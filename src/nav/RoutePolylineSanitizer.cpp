#include "RoutePolylineSanitizer.h"
#include <cmath>

namespace RoutePolylineSanitizer {

void sanitizeInto(const QPolygonF& raw, VisualRouteData& outData) {
    outData.currentRouteFragments.clear();
    outData.duplicatesRemoved = 0;
    outData.invalidPointsRemoved = 0;
    outData.discontinuitiesFound = 0;
    outData.finalPointCount = 0;
    if (raw.size() < 2) return;

    // ── Etap 1: usuń niepoprawne + duplikaty ────────────────────────
    QPolygonF clean;
    for (const QPointF& q : raw) {
        if (!std::isfinite(q.x()) || !std::isfinite(q.y())) {
            outData.invalidPointsRemoved++;
            continue;
        }
        if (!clean.isEmpty()) {
            const QPointF& last = clean.back();
            double dx = q.x() - last.x(), dy = q.y() - last.y();
            if (std::sqrt(dx*dx + dy*dy) < kRoutePointDuplicateThreshold) {
                outData.duplicatesRemoved++;
                continue;
            }
        }
        clean << q;
    }
    if (clean.size() < 2) return;

    // ── Etap 2: podział na fragmenty przy długich segmentach ───────
    // Segment > kMaxSaneRouteSegment uznawany jest za artefakt — np. resztę
    // ze starej trasy podpięte do nowej, lub diagonalna „tail" przy
    // wyjeździe z ronda. NIE łączymy go w fragmentach; zamiast tego
    // bieżący fragment zamykamy, segment zapisujemy do `rejected`, a nowy
    // fragment zaczyna się od punktu po przerwie.
    QPolygonF current;
    current << clean[0];
    for (int i = 1; i < clean.size(); ++i) {
        const QPointF& a = clean[i-1];
        const QPointF& b = clean[i];
        double dx = b.x() - a.x(), dy = b.y() - a.y();
        double d  = std::sqrt(dx*dx + dy*dy);
        if (d > kMaxSaneRouteSegment) {
            // Zamknij bieżący fragment
            if (current.size() >= 2) outData.currentRouteFragments.push_back(current);
            current.clear();
            // Zaraportuj odrzucenie
            RejectedSegment rs;
            rs.from = a;
            rs.to   = b;
            rs.length = d;
            rs.maxAllowed = kMaxSaneRouteSegment;
            rs.reasonPL = QString::fromUtf8(
                "Wykryto przerwę w trasie — segment %1 px > limit %2 px (pominięty)")
                .arg((int)d).arg((int)kMaxSaneRouteSegment);
            outData.rejected.push_back(rs);
            outData.discontinuitiesFound++;
        }
        current << b;
    }
    if (current.size() >= 2) outData.currentRouteFragments.push_back(current);

    // ── Diagnostyka ────────────────────────────────────────────────
    int total = 0;
    for (const auto& f : outData.currentRouteFragments) total += f.size();
    outData.finalPointCount = total;
}

} // namespace RoutePolylineSanitizer
