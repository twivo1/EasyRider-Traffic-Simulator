#include "RoutePicker.h"
#include "../core/RoadGeometry.h"
#include <QPainterPath>
#include <QFontMetricsF>
#include <QFont>
#include <cmath>
#include <algorithm>

namespace {
// Paleta znaczników — wyłącznie dla warstwy wyboru trasy. Celowo bez kolorów
// sygnalizacji (czerwony/żółty/zielony); różnice etapów niesie też KSZTAŁT.
const QColor kCyan("#2EC4D6"), kCyanHover("#6FE0EB"), kInk("#0B2530"), kPaper("#F4F6F6"),
             kWhite("#FFFFFF"), kLeader(232, 238, 240, 210), kTabHover("#16404D");

double laneCross(Direction d, int road) {
    switch (d) {
    case Direction::L2R: return kL2R_Y(road) + kCarW / 2.0;
    case Direction::R2L: return kR2L_Y(road) + kCarW / 2.0;
    case Direction::T2B: return kT2B_X(road) + kCarW / 2.0;
    default:             return kB2T_X(road) + kCarW / 2.0;
    }
}
QPointF dirVec(Direction d) {
    switch (d) {
    case Direction::L2R: return { 1, 0 };
    case Direction::R2L: return { -1, 0 };
    case Direction::T2B: return { 0, 1 };
    default:             return { 0, -1 };
    }
}
QRectF square(QPointF c, double side) { return QRectF(c.x() - side / 2, c.y() - side / 2, side, side); }

/// Grot strzałki (szewron) wskazujący kierunek @p v, wpisany w znacznik.
void chevron(QPainter& p, QPointF c, QPointF v, double s, const QColor& col, double w) {
    const QPointF n(-v.y(), v.x());
    QPainterPath path;
    path.moveTo(c - v * s * 0.45 + n * s * 0.55);
    path.lineTo(c + v * s * 0.45);
    path.lineTo(c - v * s * 0.45 - n * s * 0.55);
    p.setPen(QPen(col, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
}
QString plural(int n) {
    if (n == 1) return QStringLiteral("1 punkt");
    const int d = n % 10, t = n % 100;
    if (d >= 2 && d <= 4 && !(t >= 12 && t <= 14)) return QString("%1 punkty").arg(n);
    return QString("%1 punktów").arg(n);
}
} // namespace

QRectF RoutePicker::instructionRect(QSizeF vp) {
    const double w = std::min(340.0, vp.width() - 24), h = 54;
    return QRectF((vp.width() - w) / 2, 52, w, h);
}

RoutePicker::Layout RoutePicker::layout(const Input& in) {
    Layout L;
    if (!in.cps || in.stage == Stage::None) return L;
    const QRectF view(0, 0, in.viewport.width(), in.viewport.height());
    const double z = std::hypot(in.world.m11(), in.world.m12());

    // Sygnalizatory w widoku (ekran) — przeszkody.
    for (const QRectF& r : in.signalWorldRects) {
        const QRectF s = in.world.mapRect(r);
        if (s.intersects(view.adjusted(-40, -40, 40, 40))) L.signalScreenRects.push_back(s);
    }

    // ── Kotwice i podział: widoczne / poza widokiem ─────────────────────
    std::vector<Marker> ms;
    for (const Checkpoint& cp : *in.cps) {
        const bool sel = cp.id < (int)in.selectable.size() && in.selectable[cp.id];
        const bool isStart = (in.stage == Stage::End && cp.id == in.startId);
        if (!sel && !isStart) continue;
        Marker m; m.cpId = cp.id; m.selectable = sel; m.isStart = isStart;
        // Etap START i wybrany START: pas wjazdowy; etap CEL: pas wyjazdowy.
        m.dir = (in.stage == Stage::Start || isStart) ? cp.entryDir : cp.exitDir();
        const double lc = laneCross(m.dir, cp.roadIdx);
        QPointF a;
        switch (cp.edge) {
        case 0:  a = { 0.0, lc }; break;
        case 1:  a = { kCanvasW, lc }; break;
        case 2:  a = { lc, 0.0 }; break;
        default: a = { lc, kCanvasH }; break;
        }
        m.anchor = in.world.map(a);
        const QRectF inside = view.adjusted(-0.5, -0.5, 0.5, 0.5);
        if (!inside.contains(m.anchor)) {
            const double ox = std::max(view.left() - m.anchor.x(), m.anchor.x() - view.right());
            const double oy = std::max(view.top() - m.anchor.y(), m.anchor.y() - view.bottom());
            m.offSide = (ox >= oy) ? (m.anchor.x() < view.left() ? Left : Right)
                                   : (m.anchor.y() < view.top() ? Top : Bottom);
        }
        ms.push_back(m);
    }

    // ── Zakładki dla punktów poza widokiem (klikalne: przesuwają kamerę) ──
    for (int side = 0; side < 4; ++side) {
        int n = 0; double sum = 0;
        for (const Marker& m : ms)
            if (m.selectable && m.offSide == side) { ++n; sum += (side <= Right) ? m.anchor.y() : m.anchor.x(); }
        if (!n) continue;
        const double w = 118, h = 28, inset = 6;
        double along = sum / n;
        QRectF r;
        if (side <= Right) {
            along = std::clamp(along, h / 2 + inset, view.height() - h / 2 - inset);
            r = QRectF(side == Left ? inset : view.width() - w - inset, along - h / 2, w, h);
        } else {
            along = std::clamp(along, w / 2 + inset, view.width() - w / 2 - inset);
            r = QRectF(along - w / 2, side == Top ? inset : view.height() - h - inset, w, h);
        }
        L.tabs.push_back({ side, n, r });
    }

    // ── Rozmieszczenie widocznych znaczników ─────────────────────────────
    std::vector<QRectF> placedHits;
    for (const Tab& t : L.tabs) placedHits.push_back(t.rect.adjusted(-kGap, -kGap, kGap, kGap));
    if (!in.instructionRect.isEmpty()) placedHits.push_back(in.instructionRect.adjusted(-kGap, -kGap, kGap, kGap));
    // Wybrany START ma pierwszeństwo miejsca (rysowany jako pierwszy punkt odniesienia).
    std::stable_sort(ms.begin(), ms.end(), [](const Marker& a, const Marker& b) { return a.isStart > b.isStart; });

    for (Marker& m : ms) {
        if (m.offSide >= 0) { L.markers.push_back(m); continue; }
        const Checkpoint& cp = (*in.cps)[m.cpId];
        // Kierunek „do wnętrza mapy" i strona zewnętrzna pasa (z dala od osi drogi).
        QPointF inward;
        switch (cp.edge) { case 0: inward = { 1, 0 }; break; case 1: inward = { -1, 0 }; break;
                           case 2: inward = { 0, 1 }; break; default: inward = { 0, -1 }; }
        const bool horiz = cp.edge <= 1;
        const double roadCenter = horiz ? kHTop[cp.roadIdx] + kRoadW / 2.0 : kVLeft[cp.roadIdx] + kRoadW / 2.0;
        const double laneOff = laneCross(m.dir, cp.roadIdx) - roadCenter;
        const QPointF outer = horiz ? QPointF(0, laneOff >= 0 ? 1 : -1) : QPointF(laneOff >= 0 ? 1 : -1, 0);
        const double toRoadEdge = (kRoadW / 2.0 - std::abs(laneOff)) * z;
        const double shapeHalf = m.isStart || in.stage == Stage::Start ? kRadius + 3 : kSquare / 2 + 2;
        // Pierwsze skrzyżowanie od krawędzi (strefa lub powierzchnia ronda) — przeszkoda.
        const int vi = horiz ? (cp.edge == 0 ? 0 : kNumRoads - 1) : cp.roadIdx;
        const int hi = horiz ? cp.roadIdx : (cp.edge == 2 ? 0 : kNumRoads - 1);
        const QPointF ic(kVLeft[vi] + kRoadW / 2.0, kHTop[hi] + kRoadW / 2.0);
        // Rondo przy krawędzi: znacznik nie zasłania pasa ruchu ronda (okrąg r = kRingR,
        // nadwozia do kRingR + W/2) — przeszkoda kołowa, nie prostokąt obejmujący.
        const bool ring = (in.roundabouts >> (vi * kNumRoads + hi)) & 1u;
        const QPointF ringC = in.world.map(ic);
        const double ringR = (kRingR + kCarW / 2.0 + 2.0) * z;
        auto onRing = [&](const QRectF& r) {   // odległość prostokąta od środka ronda < promień
            const double dx = std::max({ r.left() - ringC.x(), 0.0, ringC.x() - r.right() });
            const double dy = std::max({ r.top() - ringC.y(), 0.0, ringC.y() - r.bottom() });
            return ring && std::hypot(dx, dy) < ringR;
        };

        auto valid = [&](QPointF c) {
            const QRectF hit = square(c, kHit), shape = square(c, 2 * shapeHalf);
            if (!view.contains(hit)) return false;
            const QRectF guard = shape.adjusted(-kGap, -kGap, kGap, kGap);
            for (const QRectF& s : L.signalScreenRects) if (guard.intersects(s)) return false;
            if (onRing(shape)) return false;
            for (const QRectF& h : placedHits) if (hit.intersects(h)) return false;
            return true;
        };
        // Kandydaci: wzdłuż pasa w głąb mapy × przesunięcia poprzeczne na obie strony
        // drogi; wybierany NAJBLIŻSZY wolny punkt (najkrótsza linia do kotwicy).
        const double toInnerEdge = (kRoadW / 2.0 + std::abs(laneOff)) * z;
        std::vector<double> lats = { 0, 6, -6, 12, -12 };
        for (double k : { 0.0, 12.0, 24.0, 36.0 }) {
            lats.push_back(toRoadEdge + shapeHalf + 3 + k);
            lats.push_back(-(toInnerEdge + shapeHalf + 3 + k));
        }
        std::vector<std::pair<double, QPointF>> cand;
        const QPointF base = m.anchor + inward * (kHit / 2 + 1);
        for (double lat : lats)
            for (double d = kHit / 2 + 1; d <= kHit / 2 + 1 + 120; d += 3) {
                const QPointF c = m.anchor + inward * d + outer * lat;
                cand.push_back({ QLineF(c, base).length(), c });
            }
        std::stable_sort(cand.begin(), cand.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        bool found = false; QPointF best;
        for (const auto& [dist, c] : cand) if (valid(c)) { best = c; found = true; break; }
        if (!found) {   // brak wolnego miejsca — znacznik w położeniu bazowym, oznaczony jako konflikt
            best = m.anchor + inward * (kHit / 2 + 1);
            m.conflict = true;
        }
        m.center = best;
        m.visible = true;
        m.hit = m.selectable ? square(best, kHit) : QRectF();
        m.leader = QLineF(best, m.anchor).length() > shapeHalf + kHit / 2 - 2 + 6;
        placedHits.push_back(square(best, kHit));
        if (m.isStart) {
            // Etykieta START: pierwsze wolne miejsce obok znacznika (bez sygnalizatorów,
            // innych znaczników, węzła i karty), w całości w widoku.
            QFont f("Segoe UI", 8, QFont::Bold);
            const double w = QFontMetricsF(f).horizontalAdvance("START") + 12, h = 17;
            // 8 kierunków × 2 odległości, w kolejności preferencji (z dala od wnętrza skrzyżowania).
            const QPointF dirs[8] = { outer, -outer, inward, -inward, outer + inward, -outer + inward, outer - inward, -outer - inward };
            for (int k = 0; k < 16; ++k) {
                const QPointF d = dirs[k % 8];
                const double dl = std::hypot(d.x(), d.y());
                const QPointF u = d / dl;
                const double off = kHit / 2 + 3 + (k >= 8 ? 14 : 0)
                                 + std::min(std::abs(u.x()) > 1e-9 ? (w / 2) / std::abs(u.x()) : 1e9,
                                            std::abs(u.y()) > 1e-9 ? (h / 2) / std::abs(u.y()) : 1e9);
                const QPointF c = best + u * off;
                QRectF lr(c.x() - w / 2, c.y() - h / 2, w, h);
                // Przy krawędzi widoku etykieta przylega do niej zamiast z niej wystawać.
                lr.moveLeft(std::clamp(lr.left(), 2.0, view.width() - w - 2));
                lr.moveTop(std::clamp(lr.top(), 2.0, view.height() - h - 2));
                if (lr.intersects(square(best, 2 * shapeHalf))) continue;   // nie na znaczniku
                bool ok = view.contains(lr) && !onRing(lr);
                for (const QRectF& s : L.signalScreenRects) if (ok && lr.adjusted(-kGap, -kGap, kGap, kGap).intersects(s)) ok = false;
                for (const QRectF& hh : placedHits) if (ok && lr.intersects(hh)) ok = false;
                if (ok) { m.label = lr; placedHits.push_back(lr); break; }
            }
        }
        L.markers.push_back(m);
    }
    return L;
}

void RoutePicker::drawInstruction(QPainter& p, const QRectF& r, Stage stage) {
    if (stage == Stage::None) return;
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor("#D0CEC8"), 1));
    p.setBrush(QColor("#F8F7F4"));
    p.drawRoundedRect(r, 6, 6);
    // Miniatura znacznika z bieżącego etapu — wiąże instrukcję z kształtem na mapie.
    const QPointF ic(r.left() + 26, r.center().y());
    if (stage == Stage::Start) {
        p.setPen(QPen(kInk, 1.5)); p.setBrush(kCyan); p.drawEllipse(ic, 10, 10);
        chevron(p, ic, { 1, 0 }, 10, kInk, 2.0);
    } else {
        p.setPen(QPen(kInk, 2)); p.setBrush(kPaper); p.drawRoundedRect(square(ic, 18), 4, 4);
        chevron(p, ic, { 1, 0 }, 9, kInk, 2.0);
    }
    QFont title("Segoe UI", 11, QFont::Bold), sub("Segoe UI", 9);
    const QRectF tr(r.left() + 46, r.top() + 7, r.width() - 54, 22), sr(r.left() + 46, r.top() + 28, r.width() - 54, 18);
    p.setFont(title); p.setPen(QColor("#242424"));
    p.drawText(tr, Qt::AlignLeft | Qt::AlignVCenter,
               stage == Stage::Start ? QString::fromUtf8("1 / 2 — WYBIERZ START") : QString::fromUtf8("2 / 2 — WYBIERZ CEL"));
    p.setFont(sub); p.setPen(QColor("#666660"));
    p.drawText(sr, Qt::AlignLeft | Qt::AlignVCenter,
               stage == Stage::Start ? QString::fromUtf8("Kliknij oznaczony punkt wjazdu na mapę.")
                                     : QString::fromUtf8("Kliknij oznaczony punkt wyjazdu z mapy."));
    p.restore();
}

void RoutePicker::draw(QPainter& p, const Layout& l, Stage stage, int hoverCp, int hoverTab) {
    if (stage == Stage::None) return;
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    for (const Marker& m : l.markers) {
        if (!m.visible) continue;
        const bool hov = (m.cpId == hoverCp) && m.selectable;
        const QPointF v = dirVec(m.dir);
        const bool circle = (stage == Stage::Start) || m.isStart;
        const double half = circle ? kRadius : kSquare / 2;
        if (m.leader) {
            const QPointF d = m.center - m.anchor;
            const double len = std::hypot(d.x(), d.y());
            const QPointF end = m.center - d / len * (half + 2);
            p.setPen(QPen(kLeader, 1.5, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(m.anchor, end);
            p.setPen(QPen(kInk, 1.2)); p.setBrush(circle ? kCyan : kPaper);
            p.drawEllipse(m.anchor, 3.2, 3.2);
        }
        if (circle) {
            // START: koło z grotem skierowanym DO mapy (wjazd).
            const double ring = (hov || m.isStart) ? 3.0 : 2.0;
            p.setPen(Qt::NoPen); p.setBrush(kWhite);
            p.drawEllipse(m.center, half + ring + (hov ? 2 : 0), half + ring + (hov ? 2 : 0));
            p.setBrush(hov ? kCyanHover : kCyan);
            p.setPen(QPen(kInk, 1.5));
            p.drawEllipse(m.center, half + (hov ? 2 : 0), half + (hov ? 2 : 0));
            chevron(p, m.center, v, half * 1.05, kInk, 2.6);
        } else {
            // CEL: zaokrąglony kwadrat z grotem skierowanym NA ZEWNĄTRZ mapy (wyjazd).
            const double s = kSquare + (hov ? 4 : 0);
            p.setPen(Qt::NoPen); p.setBrush(hov ? kCyan : kInk);
            p.drawRoundedRect(square(m.center, s + (hov ? 6 : 4)), 6, 6);
            p.setBrush(hov ? kWhite : kPaper);
            p.drawRoundedRect(square(m.center, s), 4, 4);
            chevron(p, m.center, v, s * 0.5, kInk, 2.6);
        }
        if (m.isStart && !m.label.isEmpty()) {
            QFont f("Segoe UI", 8, QFont::Bold); p.setFont(f);
            p.setPen(QPen(kWhite, 1.2)); p.setBrush(kInk); p.drawRoundedRect(m.label, 4, 4);
            p.setPen(kWhite); p.drawText(m.label, Qt::AlignCenter, "START");
        }
    }
    // Zakładki „punkty poza widokiem" — klikalne.
    for (int i = 0; i < (int)l.tabs.size(); ++i) {
        const Tab& t = l.tabs[i];
        const bool hov = (t.side == hoverTab);
        p.setPen(QPen(kCyan, 1.5)); p.setBrush(hov ? kTabHover : kInk);
        p.drawRoundedRect(t.rect, t.rect.height() / 2, t.rect.height() / 2);
        const QPointF v = t.side == Left ? QPointF(-1, 0) : t.side == Right ? QPointF(1, 0)
                        : t.side == Top ? QPointF(0, -1) : QPointF(0, 1);
        const QPointF ic(t.rect.left() + 16, t.rect.center().y());
        chevron(p, ic, v, 10, kCyan, 2.2);
        QFont f("Segoe UI", 9, QFont::Bold); p.setFont(f); p.setPen(kWhite);
        p.drawText(t.rect.adjusted(28, 0, -8, 0), Qt::AlignLeft | Qt::AlignVCenter, plural(t.count));
    }
    p.restore();
}

int RoutePicker::hitMarker(const Layout& l, QPointF pos) {
    // Hitboxy nie nachodzą na siebie (gwarantuje layout), więc wynik jest jednoznaczny.
    for (const Marker& m : l.markers)
        if (m.visible && m.selectable && m.hit.contains(pos)) return m.cpId;
    return -1;
}
int RoutePicker::hitTab(const Layout& l, QPointF pos) {
    for (const Tab& t : l.tabs) if (t.rect.contains(pos)) return t.side;
    return -1;
}
