#include "VisualRouteBuilder.h"
#include "RoutePolylineSanitizer.h"
#include "core/RoadGeometry.h"
#include <algorithm>
#include <cmath>

namespace VisualRouteBuilder {

// ── Niskopoziomowe: pojedynczy krok → punkty pasów ruchu ────────────
namespace {

/// Punkt WJAZDU na pas właściwego kierunku przy krawędzi skrzyżowania
/// (vi,hi). Liczony na środku pasa (offset kCarW/2), aby trasa biegła
/// dokładnie tam, gdzie jadą auta — a nie geometrycznym środkiem drogi.
QPointF laneEntry(Direction d, int vi, int hi, double ix, double iy) {
    switch(d) {
    case Direction::L2R: return {ix,            kL2R_Y(hi) + kCarW/2};
    case Direction::R2L: return {ix + kRoadW,   kR2L_Y(hi) + kCarW/2};
    case Direction::T2B: return {kT2B_X(vi) + kCarW/2, iy};
    default:             return {kB2T_X(vi) + kCarW/2, iy + kRoadW};
    }
}
/// Punkt WYJAZDU z pasa po przeciwnej stronie skrzyżowania (vi,hi).
/// Symetryczny do laneEntry — razem wyznaczają przejazd na wprost.
QPointF laneExit(Direction d, int vi, int hi, double ix, double iy) {
    switch(d) {
    case Direction::L2R: return {ix + kRoadW,   kL2R_Y(hi) + kCarW/2};
    case Direction::R2L: return {ix,            kR2L_Y(hi) + kCarW/2};
    case Direction::T2B: return {kT2B_X(vi) + kCarW/2, iy + kRoadW};
    default:             return {kB2T_X(vi) + kCarW/2, iy};
    }
}
// Punkt zagięcia trasy podczas skrętu — przecięcie pasa dojazdowego
// z pasem wyjazdowym (a NIE środek skrzyżowania, który leżał poza pasami
// i powodował V-spike w starszych wersjach).
QPointF laneCross(Direction a, Direction d, int vi, int hi,
                   double cx, double cy) {
    auto laneX = [&](Direction dd) -> double {
        if (dd == Direction::T2B) return kT2B_X(vi) + kCarW/2;
        if (dd == Direction::B2T) return kB2T_X(vi) + kCarW/2;
        return cx;
    };
    auto laneY = [&](Direction dd) -> double {
        if (dd == Direction::L2R) return kL2R_Y(hi) + kCarW/2;
        if (dd == Direction::R2L) return kR2L_Y(hi) + kCarW/2;
        return cy;
    };
    bool aH = (a == Direction::L2R || a == Direction::R2L);
    bool dH = (d == Direction::L2R || d == Direction::R2L);
    if (aH && !dH) return QPointF(laneX(d), laneY(a));  // poziom→pion
    if (!aH && dH) return QPointF(laneX(a), laneY(d));  // pion→poziom
    return QPointF(cx, cy);                              // (nie powinno wystąpić)
}

/// Czy punkt leży na JAKIEJKOLWIEK jezdni (poziomej lub pionowej)?
bool isOnRoad(QPointF p) {
    constexpr double tol = 2.0;
    for (int hi = 0; hi < kNumRoads; ++hi)
        if (p.y() >= kHTop[hi]-tol && p.y() <= kHTop[hi]+kRoadW+tol) return true;
    for (int vi = 0; vi < kNumRoads; ++vi)
        if (p.x() >= kVLeft[vi]-tol && p.x() <= kVLeft[vi]+kRoadW+tol) return true;
    return false;
}
/// Czy WNĘTRZE odcinka a→b wychodzi poza jezdnię (próbkowanie środków)?
bool segCrossesGrass(QPointF a, QPointF b) {
    for (double t = 0.15; t <= 0.85; t += 0.1) {
        QPointF m(a.x()+(b.x()-a.x())*t, a.y()+(b.y()-a.y())*t);
        if (!isOnRoad(m)) return true;
    }
    return false;
}

} // anonymous namespace

QPolygonF buildRawPolyline(const Route& route, QPointF carPos) {
    QPolygonF pts;
    if (route.size() < 2) return pts;

    for (int i = 0; i < (int)route.size(); i++) {
        const auto& step = route[i];
        if (step.vi < 0) { pts << step.worldPos; continue; }

        double ix = kVLeft[step.vi], iy = kHTop[step.hi];
        double cx = ix + kRoadW/2.0, cy = iy + kRoadW/2.0;
        Direction arrDir = step.dir;
        Direction depDir = (i + 1 < (int)route.size()) ? route[i+1].dir : arrDir;

        if (arrDir == depDir) {
            pts << laneEntry(arrDir, step.vi, step.hi, ix, iy)
                << laneExit (depDir, step.vi, step.hi, ix, iy);
        } else {
            pts << laneEntry(arrDir, step.vi, step.hi, ix, iy)
                << laneCross(arrDir, depDir, step.vi, step.hi, cx, cy)
                << laneExit (depDir, step.vi, step.hi, ix, iy);
        }
    }
    if (pts.size() < 2) return pts;

    // ── Przycięcie do pozycji auta (styl GPS z gier) ────────────────
    // Polilinia ma ZACZYNAĆ SIĘ przy aucie i pokazywać tylko to, co przed nim.
    //
    // PRZYCZYNA „trójkątów" przy skręcie: trasa z RouteManager::visualRoute()
    // jest już poprzedzona krokiem „pozycja auta" (worldPos == carPos), więc
    // pierwszym punktem polilinii JEST carPos. Stary trim brał go za najbliższy
    // segment (odległość 0) i nigdy nie odcinał tego, co ZA autem. Dodatkowo
    // dla skrzyżowania, które auto WŁAŚNIE minęło, trasa wciąż zawiera punkt
    // WJAZDOWY (po starej stronie) — powstawał skośny odcinek „w tył"
    // zamykający trójkąt, potrafiący wyjść poza jezdnię.
    //
    // NAPRAWA — dwa kroki:
    //  1) pomijamy powielony punkt startowy i szukamy NAJBLIŻSZEGO segmentu
    //     PRAWDZIWYCH pasów (rzut prostopadły, przycięty do [0,1]); rekonstrukcja
    //     zachowuje wierzchołek, do którego auto dopiero zmierza (t≈0), a pomija
    //     już minięte (t>0) — linia trzyma się jezdni i nie zawraca przy skręcie.
    //  2) gdy auto jest DALEKO PROSTOPADLE od trasy (przeoczyło skręt / trasa
    //     chwilowo nieaktualna), doczepienie carPos rysowałoby skośną przekątną
    //     (czasem przez trawę). Wtedy zaczynamy od RZUTU na jezdnię — bez ostrogi.
    //     Rozróżnienie: odległość od PROSTEJ segmentu (nie od jego końca) — mała
    //     dla auta jadącego wzdłuż pasa (dojazd), duża gdy auto jest z boku trasy.
    if (!carPos.isNull()) {
        int base = 0;                                   // pomiń powielony carPos
        if (std::hypot(pts[0].x()-carPos.x(), pts[0].y()-carPos.y()) < 1.5)
            base = 1;

        if (pts.size() - base >= 2) {
            // Auto jest ZAWSZE blisko POCZĄTKU visualRoute (trasa jest już
            // przycięta do pozycji auta). Ograniczamy szukanie do kilku
            // pierwszych segmentów — inaczej trasa, która zawraca w pobliże
            // startu (pętla), mogłaby dopasować daleki segment z jej końca
            // i skleić całość krótką przekątną.
            constexpr int kLocalSegs = 6;
            int    bestSeg = -1; double bestT = 0.0, bestD2 = 1e18;
            int    iEnd = std::min<int>(pts.size() - 1, base + kLocalSegs);
            for (int i = base; i < iEnd; ++i) {
                QPointF a = pts[i], b = pts[i+1];
                double abx = b.x()-a.x(), aby = b.y()-a.y();
                double len2 = abx*abx + aby*aby;
                if (len2 < 1e-6) continue;
                double t  = ((carPos.x()-a.x())*abx + (carPos.y()-a.y())*aby) / len2;
                double tC = std::clamp(t, 0.0, 1.0);
                double px = a.x()+abx*tC, py = a.y()+aby*tC;
                double dx = carPos.x()-px, dy = carPos.y()-py;
                double d2 = dx*dx + dy*dy;
                if (d2 < bestD2) { bestD2=d2; bestSeg=i; bestT=tC; }
            }
            if (bestSeg >= 0) {
                QPointF a = pts[bestSeg], b = pts[bestSeg+1];
                double abx = b.x()-a.x(), aby = b.y()-a.y();
                double segLen = std::hypot(abx, aby);
                // odległość auta od PROSTEJ zawierającej segment (nie od jego końca)
                double perpToLine = segLen > 1e-6
                    ? std::fabs((carPos.x()-a.x())*aby - (carPos.y()-a.y())*abx) / segLen
                    : std::hypot(carPos.x()-a.x(), carPos.y()-a.y());
                // rzut auta na wybrany segment (przycięty)
                QPointF np = a + (b - a) * std::clamp(bestT, 0.0, 1.0);

                constexpr double kSnapPerp = 45.0;       // auto realnie przy pasie
                QPolygonF trimmed;
                if (perpToLine <= kSnapPerp) {           // auto NA/wzdłuż trasy → start przy aucie
                    trimmed << carPos;
                    int firstVtx = (bestT < 0.001) ? bestSeg : bestSeg + 1;
                    for (int i = firstVtx; i < pts.size(); ++i) trimmed << pts[i];
                } else {                                 // auto POZA trasą → start na jezdni (bez ostrogi)
                    trimmed << np;
                    for (int i = bestSeg + 1; i < pts.size(); ++i) trimmed << pts[i];
                }
                if (trimmed.size() >= 2) pts = trimmed;
            }
        }
    }

    // ── Strażnik „nie wychodź poza drogę" na końcu trasy ────────────
    // Checkpointy leżą poza siatką dróg (na krawędziach mapy). Gdy trasa
    // chwilowo zwęża się do [auto, checkpoint], a auto nie jest z nim
    // wyrównane (np. nieaktualna trasa), odcinek do checkpointu przeciąłby
    // trawę. Wtedy odcinamy taki końcowy punkt — lepiej krótsza linia niż
    // przekątna przez trawnik. Wnętrze trasy jest już na jezdni z konstrukcji.
    while (pts.size() >= 2 && segCrossesGrass(pts[pts.size()-2], pts[pts.size()-1]))
        pts.removeLast();

    return pts;
}

/**
 * @brief Pełny pipeline produkujący dane AKTYWNEJ (pomarańczowej) trasy.
 *
 *  Buduje surową polilinię (z opcjonalnym przycięciem od @p carPos —
 *  patrz obrona przed *orange tail* w buildRawPolyline()), a następnie
 *  przepuszcza ją przez sanitizer.
 *
 *  SEPARACJA WARSTW: ta funkcja wypełnia WYŁĄCZNIE pola dotyczące bieżącej
 *  trasy (currentRouteFragments + rejected + diagnostyka liczbowa). Pola
 *  reprezentujące INNE pojęcia — drivenTrace (przebyta ścieżka) oraz
 *  historyRoutes (stare/przeliczone trasy) — pozostają nietknięte i są
 *  zarządzane wyżej (RouteManager/OverlayRenderer). Dzięki temu „stara"
 *  i „nowa" trasa nigdy nie zlewają się w jedną polilinię.
 *
 * @see VisualRouteData — opis zasady separacji pól.
 * @see buildRawPolyline — przycięcie GTA-style i kara za rzut poza segment.
 */
VisualRouteData build(const Route& route, QPointF carPos) {
    VisualRouteData data;
    QPolygonF raw = buildRawPolyline(route, carPos);
    data.rawPointCount = raw.size();
    RoutePolylineSanitizer::sanitizeInto(raw, data);
    return data;
}

} // namespace VisualRouteBuilder
