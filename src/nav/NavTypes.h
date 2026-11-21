#pragma once
#include <vector>
#include <algorithm>
#include <QPointF>
#include "core/RoadGeometry.h"

/**
 * @file NavTypes.h
 * @brief Typy nawigacji: checkpointy, kroki trasy i mapowanie intencji skrętu.
 */

/**
 * @brief Punkt wjazdu/wyjazdu na krawędzi mapy.
 *
 * Łącznie 32 checkpointy (po 8 na każdą krawędź). Numeracja ID:
 *  - 0..7   — lewa krawędź   (drogi poziome, wjazd L2R = na wschód),
 *  - 8..15  — prawa krawędź  (drogi poziome, wjazd R2L = na zachód),
 *  - 16..23 — górna krawędź  (drogi pionowe, wjazd T2B = na południe),
 *  - 24..31 — dolna krawędź  (drogi pionowe, wjazd B2T = na północ).
 *
 * Użyty jako START: auto pojawia się w @ref spawnPos i jedzie do środka.
 * Użyty jako CEL: auto opuszcza mapę przez ten punkt.
 */
struct Checkpoint {
    int       id;       ///< Identyfikator checkpointu (0..31).
    int       edge;     ///< Krawędź: 0=lewa, 1=prawa, 2=góra, 3=dół.
    int       roadIdx;  ///< Numer drogi: hi (poziome) lub vi (pionowe).
    Direction entryDir; ///< Kierunek jazdy przy WJEŹDZIE tutaj.
    QPointF   worldPos; ///< Pozycja wizualna na krawędzi (świecąca kropka).
    QPointF   spawnPos; ///< Pozycja pojawienia się (lekko poza planszą).

    /// Kierunek jazdy przy WYJEŹDZIE przez ten checkpoint.
    Direction exitDir() const { return dirOpposite(entryDir); }

    /// ID węzła grafu dla tego checkpointu (skrzyżowania używają 0..N²-1).
    int nodeId() const { return kNumRoads*kNumRoads + id; }

    /// Stan wizualny w trybie wyboru trasy.
    enum class UIState { Available, IsStart, IsEnd };
    UIState uiState = UIState::Available;
};

/**
 * @brief Buduje 32 kanoniczne checkpointy dla bieżącej siatki 8×8.
 * @return Wektor checkpointów rozmieszczonych na krawędziach mapy.
 */
inline std::vector<Checkpoint> buildCheckpoints() {
    std::vector<Checkpoint> cps;
    cps.reserve(32);
    int id = 0;
    constexpr double kInset = 30.0;  // wcięcie od krawędzi dla widoczności

    // Lewa krawędź: wjazdy L2R (na wschód)
    for (int hi=0;hi<kNumRoads;++hi) {
        double laneY = kL2R_Y(hi) + kCarW/2.0;
        cps.push_back({id++, 0, hi, Direction::L2R,
                       QPointF(kInset, laneY),
                       QPointF(-kCarL-5, kL2R_Y(hi))});
    }
    // Prawa krawędź: wjazdy R2L (na zachód)
    for (int hi=0;hi<kNumRoads;++hi) {
        double laneY = kR2L_Y(hi) + kCarW/2.0;
        cps.push_back({id++, 1, hi, Direction::R2L,
                       QPointF(kCanvasW - kInset, laneY),
                       QPointF(kCanvasW+5, kR2L_Y(hi))});
    }
    // Górna krawędź: wjazdy T2B (na południe)
    for (int vi=0;vi<kNumRoads;++vi) {
        double laneX = kT2B_X(vi) + kCarW/2.0;
        cps.push_back({id++, 2, vi, Direction::T2B,
                       QPointF(laneX, kInset),
                       QPointF(kT2B_X(vi), -kCarL-5)});
    }
    // Dolna krawędź: wjazdy B2T (na północ)
    for (int vi=0;vi<kNumRoads;++vi) {
        double laneX = kB2T_X(vi) + kCarW/2.0;
        cps.push_back({id++, 3, vi, Direction::B2T,
                       QPointF(laneX, kCanvasH - kInset),
                       QPointF(kB2T_X(vi), kCanvasH+5)});
    }
    return cps;
}

// ── Odcinki jezdni (wspólne dla analizatora ruchu i wyszukiwarki tras) ──
/**
 * @brief Indeks pozycyjny odcinka 0..kNumRoads na drodze (patrz TrafficAnalyzer).
 *
 * Odcinek s leży między skrzyżowaniem s-1 a s (0 = od krawędzi mapy do
 * skrzyżowania 0, kNumRoads = od ostatniego skrzyżowania do krawędzi).
 * from / to to sąsiednie węzły w kierunku jazdy; -1 lub >= kNumRoads
 * oznacza krawędź mapy. (Dawny wzór traktował -1 zawsze jak lewą/górną
 * krawędź, przez co odcinki brzegowe R2L/B2T miały długość ~2240 px.)
 */
inline int segmentIndex(Direction d, int from, int to) {
    auto inner = [](int n) { return n >= 0 && n < kNumRoads; };
    if (inner(from) && inner(to)) return std::max(from, to);
    const bool fwd = (d == Direction::L2R || d == Direction::T2B);
    const bool nodeIsFrom = inner(from);
    const int node = nodeIsFrom ? from : to;
    if (!inner(node)) return 0;
    return fwd ? (nodeIsFrom ? node + 1 : node) : (nodeIsFrom ? node : node + 1);
}
/// Fizyczna długość odcinka (px) — od krawędzi strefy do krawędzi strefy/mapy.
inline double segmentLength(Direction d, int from, int to) {
    const int s = segmentIndex(d, from, to);
    if (dirIsHoriz(d)) {
        const double lo = (s == 0) ? 0.0 : kVLeft[s - 1] + kRoadW;
        const double hi = (s == kNumRoads) ? kCanvasW : kVLeft[s];
        return std::max(0.0, hi - lo);
    }
    const double lo = (s == 0) ? 0.0 : kHTop[s - 1] + kRoadW;
    const double hi = (s == kNumRoads) ? kCanvasH : kHTop[s];
    return std::max(0.0, hi - lo);
}

/**
 * @brief Pojedynczy krok trasy — skrzyżowanie albo checkpoint.
 */
struct RouteStep {
    int       nodeId;   ///< ID węzła grafu.
    int       vi, hi;   ///< Skrzyżowanie (-1 dla checkpointów).
    int       cpId;     ///< ID checkpointu (-1 dla skrzyżowań).
    Direction dir;      ///< Kierunek wyjścia z tego kroku.
    QPointF   worldPos; ///< Reprezentatywna pozycja świata (do rysowania).
};
/// Trasa = uporządkowana sekwencja kroków.
using Route = std::vector<RouteStep>;

/**
 * @brief Wyznacza intencję skrętu na podstawie kierunku wjazdu i wyjazdu.
 * @param from Kierunek, z którego auto przyjeżdża.
 * @param to   Kierunek, w którym auto odjeżdża.
 * @return 0=prosto, 1=w prawo, 2=w lewo, -1=niedozwolone (zawracanie).
 */
inline int computeIntent(Direction from, Direction to) {
    //           to:  L2R  R2L  T2B  B2T
    static const int tbl[4][4] = {
    //from L2R:      prosto  inv   prawo  lewo
                      {  0,  -1,   1,   2 },
    //from R2L:        inv  prosto lewo   prawo
                      { -1,   0,   2,   1 },
    //from T2B:        lewo  prawo prosto inv
                      {  2,   1,   0,  -1 },
    //from B2T:        prawo lewo  inv    prosto
                      {  1,   2,  -1,   0 },
    };
    int f = (int)from, t = (int)to;
    return tbl[f][t];
}
