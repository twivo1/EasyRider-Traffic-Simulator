#pragma once
#include <array>
#include "NavTypes.h"
#include "core/SimulationEngine.h"

/**
 * @brief Analizator natężenia ruchu na odcinkach jezdni.
 *
 * Skanuje wszystkie obiekty silnika i wylicza statystyki dla każdego
 * skierowanego odcinka drogi: liczbę aut, średnią prędkość oraz koszty
 * używane przez @ref DynamicPathfinder.
 *
 * Klucz odcinka: (dirIdx, roadIdx, segIdx)
 *  - L2R/R2L (dirIdx 0/1): roadIdx = hi, segIdx = vi skrzyżowania wiodącego,
 *  - T2B/B2T (dirIdx 2/3): roadIdx = vi, segIdx = hi skrzyżowania wiodącego.
 */
class TrafficAnalyzer {
public:
    explicit TrafficAnalyzer(const SimulationEngine* engine)
        : m_engine(engine) {}

    /// Ponownie skanuje auta i aktualizuje statystyki.
    void update();

    /**
     * @brief Mnożnik kosztu dla skierowanego odcinka (≥ 1.0).
     * @param dir Kierunek jazdy.
     * @param roadIdx Numer drogi (hi lub vi).
     * @param from,to Sąsiednie skrzyżowania (-1 = krawędź mapy).
     * @return Wartość ≥ 1.0; większa = bardziej zakorkowany odcinek.
     */
    double segmentCostFactor(Direction dir, int roadIdx,
                              int from, int to) const;

    /// Surowa liczba aut na odcinku (dla addytywnego modelu kosztu).
    int carsOnSegment(Direction dir, int roadIdx, int from, int to) const;
    /// Liczba aut STOJĄCYCH (v < 5 px/s) na odcinku — kolejka.
    int stoppedOnSegment(Direction dir, int roadIdx, int from, int to) const;
    /// Średnia prędkość aut na odcinku (110 px/s, gdy pusty).
    double avgSpeedOnSegment(Direction dir, int roadIdx, int from, int to) const;
    /// Liczba aut zajmujących rondo (krążące, wjeżdżające, zjeżdżające).
    int ringOccupancy(int vi, int hi) const { return m_ring[vi][hi]; }

    /// Kara za czerwone światło (w sekundach) przy wjeździe na skrzyżowanie.
    double redLightPenalty(int vi, int hi, Direction approach) const;

    /// Globalny poziom zakorkowania 0..1 (do statystyk).
    double globalCongestion() const;

    /// Liczba widocznych ognisk zatorów.
    int congestionHotspots() const;

private:
    const SimulationEngine* m_engine;

    // Liczba aut na odcinek: [dirIdx 0..3][roadIdx 0..7][segIdx 0..8]
    std::array<std::array<std::array<int,kNumRoads+1>,kNumRoads>,4> m_counts{};
    std::array<std::array<std::array<double,kNumRoads+1>,kNumRoads>,4> m_avgSpeed{};
    std::array<std::array<std::array<int,kNumRoads+1>,kNumRoads>,4> m_stopped{};
    std::array<std::array<int,kNumRoads>,kNumRoads> m_ring{};
    bool key(Direction dir, int roadIdx, int from, int to, int& di, int& si) const;

    /// Mapuje "odcinek za skrzyżowaniem from" na indeks tablicy (-1→0, 0→1, ...).
    static int segIdx(int from) { return from + 1; }


};
