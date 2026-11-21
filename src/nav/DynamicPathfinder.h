#pragma once
#include "NavTypes.h"
#include "TrafficAnalyzer.h"
#include <functional>

class SimulationEngine;

/**
 * @brief Wyszukiwarka tras A* z dynamicznymi kosztami ruchu.
 *
 * Działa na skierowanym grafie 96 węzłów:
 *  - 0..63  — skrzyżowania (vi*kN+hi),
 *  - 64..71 — checkpointy lewej krawędzi,
 *  - 72..79 — checkpointy prawej krawędzi,
 *  - 80..87 — checkpointy górnej krawędzi,
 *  - 88..95 — checkpointy dolnej krawędzi.
 *
 * Stan A* = (węzeł, kod kierunku wjazdu), gdzie kod 0..3 = L2R/R2L/T2B/B2T,
 * a 4 = brak wjazdu (start). Zawracanie (U-turn) jest zabronione.
 *
 * Koszt krawędzi (węzeł A, wjazd d_in) → (węzeł B, kierunek d) to
 * PRZEWIDYWANY CZAS PRZEJAZDU [s]:
 *     movementCost(A, d_in→d)  — przejazd przez A (prosto / łuk skrętu / rondo),
 *   + segmentCost(A→B)         — odcinek: geometria, kolejka, ruch, oczekiwanie przy B.
 * Wszystkie składniki są nieujemne i skończone (FASTEST ROUTE, nie najkrótsza).
 *
 * Heurystyka: h(n) = k·|n − cel|, gdzie k = min po krawędziach (dolne
 * ograniczenie kosztu / odległość euklidesowa). Ponieważ koszt każdej krawędzi
 * ≥ jej dolne ograniczenie ≥ k·|u−v|, h jest dopuszczalna i spójna — A* zwraca
 * ten sam koszt co Dijkstra (weryfikowane testami z wyrocznią).
 */
class DynamicPathfinder {
public:
    /**
     * @brief Parametry modelu czasu przejazdu.
     *
     * Wyznaczone metodą najmniejszych kwadratów na 3007 zmierzonych przejazdach
     * krawędzi grafu (scenariusz D9, 2 ziarna, 60 aut, ronda + sygnalizacja);
     * walidacja na odłożonej połowie: MAE 2,15 s, RMSE 3,38 s, bias +0,07 s
     * (model pierwotny na tych samych danych: MAE 3,17 s, bias −1,19 s).
     * Auto jadące płynnie (≈0,8 s) jest rozróżniane od stojącego w kolejce
     * (≈1,1 s) i od wolno sunącego, które zaraz do kolejki dołączy (≈3,2 s).
     * Łuk skrętu nie jest wolniejszy od przejazdu prosto przez strefę —
     * dawna stała kara 2 s za skręt nie miała pokrycia w pomiarach.
     */
    struct CostParams {
        double vFree        = 110.0; ///< Prędkość swobodna [px/s] (kMaxSpeed).
        double queueHeadway = 1.06;  ///< Czas rozładowania jednego stojącego auta [s].
        double movingDelay  = 0.82;  ///< Opóźnienie od auta jadącego z prędkością swobodną [s].
        double movingSlowDelay = 3.2;///< Opóźnienie od auta sunącego (v→0), interpolowane liniowo [s].
        double signalScale  = 0.62;  ///< Mnożnik czasu czekania (najbliższy węzeł: część pokrywa kolejka).
        /// Mnożnik oczekiwanego czasu czekania dla dalszych węzłów (brak informacji
        /// o kolejce w chwili dojazdu → pełna wartość oczekiwana; D9n: ETA trasy).
        double signalScaleFar = 1.0;
        /// Poprawki manewru skrętu względem jazdy prosto [s]. Auta zwykłe przy braku
        /// luki jadą prosto (ich próbki zaniżają koszt skrętu — błąd selekcji), więc
        /// wartości skalibrowano na przejazdach auta nawigowanego (D9n: bias prawo
        /// −0,51 s, lewo −0,80 s przy poprawkach 0,0 / 0,07). Czas czekania na lukę
        /// przy skręcie w lewo ma „ciężki ogon" zależny od ruchu z naprzeciwka —
        /// stała go nie oddaje (próba 2,0 s pogorszyła trasy w D5s): znane ograniczenie.
        double turnRight    = 0.5;
        double turnLeft     = 0.9;
        double ringScale    = 1.0;   ///< Mnożnik geometrycznego czasu przejazdu ronda.
        double ringYield    = 0.8;   ///< Opóźnienie ustąpienia na auto zajmujące rondo [s].
        /// Stała zaniku [s] informacji o autach SUNĄCYCH (dołączą do kolejki, która
        /// zaraz się rozładuje). Pomiar opóźnionych snapshotów (D9): współczynnik
        /// 1,08 → 0,43 (2 s) → 0,25 (5 s) → 0,13 (10 s); auta stojące i jadące
        /// płynnie zachowują wartość predykcyjną (zatłoczone ciągi są trwałe).
        double slowDecay    = 3.5;
        /// Horyzont [s], do którego czas czekania na światłach liczony jest z faz
        /// (cykl jest stały i znany); dalej — wartość oczekiwana (niepewność dojazdu).
        double phaseHorizon = 8.0;
    };

    DynamicPathfinder(const SimulationEngine* engine,
                      TrafficAnalyzer*        analyzer);

    /**
     * @brief Znajduje najszybszą trasę z checkpointu startowego do docelowego.
     * @param outCost Jeśli != nullptr, zapisuje całkowity przewidywany czas [s].
     */
    Route findRoute(int startCpId, int endCpId,
                    const std::vector<Checkpoint>& checkpoints,
                    double* outCost = nullptr);

    /**
     * @brief Trasa od skrzyżowania (przeliczenie w trakcie jazdy).
     * @param forcedExit Jeśli ≥ 0 — kierunek wyjazdu z węzła startowego jest
     *        już zatwierdzony (auto w manewrze / w strefie decyzji); nowa trasa
     *        zaczyna się od legalnego przyszłego punktu decyzyjnego.
     */
    Route findRouteFrom(int startVi, int startHi, Direction startDir,
                        int endCpId,
                        const std::vector<Checkpoint>& checkpoints,
                        double* outCost = nullptr, int forcedExit = -1);

    // ── Model kosztu (publiczny: kalibracja, diagnostyka) ──────────
    /// @param arriveIn ≥ 0: przewidywany czas dojazdu [s] — oczekiwanie liczone z faz
    ///        świateł; < 0: wartość oczekiwana (stabilna, niezależna od fazy).
    double segmentCost(Direction dir, int roadIdx, int from, int to, int toVi, int toHi,
                       double departIn = -1.0, double leadTime = 0.0) const;
    double movementCost(int vi, int hi, Direction in, Direction out) const;
    CostParams& params() { return m_p; }

    // ── Interfejs weryfikacyjny (testy / wyrocznia Dijkstry) ───────
    /// Zastępuje koszt odcinka (segmentCost) własną funkcją — losowe wagi w testach.
    /// Zwrócenie +inf oznacza odcinek zamknięty (krawędź pomijana). Wartość musi
    /// być ≥ długość/vFree (inaczej heurystyka przestaje być dopuszczalna).
    using CostOverrideFn = std::function<double(Direction dir, int roadIdx, int from, int to,
                                                int toVi, int toHi)>;
    void setCostOverride(CostOverrideFn fn) { m_override = std::move(fn); }

    struct GraphEdge { int fromState; int toState; double cost; };
    /// @param startState stan startowy wyszukiwania (krawędzie z niego liczone są
    ///        z fazami świateł, jak w search()); −1 = bez wyróżnionego startu.
    std::vector<GraphEdge> exportGraph(const std::vector<Checkpoint>& cps, int startState = -1) const;
    static constexpr int stateCount() { return kStates; }
    int startStateOfCheckpoint(int cpId) const { return state(cpNode(cpId), 4); }
    int startStateOfIntersection(int vi, int hi, Direction d) const { return state(intNode(vi, hi), (int)d); }
    int goalNodeOfCheckpoint(int cpId) const { return cpNode(cpId); }
    int nodeOfState(int s) const { return stateNode(s); }
    /// Koszt przejazdu podaną trasą w bieżącym snapshocie (−1 gdy trasa nielegalna).
    double evaluateRoute(const Route& r, const std::vector<Checkpoint>& cps) const;
    int lastExpansions() const { return m_lastExpansions; }
    /// Liczba napotkanych i skorygowanych niepoprawnych kosztów (NaN/ujemne) — I8.
    int invalidCostCount() const { return m_invalidCosts; }

private:
    CostOverrideFn m_override;
    CostParams     m_p;
    double         m_hPerPx = 0.0;       ///< Współczynnik heurystyki [s/px].
    mutable int    m_lastExpansions = 0;
    mutable int    m_invalidCosts = 0;
    const SimulationEngine* m_engine;
    TrafficAnalyzer*        m_analyzer;

    static constexpr int kN      = kNumRoads;
    static constexpr int kNodes  = kN*kN + kN*4;   ///< 64 + 32 = 96 węzłów.
    static constexpr int kStates = kNodes * 5;     ///< 480 stanów.

    int  intNode(int vi, int hi)  const { return vi*kN+hi; }
    int  cpNode(int cpId)         const { return kN*kN+cpId; }
    bool isIntNode(int n)         const { return n < kN*kN; }
    int  nodeVi(int n)            const { return n / kN; }
    int  nodeHi(int n)            const { return n % kN; }

    int  state(int node, int dirCode) const { return node*5 + dirCode; }
    int  stateNode(int s)             const { return s / 5; }
    int  stateDirCode(int s)          const { return s % 5; }

    struct Edge { int toState; double cost; Direction dir; };
    std::vector<Edge> edges(int stateId, const std::vector<Checkpoint>& cps, bool isStart = false) const;

    double  heuristic(int nodeId, int goalNode, const std::vector<Checkpoint>& cps) const;
    QPointF nodeWorldPos(int nodeId, const std::vector<Checkpoint>& cps) const;
    double  segCost(Direction dir, int roadIdx, int from, int to, int toVi, int toHi, double departIn) const;
    /// Szacowany czas dojazdu od początku wyszukiwania do początku odcinka (odległość/vFree).
    double  leadTimeTo(QPointF segStart) const;
    void    setOrigin(int startState, const std::vector<Checkpoint>& cps) const;
    mutable QPointF m_org; mutable bool m_orgSet = false;
    double  sanitize(double c) const;
    void    computeHeuristicScale();

    Route search(int startState, int goalNode, int startCpId, int endCpId,
                 const std::vector<Checkpoint>& cps, double* outCost, int forcedExit);
    Route buildRoute(const std::vector<int>& prevState,
                     int goalState, int startCpId, int endCpId,
                     const std::vector<Checkpoint>& cps) const;
};
