#pragma once
#include <QObject>
#include <QTimer>
#include <unordered_map>
#include "NavTypes.h"
#include "TrafficAnalyzer.h"
#include "DynamicPathfinder.h"

class SimulationEngine;

/**
 * @brief Zarządza pełnym cyklem życia nawigacji GPS.
 *
 * Wykorzystuje mechanizm @ref SmartCarIntentFn, dzięki czemu silnik ZAWSZE
 * pyta menedżera trasy o decyzję skrętu SmartCar na każdym skrzyżowaniu —
 * eliminuje to problemy z synchronizacją "oczekującej" intencji. Co kilka
 * sekund przelicza trasę względem aktualnego ruchu i pozycji auta.
 */
class RouteManager : public QObject {
    Q_OBJECT
public:
    explicit RouteManager(SimulationEngine* engine, QObject* parent = nullptr);
    ~RouteManager();

    void tick();   ///< Wywoływane PRZED engine->update().

    /// Rozpoczyna nawigację między checkpointami start i cel.
    bool startNavigation(int startCpId, int endCpId);
    /// Przerywa nawigację i usuwa SmartCar.
    void stopNavigation();

    bool isNavigating() const { return m_navigating; }
    const Route&                    currentRoute()  const { return m_route; }
    const std::vector<Checkpoint>&  checkpoints()   const { return m_checkpoints; }

    /**
     * @brief Zwraca trasę przyciętą do aktualnej pozycji SmartCar.
     *
     * Działa jak GPS w grach (Forza Horizon, GTA V): linia zaczyna się
     * od pozycji auta i obejmuje tylko odcinki Z PRZODU. Wywoływane
     * co klatkę w MainWindow::tick(), aby widok miał aktualną trasę.
     */
    Route visualRoute() const;

    double routeCost()       const { return m_routeCost;   } ///< Przewidywany pozostały czas [s].
    int    recalcCount()     const { return m_recalcCount; } ///< Liczba przeliczeń trasy.
    int    congestionCount() const { return m_analyzer.congestionHotspots(); } ///< Liczba korków.
    double estTimeSeconds()  const { return m_routeCost; }   ///< Szacowany czas przejazdu.

signals:
    void routeChanged(Route newRoute);  ///< Trasa uległa zmianie.
    void navigationFinished();          ///< Prawidłowe dotarcie do celu.
    void navigationFailed();            ///< Auto opuściło mapę w złym miejscu.

private slots:
    void onRecalcTimer();   ///< Cykliczne przeliczenie trasy.

private:
    SimulationEngine*       m_engine;
    TrafficAnalyzer         m_analyzer;
    DynamicPathfinder       m_pathfinder;
    std::vector<Checkpoint> m_checkpoints;

    bool    m_navigating  = false;
    int     m_startCpId   = -1;
    int     m_endCpId     = -1;
    Route   m_route;
    double  m_routeCost   = 0.0;
    int     m_recalcCount = 0;
    int     m_stuckTicks  = 0;             ///< Jak długo SmartCar się nie ruszał.
    bool    m_arrivedCorrectly = false;    ///< Czy dotarł do właściwego celu.
    QPointF m_lastSmartPos;                ///< Poprzednia pozycja (detekcja utknięcia).
    QTimer  m_recalcTimer;

    // ── GPS-style real-time recalc ──────────────────────────────────
    int     m_tickCounter = 0;             ///< Licznik klatek do rate-limitingu przeliczania.
    int     m_lastZoneId  = -1;            ///< Ostatnia strefa (vi*N+hi) — recalc przy zmianie.
    Direction m_lastDir   = Direction::L2R;///< Ostatni kierunek jazdy — recalc przy zmianie.

    /// Mapa: zoneID → ABSOLUTNY kierunek wyjazdu ze skrzyżowania.
    /// Przechowujemy kierunek zamiast relatywnej intencji (prawo/lewo/prosto),
    /// bo SmartCar może dotrzeć do skrzyżowania z innej strony niż planowano.
    /// Relatywną intencję obliczamy dopiero w momencie decyzji,
    /// porównując AKTUALNY kierunek jazdy z zapisanym kierunkiem wyjazdu.
    std::unordered_map<int, Direction> m_zoneExitDirs;

    void rebuildIntentMap();           ///< Przebudowuje mapę kierunków z trasy.
    int  intentForZone(int zoneID) const; ///< Intencja dla strefy (obliczona dynamicznie).
    bool checkArrival();               ///< Sprawdza dotarcie do celu.
    void recalcFromCurrentPos();       ///< Przelicza trasę od pozycji SmartCar.

    /// Znajduje skrzyżowanie, na którym/przy którym jest SmartCar.
    bool findCarIntersection(int& outVi, int& outHi) const;
    double distanceToNextIntersection() const;
    /// Punkt planowania: najbliższy węzeł przed autem i (jeśli manewr w nim jest
    /// już zatwierdzony) wymuszony kierunek wyjazdu. Zwraca false bez auta.
    bool   planningOrigin(int& vi, int& hi, Direction& in, int& forcedExit) const;
    /// Pozostała część bieżącej trasy od stanu (vi,hi,in); pusta gdy auto zboczyło.
    Route  currentSuffix(int vi, int hi, Direction in) const;
    int    m_rerouteRejected = 0;          ///< Odrzucone alternatywy (histereza).
public:
    int    rerouteRejected() const { return m_rerouteRejected; }
    /// Dostęp do wyszukiwarki (testy porównawcze: inna polityka kosztów).
    DynamicPathfinder& pathfinder() { return m_pathfinder; }
    /// Minimalna korzyść uzasadniająca zmianę trasy: max(abs, rel·pozostały czas).
    static constexpr double kMinGainAbs = 2.5;
    static constexpr double kMinGainRel = 0.10;
private:
};
