/**
 * @file SimulationEngine.h
 * @brief Silnik symulacji: lista obiektów, fazy świateł, zasady ruchu, rozsuwanie kolizji.
 * @ingroup simulation_grp
 */
#ifndef SIMULATIONENGINE_H
#define SIMULATIONENGINE_H

#include <vector>
#include <memory>
#include <functional>
#include <random>
#include "RoadGeometry.h"

class SimulationObject;
class Car;

/// Stan pojedynczego sygnalizatora.
enum class LightState { Red, Yellow, Green };

/**
 * @brief Stan jednego skrzyżowania (sygnalizacja lub rondo).
 */
struct IntersectionState {
    bool       isRoundabout = false;             ///< Czy to rondo.
    LightState hLight       = LightState::Green; ///< Światło dla dróg poziomych.
    LightState vLight       = LightState::Red;   ///< Światło dla dróg pionowych.
    double     timer        = 0.0;               ///< Licznik czasu fazy.
    int        phase        = 0;                 ///< Numer aktualnej fazy.
};

using Direction = ::Direction;

/**
 * @brief Funkcja zwrotna dostarczająca intencję skrętu dla SmartCar.
 *
 * Wywoływana przez silnik przy każdej decyzji o skręcie auta GPS na
 * skrzyżowaniu. Zwraca: 0=prosto, 1=w prawo, 2=w lewo.
 */
using SmartCarIntentFn = std::function<int(int zoneID)>;

/**
 * @brief Silnik symulacji ruchu drogowego.
 *
 * Przechowuje obiekty (auta), stany skrzyżowań i logikę ruchu: sygnalizację,
 * zasady pierwszeństwa, ronda oraz specjalne auto GPS (SmartCar) sterowane
 * przez @ref SmartCarIntentFn. Umożliwia też generowanie ruchu i map.
 */
class SimulationEngine {
public:
    SimulationEngine();

    /// Aktualizuje całą symulację o krok czasowy @p deltaTime (sekundy).
    void update(double deltaTime);
    /// Dodaje jedno auto wjeżdżające z losowej krawędzi.
    void addCar();
    /// Dodaje pojedyncze auto w wybranym miejscu, kierunku i pasie.
    /// Używane głównie przez testy do tworzenia kontrolowanych scenariuszy.
    void addCar(QPointF pos, Direction dir, int road);
    /// Zwraca listę wszystkich obiektów na mapie.
    const std::vector<std::unique_ptr<SimulationObject>>& objects() const;
    /// Zwraca stan skrzyżowania (vi, hi).
    const IntersectionState& intersectionAt(int vi, int hi) const;

    /// Konfiguruje które skrzyżowania są rondami (generowanie mapy).
    void setRoundabouts(const std::vector<std::pair<int,int>>& positions);

    // ── Zarządzanie SmartCar ────────────────────────────────────────
    /// Tworzy auto GPS w podanej pozycji, kierunku i na danej drodze.
    void  spawnSmartCar(QPointF pos, Direction dir, int road);
    /// Usuwa auto GPS z mapy.
    void  removeSmartCar();
    /// Czy auto GPS istnieje.
    bool  hasSmartCar()      const;
    const Car* smartCar()    const { return m_smartCar; } ///< Auto GPS (stan manewru — nawigacja).

    QPointF   smartCarCenter()  const; ///< Środek auta GPS.
    Direction smartCarDir()     const; ///< Kierunek auta GPS.
    int       smartCarRoad()    const; ///< Numer drogi auta GPS.
    bool      smartCarOnRing()  const; ///< Czy auto GPS jest na rondzie.
    int       smartCarRingVI()  const; ///< Vi ronda na którym jest SmartCar (-1 jeśli nie na ringu).
    int       smartCarRingHI()  const; ///< Hi ronda na którym jest SmartCar (-1 jeśli nie na ringu).

    /**
     * @brief Rejestruje dostawcę intencji nawigacji.
     *
     * Po ustawieniu silnik wywołuje tę funkcję dla KAŻDEJ decyzji o skręcie
     * SmartCar zamiast losować kierunek.
     */
    void setSmartCarIntentFn(SmartCarIntentFn fn);

    /// Tryb zapasowy: ręczne ustawienie oczekującej intencji.
    void setSmartCarPending(int zoneID, int intent);

    // ── Tryb stałego ruchu ──────────────────────────────────────────
    /**
     * @brief Ustawia docelową liczbę zwykłych aut utrzymywaną na mapie.
     *
     * Gdy > 0, silnik po @ref removeOutside dosypuje auta do tej liczby.
     * SmartCar nie jest wliczany. 0 wyłącza tryb.
     */
    void setTargetCarCount(int n);
    int  targetCarCount() const { return m_targetCarCount; }
    int  regularCarCount() const;   ///< Aktualna liczba aut (bez SmartCar).

    /**
     * @brief Rozmieszcza @p count aut w losowych miejscach NA jezdniach.
     *
     * Auta nie są stawiane na skrzyżowaniach ani nie nachodzą na siebie.
     * Najpierw usuwa wszystkie istniejące zwykłe auta. Limit: 100.
     */
    void spawnCarsOnMap(int count);

    /// Pełny reset: usuwa wszystkie auta (zwykłe i SmartCar), resetuje stan
    /// świateł oraz target car count. Po reset() silnik jest w stanie jak
    /// świeżo po konstruktorze — bezpieczny dla powrotu do menu startowego.
    void reset();

    /// Czas oczekiwania na zielone dla podejścia (poziomego/pionowego), jeśli
    /// auto dojedzie do skrzyżowania za dtAhead s (cykl faz jest stały).
    static double signalWaitAt(const IntersectionState& s, bool horizontal, double dtAhead);
    /// Oczekiwany (średni) czas czekania na sygnalizacji przy losowej chwili
    /// dojazdu: E[w] = T_stop^2 / (2 T_cyklu). Niezależny od bieżącej fazy.
    static double signalExpectedWait(bool horizontal);

    /// Ustawia fazę sygnalizacji (scenariusze testowe: wymuszenie czerwonego).
    void setIntersectionPhase(int vi, int hi, int phase, double timer = 0.0);

    /// Ustawia ziarno generatora losowego silnika (determinizm scenariuszy).
    void setSeed(unsigned seed) { m_rng.seed(seed); }
    /// Licznik awaryjnych korekt nakładania (resolveOverlaps). W poprawnie
    /// sterowanym ruchu powinien pozostać 0 — każda korekta to skok pozycji.
    long overlapCorrections() const { return m_overlapCorrections; }

    // Cykl życia (publiczne na potrzeby testów)
    void removeOutside();     ///< Usuwa auta, które opuściły mapę.
    void maintainTraffic();   ///< Dosypuje auta do docelowej liczby.

private:
    std::vector<std::unique_ptr<SimulationObject>> m_objects;
    IntersectionState m_iState[kNumRoads][kNumRoads];
    int   m_nextSpawn       = 0;
    int   m_targetCarCount  = 0;    ///< 0 = wyłączony tryb stałego ruchu.
    Car*  m_smartCar        = nullptr;
    SmartCarIntentFn m_smartIntentFn;
    int   m_nextCarId       = 1;
    /// SmartCar oczekujący na wolne miejsce wjazdu (krawędź mapy bywa zajęta
    /// przez ruch tła — dawniej auto lądowało na innym aucie, było cofane za
    /// margines mapy i usuwane, a nawigacja kończyła się „zgubieniem trasy").
    struct PendingSpawn { QPointF pos; Direction dir; int road; };
    std::unique_ptr<PendingSpawn> m_pendingSmart;
    bool spawnSlotFree(QPointF pos, Direction dir) const;
    void trySpawnPendingSmart();
    long  m_overlapCorrections = 0;
    /// Generator losowy per-silnik (dawniej globalny static — dwa silniki
    /// w jednym procesie dzieliły stan, co łamało powtarzalność).
    std::mt19937 m_rng{42};
    /// Rozkład intencji skrętu zwykłych aut: 60% prosto, 20% prawo, 20% lewo.
    std::discrete_distribution<int> m_intentDist{60, 20, 20};
    void pushCar(std::unique_ptr<Car> car);   ///< Dodaje auto z nadaniem id.

    /// Maksymalny krok logiki [s]; update() dzieli dłuższe dt na podkroki.
    static constexpr double kMaxStep = 1.0 / 60.0;
    void step(double dt);                 ///< Jeden stały krok logiki.
    void applySafetyCaps(double dt);      ///< Bezpieczna prędkość (przeszkoda na torze).
    bool ringEntryClear(const Car* car, int vi, int hi) const; ///< Luka na rondzie dla wjazdu.
    void updateIntersections(double dt);  ///< Aktualizuje fazy świateł.
    void applyTrafficRules();             ///< Stosuje zasady ruchu dla aut.

    bool isTurnSafe(Direction newDir, int newRoad, QPointF endCenter, double margin = 1.0) const; ///< Czy pas docelowy skrętu jest wolny.
    bool isOncomingClear(Car* car, int vi, int hi, double margin = 1.0) const; ///< Czy pas przeciwny jest wolny (skręt w lewo); margin>1 = ostrzej (kontrola przed wjazdem).
    bool isRingExitSafe(Direction dir, int road, QPointF exitPos) const;  ///< Czy zjazd z ronda bezpieczny.
    bool exitLaneClear(Direction dir, int road, int vi, int hi, const Car* self) const; ///< Wolne miejsce za skrzyżowaniem w danym kierunku.
    bool isBoxExitClear(Car* car, int vi, int hi) const; ///< Czy za skrzyżowaniem jest miejsce (precyzyjny pomiar, jazda prosto).
    bool canEnterIntersection(Car* car, int vi, int hi) const; ///< Czy auto może w pełni przejechać (prosto LUB skręt).
    void resolveTurnTarget(Car* car, int vi, int hi, int intent,
                           Direction& nd, int& nr, QPointF& np) const; ///< Cel manewru skrętu.
    void decideTurn(Car* car, int vi, int hi);  ///< Wykonuje skręt auta na skrzyżowaniu.
    void refreshSmartCarPtr();                   ///< Odświeża wskaźnik na SmartCar.

    /**
     * @brief Końcowy etap gwarancji braku nakładania się aut.
     *
     * Po przesunięciu wszystkich aut rozsuwa pary, które mimo sterowania na
     * siebie zachodzą (typowo przejściowo przy skręcie lub na rondzie).
     * Tylne auto z pary jest cofane wzdłuż swojego toru tak, by tory się
     * stykały, ale nie przenikały — bez „losowych" zatrzymań na środku.
     */
    void resolveOverlaps();

    /// Rozstrzyga intencję skrętu auta w danej strefie.
    int resolveIntent(Car* car, int zoneID);
};

#endif
