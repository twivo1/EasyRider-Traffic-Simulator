#pragma once
/**
 * @file SimMonitor.h
 * @brief Niezależny walidator inwariantów symulacji (diagnostyka/testy).
 *
 * Monitor NIE ingeruje w silnik — po każdym kroku odczytuje stan wszystkich
 * aut i sprawdza inwarianty I1–I4, I7, I10 z audytu mechaniki:
 *  - I1  wartości skończone (pozycja, prędkość, kurs),
 *  - I2  auto na legalnym torze (oś pasa, okrąg ronda albo tor manewru),
 *  - I3  brak nieuzasadnionych skoków pozycji / kursu / cofnięć,
 *  - I4  brak nakładania się RZECZYWISTYCH obrysów (obrócone prostokąty, SAT),
 *  - I5  minimalny odstęp między obrysami (statystyka + licznik styków),
 *  - I7  legalne następstwa odcinków (skręt tylko na skrzyżowaniu, bez U-turn),
 *  - kontrola trajektorii między próbkami (stan w połowie kroku).
 *
 * Koszt O(n²) na krok — przeznaczony do testów, nie do pętli produkcyjnej.
 * Każde naruszenie zapisuje tick, id aut i szczegóły (odtwarzalność).
 */
#include <QPointF>
#include <QString>
#include <QStringList>
#include <unordered_map>
#include <vector>
#include "RoadGeometry.h"

class SimulationEngine;

struct MonitorStats {
    long   ticks = 0;
    double simTime = 0.0;
    int    maxCars = 0;
    int    spawned = 0;            ///< Auta zaobserwowane po raz pierwszy.
    int    exited = 0;             ///< Auta, które zniknęły przy krawędzi mapy.
    int    vanished = 0;           ///< Auta, które zniknęły w środku mapy (błąd).

    long   nonFinite = 0;          ///< I1
    long   offPath = 0;            ///< I2
    double maxOffPath = 0.0;
    long   teleports = 0;          ///< I3: przemieszczenie > v·dt + tolerancja
    double maxExcessJump = 0.0;    ///< Największy nadmiar ponad dozwolone przemieszczenie.
    long   reverseJumps = 0;       ///< I3: ruch wstecz (cofnięcie przez korektę).
    long   headingJumps = 0;       ///< I3: skok kursu > limit w jednym kroku.
    double maxHeadingStep = 0.0;   ///< [deg]
    long   overlaps = 0;           ///< I4: liczba (para, tick) z nakładaniem.
    long   overlapPairs = 0;       ///< Liczba różnych par, które kiedykolwiek się nałożyły.
    double maxPenetration = 0.0;   ///< [px]
    long   sweptHits = 0;          ///< Nakładanie w połowie kroku (między próbkami).
    long   contacts = 0;           ///< I5: odstęp < contactGap (bez nakładania).
    double minGap = 1e9;           ///< Minimalny odstęp obrysów (px) w całym przebiegu.
    long   illegalTransitions = 0; ///< I7: zmiana odcinka poza skrzyżowaniem.
    long   uturns = 0;             ///< I7: zawrócenie na skrzyżowaniu.
    long   engineCorrections = 0;  ///< Korekty resolveOverlaps (skoki wymuszone).
    double maxStoppedTime = 0.0;   ///< Najdłuższy ciągły postój jednego auta [s].
    double meanSpeed = 0.0;        ///< Średnia prędkość (po autach i krokach).
    long   speedSamples = 0;
    double sumTripTime = 0.0;      ///< Suma czasów przejazdu aut, które opuściły mapę.
    int    tripCount = 0;

    long totalViolations() const {
        return nonFinite + offPath + teleports + reverseJumps + headingJumps
             + overlaps + sweptHits + illegalTransitions + uturns + vanished;
    }
};

class SimMonitor {
public:
    /// @param contactGap Odstęp [px], poniżej którego para liczona jest jako styk.
    explicit SimMonitor(double contactGap = 1.0) : m_contactGap(contactGap) {}

    /// Wywoływać po KAŻDYM engine.update(dt) z tym samym dt.
    void observe(const SimulationEngine& engine, double dt);

    const MonitorStats& stats() const { return m_s; }
    /// Pierwsze naruszenia (tick, auta, szczegóły) — do raportu.
    const QStringList& violations() const { return m_log; }
    /// Liczba aut, które stoją (v<1) dłużej niż @p seconds w chwili obecnej.
    int carsStoppedLongerThan(double seconds) const;

    /// Pomocnicze: odstęp między obrysami dwóch aut (ujemny = penetracja).
    static double obbGap(QPointF ca, double ha, QPointF cb, double hb,
                         double L = kCarL, double W = kCarW);

private:
    struct Prev {
        QPointF c; double heading; double speed; Direction dir; int road;
        bool ring; int ringVi, ringHi; double stopped; double age;
        int lastTurnVi = -1, lastTurnHi = -1; Direction preTurnDir = Direction::L2R;
        bool seen = true;
    };
    double m_contactGap;
    long   m_corrBase = -1;   ///< Licznik korekt silnika w chwili pierwszej obserwacji.
    MonitorStats m_s;
    QStringList m_log;
    std::unordered_map<int, Prev> m_prev;
    std::unordered_map<long long, bool> m_pairSeen;

    void logV(const QString& kind, int a, int b, const QString& detail);
};
