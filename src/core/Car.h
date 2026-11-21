/**
 * @file Car.h
 * @brief Klasa Car — pojazd na drodze: pozycja, kierunek, prędkość, stan na rondzie.
 * @ingroup simulation_grp
 */
#pragma once
#include "SimulationObject.h"
#include "RoadGeometry.h"
#include "PhysicsConstants.h"
#include "Roundabout.h"
#include <QPointF>
#include <memory>
#include <vector>
#include <limits>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════
//  Tor manewru — ciągła ścieżka (skręt, wjazd i zjazd z ronda)
// ═══════════════════════════════════════════════════════════════════

/// Punkt toru: położenie środka auta i kurs (radiany, 0 = wschód, y w dół).
struct PathPose { QPointF c; double h; };

/**
 * @brief Ścieżka próbkowana z parametryzacją długością łuku.
 *
 * Używana dla manewrów, które wcześniej były realizowane jako natychmiastowe
 * przestawienie pozycji (skręt na skrzyżowaniu, wjazd/zjazd z ronda). Tor jest
 * ciągły w położeniu i stycznej (G1), więc ruch po nim nie powoduje skoków
 * położenia ani kursu.
 */
struct SampledPath {
    std::vector<PathPose> pts;
    std::vector<double>   s;       ///< Skumulowana długość do punktu i.
    double length() const { return s.empty() ? 0.0 : s.back(); }
    PathPose at(double d) const;   ///< Interpolacja położenia i kursu.
    /// Odległość punktu od toru (diagnostyka: legalność trajektorii).
    double distanceTo(QPointF p) const;

    static SampledPath arc(QPointF center, double r, double a0, double sweep, int n = 24);
    static SampledPath bezier(PathPose from, PathPose to, double k0, double k1, int n = 32);
};

/// Manewr w toku: auto jedzie po @ref SampledPath, po czym przechodzi w nowy stan.
struct Maneuver {
    enum class Kind { Turn, RingEntry, RingExit } kind;
    SampledPath path;
    double      pos = 0.0;           ///< Przebyta długość toru.
    int         vi = -1, hi = -1;    ///< Skrzyżowanie / rondo, na którym trwa manewr.
    Direction   endDir;              ///< Pas po manewrze (Turn, RingExit).
    int         endRoad = 0;
    // RingEntry: stan ronda po zakończeniu wjazdu
    double      ringTheta = 0.0, travelNeeded = 0.0;
    Direction   exitDir = Direction::L2R;
    int         exitRoad = 0;
};

// ═══════════════════════════════════════════════════════════════════
//  Car — pojedynczy pojazd na siatce dróg
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Pojedynczy pojazd poruszający się po siatce dróg.
 *
 * Stany ruchu:
 *  - **pas** — jazda prosto wzdłuż osi pasa (m_dir, m_road),
 *  - **manewr** — ruch po ciągłym torze (łuk skrętu, krzywa wjazdu/zjazdu ronda),
 *  - **rondo** — ruch po okręgu o promieniu kRingR.
 *
 * Prędkość dąży do celu (stop/slow/drive/boost) z ograniczeniami przyspieszenia
 * i hamowania; dodatkowo silnik może narzucić w danym kroku limit bezpieczny
 * (@ref capSafeSpeed) wynikający z odległości do przeszkody.
 */
class Car : public SimulationObject {
public:
    Car(QPointF pos, Direction dir, int road);
    ~Car() override;

    // ── SimulationObject overrides ───────────────────────────────
    void    update(double dt) override;
    QPointF position()    const override { return m_pos; }
    /// Prostokąt osiowy obejmujący RZECZYWISTY (obrócony) obrys auta.
    QRectF  boundingRect()const override;
    double  headingAngle()const override;

    /// Środek i kurs (radiany) — rzeczywista poza auta.
    QPointF center()     const;
    double  headingRad() const;

    bool isCircling(int vi,int hi) const override { return isOnRing() && ringVI()==vi && ringHI()==hi; }
    int  blinkerSide()   const override { return m_blinker; }
    bool blinkerFlash()  const override { return m_blinkerOn; }

    Direction dir()       const override { return m_dir; }
    int       road()      const override { return m_road; }
    double    speed()     const override { return m_speed; }
    bool      isSmartCar()const override { return m_isSmartCar; }
    /// Auto zajmuje rondo: krąży albo wykonuje wjazd/zjazd (konflikt z ringiem).
    bool      isOnRing()  const override;
    int       ringVI()    const override;
    int       ringHI()    const override;

    // ── Sterowanie prędkością ────────────────────────────────────
    void stop()  { m_target = 0; }            ///< Zatrzymaj (cel prędkości = 0).
    void slow()  { m_target = std::min(kSlowSpeed, m_max); }   ///< Jedź ostrożnie.
    void drive() { m_target = m_max; }         ///< Jedź z prędkością przelotową.
    void boost() { m_target = m_max * kBoostSpeed / kMaxSpeed; } ///< Zielone na skrzyżowaniu.
    void setTarget(double v) { m_target = std::min(v, m_max); }

    /// Bezpieczny limit prędkości w bieżącym kroku (min z kolejnych wywołań).
    void capSafeSpeed(double v, int by = 0) { if (v < m_cap) { m_cap = std::max(0.0, v); m_capBy = by; } }
    /// Diagnostyka: kto ograniczył prędkość w ostatnim kroku (id auta, 0 = reguła, −1 = brak).
    int  lastCapBy() const { return m_lastCapBy; }
    double safeCap() const { return m_cap; }

    /// Ogranicza bieżącą prędkość z góry (dawny mechanizm awaryjny).
    void capSpeed(double s) { if (m_speed > s) m_speed = s; }

    /// Cofa auto wzdłuż toru — tylko ostateczna siatka bezpieczeństwa.
    void pushBack(double dist);

    // ── Zone / decision state ────────────────────────────────────
    void markDecided()  { m_decided = true; }
    void undecide()     { m_decided = false; m_turnRetries++; }
    void resetRetries() { m_turnRetries = 0; }
    int  turnRetries()  const { return m_turnRetries; }
    int  zoneID()       const { return m_zoneID; }
    bool decided()      const { return m_decided; }

    void enterZone(int zid) { if (m_zoneID != zid) { m_zoneID = zid; } }
    void exitZone()         { m_zoneID=-1; m_decided=false; m_turnRetries=0;
                              clearBlinker(); clearPending(); }

    // ── Pending intent (set by engine's pre-decision) ────────────
    void setPending(int zID,int intent) { m_pendingZoneID=zID; m_pendingIntent=intent; }
    void clearPending()  { m_pendingZoneID = -1; }
    int  pendingZone()   const { return m_pendingZoneID; }
    int  pendingIntent() const { return m_pendingIntent; }

    // ── Blinker ──────────────────────────────────────────────────
    void setBlinker(int side) { m_blinker = side; m_blinkerTimer = 0; m_blinkerOn = true; }
    void clearBlinker()       { setBlinker(0); }

    // ── Skręt na skrzyżowaniu: łuk od bieżącej pozycji do pasu docelowego ──
    /// Rozpoczyna skręt po łuku. Zwraca false, gdy geometria nie pozwala
    /// (auto minęło punkt styczności) — wtedy auto jedzie prosto.
    bool startTurn(Direction nd, int nr, int vi, int hi);
    /// Promień łuku skrętu, gdyby rozpocząć go teraz (−1 = niemożliwy).
    double turnRadiusFromHere(Direction nd, int nr) const;
    /// Prędkość dopuszczalna na łuku o promieniu r (przyspieszenie boczne).
    static double turnSpeed(double r);

    // ── Roundabout ───────────────────────────────────────────────
    /// Rozpoczyna wjazd po krzywej stycznej do okręgu ronda.
    void enterRoundabout(int vi, int hi, int intent);
    /// Kąt na okręgu, w którym kończy się krzywa wjazdu dla kierunku @p d.
    static double ringMergeTheta(Direction d, double laneOffset);
    /// Kąt, w którym zaczyna się krzywa zjazdu na pas @p exitDir.
    static double ringDivergeTheta(Direction exitDir, double laneOffset);
    /// Poprzeczne przesunięcie osi pasa względem osi drogi (w prawo, px).
    static double laneOffset(Direction d);

    bool   ringReadyToExit() const { return m_rb && (m_rb->traveled >= m_rb->travelNeeded - 1e-9); }
    double ringTheta()     const;     ///< Kąt (także rzutowany w trakcie wjazdu/zjazdu).
    double ringTraveled()  const { return m_rb ? m_rb->traveled : 0.0; }
    QPointF ringExitPos()  const;     ///< Lewy-górny róg auta na końcu zjazdu.
    QPointF ringExitCenter() const;   ///< Środek auta na końcu zjazdu.
    Direction ringExitDir()const;
    int    ringExitRoad()  const;
    /// Rozpoczyna zjazd z ronda po krzywej stycznej.
    void   doExitRoundabout();
    /// Kolejne okrążenie (zjazd zablokowany): licznik łuku do zjazdu rośnie o 2π.
    void   extendRingLap() { if (m_rb) m_rb->travelNeeded += 2.0 * M_PI; }
    bool   inRingEntry()   const { return m_man && m_man->kind == Maneuver::Kind::RingEntry; }
    bool   inRingExit()    const { return m_man && m_man->kind == Maneuver::Kind::RingExit; }
    bool   circling()      const { return m_rb != nullptr; }

    // ── Manewr po torze (skręt / wjazd / zjazd z ronda) ──────────
    bool   inManeuver()        const { return m_man != nullptr; }
    bool   inTurn()            const { return m_man && m_man->kind == Maneuver::Kind::Turn; }
    double maneuverPathError() const;
    double turnArcRadius()     const { return inTurn() ? m_man->path.length() / (M_PI / 2) : 0.0; }
    double maneuverRemaining() const { return m_man ? m_man->path.length() - m_man->pos : 0.0; }
    int    maneuverVI()        const { return m_man ? m_man->vi : -1; }
    int    maneuverHI()        const { return m_man ? m_man->hi : -1; }
    Direction maneuverEndDir() const { return m_man ? (m_man->kind == Maneuver::Kind::RingEntry ? m_man->exitDir : m_man->endDir) : m_dir; }
    int    maneuverEndRoad()   const { return m_man ? (m_man->kind == Maneuver::Kind::RingEntry ? m_man->exitRoad : m_man->endRoad) : m_road; }

    /**
     * @brief Przewidywany tor jazdy na dystansie @p dist (próbki co @p step px).
     *
     * Bez założeń o przyszłych decyzjach: pas → prosto; manewr → reszta toru,
     * potem prosto (lub okrąg ronda); rondo → okrąg. Używany do wykrywania
     * przeszkody z przodu niezależnie od tego, na jakim odcinku grafu jest.
     */
    void futurePath(double dist, double step, std::vector<PathPose>& out) const;

    // ── SmartCar flag ────────────────────────────────────────────
    bool m_isSmartCar = false;

    // ── Scenariusze testowe: awaria / wolniejszy pojazd ──────────
    /// Auto „unieruchomione" (np. awaria): hamuje do zera i stoi, dopóki
    /// flaga jest ustawiona. Hamowanie respektuje ograniczenia modelu.
    void   setHeld(bool h)        { m_held = h; }
    bool   held() const           { return m_held; }
    /// Stała intencja na każdym skrzyżowaniu (0/1/2; −1 = losowa) — scenariusze testowe.
    void   setFixedIntent(int i)  { m_fixedIntent = i; }
    int    fixedIntent() const    { return m_fixedIntent; }
    /// Indywidualna prędkość przelotowa (pojazdy o różnej prędkości).
    void   setMaxSpeed(double v)  { m_max = v; if (m_target > v) m_target = v; }
    double maxSpeed() const       { return m_max; }

    // ── Identyfikator diagnostyczny (nadawany przez silnik) ──────
    int  id() const      { return m_id; }
    void setId(int id)   { m_id = id; }

private:
    void updateBlinker(double dt);
    void integrateSpeed(double dt);
    void advance(double dist);            ///< Przesunięcie wzdłuż bieżącego stanu.
    void setCenterLane(QPointF c);        ///< Ustawia m_pos z środka (orientacja pasa).
    void finishManeuver(double leftover);
    QPointF ringPoint(double theta) const;
    void startRingExitManeuver();

    Direction m_dir;
    int       m_road;
    QPointF   m_pos;                      ///< Lewy-górny róg (stan pas).
    QPointF   m_c;                        ///< Środek (manewr / rondo).
    double    m_h = 0.0;                  ///< Kurs (manewr / rondo), radiany.
    double    m_speed  = 0, m_target = kMaxSpeed, m_max = kMaxSpeed;
    double    m_cap = std::numeric_limits<double>::infinity();
    int       m_capBy = -1, m_lastCapBy = -1;
    int       m_zoneID = -1;
    bool      m_decided = false;   ///< Dawniej true: nowe auto pomijało decyzję na PIERWSZYM skrzyżowaniu.
    int       m_blinker = 0;
    double    m_blinkerTimer = 1;
    bool      m_blinkerOn = true;
    int       m_pendingZoneID = -1, m_pendingIntent = 0;
    int       m_turnRetries = 0;
    int       m_id = -1;
    bool      m_held = false;
    int       m_fixedIntent = -1;
    std::unique_ptr<RbState>  m_rb;
    std::unique_ptr<Maneuver> m_man;
};
