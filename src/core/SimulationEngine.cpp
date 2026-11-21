#include "SimulationEngine.h"
#include "Car.h"
#include "PhysicsConstants.h"
#include "Roundabout.h"
#include "RoadGeometry.h"
#include "Obb.h"
#include <cmath>
#include <algorithm>
#include <random>

// ═══════════════════════════════════════════════════════════════════
//  Czasy trwania faz sygnalizacji (sekundy)
// ═══════════════════════════════════════════════════════════════════
static constexpr double kTLDur[]  = {8.0, 2.0, 8.0, 2.0};  ///< Światła: zielone, żółte, zielone, żółte.
static constexpr double kRndDur[] = {6.0, 6.0};            ///< Rondo: dwie fazy przepustowości.

// ═══════════════════════════════════════════════════════════════════
//  Generatory liczb losowych
// ═══════════════════════════════════════════════════════════════════
// (generator jest teraz polem silnika: m_rng / m_intentDist)

// ── Pomocniki geometrii kolizji (lokalne) ───────────────────────────

// ════════════════════════════════════════════════════════════════════
//  Ustawianie świateł w zależności od fazy
// ════════════════════════════════════════════════════════════════════
static void applyPhase(IntersectionState& s) {
    if (s.isRoundabout) {
        s.hLight = (s.phase == 0) ? LightState::Green : LightState::Red;
        s.vLight = (s.phase == 0) ? LightState::Red   : LightState::Green;
    } else {
        switch (s.phase) {
        case 0: s.hLight = LightState::Green;  s.vLight = LightState::Red;    break;
        case 1: s.hLight = LightState::Yellow; s.vLight = LightState::Red;    break;
        case 2: s.hLight = LightState::Red;    s.vLight = LightState::Green;  break;
        case 3: s.hLight = LightState::Red;    s.vLight = LightState::Yellow; break;
        }
    }
}

// ════════════════════════════════════════════════════════════════════
//  SimulationEngine — konstrukcja i pętla
// ════════════════════════════════════════════════════════════════════
SimulationEngine::SimulationEngine() {
    // Rozłóż fazy świateł tak, by nie zmieniały się wszystkie naraz.
    for (int vi = 0; vi < kNumRoads; ++vi)
        for (int hi = 0; hi < kNumRoads; ++hi) {
            auto& s = m_iState[vi][hi];
            s.isRoundabout = kIsRoundabout(vi, hi);
            double off = (vi * kNumRoads + hi) * 1.9;   // przesunięcie fazowe
            double cyc = s.isRoundabout
                       ? kRndDur[0] + kRndDur[1]
                       : kTLDur[0] + kTLDur[1] + kTLDur[2] + kTLDur[3];
            s.timer = std::fmod(off, cyc);
            s.phase = 0;
            if (s.isRoundabout)
                while (s.timer >= kRndDur[s.phase]) { s.timer -= kRndDur[s.phase]; s.phase = (s.phase + 1) % 2; }
            else
                while (s.timer >= kTLDur[s.phase])  { s.timer -= kTLDur[s.phase];  s.phase = (s.phase + 1) % 4; }
            applyPhase(s);
        }
}

void SimulationEngine::update(double dt) {
    if (dt <= 0) return;
    // Stały krok logiki: dt (np. 0,016 × mnożnik tempa) dzielimy na podkroki
    // ≤ kMaxStep. Wyniki nie zależą od tempa symulacji ani FPS (dawniej przy
    // ×4 krok 0,064 s przeskakiwał okna decyzyjne, np. ±4 px wjazdu na rondo).
    const int n = std::max(1, int(std::ceil(dt / kMaxStep - 1e-9)));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) step(h);
}

void SimulationEngine::step(double dt) {
    updateIntersections(dt);   // 1. faza świateł
    applyTrafficRules();       // 2. decyzje o prędkości, skrętach, rondach
    applySafetyCaps(dt);       // 3. bezpieczna prędkość względem przeszkód na torze
    for (auto& o : m_objects) o->update(dt);  // 4. ruch
    resolveOverlaps();         // 5. siatka bezpieczeństwa (powinna być bezczynna)
    removeOutside();           // 6. usuń auta poza mapą
    trySpawnPendingSmart();    // 7. SmartCar ma pierwszeństwo wjazdu przed ruchem tła
    maintainTraffic();         // 8. dosyp ruch do docelowej liczby
}

void SimulationEngine::addCar() {
    int idx = m_nextSpawn;
    m_nextSpawn = (m_nextSpawn + 1) % (kNumRoads * 4);

    Direction dir; int road; QPointF pos; bool poziomo;
    if      (idx < kNumRoads)     { dir = Direction::L2R; road = idx;                 pos = {-kCarL - 5,   kL2R_Y(road)}; poziomo = true;  }
    else if (idx < kNumRoads * 2) { dir = Direction::R2L; road = idx - kNumRoads;     pos = {kCanvasW + 5, kR2L_Y(road)}; poziomo = true;  }
    else if (idx < kNumRoads * 3) { dir = Direction::T2B; road = idx - kNumRoads * 2; pos = {kT2B_X(road), -kCarL - 5};   poziomo = false; }
    else                          { dir = Direction::B2T; road = idx - kNumRoads * 3; pos = {kB2T_X(road), kCanvasH + 5}; poziomo = false; }

    // Nie wjeżdżaj, jeśli przy krawędzi nie ma wolnego miejsca.
    QRectF nr(pos.x(), pos.y(), poziomo ? kCarL : kCarW, poziomo ? kCarW : kCarL);
    QRectF safe = nr.adjusted(-kSafeGap, -kSafeGap, kSafeGap, kSafeGap);
    for (const auto& o : m_objects)
        if (o->boundingRect().intersects(safe)) return;
    if (m_pendingSmart && m_pendingSmart->dir == dir && m_pendingSmart->road == road) return;  // miejsce zarezerwowane

    pushCar(std::make_unique<Car>(pos, dir, road));
}

void SimulationEngine::addCar(QPointF pos, Direction dir, int road) {
    // Wariant z pełną kontrolą pozycji/kierunku — używany przez testy
    // do generowania kontrolowanych scenariuszy. Limit 100 aut.
    int n = 0;
    for (const auto& o : m_objects)
        if (dynamic_cast<const Car*>(o.get())) n++;
    if (n >= 100) return;
    pushCar(std::make_unique<Car>(pos, dir, road));
}

void SimulationEngine::pushCar(std::unique_ptr<Car> car) {
    car->setId(m_nextCarId++);
    m_objects.push_back(std::move(car));
}

const std::vector<std::unique_ptr<SimulationObject>>& SimulationEngine::objects() const { return m_objects; }
const IntersectionState& SimulationEngine::intersectionAt(int vi, int hi) const { return m_iState[vi][hi]; }

void SimulationEngine::setRoundabouts(const std::vector<std::pair<int,int>>& positions) {
    for (int vi = 0; vi < kNumRoads; ++vi)
        for (int hi = 0; hi < kNumRoads; ++hi)
            m_iState[vi][hi].isRoundabout = false;
    for (auto [vi, hi] : positions) {
        if (vi < 0 || vi >= kNumRoads || hi < 0 || hi >= kNumRoads) continue;
        auto& s = m_iState[vi][hi];
        s.isRoundabout = true;
        s.timer = 0; s.phase = 0;
        applyPhase(s);
    }
}

double SimulationEngine::signalWaitAt(const IntersectionState& st, bool horizontal, double dtAhead) {
    if (st.isRoundabout) return 0.0;
    const int greenPhase = horizontal ? 0 : 2;
    int ph = st.phase; double t = st.timer + std::max(0.0, dtAhead);
    while (t >= kTLDur[ph]) { t -= kTLDur[ph]; ph = (ph + 1) % 4; }
    if (ph == greenPhase) return 0.0;
    double wait = kTLDur[ph] - t;
    for (int k = (ph + 1) % 4; k != greenPhase; k = (k + 1) % 4) wait += kTLDur[k];
    return wait;
}

double SimulationEngine::signalExpectedWait(bool horizontal) {
    const double cycle = kTLDur[0] + kTLDur[1] + kTLDur[2] + kTLDur[3];
    const double green = horizontal ? kTLDur[0] : kTLDur[2];
    const double stop = cycle - green;
    return stop * stop / (2.0 * cycle);
}

void SimulationEngine::setIntersectionPhase(int vi, int hi, int phase, double timer) {
    if (vi < 0 || vi >= kNumRoads || hi < 0 || hi >= kNumRoads) return;
    auto& s = m_iState[vi][hi];
    s.phase = s.isRoundabout ? (phase % 2) : (phase % 4);
    s.timer = timer;
    applyPhase(s);
}

void SimulationEngine::updateIntersections(double dt) {
    for (int vi = 0; vi < kNumRoads; ++vi)
        for (int hi = 0; hi < kNumRoads; ++hi) {
            auto& s = m_iState[vi][hi];
            s.timer += dt;
            if (s.isRoundabout)
                while (s.timer >= kRndDur[s.phase]) { s.timer -= kRndDur[s.phase]; s.phase = (s.phase + 1) % 2; }
            else
                while (s.timer >= kTLDur[s.phase])  { s.timer -= kTLDur[s.phase];  s.phase = (s.phase + 1) % 4; }
            applyPhase(s);
        }
}

// ════════════════════════════════════════════════════════════════════
//  Kontrole bezpieczeństwa skrętu
// ════════════════════════════════════════════════════════════════════

namespace {
double headingOf(Direction d) {
    switch (d) {
    case Direction::L2R: return 0.0;
    case Direction::T2B: return M_PI / 2;
    case Direction::R2L: return M_PI;
    default:             return -M_PI / 2;
    }
}
QPointF unitVec(double h) { return { std::cos(h), std::sin(h) }; }
double laneCrossOf(Direction d, int road) {
    switch (d) {
    case Direction::L2R: return kL2R_Y(road) + kCarW / 2.0;
    case Direction::R2L: return kR2L_Y(road) + kCarW / 2.0;
    case Direction::T2B: return kT2B_X(road) + kCarW / 2.0;
    default:             return kB2T_X(road) + kCarW / 2.0;
    }
}
/// Współrzędna krawędzi strefy skrzyżowania, przez którą auto do niej wjeżdża.
double zoneEntryCoord(Direction d, int vi, int hi) {
    switch (d) {
    case Direction::L2R: return kVLeft[vi];
    case Direction::R2L: return kVLeft[vi] + kRoadW;
    case Direction::T2B: return kHTop[hi];
    default:             return kHTop[hi] + kRoadW;
    }
}
/// Promień łuku skrętu z pasa (dir) na pas (nd, nr), rozpoczętego na wjeździe do strefy.
double plannedTurnRadius(Direction dir, int vi, int hi, Direction nd, int nr) {
    return std::abs(laneCrossOf(nd, nr) - zoneEntryCoord(dir, vi, hi));
}
} // namespace

/// Środek auta po zakończeniu łuku skrętu (pas docelowy).
static QPointF turnEndCenter(const Car* car, int vi, int hi, Direction nd, int nr) {
    const Direction d = car->dir();
    const double r = plannedTurnRadius(d, vi, hi, nd, nr);
    QPointF start = car->center();
    if (dirIsHoriz(d)) start.setX(zoneEntryCoord(d, vi, hi)); else start.setY(zoneEntryCoord(d, vi, hi));
    return start + unitVec(headingOf(d)) * r + unitVec(headingOf(nd)) * r;
}

/// Czy pas docelowy skrętu (nd, nr) wokół miejsca zakończenia łuku jest wolny?
bool SimulationEngine::isTurnSafe(Direction nd, int nr, QPointF endCenter, double margin) const {
    const bool poziomo = dirIsHoriz(nd);
    QRectF cel(endCenter.x() - (poziomo ? kCarL : kCarW) / 2, endCenter.y() - (poziomo ? kCarW : kCarL) / 2,
               poziomo ? kCarL : kCarW, poziomo ? kCarW : kCarL);
    const double g = kTurnGap * margin;
    QRectF chk = cel.adjusted(-g, -g, g, g);
    for (const auto& o : m_objects) {
        const Car* oc = dynamic_cast<const Car*>(o.get());
        if (!oc) continue;
        const bool sameLane = (!oc->inManeuver() && !oc->isOnRing() && oc->dir() == nd && oc->road() == nr)
                           || (oc->inManeuver() && oc->maneuverEndDir() == nd && oc->maneuverEndRoad() == nr);
        if (!sameLane) continue;
        if (oc->boundingRect().intersects(chk)) return false;
    }
    return true;
}

/// Czy pas przeciwny jest wolny, by bezpiecznie skręcić w lewo przez skrzyżowanie?
/// Skręt w lewo przecina pas nadjeżdżających z naprzeciwka — trzeba ich przepuścić.
/// Horyzont obejmuje czas przejazdu łuku (≈1–1,5 s), więc pod uwagę brane są
/// auta, które w tym czasie dojadą do skrzyżowania.
bool SimulationEngine::isOncomingClear(Car* car, int vi, int hi, double margin) const {
    Direction oncoming = dirOpposite(car->dir());
    const double reach = 1.6 * margin * kMaxSpeed + 20.0;
    QRectF zone = kIZone(vi, hi);
    QRectF big = zone.adjusted(-reach, -reach, reach, reach);
    const QPointF iCtr(kVLeft[vi] + kRoadW / 2.0, kHTop[hi] + kRoadW / 2.0);
    for (const auto& o : m_objects) {
        const Car* oc = dynamic_cast<const Car*>(o.get());
        if (!oc || oc == car || oc->isOnRing()) continue;
        if (oc->inTurn() && oc->maneuverVI() == vi && oc->maneuverHI() == hi) {
            // Auto z naprzeciwka już skręca w tym skrzyżowaniu — tory mogą się przeciąć.
            if (oc->dir() == oncoming) return false;
            continue;
        }
        if (oc->dir() != oncoming || oc->inManeuver()) continue;
        if (dirIsHoriz(oncoming) ? oc->road() != hi : oc->road() != vi) continue;
        QPointF c = oc->center();
        if (zone.contains(c)) return false;
        if (!big.contains(c)) continue;
        const double toward = (iCtr.x() - c.x()) * std::cos(headingOf(oncoming))
                            + (iCtr.y() - c.y()) * std::sin(headingOf(oncoming));
        if (toward <= 0) continue;                  // już minęło skrzyżowanie
        const double dist = toward - kRoadW / 2 - kCarL / 2;
        if (oc->speed() < 1.0 && dist > 25.0) continue;   // stoi przed linią
        if (dist < (oc->speed() * 1.6 + 30.0) * margin) return false;
    }
    return true;
}

/// Czy pas zjazdowy z ronda ma miejsce na zakończenie zjazdu?
bool SimulationEngine::isRingExitSafe(Direction dir, int road, QPointF exitCenter) const {
    const QPointF u = unitVec(headingOf(dir));
    for (const auto& o : m_objects) {
        const Car* oc = dynamic_cast<const Car*>(o.get());
        if (!oc || oc->isOnRing() || oc->inManeuver()) continue;
        if (oc->dir() != dir || oc->road() != road) continue;
        const QPointF d = oc->center() - exitCenter;
        const double along = d.x() * u.x() + d.y() * u.y();
        // Auto zajmuje [along−L/2, along+L/2]; potrzebujemy [−L/2, L/2 + odstęp + zapas].
        if (along + kCarL / 2 > -kCarL / 2 && along - kCarL / 2 < kCarL / 2 + kStandGap + 4.0) return false;
    }
    return true;
}

/// Czy w kierunku `dir` na drodze `road`, za skrzyżowaniem (vi,hi), jest
/// dość wolnego miejsca, by auto przejechało i zatrzymało się ZA strefą?
/// KLUCZOWE: liczy się tylko STOJĄCE auta (speed < 5). Jadące auta opuszczą
/// strefę i nie blokują wjazdu — to zapobiega kaskadowemu deadlockowi.
bool SimulationEngine::exitLaneClear(Direction dir, int road, int vi, int hi,
                                      const Car* self) const {
    const double need = kCarL + kSafeGap;
    double minDist = 1e9;

    double nearEdge, farEdge;
    switch (dir) {
    case Direction::L2R: nearEdge = kVLeft[vi];          farEdge = kVLeft[vi] + kRoadW; break;
    case Direction::R2L: nearEdge = kVLeft[vi] + kRoadW; farEdge = kVLeft[vi];          break;
    case Direction::T2B: nearEdge = kHTop[hi];           farEdge = kHTop[hi] + kRoadW;  break;
    default:             nearEdge = kHTop[hi] + kRoadW;  farEdge = kHTop[hi];           break;
    }

    for (const auto& o : m_objects) {
        const Car* oc = dynamic_cast<const Car*>(o.get());
        if (!oc || oc == self || oc->isOnRing() || oc->inManeuver()) continue;
        if (oc->dir() != dir || oc->road() != road) continue;
        if (oc->speed() > 5.0) continue;
        const QRectF orr = oc->boundingRect();

        double distFromFar;
        bool pastNear;
        switch (dir) {
        case Direction::L2R: pastNear = orr.left()   > nearEdge; distFromFar = orr.left()   - farEdge; break;
        case Direction::R2L: pastNear = orr.right()  < nearEdge; distFromFar = farEdge - orr.right();  break;
        case Direction::T2B: pastNear = orr.top()    > nearEdge; distFromFar = orr.top()    - farEdge; break;
        default:             pastNear = orr.bottom() < nearEdge; distFromFar = farEdge - orr.bottom(); break;
        }
        if (pastNear && distFromFar < minDist)
            minDist = distFromFar;
    }
    return minDist >= need;
}

/// "Don't block the box" dla jazdy prosto — wyjazd w bieżącym kierunku.
bool SimulationEngine::isBoxExitClear(Car* car, int vi, int hi) const {
    return exitLaneClear(car->dir(), car->road(), vi, hi, car);
}

/// Czy auto może bezpiecznie WJECHAĆ na skrzyżowanie?
/// Dla auta zwykłego: sprawdza wyjazd na wprost (escape route).
/// Dla SmartCara: dodatkowo sprawdza, czy zamierzony skręt jest bezpieczny.
bool SimulationEngine::canEnterIntersection(Car* car, int vi, int hi) const {
    if (!isBoxExitClear(car, vi, hi)) return false;
    if (car->isSmartCar() && m_smartIntentFn) {
        int zID = vi * kNumRoads + hi;
        int intent = m_smartIntentFn(zID);
        if (intent != 0) {
            Direction nd; int nr; QPointF np;
            resolveTurnTarget(car, vi, hi, intent, nd, nr, np);
            // Kontrola przed wjazdem jest OSTRZEJSZA niż decyzja w strefie (histereza):
            // auto, które przeszło tę kontrolę, nie traci skrętu z powodu auta, które
            // w ciągu ~0,3 s dojazdu do strefy znalazło się na granicy okna.
            if (!isTurnSafe(nd, nr, np, 1.5)) return false;
            if (intent == 2 && !isOncomingClear(car, vi, hi, 1.5)) return false;
        }
    }
    return true;
}

// ── Cel manewru skrętu (kierunek, droga, środek auta po łuku) ───────
void SimulationEngine::resolveTurnTarget(Car* car, int vi, int hi, int intent,
                                          Direction& nd, int& nr, QPointF& np) const {
    const bool rt = (intent == 1);   // skręt w prawo
    const Direction dir = car->dir();
    if (dirIsHoriz(dir)) {
        nd = (dir == Direction::L2R) ? (rt ? Direction::T2B : Direction::B2T)
                                     : (rt ? Direction::B2T : Direction::T2B);
        nr = vi;
    } else {
        nd = (dir == Direction::T2B) ? (rt ? Direction::R2L : Direction::L2R)
                                     : (rt ? Direction::L2R : Direction::R2L);
        nr = hi;
    }
    np = turnEndCenter(car, vi, hi, nd, nr);
}

// ── Decyzja o skręcie (auto właśnie wjechało środkiem do strefy) ────
void SimulationEngine::decideTurn(Car* car, int vi, int hi) {
    int zID    = vi * kNumRoads + hi;
    int intent = resolveIntent(car, zID);

    if (intent == 0) { car->markDecided(); car->clearBlinker(); return; }

    Direction nd; int nr; QPointF np;
    resolveTurnTarget(car, vi, hi, intent, nd, nr, np);

    bool safe = isTurnSafe(nd, nr, np);
    if (intent == 2) safe = safe && isOncomingClear(car, vi, hi);

    if (safe && car->startTurn(nd, nr, vi, hi)) {
        car->setBlinker(intent == 1 ? 1 : 2);
        car->resetRetries();
        return;
    }
    // Auto jest JUŻ w strefie — nie wolno blokować skrzyżowania czekając na
    // skręt: jedzie prosto (SmartCar przeliczy trasę).
    car->markDecided();
    car->clearBlinker();
    car->resetRetries();
}

// ════════════════════════════════════════════════════════════════════
//  Rondo — warunek wjazdu (pierwszeństwo aut na rondzie)
// ════════════════════════════════════════════════════════════════════
/// Czy auto może teraz rozpocząć wjazd na rondo (vi,hi)?
///  • auto nadjeżdżające po okręgu nie może nas dogonić w czasie włączania,
///  • za punktem włączenia musi być miejsce na nasze auto.
/// Uwzględnia auta krążące, wjeżdżające (z innych wlotów) i zjeżdżające.
bool SimulationEngine::ringEntryClear(const Car* car, int vi, int hi) const {
    const double th3 = Car::ringMergeTheta(car->dir(), Car::laneOffset(car->dir()));
    const double tMerge = 1.2;   // czas przejazdu krzywej wjazdu (~45 px) od małej prędkości
    auto conflict = [&](double th, double v) {
        const double up   = Rb::mod2pi(th - th3) * kRingR;   // za punktem włączenia (nadjeżdża)
        const double down = Rb::mod2pi(th3 - th) * kRingR;   // przed punktem włączenia
        const double needUp = kCarL + kStandGap + 12.0 + std::max(v, kRingSpeed) * tMerge;
        return up < needUp || down < kCarL + kStandGap + 6.0;
    };
    for (const auto& o : m_objects) {
        const Car* oc = dynamic_cast<const Car*>(o.get());
        if (!oc || oc == car || !oc->isCircling(vi, hi)) continue;
        if (oc->inRingExit()) {
            // Zjeżdżające: konflikt tylko gdy jeszcze jest na wysokości naszej krzywej.
            const double th = oc->ringTheta();
            const double d = std::min(Rb::mod2pi(th - th3), Rb::mod2pi(th3 - th)) * kRingR;
            if (d < kCarL + kStandGap) return false;
            continue;
        }
        if (oc->inRingEntry()) {
            // Wjeżdżające z innego wlotu: wkrótce znajdzie się w swoim punkcie włączenia.
            if (conflict(oc->ringTheta(), oc->speed())) return false;
            if (conflict(Car::ringMergeTheta(oc->dir(), Car::laneOffset(oc->dir())), kRingSpeed)) return false;
            continue;
        }
        if (conflict(oc->ringTheta(), oc->speed())) return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════
//  Bezpieczna prędkość — przeszkoda na przewidywanym torze
// ════════════════════════════════════════════════════════════════════
/// Dla każdego auta wyznacza wolną drogę wzdłuż jego przewidywanego toru
/// (pas, łuk skrętu, wjazd/zjazd, okrąg ronda — niezależnie od odcinka grafu)
/// i ogranicza prędkość do bezpiecznej (model Gippsa):
///     v ≤ −bτ + √((bτ)² + v_L²·b/b_L + 2b·s),
/// s — wolna droga do obrysu przeszkody pomniejszona o odstęp postoju s₀,
/// v_L — prędkość przeszkody wzdłuż toru, b — opóźnienie planowane,
/// b_L — maksymalne opóźnienie przeszkody. Odstęp rośnie z prędkością,
/// a w postoju auta zachowują s₀ (nie stykają się zderzakami).
void SimulationEngine::applySafetyCaps(double dt) {
    struct Info { Car* car; QPointF c; double h; double v; };
    std::vector<Info> cars;
    cars.reserve(m_objects.size());
    for (auto& o : m_objects)
        if (Car* c = dynamic_cast<Car*>(o.get()))
            cars.push_back({ c, c->center(), c->headingRad(), c->speed() });

    const double b = kPlanBrake, tau = kReactTime + dt;
    const double step = 2.0;
    std::vector<PathPose> path;
    for (size_t i = 0; i < cars.size(); ++i) {
        Info& A = cars[i];
        const double vmax = std::max(A.v, A.car->maxSpeed() * kBoostSpeed / kMaxSpeed);
        const double look = vmax * vmax / (2 * b) + vmax * tau + kStandGap + kCarL / 2 + 8.0;
        bool pathBuilt = false;
        double freeDist = 1e9, leaderV = 0.0; int blocker = -1;
        const QPointF fwd = unitVec(A.h);
        for (size_t j = 0; j < cars.size(); ++j) {
            if (i == j) continue;
            const Info& B = cars[j];
            const QPointF d = B.c - A.c;
            if (std::abs(d.x()) > look + kCarL || std::abs(d.y()) > look + kCarL) continue;
            if (d.x() * fwd.x() + d.y() * fwd.y() <= 0) continue;   // auto za nami
            if (!pathBuilt) { A.car->futurePath(look, step, path); pathBuilt = true; }
            // Odrzucenie okręgami opisanymi (tańsze niż SAT dla każdej próbki).
            const double rA = 0.5 * std::hypot(kCarL + kStandGap, kCarW), rB = 0.5 * std::hypot(kCarL, kCarW);
            const double reach2 = (rA + rB) * (rA + rB);
            for (size_t k = 0; k < path.size(); ++k) {
                const QPointF cc = path[k].c + unitVec(path[k].h) * (kStandGap / 2);
                const QPointF dd = cc - B.c;
                if (dd.x() * dd.x() + dd.y() * dd.y() > reach2) continue;
                if (Obb::overlap(cc, path[k].h, kCarL + kStandGap, kCarW, B.c, B.h, kCarL, kCarW)) {
                    const double s = std::max(0.0, (double(k) - 1.0) * step);
                    if (s < freeDist) {
                        freeDist = s;
                        leaderV = std::max(0.0, B.v * std::cos(B.h - path[k].h));
                        blocker = B.car->id();
                    }
                    break;
                }
            }
        }
        if (freeDist < 1e8) {
            const double vSafe = -b * tau + std::sqrt(b * b * tau * tau
                               + leaderV * leaderV * b / kMaxBrake + 2 * b * freeDist);
            A.car->capSafeSpeed(vSafe, blocker);
        }
    }
}

// ════════════════════════════════════════════════════════════════════
//  applyTrafficRules — decyzje o prędkości, skrętach i rondach
// ════════════════════════════════════════════════════════════════════
void SimulationEngine::applyTrafficRules() {

    // ════ ETAP 1: auta na rondzie i w manewrach ═══════════════════════
    // Odstęp od poprzedzającego (także na ringu) zapewnia applySafetyCaps.
    for (auto& obj : m_objects) {
        Car* car = dynamic_cast<Car*>(obj.get());
        if (!car) continue;
        if (car->inTurn())      { car->setTarget(car->maxSpeed()); continue; }   // limit łuku: applySafetyCaps→turn cap
        if (car->inRingEntry()) { car->setTarget(kRingSpeed);  continue; }
        if (car->inRingExit())  { car->setTarget(kZoneMaxSpd); continue; }
        if (!car->circling()) continue;

        car->setTarget(kRingSpeed);
        // Zjazd: w punkcie rozwidlenia, jeśli na pasie zjazdu jest miejsce;
        // w przeciwnym razie kolejne okrążenie (nie wjeżdżamy w kolejkę).
        if (car->ringReadyToExit()) {
            if (isRingExitSafe(car->ringExitDir(), car->ringExitRoad(), car->ringExitCenter()))
                car->doExitRoundabout();
            else
                car->extendRingLap();
        }
    }

    // ════ ETAP 2: auta na pasach ════════════════════════════════════
    for (auto& obj : m_objects) {
        Car* car = dynamic_cast<Car*>(obj.get());
        if (!car || car->isOnRing() || car->inManeuver()) continue;

        const QRectF   rect = car->boundingRect();
        const QPointF  ctr  = rect.center();
        const Direction dir = car->dir();
        const int roadIdx   = car->road();
        const bool isH      = (dir == Direction::L2R || dir == Direction::R2L);
        bool shouldStop = false, shouldSlow = false, shouldBoost = false;

        // ── 1. Najbliższe skrzyżowanie z przodu ──────────────────
        constexpr double kStillCurrent = kRoadW + kCarL + 8;
        int nextVI = -1, nextHI = -1; double nearestDist = 1e9;
        if (isH) {
            int hi = roadIdx;
            for (int vi = 0; vi < kNumRoads; ++vi) {
                double d = (dir == Direction::L2R) ? kSL_L2R(vi) - rect.right()
                                                   : rect.left() - kSL_R2L(vi);
                if (d > -kStillCurrent && d < nearestDist) { nearestDist = d; nextVI = vi; nextHI = hi; }
            }
        } else {
            int vi = roadIdx;
            for (int hi = 0; hi < kNumRoads; ++hi) {
                double d = (dir == Direction::T2B) ? kSL_T2B(hi) - rect.bottom()
                                                   : rect.top() - kSL_B2T(hi);
                if (d > -kStillCurrent && d < nearestDist) { nearestDist = d; nextVI = vi; nextHI = hi; }
            }
        }
        bool inZone = (nextVI >= 0) && kIZone(nextVI, nextHI).contains(ctr);
        bool touchingZone = (nextVI >= 0) && kIZone(nextVI, nextHI).intersects(rect);
        const IntersectionState* iSt = (nextVI >= 0) ? &m_iState[nextVI][nextHI] : nullptr;

        // ── 2. Wczesna decyzja: kierunkowskaz kSignalDist przed stopem ──
        {
            int sVI = -1, sHI = -1; double sDist = 1e9;
            if (isH) { int hi = roadIdx;
                for (int vi = 0; vi < kNumRoads; ++vi) {
                    double d = (dir == Direction::L2R) ? kSL_L2R(vi) - rect.right()
                                                       : rect.left() - kSL_R2L(vi);
                    if (d > 0 && d < sDist) { sDist = d; sVI = vi; sHI = hi; }
                }
            } else { int vi = roadIdx;
                for (int hi = 0; hi < kNumRoads; ++hi) {
                    double d = (dir == Direction::T2B) ? kSL_T2B(hi) - rect.bottom()
                                                       : rect.top() - kSL_B2T(hi);
                    if (d > 0 && d < sDist) { sDist = d; sVI = vi; sHI = hi; }
                }
            }
            if (sVI >= 0 && sDist <= kSignalDist) {
                int zID = sVI * kNumRoads + sHI;
                // SmartCar: intencja z nawigacji odświeżana co krok (trasa może się
                // zmienić przed punktem zatwierdzenia), zwykłe auto — raz na skrzyżowanie.
                if (car->pendingZone() != zID || (car->isSmartCar() && m_smartIntentFn)) {
                    car->setPending(zID, resolveIntent(car, zID));
                    const int pi = car->pendingIntent();
                    car->setBlinker(pi == 1 ? 1 : pi == 2 ? 2 : 0);
                }
            }
        }

        // ── 3. Śledzenie wjazdu/wyjazdu ze strefy (sygnalizacja) ──
        if (iSt && !iSt->isRoundabout) {
            if (inZone && nextVI >= 0) car->enterZone(nextVI * kNumRoads + nextHI);
            else if (!inZone && car->zoneID() >= 0) car->exitZone();
        }

        // ── 4. Zatrzymanie przed czerwonym/żółtym ─────────────────
        if (nextVI >= 0 && !inZone && nearestDist >= 0 && iSt && !iSt->isRoundabout) {
            LightState lt = isH ? iSt->hLight : iSt->vLight;
            if (lt == LightState::Red || lt == LightState::Yellow) {
                if (nearestDist <= kFarBrake)  shouldSlow = true;
                if (nearestDist <= kStopBrake) shouldStop = true;
            }
        }

        // ── 4b. Decyzja wjazdu: nie wjeżdżaj, jeśli nie zdołasz w pełni
        //        przejechać skrzyżowania. Auto czeka PRZED skrzyżowaniem.
        if (nextVI >= 0 && !touchingZone && nearestDist >= 0
            && iSt && !iSt->isRoundabout) {
            if (!canEnterIntersection(car, nextVI, nextHI)) {
                if (nearestDist <= kFarBrake)  shouldSlow = true;
                if (nearestDist <= kStopBrake) shouldStop = true;
            }
        }

        // ── 4c. Zwolnienie przed łukiem skrętu ─────────────────────
        // Auto z zamiarem skrętu dojeżdża do strefy z prędkością, przy której
        // przyspieszenie boczne na łuku mieści się w limicie modelu.
        if (nextVI >= 0 && iSt && !iSt->isRoundabout && !inZone
            && car->pendingZone() == nextVI * kNumRoads + nextHI && car->pendingIntent() != 0) {
            Direction nd; int nr; QPointF np;
            resolveTurnTarget(car, nextVI, nextHI, car->pendingIntent(), nd, nr, np);
            const double r = plannedTurnRadius(dir, nextVI, nextHI, nd, nr);
            const double toZone = std::max(0.0, std::abs(zoneEntryCoord(dir, nextVI, nextHI) - (isH ? ctr.x() : ctr.y())));
            const double vTurn = Car::turnSpeed(r);
            car->capSafeSpeed(std::sqrt(vTurn * vTurn + 2 * kPlanBrake * toZone));
        }

        // ── 5. Rondo: ustąpienie pierwszeństwa i wjazd ────────────
        // Linia ustąpienia: przód auta tuż poza pasem ruchu ronda (kRbYieldR).
        // Pierwszeństwo mają auta na rondzie; wjazd nie zależy od niewidocznej
        // „sygnalizacji" (dawniej auta czekały do 6 s przed pustym rondem).
        if (nextVI >= 0 && iSt && iSt->isRoundabout) {
            const double rcx = kVLeft[nextVI] + kRoadW / 2.0;
            const double rcy = kHTop[nextHI]  + kRoadW / 2.0;
            const double along = (dir == Direction::L2R) ? rcx - ctr.x()
                               : (dir == Direction::R2L) ? ctr.x() - rcx
                               : (dir == Direction::T2B) ? rcy - ctr.y() : ctr.y() - rcy;
            const double uCommit = kRbLaneU(Car::laneOffset(dir));
            const double toCommit = along - uCommit;       // >0: przed punktem wjazdu
            if (along > 0 && toCommit > -kCarL && toCommit < kRbFarBrake) {
                const bool clear = ringEntryClear(car, nextVI, nextHI);
                const double vEntry = 0.7 * kRingSpeed;
                car->capSafeSpeed(std::sqrt(vEntry * vEntry + 2 * kPlanBrake * std::max(0.0, toCommit)));
                if (!clear)
                    car->capSafeSpeed(std::sqrt(2 * kPlanBrake * std::max(0.0, toCommit - 0.5)));
                if (clear && toCommit <= 4.0) {
                    int zID = nextVI * kNumRoads + nextHI;
                    int intent = resolveIntent(car, zID);
                    car->enterRoundabout(nextVI, nextHI, intent);
                    car->setBlinker(intent == 1 ? 1 : intent == 2 ? 2 : 0);
                    continue;
                }
            }
        }

        // Przyspieszenie w strefie skrzyżowania na zielonym.
        if (inZone && iSt && !iSt->isRoundabout) {
            LightState lt = isH ? iSt->hLight : iSt->vLight;
            if (lt == LightState::Green) shouldBoost = true;
        }

        // ── 6. Konflikt poprzeczny w strefie (sygnalizacja) ───────
        if (!inZone && !shouldStop && nextVI >= 0 && nearestDist <= kFarBrake && iSt && !iSt->isRoundabout) {
            QRectF zone = kIZone(nextVI, nextHI);
            for (const auto& oth : m_objects) {
                if (oth.get() == obj.get()) continue;
                const Car* oc = dynamic_cast<const Car*>(oth.get());
                if (!oc) continue;
                const bool oIn = zone.contains(oc->center());
                const double oh = oc->headingRad();
                const bool oHoriz = std::abs(std::cos(oh)) > std::abs(std::sin(oh));
                if (oIn && isH != oHoriz) { shouldStop = true; break; }
            }
        }

        // (Kolejka i odstęp od auta z przodu: applySafetyCaps — tor przewidywany,
        //  obrócone obrysy, odstęp zależny od prędkości.)

        // ── Zastosuj prędkość ─────────────────────────────────────
        if (shouldStop && touchingZone) {
            car->slow();
        } else if (shouldStop) car->stop();
        else if (shouldSlow)   car->slow();
        else if (shouldBoost)  car->boost();
        else                   car->drive();

        // ── 8. Decyzja o skręcie po wjeździe do strefy ────────────
        // Auto w strefie jest już zaangażowane w przejazd — decyzja zapada
        // niezależnie od fazy (dawniej skręt na żółtym był pomijany).
        if (inZone && !car->decided() && nextVI >= 0 && iSt && !iSt->isRoundabout)
            decideTurn(car, nextVI, nextHI);
    }

    // ════ ETAP 3: limit prędkości na łuku skrętu ═════════════════════
    for (auto& obj : m_objects) {
        Car* car = dynamic_cast<Car*>(obj.get());
        if (car && car->inTurn()) car->capSafeSpeed(Car::turnSpeed(car->turnArcRadius()));
    }
}

// ════════════════════════════════════════════════════════════════════
//  resolveOverlaps — ostateczna siatka bezpieczeństwa
// ════════════════════════════════════════════════════════════════════
// Zapobieganie kolizjom realizuje applySafetyCaps (przed ruchem). Ta funkcja
// sprawdza RZECZYWISTE (obrócone) obrysy po ruchu i — gdyby mimo to doszło do
// nakładania — cofa tylne auto. Każda korekta jest zliczana
// (overlapCorrections) i w testach traktowana jako błąd: to skok pozycji.
void SimulationEngine::resolveOverlaps() {
    constexpr int    kMaxIter = 20;
    constexpr double kEps     = 0.5;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        bool zmiana = false;
        int n = (int)m_objects.size();
        for (int i = 0; i < n; ++i) {
            Car* a = dynamic_cast<Car*>(m_objects[i].get());
            if (!a) continue;
            for (int j = i + 1; j < n; ++j) {
                Car* b = dynamic_cast<Car*>(m_objects[j].get());
                if (!b) continue;
                if (!a->boundingRect().intersects(b->boundingRect())) continue;
                const double pen = Obb::penetration(Obb::corners(a->center(), a->headingRad(), kCarL, kCarW),
                                                    Obb::corners(b->center(), b->headingRad(), kCarL, kCarW));
                if (pen < 0.01) continue;

                ++m_overlapCorrections;
                const QPointF ca = a->center(), cb = b->center();
                const QPointF fa = unitVec(a->headingRad()), fb = unitVec(b->headingRad());
                const bool bPrzedA = (cb - ca).x() * fa.x() + (cb - ca).y() * fa.y() > 0;
                const bool aPrzedB = (ca - cb).x() * fb.x() + (ca - cb).y() * fb.y() > 0;
                Car* rear;
                if      (bPrzedA && !aPrzedB) rear = a;
                else if (aPrzedB && !bPrzedA) rear = b;
                else rear = (a->speed() >= b->speed()) ? a : b;
                Car* other = (rear == a) ? b : a;
                if (rear->circling() && !other->circling()) rear = other;   // ring ma pierwszeństwo
                rear->pushBack(pen + kEps);
                rear->stop();
                rear->capSpeed(0.0);
                zmiana = true;
            }
        }
        if (!zmiana) break;
    }
}

void SimulationEngine::removeOutside() {
    const double mg = kCarL + 15;
    m_objects.erase(std::remove_if(m_objects.begin(), m_objects.end(),
        [mg](const std::unique_ptr<SimulationObject>& o) {
            const QPointF p = o->position();
            return p.x() < -mg || p.x() > kCanvasW + mg
                || p.y() < -mg || p.y() > kCanvasH + mg;
        }), m_objects.end());
    refreshSmartCarPtr();
}

void SimulationEngine::refreshSmartCarPtr() {
    if (!m_smartCar) return;
    for (const auto& o : m_objects)
        if (o.get() == m_smartCar) return;
    m_smartCar = nullptr;   // SmartCar został usunięty (np. opuścił mapę)
}

// ── Publiczne API SmartCar ──────────────────────────────────────────

bool SimulationEngine::spawnSlotFree(QPointF pos, Direction dir) const {
    const bool poziomo = dirIsHoriz(dir);
    QRectF nr(pos.x(), pos.y(), poziomo ? kCarL : kCarW, poziomo ? kCarW : kCarL);
    QRectF safe = nr.adjusted(-kSafeGap, -kSafeGap, kSafeGap, kSafeGap);
    for (const auto& o : m_objects)
        if (o->boundingRect().intersects(safe)) return false;
    return true;
}

void SimulationEngine::trySpawnPendingSmart() {
    if (!m_pendingSmart || !spawnSlotFree(m_pendingSmart->pos, m_pendingSmart->dir)) return;
    auto car = std::make_unique<Car>(m_pendingSmart->pos, m_pendingSmart->dir, m_pendingSmart->road);
    car->m_isSmartCar = true;
    m_smartCar = car.get();
    pushCar(std::move(car));
    m_pendingSmart.reset();
}

void SimulationEngine::spawnSmartCar(QPointF pos, Direction dir, int road) {
    removeSmartCar();   // tylko jedno auto GPS na raz
    // Wjazd dopiero, gdy miejsce na krawędzi jest wolne (jak dla zwykłych aut).
    m_pendingSmart = std::make_unique<PendingSpawn>(PendingSpawn{ pos, dir, road });
    trySpawnPendingSmart();
}

void SimulationEngine::removeSmartCar() {
    m_pendingSmart.reset();
    if (!m_smartCar) return;
    m_objects.erase(
        std::remove_if(m_objects.begin(), m_objects.end(),
            [](const std::unique_ptr<SimulationObject>& o) { return o->isSmartCar(); }),
        m_objects.end());
    m_smartCar = nullptr;
}

bool      SimulationEngine::hasSmartCar()    const { return m_smartCar != nullptr || m_pendingSmart != nullptr; }
QPointF   SimulationEngine::smartCarCenter() const {
    if (m_smartCar) return m_smartCar->center();
    if (m_pendingSmart) {
        const bool poziomo = dirIsHoriz(m_pendingSmart->dir);
        return m_pendingSmart->pos + QPointF(poziomo ? kCarL / 2 : kCarW / 2, poziomo ? kCarW / 2 : kCarL / 2);
    }
    return QPointF{};
}
Direction SimulationEngine::smartCarDir()    const { return m_smartCar ? m_smartCar->dir() : m_pendingSmart ? m_pendingSmart->dir : Direction::L2R; }
int       SimulationEngine::smartCarRoad()   const { return m_smartCar ? m_smartCar->road() : m_pendingSmart ? m_pendingSmart->road : 0; }
bool      SimulationEngine::smartCarOnRing() const { return m_smartCar ? m_smartCar->isOnRing() : false; }
int       SimulationEngine::smartCarRingVI() const { return m_smartCar ? m_smartCar->ringVI() : -1; }
int       SimulationEngine::smartCarRingHI() const { return m_smartCar ? m_smartCar->ringHI() : -1; }

void SimulationEngine::setSmartCarPending(int zoneID, int intent) {
    if (m_smartCar) m_smartCar->setPending(zoneID, intent);
}

void SimulationEngine::setSmartCarIntentFn(SmartCarIntentFn fn) {
    m_smartIntentFn = std::move(fn);
}

int SimulationEngine::resolveIntent(Car* car, int zoneID) {
    // SmartCar z aktywną nawigacją → zawsze intencja z trasy.
    if (car->isSmartCar() && m_smartIntentFn)
        return m_smartIntentFn(zoneID);
    if (car->fixedIntent() >= 0) return car->fixedIntent();
    // Zwykłe auto → wcześniej ustalona intencja lub losowa.
    if (car->pendingZone() == zoneID)
        return car->pendingIntent();
    return m_intentDist(m_rng);
}

// ── Tryb stałego ruchu ──────────────────────────────────────────────

void SimulationEngine::setTargetCarCount(int n) {
    m_targetCarCount = std::max(0, n);
}

int SimulationEngine::regularCarCount() const {
    int count = 0;
    for (const auto& o : m_objects)
        if (!o->isSmartCar()) count++;
    return count;
}

void SimulationEngine::maintainTraffic() {
    if (m_targetCarCount <= 0) return;
    int toSpawn = m_targetCarCount - regularCarCount();
    for (int i = 0; i < toSpawn; ++i) addCar();
}

// ── Rozmieszczanie aut w losowych miejscach na jezdniach ────────────
void SimulationEngine::reset() {
    m_objects.clear();
    m_smartCar = nullptr;
    m_pendingSmart.reset();
    m_targetCarCount = 0;
    m_smartIntentFn = nullptr;
    // m_iState (sygnalizacja) zostawiamy — i tak resetuje się fazami,
    // a usuwanie referencji do skrzyżowań nie ma sensu (grid jest stały).
}

void SimulationEngine::spawnCarsOnMap(int count) {
    count = std::min(count, 100);   // twardy limit — chroni przed zakorkowaniem rond

    // Usuń wszystkie zwykłe auta (zachowaj SmartCar, jeśli istnieje).
    m_objects.erase(
        std::remove_if(m_objects.begin(), m_objects.end(),
            [](const std::unique_ptr<SimulationObject>& o) { return !o->isSmartCar(); }),
        m_objects.end());

    // Zbuduj listę dozwolonych odcinków (między skrzyżowaniami, poza strefami).
    struct Seg { Direction dir; int road; double lo, hi; };
    std::vector<Seg> segs;

    for (int r = 0; r < kNumRoads; ++r) {
        for (int vi = 0; vi < kNumRoads - 1; ++vi) {
            double x0 = kVLeft[vi]   + kRoadW + kCarL;
            double x1 = kVLeft[vi+1] - kCarL;
            if (x1 - x0 > kCarL * 2) {
                segs.push_back({Direction::L2R, r, x0, x1});
                segs.push_back({Direction::R2L, r, x0, x1});
            }
        }
        for (int hi = 0; hi < kNumRoads - 1; ++hi) {
            double y0 = kHTop[hi]   + kRoadW + kCarL;
            double y1 = kHTop[hi+1] - kCarL;
            if (y1 - y0 > kCarL * 2) {
                segs.push_back({Direction::T2B, r, y0, y1});
                segs.push_back({Direction::B2T, r, y0, y1});
            }
        }
    }

    if (segs.empty()) return;
    std::uniform_int_distribution<int> segDist(0, (int)segs.size() - 1);

    for (int i = 0; i < count; ++i) {
        // Do 30 prób znalezienia miejsca bez nakładania.
        for (int attempt = 0; attempt < 30; ++attempt) {
            const Seg& seg = segs[segDist(m_rng)];
            std::uniform_real_distribution<double> posDist(seg.lo, seg.hi);
            double p = posDist(m_rng);

            QPointF pos; double w, h;
            if (seg.dir == Direction::L2R || seg.dir == Direction::R2L) {
                double laneY = (seg.dir == Direction::L2R) ? kL2R_Y(seg.road) : kR2L_Y(seg.road);
                pos = {p, laneY}; w = kCarL; h = kCarW;
            } else {
                double laneX = (seg.dir == Direction::T2B) ? kT2B_X(seg.road) : kB2T_X(seg.road);
                pos = {laneX, p}; w = kCarW; h = kCarL;
            }

            QRectF newRect(pos.x(), pos.y(), w, h);
            newRect.adjust(-4, -4, 4, 4);   // margines bezpieczeństwa

            bool overlap = false;
            for (const auto& o : m_objects)
                if (o->boundingRect().intersects(newRect)) { overlap = true; break; }
            if (!overlap) {
                pushCar(std::make_unique<Car>(pos, seg.dir, seg.road));
                break;
            }
        }
    }

    m_targetCarCount = count;   // utrzymuj tę liczbę aut
}
