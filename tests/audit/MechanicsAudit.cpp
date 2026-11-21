/**
 * @file MechanicsAudit.cpp
 * @brief Scenariusze audytu mechaniki (A: trasowanie, B: ronda, C: bezpieczeństwo,
 *        D: korki, E: stabilność). Patrz MechanicsAudit.h.
 */
#include "MechanicsAudit.h"
#include "core/SimulationEngine.h"
#include "core/Car.h"
#include "core/Roundabout.h"
#include "core/RoadGeometry.h"
#include "nav/NavTypes.h"
#include "nav/TrafficAnalyzer.h"
#include "nav/DynamicPathfinder.h"
#include "nav/RouteManager.h"

#include <QElapsedTimer>
#include <QCoreApplication>
#include <QLineF>
#include <QCryptographicHash>
#include <cstdio>
#include <cmath>
#include <map>
#include <set>
#include <queue>
#include <random>
#include <limits>
#include <algorithm>

namespace Audit {
namespace {

constexpr double kDt = 1.0 / 60.0;

// ════════════════════════════════════════════════════════════════════
//  Pomocnicze — scena
// ════════════════════════════════════════════════════════════════════
Car* lastCar(SimulationEngine& e) {
    return e.objects().empty() ? nullptr : dynamic_cast<Car*>(e.objects().back().get());
}
Car* carById(const SimulationEngine& e, int id) {
    for (const auto& o : e.objects())
        if (auto* c = dynamic_cast<Car*>(o.get())) if (c->id() == id) return c;
    return nullptr;
}
/// Lewy-górny róg auta o środku w @p along (oś jazdy) na pasie (d, road).
QPointF laneTopLeft(Direction d, int road, double along) {
    switch (d) {
    case Direction::L2R: return { along - kCarL / 2, kL2R_Y(road) };
    case Direction::R2L: return { along - kCarL / 2, kR2L_Y(road) };
    case Direction::T2B: return { kT2B_X(road), along - kCarL / 2 };
    default:             return { kB2T_X(road), along - kCarL / 2 };
    }
}
Car* place(SimulationEngine& e, Direction d, int road, double along) {
    e.addCar(laneTopLeft(d, road, along), d, road);
    return lastCar(e);
}
QPointF ringCtr(int vi, int hi) { return { kVLeft[vi] + kRoadW / 2, kHTop[hi] + kRoadW / 2 }; }
/// Środek wzdłuż osi jazdy, w odległości @p dist przed środkiem ronda (vi,hi).
double approachAlong(Direction d, int vi, int hi, double dist) {
    QPointF c = ringCtr(vi, hi);
    switch (d) {
    case Direction::L2R: return c.x() - dist;
    case Direction::R2L: return c.x() + dist;
    case Direction::T2B: return c.y() - dist;
    default:             return c.y() + dist;
    }
}
int roadFor(Direction d, int vi, int hi) { return dirIsHoriz(d) ? hi : vi; }
Direction exitDirFor(Direction in, int intent) {
    const Direction all[4] = { Direction::L2R, Direction::R2L, Direction::T2B, Direction::B2T };
    for (Direction o : all) if (computeIntent(in, o) == intent) return o;
    return in;
}
const char* dname(Direction d) {
    switch (d) { case Direction::L2R: return "L2R"; case Direction::R2L: return "R2L";
                 case Direction::T2B: return "T2B"; default: return "B2T"; }
}
bool cleanStats(const MonitorStats& s, QStringList& f, bool allowContacts = false) {
    auto chk = [&](long v, const char* what) { if (v) f << QString("%1 = %2").arg(what).arg(v); };
    chk(s.nonFinite, "NaN/inf");
    chk(s.overlaps, "nakładania obrysów (OBB)");
    chk(s.sweptHits, "przecięcia trajektorii między próbkami");
    chk(s.teleports, "teleportacje");
    chk(s.reverseJumps, "cofnięcia (korekty)");
    chk(s.headingJumps, "skoki kursu > 20°/krok");
    chk(s.offPath, "zjazdy z toru");
    chk(s.illegalTransitions, "nielegalne zmiany odcinka");
    chk(s.uturns, "zawrócenia");
    chk(s.vanished, "zniknięcia poza krawędzią");
    chk(s.engineCorrections, "korekty resolveOverlaps");
    if (!allowContacts) chk(s.contacts, "styki (odstęp < 1 px)");
    return f.isEmpty();
}

// ════════════════════════════════════════════════════════════════════
//  Obserwator rond: wjazdy, zjazdy, bezpieczeństwo wjazdu, zbędne czekanie
// ════════════════════════════════════════════════════════════════════
struct RbObserver {
    struct CarRb { bool was = false; bool circ = false; int entries = 0, exits = 0; Direction inDir{}, outDir{};
                   double waitStart = -1; double maxWait = 0; };
    std::map<int, CarRb> cars;
    long unsafeEntries = 0;
    double unjustifiedWait = 0;   ///< [s·auto] postój przed pustym rondem
    double maxApproachWait = 0;   ///< najdłuższe czekanie przed wjazdem [s]
    QStringList log;

    void observe(const SimulationEngine& e, double t, double dt) {
        for (const auto& o : e.objects()) {
            const Car* c = dynamic_cast<const Car*>(o.get());
            if (!c) continue;
            CarRb& r = cars[c->id()];
            const bool on = c->isOnRing();
            if (on && !r.was) {
                r.entries++; r.inDir = c->dir();
                if (r.waitStart >= 0) { maxApproachWait = std::max(maxApproachWait, t - r.waitStart); r.waitStart = -1; }
            }
            // Bezpieczeństwo włączenia: w chwili wejścia na okrąg auto krążące
            // za nami musi mieć drogę hamowania + odstęp postoju.
            const bool circ = c->circling();
            if (circ && !r.circ) {
                for (const auto& oo : e.objects()) {
                    const Car* y = dynamic_cast<const Car*>(oo.get());
                    if (!y || y == c || !y->circling() || y->ringVI() != c->ringVI() || y->ringHI() != c->ringHI()) continue;
                    const double up = Rb::mod2pi(y->ringTheta() - c->ringTheta());
                    const double upArc = up * kRingR;
                    const double need = kCarL + kStandGap + y->speed() * y->speed() / (2 * kBrake);
                    if (up < M_PI && upArc < need) {
                        unsafeEntries++;
                        if (log.size() < 10)
                            log << QString("t=%1 car#%2 włączyło się %3 px przed car#%4 (v=%5)")
                                       .arg(t, 0, 'f', 2).arg(c->id()).arg(upArc, 0, 'f', 1)
                                       .arg(y->id()).arg(y->speed(), 0, 'f', 0);
                    }
                }
            }
            r.circ = circ;
            if (!on && r.was) { r.exits++; r.outDir = c->dir(); }
            r.was = on;

            // Czekanie przed rondem
            if (!on) {
                for (int vi = 0; vi < kNumRoads; ++vi) for (int hi = 0; hi < kNumRoads; ++hi) {
                    if (!e.intersectionAt(vi, hi).isRoundabout) continue;
                    if (roadFor(c->dir(), vi, hi) != c->road()) continue;
                    QPointF rc = ringCtr(vi, hi), cc = c->boundingRect().center();
                    double along;
                    switch (c->dir()) {
                    case Direction::L2R: along = rc.x() - cc.x(); break;
                    case Direction::R2L: along = cc.x() - rc.x(); break;
                    case Direction::T2B: along = rc.y() - cc.y(); break;
                    default:             along = cc.y() - rc.y(); break;
                    }
                    if (along < 0 || along > 160) continue;
                    if (c->speed() < 1.0) {
                        if (r.waitStart < 0) r.waitStart = t;
                        // Zbędne: brak auta z przodu na pasie i pusty ring
                        bool ringEmpty = true, carAhead = false;
                        for (const auto& oo : e.objects()) {
                            const Car* y = dynamic_cast<const Car*>(oo.get());
                            if (!y || y == c) continue;
                            if (y->isCircling(vi, hi)) ringEmpty = false;
                            if (!y->isOnRing() && y->dir() == c->dir() && y->road() == c->road()) {
                                QPointF yc = y->boundingRect().center();
                                double ya;
                                switch (c->dir()) {
                                case Direction::L2R: ya = yc.x() - cc.x(); break;
                                case Direction::R2L: ya = cc.x() - yc.x(); break;
                                case Direction::T2B: ya = yc.y() - cc.y(); break;
                                default:             ya = cc.y() - yc.y(); break;
                                }
                                if (ya > 0 && ya < 120) carAhead = true;
                            }
                            if (y->inManeuver() && QLineF(y->boundingRect().center(), rc).length() < kRingR + 30)
                                ringEmpty = false;
                        }
                        if (ringEmpty && !carAhead && !c->held()) unjustifiedWait += dt;
                    }
                }
            }
        }
    }
};

// ════════════════════════════════════════════════════════════════════
//  Uruchamianie scenariusza dynamicznego
// ════════════════════════════════════════════════════════════════════
struct Run {
    SimulationEngine e;
    SimMonitor mon;
    RbObserver rb;
    double dt = kDt;
    long tick = 0;
    double t = 0;
    std::map<QString, double> v;   ///< zmienne scenariusza
    bool useRb = false;
    void step() {
        e.update(dt);
        tick++; t += dt;
        mon.observe(e, dt);
        if (useRb) rb.observe(e, t, dt);
    }
};

using SetupFn  = std::function<void(Run&)>;
using TickFn   = std::function<void(Run&)>;
using FinishFn = std::function<void(Run&, Result&)>;

Result runDynamic(const QString& id, const QString& group, const QString& name,
                  unsigned seed, long ticks, const SetupFn& setup,
                  const TickFn& onTick, const FinishFn& finish,
                  double dt = kDt, bool rbObs = false, bool allowContacts = false) {
    QElapsedTimer timer; timer.start();
    Result r; r.id = id; r.group = group; r.name = name; r.seed = seed;
    Run run; run.dt = dt; run.useRb = rbObs;
    run.e.setSeed(seed);
    setup(run);
    for (long i = 0; i < ticks; ++i) {
        run.step();
        if (onTick) onTick(run);
        if (run.v.count("__stop")) break;
    }
    r.ticks = run.tick; r.simTime = run.t; r.st = run.mon.stats(); r.cars = r.st.maxCars;
    cleanStats(r.st, r.failures, allowContacts);
    if (rbObs) {
        if (run.rb.unsafeEntries)
            r.failures << QString("niebezpieczne wjazdy na rondo = %1").arg(run.rb.unsafeEntries);
        r.notes << QString("zbędne czekanie przed pustym rondem %1 s·auto").arg(run.rb.unjustifiedWait, 0, 'f', 1)
                << QString("max czekanie na wjazd %1 s").arg(run.rb.maxApproachWait, 0, 'f', 1);
        for (auto& l : run.rb.log) r.violations << "RB " + l;
    }
    if (finish) finish(run, r);
    // Diagnostyka zatorów: auta stojące > 20 s — kto je blokuje (odtwarzalność).
    for (const auto& o : run.e.objects()) {
        const Car* c = dynamic_cast<const Car*>(o.get());
        if (!c || c->speed() > 1.0 || r.violations.size() > 40) continue;
        if (run.mon.carsStoppedLongerThan(20.0) == 0) break;
        QPointF p = c->center();
        r.violations << QString("STUCK car#%1 (%2,%3) h=%4° ring=%5 circ=%6 man=%7 dir=%8 road=%9 blockedBy=%10 held=%11")
                            .arg(c->id()).arg(p.x(), 0, 'f', 1).arg(p.y(), 0, 'f', 1).arg(c->headingAngle(), 0, 'f', 0)
                            .arg(c->isOnRing()).arg(c->circling()).arg(c->inManeuver()).arg(dname(c->dir())).arg(c->road())
                            .arg(c->lastCapBy()).arg(c->held());
    }
    r.notes << QString("minGap %1 px, maxΔkurs %2°/krok, max postój %3 s, dojechało %4")
                   .arg(r.st.minGap > 1e8 ? -1.0 : r.st.minGap, 0, 'f', 2)
                   .arg(r.st.maxHeadingStep, 0, 'f', 1)
                   .arg(r.st.maxStoppedTime, 0, 'f', 1).arg(r.st.exited);
    r.violations << run.mon.violations().mid(0, 12);
    r.pass = r.failures.isEmpty();
    r.runtimeMs = timer.elapsed();
    return r;
}

// ════════════════════════════════════════════════════════════════════
//  Wyrocznia: niezależny Dijkstra na wyeksportowanym grafie stanów
// ════════════════════════════════════════════════════════════════════
double dijkstraOracle(const DynamicPathfinder& pf, const std::vector<Checkpoint>& cps,
                      int startState, int goalNode) {
    auto edges = pf.exportGraph(cps, startState);
    const int N = DynamicPathfinder::stateCount();
    std::vector<std::vector<std::pair<int, double>>> adj(N);
    for (auto& e : edges) adj[e.fromState].push_back({ e.toState, e.cost });
    std::vector<double> d(N, std::numeric_limits<double>::infinity());
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    d[startState] = 0; pq.push({ 0, startState });
    while (!pq.empty()) {
        auto [du, u] = pq.top(); pq.pop();
        if (du > d[u]) continue;
        if (pf.nodeOfState(u) == goalNode) return du;
        for (auto [w, c] : adj[u]) if (du + c < d[w]) { d[w] = du + c; pq.push({ d[w], w }); }
    }
    return std::numeric_limits<double>::infinity();
}

/// Sprawdza legalność trasy: kolejne węzły sąsiednie, brak zawracania, poprawne końce.
bool routeLegal(const Route& r, int startCp, int endCp, QString& why) {
    if (r.size() < 2) { why = "trasa krótsza niż 2 kroki"; return false; }
    if (startCp >= 0 && r.front().cpId != startCp) { why = "zły start"; return false; }
    if (r.back().cpId != endCp) { why = "zły cel"; return false; }
    for (size_t i = 1; i < r.size(); ++i) {
        if (r[i].dir == dirOpposite(r[i - 1].dir) && r[i - 1].vi >= 0) { why = "zawracanie"; return false; }
        if (r[i].vi >= 0 && r[i - 1].vi >= 0) {
            int dv = r[i].vi - r[i - 1].vi, dh = r[i].hi - r[i - 1].hi;
            if (std::abs(dv) + std::abs(dh) != 1) { why = "węzły niesąsiednie"; return false; }
            Direction expect = dv > 0 ? Direction::L2R : dv < 0 ? Direction::R2L
                             : dh > 0 ? Direction::T2B : Direction::B2T;
            if (r[i].dir != expect) { why = "kierunek niezgodny z ruchem"; return false; }
        }
    }
    return true;
}

struct OracleStats { int pairs = 0, worse = 0, illegal = 0, costMismatch = 0, unreachable = 0;
                     double maxExcess = 0; long expAstar = 0; QStringList log; };

void compareAll(DynamicPathfinder& pf, const std::vector<Checkpoint>& cps, OracleStats& os,
                int pairStride = 1) {
    for (int a = 0; a < 32; a += 1)
        for (int b = 0; b < 32; b += pairStride) {
            if (a == b) continue;
            double cost = -1;
            Route r = pf.findRoute(a, b, cps, &cost);
            os.expAstar += pf.lastExpansions();
            double ref = dijkstraOracle(pf, cps, pf.startStateOfCheckpoint(a), pf.goalNodeOfCheckpoint(b));
            os.pairs++;
            if (std::isinf(ref)) { if (!r.empty()) os.illegal++; os.unreachable++; continue; }
            if (r.empty()) { os.worse++; if (os.log.size() < 8) os.log << QString("%1→%2: A* brak trasy, Dijkstra %3").arg(a).arg(b).arg(ref); continue; }
            QString why;
            if (!routeLegal(r, a, b, why)) { os.illegal++; if (os.log.size() < 8) os.log << QString("%1→%2: %3").arg(a).arg(b).arg(why); }
            double ev = pf.evaluateRoute(r, cps);
            if (std::abs(ev - cost) > 1e-6) { os.costMismatch++; if (os.log.size() < 8) os.log << QString("%1→%2: koszt zgłoszony %3 ≠ przeliczony %4").arg(a).arg(b).arg(cost).arg(ev); }
            if (cost > ref + 1e-6) {
                os.worse++; os.maxExcess = std::max(os.maxExcess, cost - ref);
                if (os.log.size() < 8) os.log << QString("%1→%2: A* %3 > Dijkstra %4").arg(a).arg(b).arg(cost, 0, 'f', 4).arg(ref, 0, 'f', 4);
            }
        }
}

Result oracleResult(const QString& id, const QString& name, unsigned seed, OracleStats& os,
                    const QString& extra = QString()) {
    Result r; r.id = id; r.group = "A"; r.name = name; r.seed = seed; r.dynamic = false;
    if (os.worse) r.failures << QString("A* droższy/niepełny niż Dijkstra: %1 par (max nadwyżka %2)").arg(os.worse).arg(os.maxExcess, 0, 'f', 4);
    if (os.illegal) r.failures << QString("nielegalne trasy: %1").arg(os.illegal);
    if (os.costMismatch) r.failures << QString("koszt zgłoszony ≠ koszt trasy: %1").arg(os.costMismatch);
    r.notes << QString("%1 par, %2 nieosiągalnych, śr. rozwinięć A* %3")
                   .arg(os.pairs).arg(os.unreachable).arg(os.pairs ? double(os.expAstar) / os.pairs : 0, 0, 'f', 1);
    if (!extra.isEmpty()) r.notes << extra;
    r.violations = os.log;
    r.pass = r.failures.isEmpty();
    return r;
}

/// Koszt bazowy odcinka (sama geometria) — do losowych wag ≥ baza.
double baseSegTime(Direction dir, int from, int to) {
    double lo, hi;
    if (dirIsHoriz(dir)) {
        int s = (dir == Direction::L2R) ? from + 1 : (from < 0 ? kNumRoads : from);
        lo = (s == 0) ? 0.0 : kVLeft[s - 1] + kRoadW; hi = (s == kNumRoads) ? kCanvasW : kVLeft[s];
    } else {
        int s = (dir == Direction::T2B) ? from + 1 : (from < 0 ? kNumRoads : from);
        lo = (s == 0) ? 0.0 : kHTop[s - 1] + kRoadW; hi = (s == kNumRoads) ? kCanvasH : kHTop[s];
    }
    return std::abs(hi - lo) / kMaxSpeed;
}

/// Rozgrzewa silnik ruchem (snapshot zatłoczenia do testów trasowania).
void warmTraffic(SimulationEngine& e, unsigned seed, int cars, double seconds) {
    e.setSeed(seed);
    e.spawnCarsOnMap(cars);
    for (int i = 0; i < int(seconds / kDt); ++i) e.update(kDt);
}

// ════════════════════════════════════════════════════════════════════
//  Prowadzenie SmartCar zadaną trasą (pomiar rzeczywistego czasu)
// ════════════════════════════════════════════════════════════════════
void installRouteIntent(SimulationEngine& e, const Route& route) {
    std::map<int, Direction> exits;
    for (size_t i = 0; i + 1 < route.size(); ++i)
        if (route[i].vi >= 0) exits[route[i].vi * kNumRoads + route[i].hi] = route[i + 1].dir;
    SimulationEngine* pe = &e;
    e.setSmartCarIntentFn([exits, pe](int z) {
        auto it = exits.find(z);
        if (it == exits.end()) return 0;
        int in = computeIntent(pe->smartCarDir(), it->second);
        return in < 0 ? 0 : in;
    });
}

// ════════════════════════════════════════════════════════════════════
//  Scenariusze
// ════════════════════════════════════════════════════════════════════

// ── A: trasowanie ────────────────────────────────────────────────────
Result A1() {
    SimulationEngine e; TrafficAnalyzer ta(&e); ta.update();
    DynamicPathfinder pf(&e, &ta); auto cps = buildCheckpoints();
    OracleStats os; compareAll(pf, cps, os);
    // Spójność kosztu z czasem fizycznym: pusta mapa, trasa na wprost 0→8.
    double c = 0; pf.findRoute(0, 8, cps, &c);
    double geo = kCanvasW / kMaxSpeed - kNumRoads * kRoadW / kMaxSpeed;
    Result r = oracleResult("A1", "A* vs Dijkstra — pusta sieć", 0, os,
                            QString("koszt 0→8 (prosto) = %1 s, czas geometryczny dróg = %2 s").arg(c, 0, 'f', 2).arg(geo, 0, 'f', 2));
    // Pusta sieć: koszt nie może zawierać fikcyjnych dziesiątek sekund.
    double c2 = 0; pf.findRoute(8, 0, cps, &c2);
    r.notes << QString("koszt 8→0 (prosto, R2L) = %1 s").arg(c2, 0, 'f', 2);
    if (std::abs(c - c2) > 3.0) {
        r.failures << QString("asymetria kosztu kierunków na pustej mapie: L2R %1 s vs R2L %2 s").arg(c, 0, 'f', 2).arg(c2, 0, 'f', 2);
        r.pass = false;
    }
    return r;
}

Result A2(unsigned seed) {
    SimulationEngine e; warmTraffic(e, seed, 70, 25.0);
    TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta); auto cps = buildCheckpoints();
    OracleStats os; compareAll(pf, cps, os);
    return oracleResult(QString("A2-%1").arg(seed), "A* vs Dijkstra — snapshot z korkami (70 aut)", seed, os);
}

Result A4(unsigned seed) {
    // Losowe wagi ≥ geometria (różne natężenia) + identyczne koszty (remisy).
    SimulationEngine e; TrafficAnalyzer ta(&e); ta.update();
    DynamicPathfinder pf(&e, &ta); auto cps = buildCheckpoints();
    OracleStats os;
    std::mt19937 rng(seed);
    for (int trial = 0; trial < 12; ++trial) {
        std::map<long, double> w;
        std::uniform_real_distribution<double> U(0.0, 1.0);
        const bool ties = (trial % 3 == 2);
        pf.setCostOverride([&](Direction d, int road, int from, int to, int, int) {
            long key = ((long)d * 16 + road) * 32 + (from + 1) * 2 + (to > from);
            auto it = w.find(key);
            if (it == w.end()) {
                double extra = ties ? std::floor(U(rng) * 3) : U(rng) * 12.0 * U(rng);
                it = w.emplace(key, baseSegTime(d, from, to) + extra).first;
            }
            return it->second;
        });
        compareAll(pf, cps, os, 5);
    }
    return oracleResult(QString("A4-%1").arg(seed), "A* vs Dijkstra — losowe wagi i remisy (12 grafów)", seed, os);
}

Result A10() {
    SimulationEngine e; TrafficAnalyzer ta(&e); ta.update();
    DynamicPathfinder pf(&e, &ta); auto cps = buildCheckpoints();
    // Zamknięcie wszystkich wjazdów do skrzyżowania (3,3) oraz odcięcie celu 8.
    pf.setCostOverride([](Direction d, int road, int from, int to, int toVi, int toHi) {
        if (toVi == 3 && toHi == 3) return std::numeric_limits<double>::infinity();
        if (dirIsHoriz(d) && road == 0 && d == Direction::L2R && to >= kNumRoads)
            return std::numeric_limits<double>::infinity();
        return baseSegTime(d, from, to);
    });
    OracleStats os; compareAll(pf, cps, os);
    Result r = oracleResult("A10", "Brak dostępnej trasy / odcinki zamknięte", 0, os);
    double c = 12345; Route rr = pf.findRoute(5, 8, cps, &c);
    if (!rr.empty()) { r.failures << "zwrócono trasę do nieosiągalnego celu"; r.pass = false; }
    for (auto& s : rr) if (s.vi == 3 && s.hi == 3) { r.failures << "trasa przez zamknięty węzeł"; r.pass = false; }
    r.notes << QString("cel 8 nieosiągalny: trasa pusta = %1").arg(rr.empty() ? "TAK" : "NIE");
    return r;
}

/// Zator: N unieruchomionych aut na pasie L2R drogi hi=0 między vi=3 a vi=4.
void buildJam(SimulationEngine& e, int n, bool hold) {
    double x = kVLeft[4] - 30;
    for (int i = 0; i < n; ++i) {
        Car* c = place(e, Direction::L2R, 0, x - i * (kCarL + 8));
        if (c) c->setHeld(hold);
    }
}

Result A3_D5(bool incident) {
    // Dwie alternatywy z 0 (lewa krawędź, hi=0) do 8 (prawa krawędź, hi=0):
    //  A) na wprost przez zatłoczony korytarz,  B) objazd przez hi=1.
    // Rzeczywisty czas mierzymy, prowadząc SmartCar KAŻDĄ trasą w identycznej
    // (deterministycznej) scenie — to jest „prawda", z którą porównujemy wybór A*.
    //  • wariant zwykły: korek powstaje z ruchu (gęsta kolumna aut, światła),
    //  • wariant „incident": 6 aut unieruchomionych na 40 s (awaria) — snapshot
    //    nie zawiera informacji o czasie trwania awarii; wynik raportujemy jako
    //    znane ograniczenie predykcji, reakcję dynamiczną sprawdza A7/A8.
    Result r; r.id = incident ? "A3i" : "A3/D5"; r.group = "A";
    r.name = incident ? "Awaria 40 s na krótszej trasie (ograniczenie snapshotu)"
                      : "Jednorazowy pluton na krótszej trasie (ograniczenie)";
    r.seed = 7; r.dynamic = true;
    auto cps = buildCheckpoints();
    auto scene = [&](SimulationEngine& e) {
        e.setSeed(7); e.setRoundabouts({});
        if (incident) buildJam(e, 6, true);
        else for (int i = 0; i < 16; ++i) { Car* c = place(e, Direction::L2R, 0, 420 + i * 58.0); c->setPending(-1, 0); }
        for (int k = 0; k < (incident ? 120 : 360); ++k) e.update(kDt);
    };
    // Trasy kandydackie: wymuszone wagami (prosto / objazd).
    SimulationEngine e0; scene(e0);
    TrafficAnalyzer ta(&e0); ta.update(); DynamicPathfinder pf(&e0, &ta);
    double chosenCost = 0; Route chosen = pf.findRoute(0, 8, cps, &chosenCost);
    Route straight, detour;
    {
        DynamicPathfinder p2(&e0, &ta);
        p2.setCostOverride([](Direction d, int, int from, int to, int, int) {
            return baseSegTime(d, from, to) + (dirIsHoriz(d) ? 0 : 50); });
        straight = p2.findRoute(0, 8, cps);
        p2.setCostOverride([](Direction d, int road, int from, int to, int, int) {
            double b = baseSegTime(d, from, to);
            if (d == Direction::L2R && road == 0 && from >= 2) return b + 100;
            return b; });
        detour = p2.findRoute(0, 8, cps);
    }
    double costStraight = pf.evaluateRoute(straight, cps), costDetour = pf.evaluateRoute(detour, cps);
    // Rozbicie kosztu trasy na prosto: odcinki korytarza hi=0 (cechy z analizatora).
    {
        QStringList parts;
        for (int sgi = 0; sgi <= kNumRoads; ++sgi) {
            const int from = sgi - 1, to = sgi < kNumRoads ? sgi : kNumRoads;
            parts << QString("s%1:%2/%3@%4").arg(sgi).arg(ta.stoppedOnSegment(Direction::L2R, 0, from, to))
                         .arg(ta.carsOnSegment(Direction::L2R, 0, from, to))
                         .arg(ta.avgSpeedOnSegment(Direction::L2R, 0, from, to), 0, 'f', 0);
        }
        r.notes << "korytarz hi=0 (stojące/wszystkie@v): " + parts.join(" ");
    }
    auto drive = [&](const Route& route, MonitorStats& st) -> double {
        Run run; scene(run.e);
        // awaria (wariant incident) kończy się po 40 s
        run.e.spawnSmartCar(cps[0].spawnPos, cps[0].entryDir, cps[0].roadIdx);
        installRouteIntent(run.e, route);
        for (long i = 0; i < 60 * 120; ++i) {
            if (run.t > 40.0) for (auto& o : run.e.objects()) if (auto* c = dynamic_cast<Car*>(o.get())) c->setHeld(false);
            run.step();
            if (!run.e.hasSmartCar()) break;
        }
        st = run.mon.stats();
        return run.e.hasSmartCar() ? 1e9 : run.t;
    };
    MonitorStats s1, s2, s3;
    double tStraight = drive(straight, s1), tDetour = drive(detour, s2), tChosen = drive(chosen, s3);
    auto key = [](const Route& x) { QString k; for (auto& st : x) k += QString::number(st.nodeId) + ","; return k; };
    const QString kind = key(chosen) == key(straight) ? "prosto" : key(chosen) == key(detour) ? "objazd" : "inna";
    r.notes << QString("przewidywane: prosto %1 s, objazd %2 s, wybrana %3 s; rzeczywiste: prosto %4 s, objazd %5 s, wybrana %6 s")
                   .arg(costStraight, 0, 'f', 1).arg(costDetour, 0, 'f', 1).arg(chosenCost, 0, 'f', 1)
                   .arg(tStraight, 0, 'f', 1).arg(tDetour, 0, 'f', 1).arg(tChosen, 0, 'f', 1);
    r.notes << QString("wybór A*: %1").arg(kind);
    const double best = std::min(tStraight, tDetour);
    if (tChosen > best + 3.0) {
        const QString msg = QString("A* wybrał trasę wolniejszą o %1 s od najlepszej sprawdzonej").arg(tChosen - best, 0, 'f', 1);
        if (incident) r.notes << "OGRANICZENIE: " + msg + " — awaria wygląda w snapshocie jak zwykła kolejka";
        else r.notes << "OGRANICZENIE: " + msg + " — jednorazowy pluton stoi na czerwonym w chwili decyzji, "
                        "a rozładowuje się przed dojazdem (w ruchu ustalonym kolejki odnawiają się co cykl; patrz D5s)";
    }
    { QStringList f3; cleanStats(s3, f3); for (auto& x : f3) r.failures << "jazda trasą wybraną: " + x; }
    QStringList f1, f2; cleanStats(s1, f1); cleanStats(s2, f2);
    for (auto& x : f1) r.failures << "jazda prosto: " + x;
    for (auto& x : f2) r.failures << "jazda objazdem: " + x;
    r.st = s2; r.ticks = s2.ticks; r.simTime = s2.simTime; r.cars = s2.maxCars;
    r.pass = r.failures.isEmpty();
    return r;
}

Result A5A6() {
    // Rondo (4,4) w środku: trasa przez rondo; następnie kolejka na wlocie → objazd.
    Result r; r.id = "A5/A6"; r.group = "A"; r.name = "Trasa przez rondo / omijająca zakorkowane rondo"; r.dynamic = false;
    auto cps = buildCheckpoints();
    SimulationEngine e; e.setRoundabouts({ { 4, 4 } });
    TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta);
    double c1 = 0; Route through = pf.findRoute(4, 12, cps, &c1);   // hi=4 L2R → prawa krawędź hi=4
    bool viaRb = false; for (auto& s : through) if (s.vi == 4 && s.hi == 4) viaRb = true;
    OracleStats os; compareAll(pf, cps, os, 3);
    // Kolejka 6 stojących aut przed rondem na wlocie L2R.
    for (int i = 0; i < 6; ++i) { Car* c = place(e, Direction::L2R, 4, approachAlong(Direction::L2R, 4, 4, 120 + i * 48)); c->setHeld(true); }
    for (int k = 0; k < 120; ++k) e.update(kDt);
    ta.update();
    double c2 = 0; Route avoid = pf.findRoute(4, 12, cps, &c2);
    bool viaRb2 = false; for (auto& s : avoid) if (s.vi == 4 && s.hi == 4) viaRb2 = true;
    compareAll(pf, cps, os, 3);
    r = oracleResult("A5/A6", "Trasa przez rondo / omijająca zakorkowane rondo", 0, os);
    r.notes << QString("pusto: koszt %1 s, przez rondo: %2; z kolejką: koszt %3 s, przez rondo: %4")
                   .arg(c1, 0, 'f', 1).arg(viaRb ? "TAK" : "NIE").arg(c2, 0, 'f', 1).arg(viaRb2 ? "TAK" : "NIE");
    if (!viaRb) r.failures << "na pustej mapie trasa na wprost nie przechodzi przez rondo";
    if (c2 <= c1) r.failures << "kolejka przed rondem nie podniosła kosztu";
    r.pass = r.failures.isEmpty();
    return r;
}

/// A7/A8/A9: dynamiczny rerouting z RouteManager.
Result rerouteScenario(const QString& id, const QString& name, unsigned seed, bool createJam, int bgCars) {
    Result r; r.id = id; r.group = "A"; r.name = name; r.seed = seed;
    Run run; run.e.setSeed(seed); run.e.setRoundabouts({ { 2, 2 }, { 5, 5 } });
    if (bgCars) { run.e.spawnCarsOnMap(bgCars); run.e.setTargetCarCount(bgCars); }
    RouteManager rm(&run.e);
    rm.startNavigation(0, 13);    // lewa hi=0 → prawa krawędź hi=5
    Route first = rm.currentRoute();
    int changes = 0, flips = 0; std::vector<Route> history{ first };
    bool finished = false, failed = false;
    QObject::connect(&rm, &RouteManager::navigationFinished, [&] { finished = true; });
    QObject::connect(&rm, &RouteManager::navigationFailed, [&] { failed = true; });
    QObject::connect(&rm, &RouteManager::routeChanged, [&](Route nr) {
        changes++;
        // flip = powrót do trasy sprzed dwóch zmian (A→B→A)
        if (history.size() >= 2) {
            const Route& old = history[history.size() - 2];
            auto key = [](const Route& x) { QString k; for (auto& s : x) if (s.vi >= 0) k += QString("%1,%2;").arg(s.vi).arg(s.hi); return k; };
            QString kn = key(nr), ko = key(old);
            if (!kn.isEmpty() && ko.endsWith(kn) && kn != key(history.back()) && !key(history.back()).endsWith(kn)) flips++;
        }
        history.push_back(nr);
    });
    bool jammed = false;
    for (long i = 0; i < 60 * 150 && !finished && !failed; ++i) {
        if (createJam && !jammed && run.t > 3.0) {
            // Zator na trasie przed autem: pierwszy poziomy odcinek planu dalej niż 600 px.
            const Route& cur = rm.currentRoute();
            QPointF sc = run.e.smartCarCenter();
            for (size_t k = 0; k + 1 < cur.size(); ++k) {
                const RouteStep& a = cur[k]; const RouteStep& b = cur[k + 1];
                if (a.vi < 0 || b.vi < 0 || !dirIsHoriz(b.dir) || b.dir != Direction::L2R) continue;
                if (kVLeft[a.vi] - sc.x() < 500) continue;
                double x = kVLeft[b.vi] - 30;
                for (int n = 0; n < 5; ++n) { Car* c = place(run.e, Direction::L2R, a.hi, x - n * 48); if (c) c->setHeld(true); }
                r.notes << QString("zator utworzony t=%1 s na hi=%2 między vi=%3 a vi=%4").arg(run.t, 0, 'f', 1).arg(a.hi).arg(a.vi).arg(b.vi);
                run.v["jamVi"] = b.vi; run.v["jamHi"] = a.hi;
                jammed = true; break;
            }
        }
        rm.tick();
        run.step();
        QCoreApplication::processEvents();
    }
    r.st = run.mon.stats(); r.ticks = run.tick; r.simTime = run.t; r.cars = r.st.maxCars;
    cleanStats(r.st, r.failures, bgCars > 0);
    // Mechanika zwykłych aut w tle (styk/overlap) oceniana jest w grupach C/E —
    // tu liczymy wszystko, ale nie przypisujemy ich trasowaniu.
    r.notes << QString("zmian trasy: %1, powrotów A→B→A: %2, dotarł: %3, zgubił: %4, czas %5 s")
                   .arg(changes).arg(flips).arg(finished ? "TAK" : "NIE").arg(failed ? "TAK" : "NIE").arg(run.t, 0, 'f', 1);
    if (!finished) r.failures << "SmartCar nie dotarł do celu";
    if (flips > 0) r.failures << QString("oscylacja tras A→B→A: %1").arg(flips);
    if (!createJam && changes > 4) r.failures << QString("zbyt wiele zmian trasy bez istotnej przyczyny: %1").arg(changes);
    if (createJam && run.v.count("jamVi")) {
        int jv = (int)run.v["jamVi"], jh = (int)run.v["jamHi"];
        bool still = false; const Route& fin = history.back();
        for (size_t k = 0; k + 1 < fin.size(); ++k)
            if (fin[k + 1].vi == jv && fin[k + 1].hi == jh && fin[k + 1].dir == Direction::L2R) still = true;
        r.notes << QString("trasa po zatorze omija go: %1").arg(still ? "NIE" : "TAK");
        if (changes == 0) r.failures << "brak reakcji trasowania na zator przed autem";
    }
    r.violations = run.mon.violations().mid(0, 10);
    r.pass = r.failures.isEmpty();
    return r;
}

// ── B: ronda ─────────────────────────────────────────────────────────
Result Bsingle(const QString& id, const QString& name, int intent) {
    // 4 przebiegi (każdy kierunek) w osobnych silnikach, statystyki sumowane.
    Result agg; agg.id = id; agg.group = "B"; agg.name = name; agg.seed = 0;
    const Direction dirs[4] = { Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T };
    for (Direction d : dirs) {
        Result r = runDynamic(id, "B", name, 0, 60 * 20,
            [&](Run& run) {
                run.e.setRoundabouts({ { 4, 4 } });
                // Pion: kwartał ma 180 px — start za skrzyżowaniem (4,3)/(4,5), inaczej
                // intencja byłaby rozstrzygnięta wcześniej na tamtym skrzyżowaniu.
                Car* c = place(run.e, d, 4, approachAlong(d, 4, 4, dirIsHoriz(d) ? 260 : 125));
                c->setPending(4 * kNumRoads + 4, intent);
                run.v["id"] = c->id();
            },
            [&](Run& run) {
                Car* c = carById(run.e, (int)run.v["id"]);
                if (c && c->isOnRing()) run.v["entered"] = 1;
                if (c && run.v.count("entered") && !c->isOnRing() && !c->inManeuver() && !run.v.count("outDir")) {
                    run.v["outDir"] = (int)c->dir();
                    run.v["exitT"] = run.t;
                }
                if (run.v.count("exitT") && run.t > run.v["exitT"] + 2.0) run.v["__stop"] = 1;
            },
            [&](Run& run, Result& r) {
                Direction want = exitDirFor(d, intent);
                if (!run.v.count("entered")) r.failures << QString("%1: auto nie wjechało na rondo").arg(dname(d));
                else if (!run.v.count("outDir")) r.failures << QString("%1: auto nie zjechało z ronda").arg(dname(d));
                else if ((Direction)(int)run.v["outDir"] != want)
                    r.failures << QString("%1: zły zjazd %2 (oczekiwano %3)").arg(dname(d)).arg(dname((Direction)(int)run.v["outDir"])).arg(dname(want));
            }, kDt, true);
        agg.ticks += r.ticks; agg.simTime += r.simTime; agg.cars = std::max(agg.cars, r.cars);
        auto& a = agg.st; const auto& s = r.st;
        a.overlaps += s.overlaps; a.teleports += s.teleports; a.headingJumps += s.headingJumps;
        a.reverseJumps += s.reverseJumps; a.offPath += s.offPath; a.contacts += s.contacts;
        a.engineCorrections += s.engineCorrections; a.sweptHits += s.sweptHits;
        a.maxHeadingStep = std::max(a.maxHeadingStep, s.maxHeadingStep);
        a.maxExcessJump = std::max(a.maxExcessJump, s.maxExcessJump);
        a.minGap = std::min(a.minGap, s.minGap); a.exited += s.exited; a.maxCars = std::max(a.maxCars, s.maxCars);
        for (auto& f : r.failures) agg.failures << QString("[%1] %2").arg(dname(d)).arg(f);
        agg.violations << r.violations.mid(0, 3);
        if (d == Direction::L2R) agg.notes << r.notes;
    }
    agg.notes << QString("max nadmiar skoku %1 px, max Δkurs %2°").arg(agg.st.maxExcessJump, 0, 'f', 1).arg(agg.st.maxHeadingStep, 0, 'f', 1);
    agg.pass = agg.failures.isEmpty();
    return agg;
}

Result B4() {
    return runDynamic("B4", "B", "Kilka pojazdów na rondzie (5 aut, 4 wloty)", 11, 60 * 40,
        [](Run& run) {
            run.e.setRoundabouts({ { 4, 4 } });
            const Direction dirs[4] = { Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T };
            int k = 0;
            for (Direction d : dirs) { Car* c = place(run.e, d, 4, approachAlong(d, 4, 4, 200 + 30 * k)); c->setPending(36, k % 3); k++; }
            Car* c = place(run.e, Direction::L2R, 4, approachAlong(Direction::L2R, 4, 4, 290)); c->setPending(36, 2);
        },
        [](Run& run) { if (run.e.objects().empty()) run.v["__stop"] = 1; },
        [](Run& run, Result& r) {
            int through = 0; for (auto& [id, c] : run.rb.cars) if (c.exits > 0) through++;
            r.notes << QString("przejechało rondo %1/5").arg(through);
            if (through < 5) r.failures << QString("tylko %1/5 aut przejechało rondo").arg(through);
        }, kDt, true);
}

Result B5B6(unsigned seed) {
    // Ciągły dopływ z 4 stron jednocześnie: ustępowanie i jednoczesne wjazdy.
    return runDynamic(QString("B5/B6-%1").arg(seed), "B", "Ustępowanie + jednoczesny wjazd z 4 stron", seed, 60 * 90,
        [](Run& run) {
            run.e.setRoundabouts({ { 4, 4 } });
            const Direction dirs[4] = { Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T };
            for (int wave = 0; wave < 4; ++wave)
                for (Direction d : dirs) { Car* c = place(run.e, d, 4, approachAlong(d, 4, 4, 140 + wave * 70)); c->setFixedIntent(wave % 3); }
        },
        nullptr,
        [](Run& run, Result& r) {
            // Liczymy auta, które WJECHAŁY na rondo (część skręca wcześniej na (4,3)/(4,5)).
            int through = 0, n = 0; for (auto& [id, c] : run.rb.cars) if (c.entries > 0) { n++; if (c.exits > 0) through++; }
            const int jam = run.mon.carsStoppedLongerThan(30.0);
            r.notes << QString("wjechało na rondo %1, zjechało %2, stojących > 30 s: %3").arg(n).arg(through).arg(jam);
            if (n < 8) r.failures << QString("za mało aut dotarło do ronda (%1) — scenariusz nie sprawdza konfliktów").arg(n);
            if (through < n) r.failures << QString("utknęło na rondzie %1 aut").arg(n - through);
            if (jam > 0) r.failures << QString("%1 aut zakleszczonych (> 30 s)").arg(jam);
        }, kDt, true);
}

Result B7() {
    // Zablokowany wylot: auto stoi (awaria) tuż za zjazdem wschodnim; po 12 s rusza.
    return runDynamic("B7", "B", "Zablokowany wylot ronda (awaria 12 s)", 3, 60 * 45,
        [](Run& run) {
            run.e.setRoundabouts({ { 4, 4 } });
            Car* blk = place(run.e, Direction::L2R, 4, ringCtr(4, 4).x() + kRingR + 36);
            blk->setHeld(true); run.v["blk"] = blk->id();
            for (int i = 0; i < 3; ++i) { Car* c = place(run.e, Direction::L2R, 4, approachAlong(Direction::L2R, 4, 4, 200 + i * 60)); c->setFixedIntent(0); }
            Car* t = place(run.e, Direction::T2B, 4, approachAlong(Direction::T2B, 4, 4, 125)); t->setFixedIntent(2);  // też na wschód
        },
        [](Run& run) {
            if (run.t > 12.0) if (Car* b = carById(run.e, (int)run.v["blk"])) b->setHeld(false);
            if (run.e.objects().empty()) run.v["__stop"] = 1;
        },
        [](Run& run, Result& r) {
            int through = 0, n = 0; for (auto& [id, c] : run.rb.cars) if (c.entries > 0) { n++; if (c.exits > 0) through++; }
            r.notes << QString("wjechało %1, przejechało rondo %2 (po odblokowaniu wylotu)").arg(n).arg(through);
            if (n < 4 || through < n) r.failures << QString("po odblokowaniu wylotu przejechało tylko %1/%2").arg(through).arg(n);
        }, kDt, true);
}

Result B10(unsigned seed) {
    return runDynamic(QString("B10-%1").arg(seed), "B", "Duże natężenie na 6 rondach (80 aut, 120 s)", seed, 60 * 120,
        [](Run& run) {
            run.e.setRoundabouts({ { 1, 2 }, { 2, 5 }, { 4, 4 }, { 6, 3 }, { 5, 6 }, { 3, 1 } });
            run.e.spawnCarsOnMap(80); run.e.setTargetCarCount(80);
        }, nullptr,
        [](Run&, Result& r) { if (r.st.exited < 20) r.failures << QString("niska przepustowość sieci: %1 aut dojechało").arg(r.st.exited); },
        kDt, true);
}

// ── C: bezpieczeństwo ────────────────────────────────────────────────
Result C1() {
    return runDynamic("C1", "C", "Jazda za wolniejszym pojazdem (50 vs 110 px/s)", 1, 60 * 30,
        [](Run& run) {
            run.e.setRoundabouts({});
            Car* l = place(run.e, Direction::L2R, 3, 300); l->setMaxSpeed(50); l->setFixedIntent(0);
            place(run.e, Direction::L2R, 3, 150)->setFixedIntent(0);
        }, nullptr, nullptr);
}
Result C2C3(const QString& id, const QString& name, double headStart, double holdAt) {
    return runDynamic(id, "C", name, 2, 60 * 25,
        [=](Run& run) {
            run.e.setRoundabouts({});
            // Odcinek 2-3 bez świateł po drodze: lider i 3 auta za nim.
            Car* l = place(run.e, Direction::T2B, 2, kHTop[0] + kRoadW + 60 + headStart);
            run.v["lid"] = l->id(); l->setFixedIntent(0);
            for (int i = 1; i <= 3; ++i) place(run.e, Direction::T2B, 2, kHTop[0] + kRoadW + 60 + headStart - i * 70.0)->setFixedIntent(0);
            run.e.setIntersectionPhase(2, 1, 2); run.e.setIntersectionPhase(2, 2, 2); run.e.setIntersectionPhase(2, 3, 2);
        },
        [=](Run& run) {
            Car* l = carById(run.e, (int)run.v["lid"]);
            if (l) l->setHeld(run.t > holdAt && run.t < holdAt + 5.0);
        }, nullptr);
}
Result C4C5() {
    // 8 aut dojeżdża do czerwonego na (3,3); zielone po 10 s.
    return runDynamic("C4/C5/D1-3", "C", "Kolejka na czerwonym i ruszanie (8 aut)", 4, 60 * 40,
        [](Run& run) {
            run.e.setRoundabouts({});
            run.e.setIntersectionPhase(3, 3, 2);           // H czerwone przez 10 s
            run.e.setIntersectionPhase(2, 3, 0, 0.0);
            for (int i = 0; i < 8; ++i) { Car* c = place(run.e, Direction::L2R, 3, kVLeft[3] - 80 - i * 75.0); c->setFixedIntent(0); }
        },
        [](Run& run) {
            TrafficAnalyzer ta(&run.e); ta.update();
            int q = ta.carsOnSegment(Direction::L2R, 3, 2, 3);
            if (std::abs(run.t - 9.5) < kDt / 2) {
                int stopped = 0; for (auto& o : run.e.objects()) if (o->speed() < 1) stopped++;
                run.v["q95"] = q; run.v["stopped95"] = stopped;
            }
            run.v["qmax"] = std::max(run.v["qmax"], (double)q);
            if (run.t > 10 && q == 0 && !run.v.count("cleared")) run.v["cleared"] = run.t;
        },
        [](Run& run, Result& r) {
            double len = kVLeft[3] - (kVLeft[2] + kRoadW);
            int cap = int(len / kCarL);
            r.notes << QString("kolejka(t=9.5s): %1 aut na odcinku 2→3, stoi %2; max na odcinku %3 (fizyczny limit %4); odcinek pusty po %5 s")
                           .arg(run.v["q95"]).arg(run.v["stopped95"]).arg(run.v["qmax"]).arg(cap)
                           .arg(run.v.count("cleared") ? run.v["cleared"] : -1, 0, 'f', 1);
            if (run.v["stopped95"] < 4) r.failures << "kolejka nie powstała przed czerwonym";
            if (run.v["qmax"] > cap) r.failures << "więcej aut na odcinku niż fizycznie się mieści";
            if (!run.v.count("cleared")) r.failures << "kolejka nie rozładowała się po zielonym";
        });
}
Result C6(unsigned seed) {
    // Skrzyżowanie z sygnalizacją: ruch z 4 stron + skręty.
    return runDynamic(QString("C6-%1").arg(seed), "C", "Skrzyżowanie: 4 wloty, skręty, 60 s", seed, 60 * 60,
        [](Run& run) {
            run.e.setRoundabouts({});
            const Direction dirs[4] = { Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T };
            int k = 0;
            for (int wave = 0; wave < 4; ++wave)
                for (Direction d : dirs) { Car* c = place(run.e, d, 3, approachAlong(d, 3, 3, 120 + wave * 65)); c->setFixedIntent((k++) % 3); }
        },
        [](Run& run) {
            // Auto „przejechało", gdy było w strefie (3,3), a teraz jest > 60 px od niej.
            const QRectF z = kIZone(3, 3);
            for (auto& o : run.e.objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                const QString key = QString("in%1").arg(c->id());
                if (z.contains(c->center())) run.v[key] = 1;
                else if (run.v.count(key) && !z.adjusted(-60, -60, 60, 60).contains(c->center())) run.v[QString("x%1").arg(c->id())] = 1;
            }
        },
        [](Run& run, Result& r) {
            int crossed = 0, entered = 0;
            for (auto& [k, v] : run.v) { if (k.startsWith('x')) crossed++; if (k.startsWith("in")) entered++; }
            const int jam = run.mon.carsStoppedLongerThan(30.0);
            r.notes << QString("wjechało do strefy (3,3) %1, przejechało %2, stojących > 30 s: %3").arg(entered).arg(crossed).arg(jam);
            if (entered < 12) r.failures << QString("do skrzyżowania dotarło tylko %1 aut").arg(entered);
            if (crossed < entered) r.failures << QString("%1 aut nie opuściło skrzyżowania").arg(entered - crossed);
            if (jam > 0) r.failures << QString("%1 aut zakleszczonych (> 30 s)").arg(jam);
        });
}
Result C7C8() {
    // C7: auto na rondzie zatrzymuje się (awaria 6 s) — dojeżdżające czekają.
    // C8: zjazd z ronda do kolejki (stojące auto 70 px za zjazdem).
    return runDynamic("C7/C8", "C", "Dojazd do zablokowanego ronda + zjazd do kolejki", 5, 60 * 50,
        [](Run& run) {
            run.e.setRoundabouts({ { 4, 4 } });
            Car* q = place(run.e, Direction::T2B, 4, ringCtr(4, 4).y() + kRingR + 70); q->setHeld(true); run.v["q"] = q->id();
            for (int i = 0; i < 3; ++i) { Car* c = place(run.e, Direction::L2R, 4, approachAlong(Direction::L2R, 4, 4, 180 + i * 60)); c->setPending(36, 1); }
            for (int i = 0; i < 2; ++i) { Car* c = place(run.e, Direction::B2T, 4, approachAlong(Direction::B2T, 4, 4, 200 + i * 60)); c->setPending(36, 0); }
        },
        [](Run& run) {
            if (run.t > 15) if (Car* q = carById(run.e, (int)run.v["q"])) q->setHeld(false);
            if (run.e.objects().empty()) run.v["__stop"] = 1;
        },
        [](Run& run, Result& r) {
            int through = 0; for (auto& [id, c] : run.rb.cars) if (c.exits > 0) through++;
            r.notes << QString("przejechało %1/5").arg(through);
            if (through < 5) r.failures << QString("przejechało tylko %1/5").arg(through);
        }, kDt, true);
}
Result C9(unsigned seed, int cars) {
    return runDynamic(QString("C9-%1").arg(seed), "C", QString("Duże obciążenie: %1 aut, 60 s").arg(cars), seed, 60 * 60,
        [=](Run& run) { run.e.spawnCarsOnMap(cars); run.e.setTargetCarCount(cars); }, nullptr, nullptr);
}

// ── D: korki ─────────────────────────────────────────────────────────
Result D4D6D7D8() {
    Result r; r.id = "D4/D6-8"; r.group = "D"; r.name = "Koszty: wzrost, stabilność, puste i gęste drogi"; r.dynamic = false;
    auto cps = buildCheckpoints();
    // D7: pusta sieć — wszystkie koszty skończone, ≥ czas geometryczny.
    {
        SimulationEngine e; TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta);
        int bad = 0;
        for (auto& ed : pf.exportGraph(cps)) { if (!std::isfinite(ed.cost) || ed.cost <= 0) bad++; }
        r.notes << QString("D7 pusta sieć: %1 krawędzi niepoprawnych (NaN/inf/≤0)").arg(bad);
        if (bad) r.failures << "D7: niepoprawne koszty na pustej sieci";
    }
    // D4: zator podnosi koszt odcinka; ruch płynny mniej niż stojący.
    {
        SimulationEngine moving, stopped;
        // 4 auta rozpędzane wcześniej (zielone na (2,0) i (3,0)), mierzone, gdy WSZYSTKIE
        // są na odcinku (3,0)→(4,0) i jadą ≥ 100 px/s — rzeczywiście płynny ruch.
        moving.setRoundabouts({});
        for (int i = 0; i < 2; ++i) place(moving, Direction::L2R, 0, kVLeft[2] - 260 - i * 100)->setFixedIntent(0);
        bool ready = false;
        for (int k = 0; k < 900 && !ready; ++k) {
            for (int g = 1; g <= 4; ++g) moving.setIntersectionPhase(g, 0, 0, 0.0);
            moving.update(kDt);
            TrafficAnalyzer t0(&moving); t0.update();
            int fast = 0; for (auto& o : moving.objects()) if (o->speed() >= 90) fast++;
            ready = t0.carsOnSegment(Direction::L2R, 0, 3, 4) == 2 && fast == 2;
        }
        if (!ready) r.failures << "D4: nie udało się ustawić płynnej kolumny na odcinku (3,0)→(4,0)";
        for (int i = 0; i < 2; ++i) { Car* c = place(stopped, Direction::L2R, 0, kVLeft[4] - 30 - i * 48); c->setHeld(true); }
        for (int k = 0; k < 40; ++k) stopped.update(kDt);
        SimulationEngine empty;
        auto edgeCost = [&](SimulationEngine& e) {
            TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta);
            int s = pf.startStateOfIntersection(3, 0, Direction::L2R);
            for (auto& ed : pf.exportGraph(cps))
                if (ed.fromState == s && pf.nodeOfState(ed.toState) == 4 * kNumRoads + 0) return ed.cost;
            return -1.0;
        };
        double cE = edgeCost(empty), cM = edgeCost(moving), cS = edgeCost(stopped);
        TrafficAnalyzer tm(&moving); tm.update(); int nm = tm.carsOnSegment(Direction::L2R, 0, 3, 4);
        r.notes << QString("D4 koszt odcinka (3,0)→(4,0): pusty %1 s, 2 auta płynnie(%2 na odc.) %3 s, 2 auta stojące %4 s")
                       .arg(cE, 0, 'f', 2).arg(nm).arg(cM, 0, 'f', 2).arg(cS, 0, 'f', 2);
        if (!(cS > cE)) r.failures << "D4: stojący zator nie podnosi kosztu";
        // Margines wynika z kalibracji (D9): auto stojące ≈ 1,06 s, jadące płynnie
        // ≈ 0,82 s (też ustawi się przed nami w kolejce na kolejnych światłach).
        if (!(cS > cM + 0.2)) r.failures << "D4/6.1: zator (auta stoją) wyceniony tak samo jak płynny ruch";
        if (!(cM >= cE)) r.failures << "D4: ruch na odcinku obniżył koszt poniżej pustej drogi";
    }
    // D6: niewielka zmiana ruchu (1 auto jadące płynnie) nie zmienia wyboru trasy.
    {
        SimulationEngine e; warmTraffic(e, 21, 40, 20.0);
        TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta);
        int changed = 0, total = 0; double maxJump = 0;
        std::vector<Route> before; std::vector<double> cb;
        for (int a = 0; a < 32; a += 3) { int b = (a + 13) % 32; double c = 0; before.push_back(pf.findRoute(a, b, cps, &c)); cb.push_back(c); }
        for (int k = 0; k < 6; ++k) e.update(kDt);   // 0,1 s ruchu
        ta.update();
        int idx = 0;
        for (int a = 0; a < 32; a += 3) {
            int b = (a + 13) % 32; double c = 0; Route rr = pf.findRoute(a, b, cps, &c);
            total++; maxJump = std::max(maxJump, std::abs(c - cb[idx]));
            if (rr.size() != before[idx].size()) changed++;
            else for (size_t q = 0; q < rr.size(); ++q) if (rr[q].nodeId != before[idx][q].nodeId) { changed++; break; }
            idx++;
        }
        r.notes << QString("D6 po 0,1 s ruchu: zmienione trasy %1/%2, max zmiana kosztu %3 s").arg(changed).arg(total).arg(maxJump, 0, 'f', 2);
        if (maxJump > 6.0) r.failures << QString("D6: skok kosztu %1 s po 0,1 s ruchu").arg(maxJump, 0, 'f', 1);
    }
    // D8: gęsto — 100 aut: koszty skończone, dodatnie.
    {
        SimulationEngine e; warmTraffic(e, 31, 100, 30.0);
        TrafficAnalyzer ta(&e); ta.update(); DynamicPathfinder pf(&e, &ta);
        int bad = 0; double mx = 0;
        for (auto& ed : pf.exportGraph(cps)) { if (!std::isfinite(ed.cost) || ed.cost <= 0) bad++; mx = std::max(mx, ed.cost); }
        r.notes << QString("D8 100 aut: niepoprawnych krawędzi %1, max koszt krawędzi %2 s").arg(bad).arg(mx, 0, 'f', 1);
        if (bad) r.failures << "D8: niepoprawne koszty przy dużym zagęszczeniu";
    }
    r.pass = r.failures.isEmpty();
    return r;
}

/// D9: kalibracja — przewidywany vs rzeczywisty czas przejazdu odcinka.
Result D9(unsigned seed) {
    // Kalibracja: dla każdego rzeczywistego przejazdu krawędzi grafu
    // (wyjazd z węzła A → wyjazd z węzła B) porównujemy czas zmierzony z
    // kosztem przewidzianym przez wyszukiwarkę w chwili wyjazdu z A
    // (koszt odcinka A→B + koszt manewru w B). Ten sam podział co w grafie A*.
    Result r; r.id = QString("D9-%1").arg(seed); r.group = "D"; r.name = "Predykcja czasu krawędzi vs pomiar (60 aut, 180 s)"; r.seed = seed;
    Run run; run.e.setSeed(seed); run.e.setRoundabouts({ { 2, 2 }, { 5, 5 }, { 3, 6 }, { 6, 2 } });
    run.e.spawnCarsOnMap(60); run.e.setTargetCarCount(60);
    TrafficAnalyzer ta(&run.e); DynamicPathfinder pf(&run.e, &ta);
    const bool dump = qEnvironmentVariableIsSet("EASYRIDER_CAL");
    struct Open { int vi, hi; Direction d; double t0, predSeg; int nStop, nMove; double avgV, len, expW, exactW; int bRing, occ; double oldRed; };
    std::map<int, Open> open;
    std::map<int, std::pair<int,int>> inNode;     // id → węzeł, w którym auto jest (strefa / rondo)
    // Historia snapshotów (co 0,5 s) — do pomiaru, jak szybko dezaktualizuje się
    // informacja o zatłoczeniu odcinka (predykcja dla krawędzi odległych w czasie).
    std::vector<std::pair<double, TrafficAnalyzer>> hist;
    int sampleNo = 0;
    double se = 0, sae = 0, sumA = 0, sumP = 0; int n = 0;
    double seOld = 0, saeOld = 0, sumOld = 0;
    auto nodeAt = [&](const Car* c, int& vi, int& hi) {
        if (c->isOnRing()) { vi = c->ringVI(); hi = c->ringHI(); return true; }
        const QPointF p = c->center();
        for (int v = 0; v < kNumRoads; ++v) for (int h = 0; h < kNumRoads; ++h)
            if (!run.e.intersectionAt(v, h).isRoundabout && kIZone(v, h).contains(p)) { vi = v; hi = h; return true; }
        return false;
    };
    for (long i = 0; i < 60 * 180; ++i) {
        run.step();
        ta.update();
        if (dump && i % 30 == 0) hist.push_back({ run.t, ta });
        for (auto& o : run.e.objects()) {
            const Car* c = dynamic_cast<const Car*>(o.get());
            if (!c) continue;
            int vi = -1, hi = -1;
            const bool inside = nodeAt(c, vi, hi);
            auto it = inNode.find(c->id());
            if (inside) { inNode[c->id()] = { vi, hi }; continue; }
            if (it == inNode.end()) continue;
            if (c->inManeuver()) continue;
            // Wyjazd z węzła it->second
            const int av = it->second.first, ah = it->second.second;
            inNode.erase(it);
            const Direction d = c->dir();
            auto op = open.find(c->id());
            if (op != open.end()) {
                const Open& O = op->second;
                int nv = O.vi + (O.d == Direction::L2R) - (O.d == Direction::R2L);
                int nh = O.hi + (O.d == Direction::T2B) - (O.d == Direction::B2T);
                if (nv == av && nh == ah) {
                    const double actual = run.t - O.t0;
                    const int mv = std::max(0, computeIntent(O.d, d));
                    const double pred = O.predSeg + pf.movementCost(av, ah, O.d, d);
                    if (actual < 120) {
                        double e2 = pred - actual;
                        se += e2 * e2; sae += std::abs(e2); sumA += actual; sumP += pred; n++;
                        // Pierwotny model: len/110 + 0,35·n (+ (n−4)²·0,5 dla n≥5) + kara światła + 2 s za skręt.
                        const int nAll = O.nStop + O.nMove;
                        double oldPred = O.len / 110.0 + 0.35 * nAll + (nAll >= 5 ? (nAll - 4) * (nAll - 4) * 0.5 : 0.0)
                                       + O.oldRed + (mv != 0 ? 2.0 : 0.0);
                        double eo = oldPred - actual; seOld += eo * eo; saeOld += std::abs(eo); sumOld += oldPred;
                        if (dump) {
                            std::printf("CAL,%.1f,%d,%d,%.1f,%d,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f\n", O.len, O.nStop, O.nMove, O.avgV,
                                        O.bRing, mv, O.occ, O.expW, O.exactW, pred, actual, oldPred);
                            // Te same cechy odcinka z snapshotów sprzed Δ sekund.
                            const bool hz = dirIsHoriz(O.d);
                            const int road = hz ? O.hi : O.vi, from = hz ? O.vi : O.hi, to = hz ? av : ah;
                            for (double lag : { 2.0, 5.0, 10.0, 15.0, 20.0, 30.0 }) {
                                const double tq = O.t0 - lag;
                                if (tq < 0) continue;
                                auto best = std::min_element(hist.begin(), hist.end(), [&](const auto& x, const auto& y) {
                                    return std::abs(x.first - tq) < std::abs(y.first - tq); });
                                if (best == hist.end() || std::abs(best->first - tq) > 0.6) continue;
                                const TrafficAnalyzer& h = best->second;
                                std::printf("LAG,%d,%.0f,%d,%d,%.1f\n", sampleNo, lag, h.stoppedOnSegment(O.d, road, from, to),
                                            h.carsOnSegment(O.d, road, from, to), h.avgSpeedOnSegment(O.d, road, from, to));
                            }
                            sampleNo++;
                        }
                    }
                }
            }
            // Otwórz kolejną krawędź: z (av,ah) w kierunku d
            int nv = av + (d == Direction::L2R) - (d == Direction::R2L);
            int nh = ah + (d == Direction::T2B) - (d == Direction::B2T);
            if (nv < 0 || nv >= kNumRoads || nh < 0 || nh >= kNumRoads) { open.erase(c->id()); continue; }
            const bool horiz = dirIsHoriz(d);
            const int road = horiz ? ah : av, from = horiz ? av : ah, to = horiz ? nv : nh;
            Open O{ av, ah, d, run.t, pf.segmentCost(d, road, from, to, nv, nh, 0.0),
                    ta.stoppedOnSegment(d, road, from, to), ta.carsOnSegment(d, road, from, to) - ta.stoppedOnSegment(d, road, from, to),
                    ta.avgSpeedOnSegment(d, road, from, to), segmentLength(d, from, to),
                    SimulationEngine::signalExpectedWait(horiz),
                    SimulationEngine::signalWaitAt(run.e.intersectionAt(nv, nh), horiz, segmentLength(d, from, to) / kMaxSpeed),
                    run.e.intersectionAt(nv, nh).isRoundabout ? 1 : 0, ta.ringOccupancy(nv, nh), 0.0 };
            {   // Pierwotny model (do porównania): kara wg BIEŻĄCEGO światła przy B.
                const auto& st = run.e.intersectionAt(nv, nh);
                const LightState lt = horiz ? st.hLight : st.vLight;
                O.oldRed = st.isRoundabout ? 1.0 : lt == LightState::Red ? 4.0 : lt == LightState::Yellow ? 1.5 : 0.0;
            }
            open[c->id()] = O;
        }
    }
    r.st = run.mon.stats(); r.ticks = run.tick; r.simTime = run.t; r.cars = r.st.maxCars;
    cleanStats(r.st, r.failures);
    const double mae = n ? sae / n : 0, rmse = n ? std::sqrt(se / n) : 0, bias = n ? (sumP - sumA) / n : 0;
    r.notes << QString("próbek %1, śr. rzeczywisty %2 s, śr. przewidywany %3 s, MAE %4 s, RMSE %5 s, bias %6 s")
                   .arg(n).arg(n ? sumA / n : 0, 0, 'f', 2).arg(n ? sumP / n : 0, 0, 'f', 2)
                   .arg(mae, 0, 'f', 2).arg(rmse, 0, 'f', 2).arg(bias, 0, 'f', 2);
    if (n) r.notes << QString("model pierwotny na tych samych próbkach: MAE %1 s, RMSE %2 s, bias %3 s")
                          .arg(saeOld / n, 0, 'f', 2).arg(std::sqrt(seOld / n), 0, 'f', 2).arg((sumOld - sumA) / n, 0, 'f', 2);
    if (n < 200) r.failures << "za mało próbek kalibracji";
    // Kryterium: predykcja nieobciążona (|bias| < 1 s) i MAE < 2,5 s na krawędź.
    if (std::abs(bias) > 1.0) r.failures << QString("predykcja obciążona: bias %1 s").arg(bias, 0, 'f', 2);
    if (mae > 2.5) r.failures << QString("duży błąd predykcji: MAE %1 s").arg(mae, 0, 'f', 2);
    r.pass = r.failures.isEmpty();
    return r;
}

/// D5s: czy uwzględnianie korków skraca RZECZYWISTY czas przejazdu?
/// Dla wielu ziaren i par start–cel w ustalonym ruchu (60 aut, ronda, 15 s
/// rozgrzewki) porównujemy trasę A* (koszty z korkami) z trasą najkrótszą
/// geometrycznie, prowadząc SmartCar każdą z nich w IDENTYCZNEJ scenie.
Result D5stat() {
    Result r; r.id = "D5s"; r.group = "D"; r.name = "Nawigacja z korkami vs najkrótsza: rzeczywisty czas (12 par)";
    r.dynamic = true;
    auto cps = buildCheckpoints();
    const int pairs[12][2] = { {0,13},{3,8},{16,27},{19,24},{1,30},{10,22},{26,5},{29,12},{6,18},{14,1},{21,31},{24,9} };
    double sumDiff = 0; int differ = 0, wins = 0, losses = 0;
    QStringList rows;
    for (int k = 0; k < 12; ++k) {
        const unsigned seed = 500 + k;
        auto scene = [&](SimulationEngine& e) {
            e.setSeed(seed); e.setRoundabouts({ { 2, 2 }, { 5, 5 }, { 3, 6 }, { 6, 2 } });
            e.spawnCarsOnMap(60); e.setTargetCarCount(60);
            for (int i = 0; i < 60 * 15; ++i) e.update(kDt);
        };
        const int a = pairs[k][0], b = pairs[k][1];
        // Jazda z nawigacją (RouteManager, z przeliczaniem) — polityka z korkami
        // albo geometryczna (koszt odcinka = długość / prędkość swobodna).
        auto drive = [&](bool geometric, MonitorStats& st, int& changes, bool& ok, double& predicted) {
            Run run; scene(run.e);
            RouteManager rm(&run.e);
            if (geometric) rm.pathfinder().setCostOverride([](Direction d, int, int from, int to, int, int) {
                return segmentLength(d, from, to) / kMaxSpeed; });
            bool fin = false, fail = false;
            QObject::connect(&rm, &RouteManager::navigationFinished, [&] { fin = true; });
            QObject::connect(&rm, &RouteManager::navigationFailed, [&] { fail = true; });
            rm.startNavigation(a, b);
            predicted = rm.routeCost();
            const QString startKey = [&] { QString q; for (auto& x : rm.currentRoute()) q += QString::number(x.nodeId) + ","; return q; }();
            run.v["dummy"] = 0;
            for (long i = 0; i < 60 * 180 && !fin && !fail; ++i) { rm.tick(); run.step(); }
            st = run.mon.stats(); changes = rm.recalcCount(); ok = fin;
            return std::make_pair(run.t, startKey);
        };
        MonitorStats sa, sb; int chA = 0, chB = 0; bool okA = false, okB = false; double cA = 0, cB = 0;
        auto [tA, kA] = drive(false, sa, chA, okA, cA);
        auto [tB, kB] = drive(true, sb, chB, okB, cB);
        if (kA == kB) { rows << QString("%1→%2: ta sama trasa początkowa").arg(a).arg(b); continue; }
        QStringList f; cleanStats(sa, f, true); cleanStats(sb, f, true);
        for (auto& x : f) r.failures << QString("%1→%2: %3").arg(a).arg(b).arg(x);
        if (!okA || !okB) { r.failures << QString("%1→%2: nie dotarł do celu (z korkami: %3, geometryczna: %4)")
                                             .arg(a).arg(b).arg(okA).arg(okB); continue; }
        differ++; sumDiff += tA - tB;
        if (tA < tB - 1.0) wins++; else if (tA > tB + 1.0) losses++;
        rows << QString("%1→%2: z korkami %3 s (przewid. %4, zmian %5), najkrótsza %6 s (zmian %7)").arg(a).arg(b)
                    .arg(tA, 0, 'f', 1).arg(cA, 0, 'f', 1).arg(chA).arg(tB, 0, 'f', 1).arg(chB);
        r.st.overlaps += sa.overlaps + sb.overlaps; r.st.contacts += sa.contacts + sb.contacts;
        r.st.teleports += sa.teleports + sb.teleports; r.st.headingJumps += sa.headingJumps + sb.headingJumps;
        r.st.engineCorrections += sa.engineCorrections + sb.engineCorrections;
        r.ticks += sa.ticks + sb.ticks; r.simTime += sa.simTime + sb.simTime; r.cars = std::max(r.cars, sa.maxCars);
        r.st.minGap = std::min({ r.st.minGap, sa.minGap, sb.minGap });
    }
    r.notes << rows;
    r.notes << QString("różne trasy: %1/12; z korkami szybciej: %2, wolniej: %3; śr. różnica %4 s (ujemna = zysk)")
                   .arg(differ).arg(wins).arg(losses).arg(differ ? sumDiff / differ : 0, 0, 'f', 2);
    if (differ < 3) r.failures << "za mało przypadków z różnymi trasami";
    if (differ && sumDiff / differ > 0.0) r.failures << "trasy z korkami są średnio WOLNIEJSZE od najkrótszych";
    r.pass = r.failures.isEmpty();
    return r;
}

/// D9n: kalibracja na przejazdach SAMEGO SmartCara (auto nawigowane MUSI
/// wykonać zaplanowany skręt — czeka na lukę; zwykłe auta jadą wtedy prosto,
/// więc ich próbki zaniżają koszt skrętu: błąd selekcji). Wyniki: linie NAV
/// (EASYRIDER_CAL=1) + MAE/bias przewidywania krawędzi.
Result D9nav() {
    Result r; r.id = "D9n"; r.group = "D"; r.name = "Predykcja krawędzi dla auta nawigowanego (30 przejazdów)"; r.dynamic = true;
    const bool dump = qEnvironmentVariableIsSet("EASYRIDER_CAL");
    auto cps = buildCheckpoints();
    std::mt19937 rng(77);
    double sae = 0, sb = 0; int n = 0, arrived = 0, trips = 0; double tripPred = 0, tripAct = 0;
    double saeMv[3] = {0,0,0}, sbMv[3] = {0,0,0}; int nMv[3] = {0,0,0};
    for (int k = 0; k < 30; ++k) {
        int a = rng() % 32, b = rng() % 32;
        if (cps[a].edge == cps[b].edge) b = (b + 8) % 32;
        Run run; run.e.setSeed(900 + k); run.e.setRoundabouts({ { 2, 2 }, { 5, 5 }, { 3, 6 }, { 6, 2 } });
        run.e.spawnCarsOnMap(60); run.e.setTargetCarCount(60);
        for (int i = 0; i < 60 * 15; ++i) run.e.update(kDt);
        RouteManager rm(&run.e);
        bool fin = false, fail = false;
        QObject::connect(&rm, &RouteManager::navigationFinished, [&] { fin = true; });
        QObject::connect(&rm, &RouteManager::navigationFailed, [&] { fail = true; });
        if (!rm.startNavigation(a, b)) continue;
        trips++;
        const double predTrip = rm.routeCost();
        int edgesHere = 0; double edgeSum = 0;
        TrafficAnalyzer ta(&run.e); DynamicPathfinder pf(&run.e, &ta);
        int inVi = -1, inHi = -1; bool open = false;
        int ov = 0, oh = 0; Direction od = Direction::L2R; double t0 = 0, predSeg = 0; int nS = 0, nA = 0; double av = 0, len = 0, exW = 0;
        for (long i = 0; i < 60 * 180 && !fin && !fail; ++i) {
            rm.tick(); run.step();
            const Car* c = run.e.smartCar();
            if (!c) continue;
            int vi = -1, hi = -1; bool inside = false;
            if (c->isOnRing()) { vi = c->ringVI(); hi = c->ringHI(); inside = true; }
            else for (int v = 0; v < kNumRoads && !inside; ++v) for (int h = 0; h < kNumRoads; ++h)
                if (!run.e.intersectionAt(v, h).isRoundabout && kIZone(v, h).contains(c->center())) { vi = v; hi = h; inside = true; break; }
            if (inside || c->inManeuver()) { if (inside) { inVi = vi; inHi = hi; } continue; }
            if (inVi < 0) continue;
            const int av2 = inVi, ah2 = inHi; inVi = -1;
            const Direction d = c->dir();
            if (open) {
                int nv = ov + (od == Direction::L2R) - (od == Direction::R2L), nh = oh + (od == Direction::T2B) - (od == Direction::B2T);
                if (nv == av2 && nh == ah2) {
                    const double actual = run.t - t0;
                    const int mv = std::max(0, computeIntent(od, d));
                    const double pred = predSeg + pf.movementCost(av2, ah2, od, d);
                    sae += std::abs(pred - actual); sb += pred - actual; n++; edgesHere++; edgeSum += actual;
                    saeMv[mv] += std::abs(pred - actual); sbMv[mv] += pred - actual; nMv[mv]++;
                    if (dump) std::printf("NAV,%.1f,%d,%d,%.1f,%d,%d,%d,%.2f,%.2f,%.3f,%.3f\n", len, nS, nA - nS, av,
                                          run.e.intersectionAt(av2, ah2).isRoundabout ? 1 : 0, mv, 0, exW, exW, pred, actual);
                }
            }
            ta.update();
            int nv = av2 + (d == Direction::L2R) - (d == Direction::R2L), nh = ah2 + (d == Direction::T2B) - (d == Direction::B2T);
            open = nv >= 0 && nv < kNumRoads && nh >= 0 && nh < kNumRoads;
            if (open) {
                const bool hz = dirIsHoriz(d);
                const int road = hz ? ah2 : av2, from = hz ? av2 : ah2, to = hz ? nv : nh;
                ov = av2; oh = ah2; od = d; t0 = run.t;
                predSeg = pf.segmentCost(d, road, from, to, nv, nh, 0.0);
                nS = ta.stoppedOnSegment(d, road, from, to); nA = ta.carsOnSegment(d, road, from, to);
                av = ta.avgSpeedOnSegment(d, road, from, to); len = segmentLength(d, from, to);
                exW = SimulationEngine::signalWaitAt(run.e.intersectionAt(nv, nh), hz, len / kMaxSpeed);
            }
        }
        if (fin) { arrived++; tripPred += predTrip; tripAct += run.t; }
        r.notes << QString("  trip %1→%2: przewid. %3 s, rzecz. %4 s, %5, zmian %6, krawędzi zmierzonych %7 (%8 s)")
                       .arg(a).arg(b).arg(predTrip, 0, 'f', 1).arg(run.t, 0, 'f', 1)
                       .arg(fin ? "OK" : fail ? "ZGUBIŁ" : "TIMEOUT").arg(rm.recalcCount()).arg(edgesHere).arg(edgeSum, 0, 'f', 1);
        if (!fin) {
            const Car* c = run.e.smartCar();
            if (c) r.notes << QString("     stan: (%1,%2) dir=%3 road=%4 v=%5 ring=%6 man=%7 blockedBy=%8")
                                  .arg(c->center().x(), 0, 'f', 0).arg(c->center().y(), 0, 'f', 0).arg(dname(c->dir())).arg(c->road())
                                  .arg(c->speed(), 0, 'f', 0).arg(c->isOnRing()).arg(c->inManeuver()).arg(c->lastCapBy());
        }
        MonitorStats st = run.mon.stats();
        r.st.overlaps += st.overlaps; r.st.teleports += st.teleports; r.st.headingJumps += st.headingJumps;
        r.st.engineCorrections += st.engineCorrections; r.ticks += st.ticks; r.simTime += st.simTime;
        r.cars = std::max(r.cars, st.maxCars);
    }
    const char* mvn[3] = { "prosto", "prawo", "lewo" };
    r.notes << QString("przejazdów %1, dotarło %2; próbek krawędzi %3, MAE %4 s, bias %5 s")
                   .arg(trips).arg(arrived).arg(n).arg(n ? sae / n : 0, 0, 'f', 2).arg(n ? sb / n : 0, 0, 'f', 2);
    r.notes << QString("ETA całej trasy (przy starcie): suma przewidywana %1 s, rzeczywista %2 s (%3%)")
                   .arg(tripPred, 0, 'f', 0).arg(tripAct, 0, 'f', 0).arg(tripAct > 0 ? 100.0 * (tripPred - tripAct) / tripAct : 0, 0, 'f', 1);
    for (int m = 0; m < 3; ++m)
        r.notes << QString("  manewr %1 w węźle docelowym: n=%2, MAE %3 s, bias %4 s").arg(mvn[m]).arg(nMv[m])
                       .arg(nMv[m] ? saeMv[m] / nMv[m] : 0, 0, 'f', 2).arg(nMv[m] ? sbMv[m] / nMv[m] : 0, 0, 'f', 2);
    if (arrived < trips) r.failures << QString("nie dotarło %1 z %2 przejazdów").arg(trips - arrived).arg(trips);
    for (int m = 0; m < 3; ++m)
        if (nMv[m] >= 10 && std::abs(sbMv[m] / nMv[m]) > 2.0)
            r.failures << QString("predykcja obciążona dla manewru %1: bias %2 s").arg(mvn[m]).arg(sbMv[m] / nMv[m], 0, 'f', 2);
    r.pass = r.failures.isEmpty();
    return r;
}

/// V: walidacja na ziarnach NIEUŻYWANYCH w kalibracji. Pętla jak w
/// MainWindow::tick: rm.tick(), potem engine.update(0,016 × tempo).
/// Mierzy: dotarcie, zgodność wyjazdu z pierwszego węzła z planem, ETA vs czas
/// rzeczywisty, inwarianty (SimMonitor) — przy ×1 i ×4.
Result Validate(const QString& id, double mult, int trips, unsigned seedBase) {
    Result r; r.id = id; r.group = "V"; r.dynamic = true; r.seed = seedBase;
    r.name = QString("Walidacja tras: %1 przejazdów ×%2 (nowe ziarna)").arg(trips).arg(mult, 0, 'f', 0);
    auto cps = buildCheckpoints();
    std::mt19937 rng(seedBase);
    int arrived = 0, failed = 0, timeouts = 0, firstOk = 0, firstN = 0, rings = 0;
    double sae = 0, sb = 0, sumP = 0, sumA = 0; int n = 0; double worst = 0; QString worstTrip;
    const std::vector<std::vector<std::pair<int,int>>> layouts = {
        { {1,2},{2,5},{4,4},{6,3},{5,6} }, { {2,2},{5,5},{3,6},{6,2} }, { {1,1},{3,4},{6,6},{4,1},{2,6} }, {} };
    for (int k = 0; k < trips; ++k) {
        int a = rng() % 32, b = rng() % 32;
        while (cps[a].edge == cps[b].edge) b = rng() % 32;
        const int cars = 30 + int(rng() % 41);           // 30..70 aut
        Run run; run.dt = 0.016 * mult; run.e.setSeed(seedBase + k);
        run.e.setRoundabouts(layouts[k % layouts.size()]);
        run.e.spawnCarsOnMap(cars); run.e.setTargetCarCount(cars);
        for (int i = 0; i < 900; ++i) run.e.update(1.0 / 60.0);
        RouteManager rm(&run.e);
        bool fin = false, fail = false;
        QObject::connect(&rm, &RouteManager::navigationFinished, [&] { fin = true; });
        QObject::connect(&rm, &RouteManager::navigationFailed, [&] { fail = true; });
        if (!rm.startNavigation(a, b)) { r.failures << QString("%1→%2: brak trasy").arg(a).arg(b); continue; }
        const double pred = rm.routeCost();
        const Route r0 = rm.currentRoute();
        int fVi = -1, fHi = -1; Direction planned = Direction::L2R;
        if (r0.size() >= 3 && r0[1].vi >= 0) { fVi = r0[1].vi; fHi = r0[1].hi; planned = r0[2].dir; }
        bool atFirst = false, judged = fVi < 0, wasRing = false;
        long frames = 0;
        while (!fin && !fail && run.t < 300.0) {
            rm.tick(); run.step(); ++frames;
            const Car* sc = run.e.smartCar();
            if (!sc) continue;
            if (sc->isOnRing() && !wasRing) rings++;
            wasRing = sc->isOnRing();
            if (!judged) {
                const bool here = (sc->isOnRing() && sc->ringVI() == fVi && sc->ringHI() == fHi)
                               || (sc->inManeuver() && sc->maneuverVI() == fVi && sc->maneuverHI() == fHi)
                               || kIZone(fVi, fHi).contains(sc->center());
                if (here) {
                    atFirst = true;
                    const Route& cr = rm.currentRoute();
                    for (size_t q = 0; q + 1 < cr.size(); ++q) if (cr[q].vi == fVi && cr[q].hi == fHi) planned = cr[q + 1].dir;
                } else if (atFirst && !sc->inManeuver() && !sc->isOnRing()) {
                    judged = true; firstN++;
                    if (sc->dir() == planned) firstOk++;
                    else r.failures << QString("%1→%2 (seed %3): wyjazd z pierwszego węzła (%4,%5) niezgodny z planem")
                                           .arg(a).arg(b).arg(seedBase + k).arg(fVi).arg(fHi);
                }
            }
        }
        if (fin) {
            arrived++;
            const double e = pred - run.t; sae += std::abs(e); sb += e; sumP += pred; sumA += run.t; n++;
            if (std::abs(e) > std::abs(worst)) { worst = e; worstTrip = QString("%1→%2 seed %3: przewid. %4 s, rzecz. %5 s").arg(a).arg(b).arg(seedBase + k).arg(pred, 0, 'f', 1).arg(run.t, 0, 'f', 1); }
        } else if (fail) { failed++; r.failures << QString("%1→%2 (seed %3): auto zgubiło trasę").arg(a).arg(b).arg(seedBase + k); }
        else { timeouts++; r.failures << QString("%1→%2 (seed %3): nie dotarło w 300 s").arg(a).arg(b).arg(seedBase + k); }
        const MonitorStats st = run.mon.stats();
        QStringList f; cleanStats(st, f);
        for (auto& x : f) r.failures << QString("%1→%2 (seed %3): %4").arg(a).arg(b).arg(seedBase + k).arg(x);
        r.st.overlaps += st.overlaps + st.sweptHits; r.st.contacts += st.contacts; r.st.teleports += st.teleports + st.reverseJumps;
        r.st.headingJumps += st.headingJumps; r.st.engineCorrections += st.engineCorrections;
        r.st.minGap = std::min(r.st.minGap, st.minGap); r.ticks += st.ticks; r.simTime += st.simTime; r.cars = std::max(r.cars, st.maxCars);
    }
    r.notes << QString("dotarło %1/%2, zgubiło %3, przekroczony czas %4; wyjazd z 1. węzła zgodny z planem %5/%6; wjazdów SmartCara na ronda %7")
                   .arg(arrived).arg(trips).arg(failed).arg(timeouts).arg(firstOk).arg(firstN).arg(rings);
    r.notes << QString("ETA trasy: MAE %1 s, bias %2 s, Σ przewid. %3 s vs Σ rzecz. %4 s (%5%)")
                   .arg(n ? sae / n : 0, 0, 'f', 1).arg(n ? sb / n : 0, 0, 'f', 1).arg(sumP, 0, 'f', 0).arg(sumA, 0, 'f', 0)
                   .arg(sumA > 0 ? 100.0 * (sumP - sumA) / sumA : 0, 0, 'f', 1);
    if (!worstTrip.isEmpty()) r.notes << "największy błąd ETA: " + worstTrip;
    r.pass = r.failures.isEmpty();
    return r;
}

// ── E: stabilność ────────────────────────────────────────────────────
Result Esoak(const QString& id, const QString& name, unsigned seed, int cars, double seconds, double dt) {
    return runDynamic(id, "E", name, seed, long(seconds / dt),
        [=](Run& run) {
            run.e.setRoundabouts({ { 1, 2 }, { 2, 5 }, { 4, 4 }, { 6, 3 }, { 5, 6 } });
            run.e.spawnCarsOnMap(cars); run.e.setTargetCarCount(cars);
        }, nullptr,
        [=](Run& run, Result& r) {
            int longStop = run.mon.carsStoppedLongerThan(60.0);
            r.notes << QString("aut stojących > 60 s na końcu: %1").arg(longStop);
            if (longStop > 0) r.failures << QString("%1 aut trwale zablokowanych (>60 s)").arg(longStop);
            // Przepustowość: co najmniej jeden przejazd na auto na 2 minuty (wykrywa
            // zakleszczenie sieci, nie karze za zwykłe czekanie na światłach).
            const int need = int(cars * seconds / 120.0);
            if (r.st.exited < need) r.failures << QString("niska przepustowość: %1 aut dojechało (min %2)").arg(r.st.exited).arg(need);
        }, dt);
}

QByteArray traceHash(unsigned seed) {
    SimulationEngine e; e.setSeed(seed); e.spawnCarsOnMap(40); e.setTargetCarCount(40);
    QCryptographicHash h(QCryptographicHash::Sha1);
    for (int i = 0; i < 60 * 30; ++i) {
        e.update(kDt);
        for (auto& o : e.objects()) {
            QPointF p = o->position(); double v = o->speed();
            h.addData(QByteArrayView(reinterpret_cast<const char*>(&p), sizeof(p)));
            h.addData(QByteArrayView(reinterpret_cast<const char*>(&v), sizeof(v)));
        }
    }
    return h.result().toHex().left(12);
}
Result E5E6() {
    Result r; r.id = "E5/E6"; r.group = "E"; r.name = "Determinizm (ten sam seed) + pauza/wznowienie"; r.dynamic = false; r.seed = 77;
    QByteArray a = traceHash(77), b = traceHash(77), c = traceHash(78);
    r.notes << QString("hash seed77 #1=%1 #2=%2, seed78=%3").arg(QString(a)).arg(QString(b)).arg(QString(c));
    if (a != b) r.failures << "E6: ten sam seed daje różne przebiegi";
    if (a == c) r.notes << "uwaga: różne seedy dały identyczny przebieg";
    // E5: pauza = update(0) — stan nie może się zmienić.
    SimulationEngine e; e.setSeed(5); e.spawnCarsOnMap(30);
    for (int i = 0; i < 300; ++i) e.update(kDt);
    std::vector<QPointF> before; for (auto& o : e.objects()) before.push_back(o->position());
    for (int i = 0; i < 100; ++i) e.update(0.0);
    int moved = 0; size_t k = 0; for (auto& o : e.objects()) { if (k < before.size() && o->position() != before[k]) moved++; k++; }
    r.notes << QString("E5 pauza: aut przesuniętych w czasie pauzy %1").arg(moved);
    if (moved) r.failures << "E5: auta poruszają się podczas pauzy";
    r.pass = r.failures.isEmpty();
    return r;
}

} // namespace

// ════════════════════════════════════════════════════════════════════
//  Lista scenariuszy
// ════════════════════════════════════════════════════════════════════
std::vector<Scenario> allScenarios() {
    std::vector<Scenario> s;
    auto add = [&](const QString& id, const QString& g, const QString& n, std::function<Result()> f) { s.push_back({ id, g, n, f }); };
    add("A1", "A", "A* vs Dijkstra — pusta sieć", A1);
    for (unsigned seed : { 101u, 202u, 303u }) add(QString("A2-%1").arg(seed), "A", "A* vs Dijkstra — korki", [=] { return A2(seed); });
    for (unsigned seed : { 1u, 2u, 3u }) add(QString("A4-%1").arg(seed), "A", "A* vs Dijkstra — losowe wagi", [=] { return A4(seed); });
    add("A3/D5", "A", "Pluton na krótszej trasie", [] { return A3_D5(false); });
    add("D5s", "D", "Trasy z korkami vs najkrótsze — pomiar", D5stat);
    add("D9n", "D", "Kalibracja: auto nawigowane", D9nav);
    add("A3i", "A", "Awaria na krótszej trasie", [] { return A3_D5(true); });
    add("A5/A6", "A", "Rondo na trasie / omijanie", A5A6);
    add("A7/A8", "A", "Zator na trasie w trakcie jazdy", [] { return rerouteScenario("A7/A8", "Zmiana kosztów w trakcie jazdy + rerouting bez skoków", 8, true, 0); });
    add("A9", "A", "Brak oscylacji tras (40 aut w tle)", [] { return rerouteScenario("A9", "Brak oscylacji tras przy małych zmianach (40 aut)", 9, false, 40); });
    add("A10", "A", "Brak trasy", A10);
    add("B1", "B", "Rondo na wprost (4 wloty)", [] { return Bsingle("B1", "Jeden pojazd — rondo na wprost (4 wloty)", 0); });
    add("B2", "B", "Rondo — pierwszy zjazd", [] { return Bsingle("B2", "Wjazd i pierwszy zjazd (w prawo)", 1); });
    add("B3", "B", "Rondo — dalszy zjazd", [] { return Bsingle("B3", "Wjazd i dalszy zjazd (w lewo)", 2); });
    add("B4", "B", "Kilka aut na rondzie", B4);
    for (unsigned seed : { 6u }) add(QString("B5/B6-%1").arg(seed), "B", "Ustępowanie i jednoczesny wjazd", [=] { return B5B6(seed); });
    add("B7", "B", "Zablokowany wylot", B7);
    for (unsigned seed : { 12u, 13u }) add(QString("B10-%1").arg(seed), "B", "Duże natężenie rond", [=] { return B10(seed); });
    add("C1", "C", "Wolniejszy lider", C1);
    add("C2", "C", "Hamowanie lidera", [] { return C2C3("C2", "Hamowanie lidera (kolumna 4 aut)", 0, 3.0); });
    add("C3", "C", "Nagłe zatrzymanie lidera", [] { return C2C3("C3", "Nagłe zatrzymanie lidera przy pełnej prędkości", 120, 2.2); });
    add("C4/C5", "C", "Kolejka i ruszanie", C4C5);
    for (unsigned seed : { 14u, 15u }) add(QString("C6-%1").arg(seed), "C", "Skrzyżowanie 4 wloty", [=] { return C6(seed); });
    add("C7/C8", "C", "Zablokowane rondo / zjazd do kolejki", C7C8);
    for (unsigned seed : { 16u, 17u, 18u }) add(QString("C9-%1").arg(seed), "C", "100 aut", [=] { return C9(seed, 100); });
    add("D4/D6-8", "D", "Koszty", D4D6D7D8);
    for (unsigned seed : { 19u, 20u }) add(QString("D9-%1").arg(seed), "D", "Kalibracja predykcji", [=] { return D9(seed); });
    add("E1", "E", "Długi przebieg, 8 aut", [] { return Esoak("E1", "Długi przebieg: 8 aut, 600 s", 41, 8, 600, kDt); });
    for (unsigned seed : { 42u, 43u }) add(QString("E2-%1").arg(seed), "E", "30 aut, 300 s", [=] { return Esoak(QString("E2-%1").arg(seed), "Długi przebieg: 30 aut, 300 s", seed, 30, 300, kDt); });
    add("E3", "E", "70 aut, 300 s", [] { return Esoak("E3", "Zwiększone natężenie: 70 aut, 300 s", 44, 70, 300, kDt); });
    add("E4", "E", "Przyspieszona symulacja ×4", [] { return Esoak("E4", "Przyspieszona symulacja ×4 (dt=0,064 s), 40 aut", 45, 40, 240, 0.064); });
    add("E5/E6", "E", "Determinizm + pauza", E5E6);
    add("V1", "V", "Walidacja tras ×1", [] { return Validate("V1", 1.0, 30, 7100); });
    add("V4", "V", "Walidacja tras ×4", [] { return Validate("V4", 4.0, 20, 7300); });
    // S: stress bezpieczeństwa na wielu ziarnach (mapa z rondami i sygnalizacją).
    for (unsigned seed : { 61u, 62u, 63u, 64u, 65u, 66u, 67u, 68u })
        add(QString("S-%1").arg(seed), "S", "Stress 100 aut", [=] {
            return Esoak(QString("S-%1").arg(seed), "Stress: 100 aut, 6 rond, 120 s", seed, 100, 120, kDt); });
    for (unsigned seed : { 71u, 72u })
        add(QString("S4x-%1").arg(seed), "S", "Stress ×4", [=] {
            return Esoak(QString("S4x-%1").arg(seed), "Stress ×4 (dt=0,064 s): 100 aut, 120 s", seed, 100, 120, 0.064); });
    return s;
}

// ════════════════════════════════════════════════════════════════════
//  Runner konsolowy
// ════════════════════════════════════════════════════════════════════
int runCli(const QString& filter) {
    std::vector<Result> res;
    for (auto& sc : allScenarios()) {
        if (!filter.isEmpty() && !sc.id.startsWith(filter) && !sc.group.startsWith(filter)) continue;
        std::fprintf(stderr, "… %s\n", sc.id.toUtf8().constData());
        res.push_back(sc.run());
    }
    std::printf("\n%-11s %-56s %5s %7s %7s %4s %6s %6s %6s %6s %6s %7s %s\n",
                "ID", "Scenariusz", "seed", "ticki", "t[s]", "aut", "kolizj", "styki", "telep", "kurs", "korekt", "minGap", "wynik");
    int fails = 0;
    for (auto& r : res) {
        QString nm = r.name.left(56);
        if (r.dynamic)
            std::printf("%-11s %-56s %5u %7ld %7.1f %4d %6ld %6ld %6ld %6ld %6ld %7.2f %s\n",
                        r.id.toUtf8().constData(), nm.toUtf8().constData(), r.seed, r.ticks, r.simTime, r.cars,
                        r.st.overlaps + r.st.sweptHits, r.st.contacts, r.st.teleports + r.st.reverseJumps,
                        r.st.headingJumps, r.st.engineCorrections, r.st.minGap > 1e8 ? -1.0 : r.st.minGap,
                        r.pass ? "PASS" : "FAIL");
        else
            std::printf("%-11s %-56s %5u %7s %7s %4s %6s %6s %6s %6s %6s %7s %s\n",
                        r.id.toUtf8().constData(), nm.toUtf8().constData(), r.seed, "-", "-", "-", "-", "-", "-", "-", "-", "-",
                        r.pass ? "PASS" : "FAIL");
        if (!r.pass) fails++;
    }
    std::printf("\n=== AUDYT: %d / %d PASS ===\n\n", int(res.size()) - fails, int(res.size()));
    for (auto& r : res) {
        std::printf("[%s] %s — %s  (%.0f ms)\n", r.pass ? "PASS" : "FAIL", r.id.toUtf8().constData(), r.name.toUtf8().constData(), r.runtimeMs);
        for (auto& n : r.notes) std::printf("    • %s\n", n.toUtf8().constData());
        for (auto& f : r.failures) std::printf("    ✗ %s\n", f.toUtf8().constData());
        if (!r.pass) for (auto& v : r.violations.mid(0, 6)) std::printf("      ↳ %s\n", v.toUtf8().constData());
    }
    std::fflush(stdout);
    return fails == 0 ? 0 : 1;
}

// ════════════════════════════════════════════════════════════════════
//  Adapter do infrastruktury testów (kategoria GUI/headless)
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> auditTests() {
    std::vector<TestCase> out;
    for (auto& sc : allScenarios()) {
        TestCase tc;
        tc.id = "audit." + sc.id; tc.name = sc.id + " " + sc.name; tc.category = "Audyt mechaniki";
        tc.description = QString::fromUtf8("Deterministyczny scenariusz audytu (SimMonitor: obrócone obrysy, skoki, legalność).");
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1; tc.maxFramesPerAttempt = 2; tc.isStatic = true;
        tc.declareMilestones = [](int) { return std::vector<Milestone>{ MS("done", QString::fromUtf8("Scenariusz wykonany i zweryfikowany")) }; };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        auto fn = sc.run;
        tc.updateAttempt = [fn](AttemptCtx& ctx) -> bool {
            Result r = fn();
            for (auto& n : r.notes) ctx.result->events << n;
            for (auto& v : r.violations.mid(0, 8)) ctx.result->events << v;
            measure(ctx, QString::fromUtf8("Kolizje/overlapy"), QString::number(r.st.overlaps + r.st.sweptHits));
            measure(ctx, QString::fromUtf8("Teleportacje"), QString::number(r.st.teleports + r.st.reverseJumps));
            measure(ctx, QString::fromUtf8("Skoki kursu"), QString::number(r.st.headingJumps));
            measure(ctx, QString::fromUtf8("Ticki"), QString::number(r.ticks));
            if (r.pass) { reach(ctx, "done"); markPass(ctx); }
            else markFail(ctx, r.failures.join("; "));
            return true;
        };
        out.push_back(tc);
    }
    return out;
}

} // namespace Audit
