#include "DynamicPathfinder.h"
#include "core/SimulationEngine.h"
#include "core/Car.h"
#include "core/Roundabout.h"
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <tuple>

namespace {
/// Długość toru ronda (krzywa wjazdu + łuk + krzywa zjazdu) dla manewru in→out [px]
/// oraz długość odcinków jezdni, które ten tor „zastępuje" (od punktu wjazdu do
/// krawędzi strefy i od krawędzi strefy do końca zjazdu) — są już w kosztach odcinków.
void ringPathLength(Direction in, Direction out, double& path, double& overlap) {
    const double wIn = Car::laneOffset(in), wOut = Car::laneOffset(out);
    const double th3 = Car::ringMergeTheta(in, wIn);
    const double thx = Car::ringDivergeTheta(out, wOut);
    const double arc = Rb::mod2pi(th3 - thx) * kRingR;
    // Cięciwy krzywych (lokalnie: start na osi pasa w odległości kRbLaneU od środka).
    auto curve = [](double w, double thLocal) {
        const double u = kRbLaneU(w);
        const double dx = kRingR * std::cos(thLocal) + u, dy = kRingR * std::sin(thLocal) - w;
        return 1.08 * std::hypot(dx, dy);   // krzywa Béziera ≈ 8% dłuższa od cięciwy
    };
    const double thInLocal = M_PI - std::asin(wIn / kRingR) - kRbMergeDelta;
    const double thOutLocal = M_PI - std::asin(wOut / kRingR) - kRbMergeDelta;  // zjazd = lustro wjazdu
    path = curve(wIn, thInLocal) + arc + curve(wOut, thOutLocal);
    overlap = (kRbLaneU(wIn) - kRoadW / 2.0) + (kRbLaneU(wOut) - kRoadW / 2.0);
}
} // namespace

DynamicPathfinder::DynamicPathfinder(const SimulationEngine* engine,
                                      TrafficAnalyzer*        analyzer)
    : m_engine(engine), m_analyzer(analyzer) {
    computeHeuristicScale();
}

// ════════════════════════════════════════════════════════════════════
//  Model czasu przejazdu
// ════════════════════════════════════════════════════════════════════
/// Odcinek A→B: jazda swobodna + kolejka (auta stojące rozładowują się
/// kolejno) + spowolnienie od aut jadących + oczekiwanie przy węźle B
/// (sygnalizacja: wartość oczekiwana dla losowej chwili dojazdu — stabilna,
/// nie zależy od bieżącej fazy; rondo: ustąpienie autom na rondzie).
double DynamicPathfinder::segmentCost(Direction dir, int roadIdx, int from, int to,
                                      int toVi, int toHi, double departIn, double leadTime) const {
    const double len = segmentLength(dir, from, to);
    double t = len / m_p.vFree;
    if (m_analyzer) {
        const int nAll  = m_analyzer->carsOnSegment(dir, roadIdx, from, to);
        const int nStop = m_analyzer->stoppedOnSegment(dir, roadIdx, from, to);
        const int nMove = std::max(0, nAll - nStop);
        t += m_p.queueHeadway * nStop;
        if (nMove > 0) {
            // Średnia prędkość aut JADĄCYCH (stojące mają v≈0).
            const double vMove = m_analyzer->avgSpeedOnSegment(dir, roadIdx, from, to) * nAll / nMove;
            const double slow = std::clamp(1.0 - vMove / m_p.vFree, 0.0, 1.0);
            const double fade = std::exp(-std::max(0.0, leadTime) / m_p.slowDecay);
            t += nMove * (m_p.movingDelay + (m_p.movingSlowDelay - m_p.movingDelay) * slow * fade);
        }
    }
    if (toVi >= 0 && toHi >= 0 && m_engine) {
        const auto& st = m_engine->intersectionAt(toVi, toHi);
        if (st.isRoundabout) {
            if (m_analyzer) t += m_p.ringYield * m_analyzer->ringOccupancy(toVi, toHi);
        } else {
            const bool h = dirIsHoriz(dir);
            const double arrive = departIn + len / m_p.vFree;
            if (departIn >= 0 && arrive <= m_p.phaseHorizon)
                t += m_p.signalScale * SimulationEngine::signalWaitAt(st, h, arrive);
            else
                t += m_p.signalScaleFar * SimulationEngine::signalExpectedWait(h);
        }
    }
    return t;
}

/// Przejazd przez węzeł (vi,hi) z wjazdu `in` na wyjazd `out`.
double DynamicPathfinder::movementCost(int vi, int hi, Direction in, Direction out) const {
    const bool ring = m_engine && m_engine->intersectionAt(vi, hi).isRoundabout;
    if (ring) {
        double path = 0, overlap = 0;
        ringPathLength(in, out, path, overlap);
        return std::max(kRoadW / m_p.vFree, m_p.ringScale * path / kRingSpeed - overlap / m_p.vFree);
    }
    double t = kRoadW / m_p.vFree;
    const int intent = computeIntent(in, out);
    if (intent == 1) t += m_p.turnRight;
    else if (intent == 2) t += m_p.turnLeft;
    return t;
}

double DynamicPathfinder::sanitize(double c) const {
    if (std::isinf(c) && c > 0) return c;          // odcinek zamknięty (obsługiwany)
    if (!std::isfinite(c) || c < 0) { ++m_invalidCosts; return 0.0; }
    return c;
}

double DynamicPathfinder::segCost(Direction dir, int roadIdx, int from, int to,
                                  int toVi, int toHi, double departIn) const {
    if (m_override) return sanitize(m_override(dir, roadIdx, from, to, toVi, toHi));
    // Początek odcinka: krawędź strefy węzła wyjściowego (lub krawędź mapy).
    const int s = segmentIndex(dir, from, to);
    QPointF a;
    if (dirIsHoriz(dir)) {
        const double x = (dir == Direction::L2R) ? (s == 0 ? 0.0 : kVLeft[s - 1] + kRoadW)
                                                 : (s == kNumRoads ? kCanvasW : kVLeft[s]);
        a = { x, kHTop[roadIdx] + kRoadW / 2.0 };
    } else {
        const double y = (dir == Direction::T2B) ? (s == 0 ? 0.0 : kHTop[s - 1] + kRoadW)
                                                 : (s == kNumRoads ? kCanvasH : kHTop[s]);
        a = { kVLeft[roadIdx] + kRoadW / 2.0, y };
    }
    return sanitize(segmentCost(dir, roadIdx, from, to, toVi, toHi, departIn, leadTimeTo(a)));
}

double DynamicPathfinder::leadTimeTo(QPointF p) const {
    if (!m_orgSet) return 0.0;
    // Odległość Manhattan (siatka) / prędkość swobodna, pomniejszona o pół strefy.
    const double d = std::abs(p.x() - m_org.x()) + std::abs(p.y() - m_org.y()) - kRoadW;
    return std::max(0.0, d) / m_p.vFree;
}

void DynamicPathfinder::setOrigin(int startState, const std::vector<Checkpoint>& cps) const {
    if (startState < 0) { m_orgSet = false; return; }
    m_org = nodeWorldPos(stateNode(startState), cps);
    m_orgSet = true;
}

// ── Pozycja węzła w przestrzeni świata ──────────────────────────────
QPointF DynamicPathfinder::nodeWorldPos(int nodeId, const std::vector<Checkpoint>& cps) const {
    if (isIntNode(nodeId)) {
        int vi = nodeVi(nodeId), hi = nodeHi(nodeId);
        return {kVLeft[vi]+kRoadW/2.0, kHTop[hi]+kRoadW/2.0};
    }
    int cpId = nodeId - kN*kN;
    if (cpId >= 0 && cpId < (int)cps.size()) return cps[cpId].worldPos;
    return {};
}

/// k = min po krawędziach (dolne ograniczenie kosztu / odległość euklidesowa).
/// Dolne ograniczenie: długość/vFree + minimalny koszt manewru w węźle
/// wyjściowym (niezależny od ruchu). Ponieważ każdy koszt ≥ dolne ograniczenie,
/// h(n) = k·|n−cel| jest dopuszczalna i spójna (nierówność trójkąta).
void DynamicPathfinder::computeHeuristicScale() {
    const auto cps = buildCheckpoints();
    double k = std::numeric_limits<double>::infinity();
    const Direction dirs[4] = { Direction::L2R, Direction::R2L, Direction::T2B, Direction::B2T };
    for (int vi = 0; vi < kN; ++vi) for (int hi = 0; hi < kN; ++hi) {
        double minMove = std::numeric_limits<double>::infinity();
        for (Direction a : dirs) for (Direction b : dirs)
            if (b != dirOpposite(a)) minMove = std::min(minMove, movementCost(vi, hi, a, b));
        for (Direction d : dirs) {
            const bool h = dirIsHoriz(d);
            const int nvi = vi + (d == Direction::L2R) - (d == Direction::R2L);
            const int nhi = hi + (d == Direction::T2B) - (d == Direction::B2T);
            int toNode;
            if (nvi < 0 || nvi >= kN || nhi < 0 || nhi >= kN) {
                const int cp = (d == Direction::L2R) ? 8 + hi : (d == Direction::R2L) ? hi
                             : (d == Direction::T2B) ? 24 + vi : 16 + vi;
                toNode = cpNode(cp);
            } else toNode = intNode(nvi, nhi);
            const int from = h ? vi : hi;
            const int to = h ? (nvi < 0 || nvi >= kN ? (d == Direction::L2R ? kN : -1) : nvi)
                             : (nhi < 0 || nhi >= kN ? (d == Direction::T2B ? kN : -1) : nhi);
            const double lb = segmentLength(d, from, to) / m_p.vFree + minMove;
            const QPointF a = nodeWorldPos(intNode(vi, hi), cps), b = nodeWorldPos(toNode, cps);
            const double dist = std::hypot(a.x() - b.x(), a.y() - b.y());
            if (dist > 1e-6) k = std::min(k, lb / dist);
        }
    }
    // Krawędzie wjazdowe z checkpointów (bez manewru na starcie).
    for (const auto& cp : cps) {
        const Direction d = cp.entryDir;
        const bool h = dirIsHoriz(d);
        const int first = (d == Direction::L2R || d == Direction::T2B) ? 0 : kN - 1;
        const int vi = h ? first : cp.roadIdx, hi = h ? cp.roadIdx : first;
        const double lb = segmentLength(d, -1, first) / m_p.vFree;
        const QPointF a = cp.worldPos, b = nodeWorldPos(intNode(vi, hi), cps);
        const double dist = std::hypot(a.x() - b.x(), a.y() - b.y());
        if (dist > 1e-6) k = std::min(k, lb / dist);
    }
    m_hPerPx = std::isfinite(k) ? std::max(0.0, k) : 0.0;
}

double DynamicPathfinder::heuristic(int nodeId, int goalNode, const std::vector<Checkpoint>& cps) const {
    const QPointF a = nodeWorldPos(nodeId, cps), b = nodeWorldPos(goalNode, cps);
    return m_hPerPx * std::hypot(a.x() - b.x(), a.y() - b.y());
}

// ── Generator krawędzi grafu ────────────────────────────────────────
std::vector<DynamicPathfinder::Edge>
DynamicPathfinder::edges(int stateId, const std::vector<Checkpoint>& cps, bool isStart) const {
    std::vector<Edge> result;
    int node    = stateNode(stateId);
    int dirCode = stateDirCode(stateId);
    Direction prevDir = (dirCode < 4) ? (Direction)dirCode : Direction::L2R;
    bool hasPrev = (dirCode < 4);

    auto addEdge = [&](int toNode, Direction tDir, double cost) {
        if (std::isinf(cost)) return;   // odcinek zamknięty
        result.push_back({state(toNode, (int)tDir), cost, tDir});
    };

    if (!isIntNode(node)) {
        int cpId = node - kN*kN;
        if (cpId < 0 || cpId >= (int)cps.size() || hasPrev) return result;
        const Checkpoint& cp = cps[cpId];
        Direction d = cp.entryDir;
        int ri = cp.roadIdx;
        if (dirIsHoriz(d)) {
            int vi = (d==Direction::L2R) ? 0 : kN-1;
            addEdge(intNode(vi, ri), d, segCost(d, ri, -1, vi, vi, ri, isStart ? 0.0 : -1.0));
        } else {
            int hi = (d==Direction::T2B) ? 0 : kN-1;
            addEdge(intNode(ri, hi), d, segCost(d, ri, -1, hi, ri, hi, isStart ? 0.0 : -1.0));
        }
        return result;
    }

    int vi = nodeVi(node), hi = nodeHi(node);
    const Direction dirs[4] = {Direction::L2R, Direction::R2L, Direction::T2B, Direction::B2T};
    for (Direction d : dirs) {
        if (hasPrev && d == dirOpposite(prevDir)) continue; // zakaz zawracania
        const double move = hasPrev ? movementCost(vi, hi, prevDir, d) : movementCost(vi, hi, d, d);
        // Pierwsza krawędź wyszukiwania: oczekiwanie przy następnym węźle z faz świateł.
        const double dep = isStart ? move : -1.0;
        if (dirIsHoriz(d)) {
            int nvi = vi + (d==Direction::L2R ? 1 : -1);
            if (nvi < 0 || nvi >= kN) {
                if (d==Direction::L2R) addEdge(cpNode(8 + hi), d, move + segCost(d, hi, vi, kN, -1, -1, dep));
                else                   addEdge(cpNode(hi),     d, move + segCost(d, hi, vi, -1, -1, -1, dep));
            } else {
                addEdge(intNode(nvi, hi), d, move + segCost(d, hi, vi, nvi, nvi, hi, dep));
            }
        } else {
            int nhi = hi + (d==Direction::T2B ? 1 : -1);
            if (nhi < 0 || nhi >= kN) {
                if (d==Direction::T2B) addEdge(cpNode(24 + vi), d, move + segCost(d, vi, hi, kN, -1, -1, dep));
                else                   addEdge(cpNode(16 + vi), d, move + segCost(d, vi, hi, -1, -1, -1, dep));
            } else {
                addEdge(intNode(vi, nhi), d, move + segCost(d, vi, hi, nhi, vi, nhi, dep));
            }
        }
    }
    return result;
}

// ════════════════════════════════════════════════════════════════════
//  A*
// ════════════════════════════════════════════════════════════════════
// Wpis kolejki niesie g; wpis jest przestarzały, gdy g > najlepsze znane g
// stanu (dawniej porównywano g z f = g + h, co przy h > 0 odrzucało KAŻDY
// wpis — błąd ukryty, bo heurystyka zwracała 0 dla celów-checkpointów).
Route DynamicPathfinder::search(int startState, int goalNode, int startCpId, int endCpId,
                                const std::vector<Checkpoint>& cps, double* outCost, int forcedExit) {
    constexpr double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g(kStates, INF);
    std::vector<int>    prev(kStates, -1);
    std::vector<char>   closed(kStates, 0);
    m_lastExpansions = 0;

    using Entry = std::tuple<double, double, int>;   // f, g, stan
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    setOrigin(startState, cps);
    g[startState] = 0.0;
    pq.push({heuristic(stateNode(startState), goalNode, cps), 0.0, startState});

    while (!pq.empty()) {
        auto [f, gc, cur] = pq.top(); pq.pop();
        if (closed[cur] || gc > g[cur] + 1e-12) continue;
        closed[cur] = 1;
        ++m_lastExpansions;
        if (stateNode(cur) == goalNode) {
            if (outCost) *outCost = g[cur];
            return buildRoute(prev, cur, startCpId, endCpId, cps);
        }
        for (const auto& e : edges(cur, cps, cur == startState)) {
            if (cur == startState && forcedExit >= 0 && (int)e.dir != forcedExit) continue;
            const double ng = g[cur] + e.cost;
            if (ng < g[e.toState] - 1e-12) {
                g[e.toState] = ng;
                prev[e.toState] = cur;
                pq.push({ng + heuristic(stateNode(e.toState), goalNode, cps), ng, e.toState});
            }
        }
    }
    return {};
}

Route DynamicPathfinder::findRoute(int startCpId, int endCpId,
                                    const std::vector<Checkpoint>& cps, double* outCost) {
    if (startCpId == endCpId) return {};
    if (startCpId < 0 || endCpId < 0 || startCpId >= (int)cps.size() || endCpId >= (int)cps.size()) return {};
    return search(state(cpNode(startCpId), 4), cpNode(endCpId), startCpId, endCpId, cps, outCost, -1);
}

Route DynamicPathfinder::findRouteFrom(int startVi, int startHi, Direction startDir, int endCpId,
                                        const std::vector<Checkpoint>& cps, double* outCost, int forcedExit) {
    if (endCpId < 0 || endCpId >= (int)cps.size()) return {};
    return search(state(intNode(startVi, startHi), (int)startDir), cpNode(endCpId), -1, endCpId, cps, outCost, forcedExit);
}

// ── Odtworzenie trasy ────────────────────────────────────────────────
Route DynamicPathfinder::buildRoute(const std::vector<int>& prevState,
                                     int goalState, int startCpId, int endCpId,
                                     const std::vector<Checkpoint>& cps) const {
    (void)startCpId; (void)endCpId;
    std::vector<int> states;
    for (int s = goalState; s != -1; s = prevState[s]) states.push_back(s);
    std::reverse(states.begin(), states.end());

    Route route;
    for (int i = 0; i < (int)states.size(); i++) {
        int s = states[i], node = stateNode(s), dc = stateDirCode(s);
        Direction d = (dc < 4) ? (Direction)dc : Direction::L2R;
        RouteStep step;
        step.nodeId = node;
        step.dir    = d;
        if (isIntNode(node)) {
            step.vi = nodeVi(node); step.hi = nodeHi(node); step.cpId = -1;
            step.worldPos = nodeWorldPos(node, cps);
        } else {
            step.vi = step.hi = -1;
            step.cpId = node - kN*kN;
            if (step.cpId >= 0 && step.cpId < (int)cps.size()) {
                step.worldPos = cps[step.cpId].worldPos;
                if (i == 0) step.dir = cps[step.cpId].entryDir;  // start: wjazd
            }
        }
        route.push_back(step);
    }
    return route;
}

// ── Interfejs weryfikacyjny ──────────────────────────────────────────
std::vector<DynamicPathfinder::GraphEdge>
DynamicPathfinder::exportGraph(const std::vector<Checkpoint>& cps, int startState) const {
    std::vector<GraphEdge> out;
    setOrigin(startState, cps);
    for (int s = 0; s < kStates; ++s) {
        const int node = stateNode(s), dc = stateDirCode(s);
        if (!isIntNode(node) && dc != 4) continue;
        for (const auto& e : edges(s, cps, s == startState)) out.push_back({s, e.toState, e.cost});
    }
    return out;
}

double DynamicPathfinder::evaluateRoute(const Route& r, const std::vector<Checkpoint>& cps) const {
    if (r.size() < 2) return -1.0;
    int cur = (r[0].cpId >= 0) ? state(cpNode(r[0].cpId), 4) : state(intNode(r[0].vi, r[0].hi), (int)r[0].dir);
    setOrigin(cur, cps);
    double total = 0.0;
    for (size_t i = 1; i < r.size(); ++i) {
        const int nodeId = (r[i].cpId >= 0) ? cpNode(r[i].cpId) : intNode(r[i].vi, r[i].hi);
        const int want = state(nodeId, (int)r[i].dir);
        bool found = false;
        for (const auto& e : edges(cur, cps, i == 1))
            if (e.toState == want) { total += e.cost; found = true; break; }
        if (!found) return -1.0;
        cur = want;
    }
    return total;
}
