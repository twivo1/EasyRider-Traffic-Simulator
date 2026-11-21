#include "RouteManager.h"
#include "core/SimulationEngine.h"
#include "core/Car.h"
#include <algorithm>
#include <cmath>

// ── Stałe nawigacji ─────────────────────────────────────────────────
// Dawniej recalc co 8 klatek + na każdą zmianę kierunku → flicker trasy.
// Teraz: po przejechaniu skrzyżowania (zoneChange), co 90 klatek (~1.5s)
// jako fallback oraz przy utknięciu. Próg kosztu 15% — nowa trasa zostaje
// przyjęta tylko gdy jest WYRAŹNIE lepsza (hysteresis).
static constexpr double kRecalcMs       = 5000.0;  // backup timer (1.5s tick + 5s timer)
static constexpr double kRecalcThr      = 0.15;    // 15% — trasa musi być wyraźnie lepsza
static constexpr int    kPeriodicTicks  = 90;      // co ~1.5 s (60 FPS) okresowa odśwież
static constexpr int    kStuckThreshold = 120;     // ~2 s przy 60 FPS
static constexpr double kStuckRadius    = 8.0;     // ruch w px, by uznać auto za jadące
static constexpr double kCommitDist     = 90.0;    // px — strefa „zatwierdzenia" przed skrzyżowaniem

RouteManager::RouteManager(SimulationEngine* engine, QObject* parent)
    : QObject(parent)
    , m_engine(engine)
    , m_analyzer(engine)
    , m_pathfinder(engine, &m_analyzer)
    , m_checkpoints(buildCheckpoints())
{
    connect(&m_recalcTimer, &QTimer::timeout, this, &RouteManager::onRecalcTimer);
}
RouteManager::~RouteManager() = default;

// ── Odczyt intencji skrętu ───────────────────────────────────────────
// KLUCZOWA NAPRAWA: intencja jest obliczana DYNAMICZNIE z aktualnego
// kierunku jazdy SmartCar i zapisanego absolutnego kierunku wyjazdu.
// Dzięki temu nawet jeśli SmartCar dotrze do skrzyżowania z innej strony
// niż planował A*, skręt będzie poprawny (bo cel wyjazdu się nie zmienia).
int RouteManager::intentForZone(int zoneID) const {
    if (!m_engine->hasSmartCar()) return 0;

    Direction carDir = m_engine->smartCarDir();
    // Jeśli auto jest na rondzie, użyj kierunku wjazdu na rondo
    // (bo m_dir nie zmienia się na rondzie — auto jedzie po łuku).

    // Sprawdź absolutny kierunek wyjazdu z mapy trasy
    auto it = m_zoneExitDirs.find(zoneID);
    if (it != m_zoneExitDirs.end()) {
        Direction exitDir = it->second;
        int intent = computeIntent(carDir, exitDir);
        // Jeśli wyliczona intencja to zawracanie (-1), jedź prosto —
        // auto jest z innej strony niż trasa, przeliczenie trasy poprawi to.
        if (intent < 0) return 0;
        return intent;
    }

    // AWARYJNIE (poza trasą): SmartCar przegapił skręt lub trasa nie
    // obejmuje tego skrzyżowania. Skieruj go ku celowi.
    if (m_endCpId < 0) return 0;

    int vi = zoneID / kNumRoads;
    int hi = zoneID % kNumRoads;
    const Checkpoint& endCp = m_checkpoints[m_endCpId];

    double dx = endCp.worldPos.x() - (kVLeft[vi] + kRoadW / 2.0);
    double dy = endCp.worldPos.y() - (kHTop[hi] + kRoadW / 2.0);

    // Decyzja: czy skręcić w stronę celu?
    if (dirIsHoriz(carDir)) {
        if (std::abs(dy) > kRoadW * 2) {
            Direction want = (dy > 0) ? Direction::T2B : Direction::B2T;
            int intent = computeIntent(carDir, want);
            if (intent > 0) return intent;
        }
    } else {
        if (std::abs(dx) > kRoadW * 2) {
            Direction want = (dx > 0) ? Direction::L2R : Direction::R2L;
            int intent = computeIntent(carDir, want);
            if (intent > 0) return intent;
        }
    }
    return 0; // prosto
}

/// Buduje mapę absolutnych kierunków wyjazdu z trasy A*.
/// Zamiast relatywnych intencji (prawo/lewo/prosto) zapisujemy
/// docelowy kierunek jazdy — dzięki temu intencja jest obliczana
/// dopiero w momencie decyzji z AKTUALNEGO kierunku SmartCar.
void RouteManager::rebuildIntentMap() {
    m_zoneExitDirs.clear();
    for (int i = 0; i + 1 < (int)m_route.size(); i++) {
        const RouteStep& cur = m_route[i];
        const RouteStep& nxt = m_route[i+1];
        if (cur.vi < 0 || cur.hi < 0) continue;
        // Zapisz absolutny kierunek wyjazdu (Direction z następnego kroku)
        m_zoneExitDirs[cur.vi * kNumRoads + cur.hi] = nxt.dir;
    }
}

// ── Rozpoczęcie nawigacji ────────────────────────────────────────────
bool RouteManager::startNavigation(int startCpId, int endCpId) {
    if (startCpId < 0 || endCpId < 0 || startCpId == endCpId) return false;
    if (startCpId >= (int)m_checkpoints.size()) return false;
    if (endCpId   >= (int)m_checkpoints.size()) return false;

    stopNavigation();
    m_startCpId = startCpId;
    m_endCpId   = endCpId;

    m_analyzer.update();
    double cost = 0;
    m_route = m_pathfinder.findRoute(startCpId, endCpId, m_checkpoints, &cost);
    if (m_route.empty()) return false;

    m_routeCost   = cost;
    m_recalcCount = 0;
    m_stuckTicks  = 0;
    m_tickCounter = 0;
    m_lastZoneId  = -1;
    m_lastDir     = m_checkpoints[startCpId].entryDir;
    m_arrivedCorrectly = false;
    m_navigating  = true;

    rebuildIntentMap();
    m_engine->setSmartCarIntentFn([this](int zoneID) -> int {
        return intentForZone(zoneID);
    });

    const Checkpoint& cp = m_checkpoints[startCpId];
    m_engine->spawnSmartCar(cp.spawnPos, cp.entryDir, cp.roadIdx);
    m_lastSmartPos = m_engine->smartCarCenter();

    m_recalcTimer.start((int)kRecalcMs);
    emit routeChanged(m_route);
    return true;
}

void RouteManager::stopNavigation() {
    m_recalcTimer.stop();
    m_engine->setSmartCarIntentFn(nullptr);
    m_engine->removeSmartCar();
    m_navigating = false;
    m_route.clear();
    m_zoneExitDirs.clear();
    m_routeCost = 0.0;
    m_stuckTicks = 0;
}

// ── Tick — GPS-like real-time przeliczanie trasy ─────────────────────
//
// Jak w Forza Horizon / GTA V: trasa ZAWSZE odpowiada aktualnej pozycji
// i kierunkowi jazdy SmartCar. Przeliczanie A* następuje natychmiast gdy:
//   • Auto wjedzie do nowej strefy (skrzyżowania) → zmienił się zoneId
//   • Auto zmieni kierunek (np. skręciło) → zmienił się m_dir
//   • Co 8 klatek jako fallback (~7.5x/s)
//   • Auto utknęło (nie rusza się przez 2 sekundy)
//
// Brak progu kosztu — KAŻDE przeliczenie aktualizuje trasę. Dzięki temu
// żółta linia nigdy nie pokazuje „starej" ścieżki niezgodnej z ruchem auta.
// ── Tick — STABILNA nawigacja w stylu GPS ────────────────────────────
//
// Triggery przeliczenia (każdy może wywołać recalc):
//   • Zmiana strefy — auto przejechało skrzyżowanie (najpewniejszy moment)
//   • Co 90 klatek (~1,5 s) — adaptacja do zmian ruchu
//   • Utknięcie — auto nie ruszało się przez 2 s
//
// COMMIT-ZONE LOCK: gdy auto jest bliżej niż kCommitDist od następnego
// skrzyżowania, NIE przeliczamy — auto już „commitowało" się do skrętu
// (kierunkowskaz włączony, intencja ustalona). Zmiana trasy pod nosem
// powodowała wcześniej flicker i zgubienie skrętu.
//
// Zmiana kierunku (dirChanged) USUNIĘTA — na rondzie kierunek zmienia
// się ciągle, co powodowało nieustanny flicker trasy.
void RouteManager::tick() {
    if (!m_navigating) return;
    if (!m_engine->hasSmartCar()) {
        m_navigating = false;
        m_recalcTimer.stop();
        m_engine->setSmartCarIntentFn(nullptr);
        if (m_arrivedCorrectly) emit navigationFinished();
        else                    emit navigationFailed();
        return;
    }
    if (checkArrival()) return;

    // ── Stan: bieżąca strefa i odległość do następnego skrzyżowania ──
    int curVi, curHi;
    bool hasZone = findCarIntersection(curVi, curHi);
    int curZone = hasZone ? (curVi * kNumRoads + curHi) : -1;
    bool zoneChanged = (curZone != m_lastZoneId && curZone >= 0);
    if (zoneChanged) m_lastZoneId = curZone;

    // Dystans przedniej krawędzi auta do najbliższego skrzyżowania.
    double distToNext = distanceToNextIntersection();
    bool inCommitZone = (distToNext > 0 && distToNext < kCommitDist);

    // ── Decyzja: czy w ogóle pora przeliczać? ────────────────────────
    m_tickCounter++;
    bool wantRecalc = false;
    if (zoneChanged)                          wantRecalc = true;
    else if (m_tickCounter >= kPeriodicTicks) wantRecalc = true;

    // Przeliczenie jest bezpieczne także w strefie zatwierdzenia: manewr na
    // najbliższym węźle jest wtedy wymuszony (planningOrigin), więc nowa trasa
    // zaczyna się od legalnego przyszłego punktu decyzyjnego.
    (void)inCommitZone;
    if (wantRecalc) {
        m_tickCounter = 0;
        recalcFromCurrentPos();
    }

    // ── Detekcja utknięcia (priorytet — także w commit-zone) ────────
    QPointF pos = m_engine->smartCarCenter();
    double ddx = pos.x() - m_lastSmartPos.x();
    double ddy = pos.y() - m_lastSmartPos.y();
    if (ddx*ddx + ddy*ddy < kStuckRadius*kStuckRadius) {
        m_stuckTicks++;
    } else {
        m_stuckTicks = 0;
        m_lastSmartPos = pos;
    }
    if (m_stuckTicks >= kStuckThreshold) {
        m_stuckTicks = 0;
        m_tickCounter = 0;
        recalcFromCurrentPos();
    }
}

// ── Dystans od przedniej krawędzi SmartCar do najbliższego skrzyżowania ─
// Zwraca odległość w px lub ujemną gdy auto jest już w strefie.
double RouteManager::distanceToNextIntersection() const {
    if (!m_engine->hasSmartCar()) return -1.0;
    int vi, hi;
    if (!findCarIntersection(vi, hi)) return -1.0;
    QPointF pos = m_engine->smartCarCenter();
    Direction dir = m_engine->smartCarDir();
    const double half = kCarL / 2.0;
    switch (dir) {
    case Direction::L2R: return kVLeft[vi]          - (pos.x() + half);
    case Direction::R2L: return (pos.x() - half)    - (kVLeft[vi] + kRoadW);
    case Direction::T2B: return kHTop[hi]           - (pos.y() + half);
    default:             return (pos.y() - half)    - (kHTop[hi] + kRoadW);
    }
}

// ── Znajdź NAJBLIŻSZE skrzyżowanie PRZED SmartCar ────────────────────
bool RouteManager::findCarIntersection(int& outVi, int& outHi) const {
    if (!m_engine->hasSmartCar()) return false;
    QPointF   pos = m_engine->smartCarCenter();
    Direction dir = m_engine->smartCarDir();

    if (dir == Direction::L2R || dir == Direction::R2L) {
        int hi = m_engine->smartCarRoad();
        int vi = -1;
        if (dir == Direction::L2R) {
            // Następna droga pionowa z przodu (wschód)
            for (int v = 0; v < kNumRoads; v++)
                if (kVLeft[v] + kRoadW > pos.x()) { vi = v; break; }
        } else {
            // Następna droga pionowa z przodu (zachód)
            for (int v = kNumRoads-1; v >= 0; v--)
                if (kVLeft[v] < pos.x()) { vi = v; break; }
        }
        if (vi < 0) vi = (dir == Direction::L2R) ? kNumRoads-1 : 0;
        outVi = std::clamp(vi, 0, kNumRoads-1);
        outHi = std::clamp(hi, 0, kNumRoads-1);
    } else {
        int vi = m_engine->smartCarRoad();
        int hi = -1;
        if (dir == Direction::T2B) {
            // Następna droga pozioma z przodu (południe)
            for (int h = 0; h < kNumRoads; h++)
                if (kHTop[h] + kRoadW > pos.y()) { hi = h; break; }
        } else {
            // Następna droga pozioma z przodu (północ)
            for (int h = kNumRoads-1; h >= 0; h--)
                if (kHTop[h] < pos.y()) { hi = h; break; }
        }
        if (hi < 0) hi = (dir == Direction::T2B) ? kNumRoads-1 : 0;
        outVi = std::clamp(vi, 0, kNumRoads-1);
        outHi = std::clamp(hi, 0, kNumRoads-1);
    }
    return true;
}

// ── Punkt planowania ─────────────────────────────────────────────────
// Trasa nie może zmienić manewru, który auto już wykonuje lub do którego się
// zobowiązało (łuk skrętu, rondo, strefa skrzyżowania, ostatnie kCommitDist px):
// wtedy kierunek wyjazdu z najbliższego węzła jest wymuszony.
bool RouteManager::planningOrigin(int& vi, int& hi, Direction& in, int& forcedExit) const {
    const Car* car = m_engine->smartCar();
    if (!car) return false;
    forcedExit = -1;
    in = car->dir();
    if (car->isOnRing()) {
        vi = car->ringVI(); hi = car->ringHI();
        forcedExit = (int)car->ringExitDir();
        return true;
    }
    if (car->inTurn()) {
        vi = car->maneuverVI(); hi = car->maneuverHI();
        forcedExit = (int)car->maneuverEndDir();
        return true;
    }
    if (!findCarIntersection(vi, hi)) return false;
    const bool inZone = kIZone(vi, hi).contains(car->center());
    if (inZone && car->decided()) { forcedExit = (int)car->dir(); return true; }
    const double dist = distanceToNextIntersection();
    if (inZone || dist < kCommitDist) {
        auto it = m_zoneExitDirs.find(vi * kNumRoads + hi);
        if (it != m_zoneExitDirs.end() && computeIntent(in, it->second) >= 0) forcedExit = (int)it->second;
        else forcedExit = (int)in;
    }
    return true;
}

Route RouteManager::currentSuffix(int vi, int hi, Direction in) const {
    for (size_t k = 0; k < m_route.size(); ++k)
        if (m_route[k].vi == vi && m_route[k].hi == hi && m_route[k].dir == in)
            return Route(m_route.begin() + k, m_route.end());
    return {};
}

// ── Przeliczenie trasy od bieżącej pozycji SmartCar ──────────────────
// HISTEREZA (poprawiona): nowa trasa jest porównywana z POZOSTAŁĄ częścią
// bieżącej trasy, wycenioną w TYM SAMYM snapshocie kosztów i od TEGO SAMEGO
// stanu startowego. (Dawniej porównywano koszt nowej trasy od bieżącej
// pozycji z kosztem całej starej trasy od startu — po przejechaniu 15%
// drogi każda inna trasa była „lepsza", co dawało zbędne przełączenia.)
// Zmiana następuje, gdy bieżąca trasa jest nieaktualna (auto zboczyło) albo
// zysk ≥ max(kMinGainAbs, kMinGainRel · pozostały czas).
void RouteManager::recalcFromCurrentPos() {
    if (!m_engine->hasSmartCar()) return;
    int vi, hi, forced; Direction in;
    if (!planningOrigin(vi, hi, in, forced)) return;

    m_analyzer.update();
    double newCost = 0;
    Route newRoute = m_pathfinder.findRouteFrom(vi, hi, in, m_endCpId, m_checkpoints, &newCost, forced);
    if (newRoute.empty()) return;

    Route suffix = currentSuffix(vi, hi, in);
    double curCost = suffix.size() >= 2 ? m_pathfinder.evaluateRoute(suffix, m_checkpoints) : -1.0;
    bool currentValid = curCost >= 0.0;
    if (currentValid && forced >= 0 && suffix.size() >= 2 && (int)suffix[1].dir != forced)
        currentValid = false;   // auto jest już zobowiązane do innego manewru

    bool same = currentValid && suffix.size() == newRoute.size();
    if (same)
        for (size_t i = 0; i < newRoute.size(); ++i)
            if (newRoute[i].nodeId != suffix[i].nodeId || newRoute[i].dir != suffix[i].dir) { same = false; break; }
    if (same) { m_routeCost = curCost; return; }

    bool accept = !currentValid;
    if (!accept) {
        const double gain = curCost - newCost;
        accept = gain >= std::max(kMinGainAbs, kMinGainRel * curCost);
    }
    if (accept) {
        m_route     = newRoute;
        m_routeCost = newCost;
        m_recalcCount++;
        rebuildIntentMap();
        emit this->routeChanged(m_route);
    } else {
        m_rerouteRejected++;
        m_routeCost = curCost;
    }
}

// ── Cykliczne przeliczanie trasy (na timerze) ────────────────────────
void RouteManager::onRecalcTimer() {
    if (!m_navigating || !m_engine->hasSmartCar()) return;
    recalcFromCurrentPos();  // z progiem porównania kosztu
}

// ── Wykrywanie dotarcia do celu ──────────────────────────────────────
bool RouteManager::checkArrival() {
    if (!m_engine->hasSmartCar()) return false;
    QPointF   pos  = m_engine->smartCarCenter();
    Direction dir  = m_engine->smartCarDir();
    int       road = m_engine->smartCarRoad();

    if (m_endCpId < 0 || m_endCpId >= (int)m_checkpoints.size()) return false;
    const Checkpoint& endCp = m_checkpoints[m_endCpId];

    if (road != endCp.roadIdx) return false;
    Direction exitD = endCp.exitDir();
    if (dir != exitD) return false;

    constexpr double kEdge = 60.0;
    bool atEdge = false;
    switch (exitD) {
    case Direction::L2R: atEdge = (pos.x() > kCanvasW - kEdge); break;
    case Direction::R2L: atEdge = (pos.x() < kEdge);            break;
    case Direction::T2B: atEdge = (pos.y() > kCanvasH - kEdge); break;
    case Direction::B2T: atEdge = (pos.y() < kEdge);            break;
    }

    if (atEdge) {
        m_arrivedCorrectly = true;
        return true;
    }
    return false;
}

// ════════════════════════════════════════════════════════════════════
//  visualRoute — trasa przycięta do aktualnej pozycji SmartCar
// ════════════════════════════════════════════════════════════════════
// Działa jak GPS: linia ZACZYNA SIĘ od pozycji auta i obejmuje
// tylko odcinki jeszcze PRZED nim. Minięte skrzyżowania znikają.
Route RouteManager::visualRoute() const {
    if (!m_navigating || !m_engine->hasSmartCar() || m_route.size() < 2)
        return m_route;

    QPointF carPos = m_engine->smartCarCenter();
    Direction carDir = m_engine->smartCarDir();
    bool carOnRing = m_engine->smartCarOnRing();

    // ── Na rondzie: zwróć TYLKO kroki PO rondzie, BEZ carPos ───────
    // Polilinia musi być STATYCZNA — auto krąży po ringu, a linia trasy
    // nie może się ruszać z nim. Bez tego carPos prepend tworzyłby
    // odcinek od (rotującego) auta do (stałego) laneEntry — efekt
    // „wymachującego ogona" przy każdym okrążeniu.
    if (carOnRing) {
        int ringVi = m_engine->smartCarRingVI();
        int ringHi = m_engine->smartCarRingHI();
        Route after;
        int lastRingIdx = -1;
        for (int i = 0; i < (int)m_route.size(); i++)
            if (m_route[i].vi == ringVi && m_route[i].hi == ringHi)
                lastRingIdx = i;
        int start = (lastRingIdx >= 0) ? lastRingIdx + 1 : 0;
        for (int i = start; i < (int)m_route.size(); i++)
            after.push_back(m_route[i]);
        return after;
    }

    // Szukaj pierwszego kroku trasy, którego auto JESZCZE NIE minęło.
    // Krok jest „minięty", gdy auto jest za jego pozycją w kierunku jazdy
    // tego kroku (z marginesem na szerokość skrzyżowania + rozmiar auta).
    //
    // KLUCZOWE dla artefaktu „orange tail": dla kroku skrętu sprawdzamy
    // przejście W OBU KIERUNKACH (wjazdowym = step.dir ORAZ wyjazdowym =
    // next.dir). Po skręcie auto już jedzie nowym kierunkiem (np. T2B), ale
    // step.dir nadal pamięta wjazd L2R — bez sprawdzenia kierunku
    // wyjazdowego skrzyżowanie po skręcie nie byłoby oznaczone jako minięte
    // i polilinia rysowała by „ogon" do starego pasa.
    auto passedInDir = [&](const RouteStep& step, Direction d) -> bool {
        double margin = kRoadW + kCarL;
        switch (d) {
        case Direction::L2R: return carPos.x() > step.worldPos.x() + margin;
        case Direction::R2L: return carPos.x() < step.worldPos.x() - margin;
        case Direction::T2B: return carPos.y() > step.worldPos.y() + margin;
        case Direction::B2T: return carPos.y() < step.worldPos.y() - margin;
        }
        return false;
    };
    int firstAhead = 0;
    for (int i = 0; i < (int)m_route.size(); i++) {
        const RouteStep& step = m_route[i];
        if (step.vi < 0 && step.hi < 0) {
            // Checkpoint — minięty jeśli auto jest daleko od niego
            double dx = carPos.x() - step.worldPos.x();
            double dy = carPos.y() - step.worldPos.y();
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist > kRoadW * 2) { firstAhead = i + 1; continue; }
            break;
        }
        bool passed = passedInDir(step, step.dir);
        if (!passed && i + 1 < (int)m_route.size()) {
            Direction exitDir = m_route[i + 1].dir;
            if (exitDir != step.dir) passed = passedInDir(step, exitDir);
        }
        if (passed) firstAhead = i + 1;
        else break;
    }

    // Cały przód trasy minięty → pokaż tylko cel
    if (firstAhead >= (int)m_route.size()) {
        Route r;
        RouteStep carStep;
        carStep.nodeId = -1; carStep.vi = -1; carStep.hi = -1; carStep.cpId = -1;
        carStep.dir = carDir; carStep.worldPos = carPos;
        r.push_back(carStep);
        r.push_back(m_route.back());
        return r;
    }

    // Zbuduj trasę wizualną: pozycja auta → pierwszy nie-minięty krok → reszta
    Route visual;

    // Punkt 0: aktualna pozycja SmartCar
    RouteStep carStep;
    carStep.nodeId = -1; carStep.vi = -1; carStep.hi = -1; carStep.cpId = -1;
    carStep.dir = carDir;
    carStep.worldPos = carPos;
    visual.push_back(carStep);

    // Reszta trasy od pierwszego nie-miniętego kroku
    for (int i = firstAhead; i < (int)m_route.size(); i++)
        visual.push_back(m_route[i]);

    return visual;
}
