#include "TrafficAnalyzer.h"
#include "core/SimulationObject.h"
#include <cmath>
#include <algorithm>

// ── Klasyfikacja auta do odcinka drogi ──────────────────────────────
// Zwraca (dirIdx, roadIdx, segIdx), gdzie segIdx = from+1 (patrz nagłówek).
// Zwraca false, gdy auto jest w strefie skrzyżowania (pomijane w koszcie).
static bool classifyCar(const SimulationObject& obj,
                         int& dirIdx, int& roadIdx, int& seg)
{
    Direction d = obj.dir();
    int       r = obj.road();
    QPointF   c = obj.boundingRect().center();
    dirIdx  = (int)d;
    roadIdx = r;

    if (d == Direction::L2R || d == Direction::R2L) {
        // Auto poziome, road = hi
        double cx = c.x();
        // Znajdź odcinek pionowy, w którym jest środek auta
        if (cx < kVLeft[0]) { seg = 0; return true; }          // przed vi=0
        for (int vi=0; vi<kNumRoads-1; ++vi) {
            double segStart = kVLeft[vi]  + kRoadW;
            double segEnd   = kVLeft[vi+1];
            if (cx >= segStart && cx < segEnd) { seg = vi+1; return true; }
        }
        if (cx >= kVLeft[kNumRoads-1]+kRoadW) { seg = kNumRoads; return true; } // za vi=7
        return false; // w strefie skrzyżowania
    } else {
        // Auto pionowe, road = vi
        double cy = c.y();
        if (cy < kHTop[0]) { seg = 0; return true; }
        for (int hi=0; hi<kNumRoads-1; ++hi) {
            double segStart = kHTop[hi]  + kRoadW;
            double segEnd   = kHTop[hi+1];
            if (cy >= segStart && cy < segEnd) { seg = hi+1; return true; }
        }
        if (cy >= kHTop[kNumRoads-1]+kRoadW) { seg = kNumRoads; return true; }
        return false;
    }
}

void TrafficAnalyzer::update() {
    // Zerowanie
    for (auto& a : m_counts)   for (auto& b : a)   b.fill(0);
    for (auto& a : m_stopped)  for (auto& b : a)   b.fill(0);
    for (auto& a : m_ring) a.fill(0);
    for (auto& a : m_avgSpeed) for (auto& b : a)   b.fill(110.0);

    std::array<std::array<std::array<double,kNumRoads+1>,kNumRoads>,4> speedSum{};
    for (auto& a : speedSum) for (auto& b : a) b.fill(0.0);

    for (const auto& obj : m_engine->objects()) {
        if (obj->isOnRing()) {   // auto na rondzie nie jest na żadnym odcinku
            if (obj->ringVI() >= 0) m_ring[obj->ringVI()][obj->ringHI()]++;
            continue;
        }
        int di, ri, si;
        if (!classifyCar(*obj, di, ri, si)) continue;
        m_counts[di][ri][si]++;
        if (obj->speed() < 5.0) m_stopped[di][ri][si]++;
        speedSum[di][ri][si] += obj->speed();
    }

    // Wylicz średnią prędkość na każdym odcinku
    for (int d=0;d<4;++d)
        for (int r=0;r<kNumRoads;++r)
            for (int s=0;s<=kNumRoads;++s) {
                int n = m_counts[d][r][s];
                m_avgSpeed[d][r][s] = (n > 0) ? speedSum[d][r][s] / n : 110.0;
            }
}

bool TrafficAnalyzer::key(Direction dir, int roadIdx, int from, int to, int& di, int& si) const {
    di = (int)dir;
    if (di < 0 || di >= 4 || roadIdx < 0 || roadIdx >= kNumRoads) return false;
    si = std::clamp(segmentIndex(dir, from, to), 0, kNumRoads);
    return true;
}

double TrafficAnalyzer::segmentCostFactor(Direction dir, int roadIdx,
                                           int from, int to) const {
    int di, si;
    if (!key(dir, roadIdx, from, to, di, si)) return 1.0;

    int    n     = m_counts[di][roadIdx][si];
    double spd   = m_avgSpeed[di][roadIdx][si];
    double len   = segmentLength(dir, from, to);
    if (len < 1.0) return 1.0;

    // Pojemność: odcinek "zapełnia się" przy niewielu autach (większa czułość).
    // Przy odstępie kCarL*1.8 już ~2-3 auta tworzą zauważalny korek.
    double cap = std::max(1.5, len / (kCarL * 1.8));
    double occ = std::min(1.0, (double)n / cap);

    // Współczynnik kosztu: 1.0 (płynny ruch) → 8.0 (niemal zatrzymany)
    double trafficFactor = 1.0 + 7.0 * occ * occ;

    // Każde auto dokłada małą karę bazową (nawet 1 auto spowalnia ruch)
    trafficFactor += n * 0.25;

    // Czynnik prędkości: o ile wolniejszy jest ruch od prędkości maksymalnej?
    double speedFactor = (spd < 5.0) ? 8.0 : (110.0 / spd);
    speedFactor = std::clamp(speedFactor, 1.0, 8.0);

    // Połącz oba czynniki (liczba aut z większą wagą)
    return 0.6 * trafficFactor + 0.4 * speedFactor;
}

double TrafficAnalyzer::redLightPenalty(int vi, int hi, Direction approach) const {
    const auto& s = m_engine->intersectionAt(vi, hi);
    if (s.isRoundabout) return 1.0;  // rondo: małe opóźnienie na ustąpienie
    LightState lt = dirIsHoriz(approach) ? s.hLight : s.vLight;
    switch (lt) {
    case LightState::Red:    return 4.0;   // ~4 s average wait
    case LightState::Yellow: return 1.5;
    default:                 return 0.0;
    }
}

double TrafficAnalyzer::globalCongestion() const {
    double sum = 0; int cnt = 0;
    for (int d=0;d<4;++d)
        for (int r=0;r<kNumRoads;++r)
            for (int s=1;s<kNumRoads;++s) { // pomiń odcinki skrajne
                double c = segmentCostFactor((Direction)d, r, s-1, s);
                sum += (c - 1.0) / 7.0; // normalizacja do 0..1
                cnt++;
            }
    return cnt > 0 ? sum / cnt : 0.0;
}

int TrafficAnalyzer::congestionHotspots() const {
    int hot = 0;
    for (int d=0;d<4;++d)
        for (int r=0;r<kNumRoads;++r)
            for (int s=1;s<kNumRoads;++s)
                if (segmentCostFactor((Direction)d, r, s-1, s) > 2.5)
                    hot++;
    return hot;
}

int TrafficAnalyzer::carsOnSegment(Direction dir, int roadIdx, int from, int to) const {
    int di, si;
    return key(dir, roadIdx, from, to, di, si) ? m_counts[di][roadIdx][si] : 0;
}
int TrafficAnalyzer::stoppedOnSegment(Direction dir, int roadIdx, int from, int to) const {
    int di, si;
    return key(dir, roadIdx, from, to, di, si) ? m_stopped[di][roadIdx][si] : 0;
}
double TrafficAnalyzer::avgSpeedOnSegment(Direction dir, int roadIdx, int from, int to) const {
    int di, si;
    return key(dir, roadIdx, from, to, di, si) ? m_avgSpeed[di][roadIdx][si] : 110.0;
}
