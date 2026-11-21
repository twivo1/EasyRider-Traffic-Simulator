#include "SimMonitor.h"
#include "SimulationEngine.h"
#include "Car.h"
#include <cmath>
#include <algorithm>
#include <array>

namespace {
constexpr double kDeg = 180.0 / M_PI;

using Quad = std::array<QPointF, 4>;

Quad corners(QPointF c, double hRad, double L, double W) {
    const double cs = std::cos(hRad), sn = std::sin(hRad);
    const QPointF ax(cs * L / 2, sn * L / 2);      // oś wzdłużna
    const QPointF ay(-sn * W / 2, cs * W / 2);     // oś poprzeczna
    return { c + ax + ay, c + ax - ay, c - ax - ay, c - ax + ay };
}

double dot(QPointF a, QPointF b) { return a.x() * b.x() + a.y() * b.y(); }

/// Penetracja SAT (>0 = nakładanie o tyle px wzdłuż osi minimalnej).
double satPenetration(const Quad& a, const Quad& b) {
    double best = 1e18;
    const Quad* polys[2] = { &a, &b };
    for (const Quad* p : polys) {
        for (int i = 0; i < 2; ++i) {
            QPointF e = (*p)[(i + 1) % 4] - (*p)[i];
            QPointF n(-e.y(), e.x());
            const double len = std::hypot(n.x(), n.y());
            if (len < 1e-12) continue;
            n /= len;
            double amin = 1e18, amax = -1e18, bmin = 1e18, bmax = -1e18;
            for (const auto& q : a) { double t = dot(q, n); amin = std::min(amin, t); amax = std::max(amax, t); }
            for (const auto& q : b) { double t = dot(q, n); bmin = std::min(bmin, t); bmax = std::max(bmax, t); }
            const double ov = std::min(amax, bmax) - std::max(amin, bmin);
            if (ov <= 0) return ov;          // separacja
            best = std::min(best, ov);
        }
    }
    return best;
}

double pointSegDist(QPointF p, QPointF a, QPointF b) {
    QPointF ab = b - a;
    double t = dot(p - a, ab) / std::max(1e-12, dot(ab, ab));
    t = std::clamp(t, 0.0, 1.0);
    QPointF q = a + ab * t;
    return std::hypot(p.x() - q.x(), p.y() - q.y());
}

double polyDistance(const Quad& a, const Quad& b) {
    double d = 1e18;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            d = std::min(d, pointSegDist(a[i], b[j], b[(j + 1) % 4]));
            d = std::min(d, pointSegDist(b[i], a[j], a[(j + 1) % 4]));
        }
    return d;
}

bool isFin(double v) { return std::isfinite(v); }

QPointF ringCenter(int vi, int hi) {
    return { kVLeft[vi] + kRoadW / 2.0, kHTop[hi] + kRoadW / 2.0 };
}
} // namespace

double SimMonitor::obbGap(QPointF ca, double ha, QPointF cb, double hb, double L, double W) {
    const Quad qa = corners(ca, ha, L, W), qb = corners(cb, hb, L, W);
    const double pen = satPenetration(qa, qb);
    if (pen > 0) return -pen;
    return polyDistance(qa, qb);
}

void SimMonitor::logV(const QString& kind, int a, int b, const QString& detail) {
    if (m_log.size() >= 60) return;
    m_log << QString("tick %1 (t=%2s) %3 car#%4%5: %6")
                 .arg(m_s.ticks).arg(m_s.simTime, 0, 'f', 3).arg(kind).arg(a)
                 .arg(b >= 0 ? QString(" vs car#%1").arg(b) : QString())
                 .arg(detail);
}

int SimMonitor::carsStoppedLongerThan(double seconds) const {
    int n = 0;
    for (const auto& [id, p] : m_prev) if (p.stopped > seconds) ++n;
    return n;
}

void SimMonitor::observe(const SimulationEngine& engine, double dt) {
    m_s.ticks++;
    m_s.simTime += dt;
    // Korekty liczone od początku TEJ obserwacji (licznik silnika jest skumulowany).
    if (m_corrBase < 0) m_corrBase = engine.overlapCorrections();
    m_s.engineCorrections = engine.overlapCorrections() - m_corrBase;

    struct Cur { const Car* car; int id; QPointF c; double h; double v; };
    std::vector<Cur> cur;
    cur.reserve(engine.objects().size());
    for (const auto& o : engine.objects()) {
        const Car* car = dynamic_cast<const Car*>(o.get());
        if (!car) continue;
        cur.push_back({ car, car->id(), car->center(), car->headingRad(), car->speed() });
    }
    m_s.maxCars = std::max<int>(m_s.maxCars, (int)cur.size());

    for (auto& [id, p] : m_prev) p.seen = false;

    // ── Inwarianty jednego auta ─────────────────────────────────────
    for (const Cur& k : cur) {
        const Car* car = k.car;
        // I1 — wartości skończone
        if (!isFin(k.c.x()) || !isFin(k.c.y()) || !isFin(k.v) || !isFin(k.h)) {
            m_s.nonFinite++;
            logV("NON-FINITE", k.id, -1, "pozycja/prędkość/kurs = NaN/inf");
            continue;
        }
        m_s.meanSpeed += k.v; m_s.speedSamples++;

        // I2 — legalny tor
        double off = 0.0;
        if (car->inManeuver()) {
            off = car->maneuverPathError();
        } else if (car->isOnRing()) {
            const QPointF rc = ringCenter(car->ringVI(), car->ringHI());
            off = std::abs(std::hypot(k.c.x() - rc.x(), k.c.y() - rc.y()) - kRingR);
        } else {
            switch (car->dir()) {
            case Direction::L2R: off = std::abs(k.c.y() - (kL2R_Y(car->road()) + kCarW / 2)); break;
            case Direction::R2L: off = std::abs(k.c.y() - (kR2L_Y(car->road()) + kCarW / 2)); break;
            case Direction::T2B: off = std::abs(k.c.x() - (kT2B_X(car->road()) + kCarW / 2)); break;
            case Direction::B2T: off = std::abs(k.c.x() - (kB2T_X(car->road()) + kCarW / 2)); break;
            }
        }
        m_s.maxOffPath = std::max(m_s.maxOffPath, off);
        if (off > 0.5) {
            m_s.offPath++;
            logV("OFF-PATH", k.id, -1, QString("odchyłka od toru %1 px").arg(off, 0, 'f', 2));
        }

        auto it = m_prev.find(k.id);
        if (it == m_prev.end()) {
            m_s.spawned++;
            Prev np{ k.c, k.h, k.v, car->dir(), car->road(), car->isOnRing(),
                     car->ringVI(), car->ringHI(), 0.0, 0.0 };
            m_prev.emplace(k.id, np);
            continue;
        }
        Prev& p = it->second;
        p.seen = true;
        p.age += dt;

        // I3 — ciągłość położenia
        const QPointF d = k.c - p.c;
        const double dist = std::hypot(d.x(), d.y());
        const double allowed = std::max(p.speed, k.v) * dt * 1.05 + 0.3;
        if (dist > allowed) {
            m_s.teleports++;
            m_s.maxExcessJump = std::max(m_s.maxExcessJump, dist - allowed);
            logV("TELEPORT", k.id, -1,
                 QString("Δ=%1 px > dozwolone %2 px (v=%3→%4, ring %5→%6)")
                     .arg(dist, 0, 'f', 2).arg(allowed, 0, 'f', 2)
                     .arg(p.speed, 0, 'f', 1).arg(k.v, 0, 'f', 1)
                     .arg(p.ring).arg(car->isOnRing()));
        }
        // Cofnięcie: rzut przemieszczenia na poprzedni kurs < 0
        const double along = d.x() * std::cos(p.heading) + d.y() * std::sin(p.heading);
        if (along < -0.3) {
            m_s.reverseJumps++;
            logV("REVERSE", k.id, -1, QString("cofnięcie o %1 px").arg(-along, 0, 'f', 2));
        }
        // Ciągłość kursu
        const double dh = std::abs(std::remainder((k.h - p.heading) * kDeg, 360.0));
        m_s.maxHeadingStep = std::max(m_s.maxHeadingStep, dh);
        // Próg na próbkę: 20° lub fizyczny limit prędkości kątowej (300°/s ≈ łuk
        // r = 8 px przy ~40 px/s) × dt — gdy próbki są rzadsze niż 1/60 s
        // (np. GUI ×4 pominie klatkę), stały próg dawałby fałszywe alarmy.
        if (dh > std::max(20.0, 300.0 * dt * 1.2)) {
            m_s.headingJumps++;
            logV("HEADING-JUMP", k.id, -1, QString("Δkurs=%1°").arg(dh, 0, 'f', 1));
        }

        // I7 — legalne następstwa odcinków
        const bool laneNow = !car->isOnRing() && !car->inManeuver();
        if (laneNow && (car->dir() != p.dir || car->road() != p.road)) {
            int vi = -1, hi = -1;
            const bool hNew = dirIsHoriz(car->dir()), hOld = dirIsHoriz(p.dir);
            bool legal = true;
            if (p.ring) {
                vi = p.ringVi; hi = p.ringHi;
                legal = (hNew ? car->road() == hi : car->road() == vi);
            } else if (hNew != hOld) {
                vi = hNew ? p.road : car->road();
                hi = hNew ? car->road() : p.road;
                const QRectF zone = kIZone(vi, hi).adjusted(-kCarL, -kCarL, kCarL, kCarL);
                legal = zone.contains(k.c);
            } else {
                legal = false;
                if (car->dir() == dirOpposite(p.dir)) {
                    m_s.uturns++;
                    logV("U-TURN", k.id, -1, "zawrócenie bez skrzyżowania");
                }
            }
            if (!legal) {
                m_s.illegalTransitions++;
                logV("ILLEGAL-SEGMENT", k.id, -1,
                     QString("(%1,%2)→(%3,%4) poza skrzyżowaniem")
                         .arg((int)p.dir).arg(p.road).arg((int)car->dir()).arg(car->road()));
            }
            // Dwa skręty na tym samym skrzyżowaniu kończące się zawróceniem
            if (vi >= 0 && p.lastTurnVi == vi && p.lastTurnHi == hi
                && car->dir() == dirOpposite(p.preTurnDir)) {
                m_s.uturns++;
                logV("U-TURN", k.id, -1, QString("podwójny skręt na (%1,%2)").arg(vi).arg(hi));
            }
            if (vi >= 0 && !(p.lastTurnVi == vi && p.lastTurnHi == hi)) {
                p.lastTurnVi = vi; p.lastTurnHi = hi; p.preTurnDir = p.dir;
            }
        }

        if (k.v < 1.0) p.stopped += dt; else p.stopped = 0.0;
        m_s.maxStoppedTime = std::max(m_s.maxStoppedTime, p.stopped);
    }

    // ── Pary aut: nakładanie, odstęp, stan w połowie kroku ──────────
    const double reach = std::hypot(kCarL, kCarW) + 4.0;
    for (size_t i = 0; i < cur.size(); ++i) {
        for (size_t j = i + 1; j < cur.size(); ++j) {
            const Cur& a = cur[i]; const Cur& b = cur[j];
            if (std::abs(a.c.x() - b.c.x()) > reach || std::abs(a.c.y() - b.c.y()) > reach) continue;
            const double gap = obbGap(a.c, a.h, b.c, b.h);
            m_s.minGap = std::min(m_s.minGap, gap);
            if (gap < 0) {
                m_s.overlaps++;
                m_s.maxPenetration = std::max(m_s.maxPenetration, -gap);
                const long long key = (long long)std::min(a.id, b.id) * 1000003LL + std::max(a.id, b.id);
                if (!m_pairSeen.count(key)) { m_pairSeen[key] = true; m_s.overlapPairs++; }
                logV("OVERLAP", a.id, b.id, QString("penetracja %1 px, A=(%2,%3) B=(%4,%5)")
                         .arg(-gap, 0, 'f', 2).arg(a.c.x(), 0, 'f', 1).arg(a.c.y(), 0, 'f', 1)
                         .arg(b.c.x(), 0, 'f', 1).arg(b.c.y(), 0, 'f', 1));
                continue;
            }
            if (gap < m_contactGap) {
                m_s.contacts++;
                logV("CONTACT", a.id, b.id, QString("odstęp %1 px").arg(gap, 0, 'f', 2));
            }
            // Stan w połowie kroku (konserwatywna kontrola trajektorii)
            auto pa = m_prev.find(a.id), pb = m_prev.find(b.id);
            if (pa != m_prev.end() && pb != m_prev.end() && pa->second.age > 0 && pb->second.age > 0) {
                auto mid = [](const Prev& p, const Cur& c, QPointF& mc, double& mh) {
                    mc = (p.c + c.c) / 2.0;
                    mh = p.heading + std::remainder(c.h - p.heading, 2 * M_PI) / 2.0;
                };
                QPointF ma, mb; double ha, hb;
                mid(pa->second, a, ma, ha); mid(pb->second, b, mb, hb);
                if (obbGap(ma, ha, mb, hb) < 0) {
                    m_s.sweptHits++;
                    logV("SWEPT", a.id, b.id, "obrysy przecinają się między próbkami");
                }
            }
        }
    }

    // ── Auta usunięte w tym kroku ───────────────────────────────────
    for (auto it = m_prev.begin(); it != m_prev.end();) {
        if (it->second.seen) {
            // aktualizacja stanu poprzedniego
            ++it; continue;
        }
        const QPointF c = it->second.c;
        const bool atEdge = c.x() < 90 || c.x() > kCanvasW - 90 || c.y() < 90 || c.y() > kCanvasH - 90;
        if (atEdge) {
            m_s.exited++;
            m_s.sumTripTime += it->second.age; m_s.tripCount++;
        } else {
            m_s.vanished++;
            logV("VANISHED", it->first, -1, QString("zniknęło w (%1,%2)").arg(c.x()).arg(c.y()));
        }
        it = m_prev.erase(it);
    }
    for (const Cur& k : cur) {
        auto it = m_prev.find(k.id);
        if (it == m_prev.end()) continue;
        Prev& p = it->second;
        p.c = k.c; p.heading = k.h; p.speed = k.v;
        p.dir = k.car->dir(); p.road = k.car->road();
        p.ring = k.car->isOnRing(); p.ringVi = k.car->ringVI(); p.ringHi = k.car->ringHI();
    }
}
