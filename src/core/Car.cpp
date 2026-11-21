#include "Car.h"
#include <algorithm>

// ═══════════════════════════════════════════════════════════════════
//  SampledPath
// ═══════════════════════════════════════════════════════════════════
namespace {
double dirHeading(Direction d) {
    switch (d) {
    case Direction::L2R: return 0.0;
    case Direction::T2B: return M_PI / 2;
    case Direction::R2L: return M_PI;
    default:             return -M_PI / 2;
    }
}
QPointF unit(double h) { return { std::cos(h), std::sin(h) }; }
double ringHeading(double theta) { return std::atan2(-std::cos(theta), std::sin(theta)); }
QPointF ringCenterOf(int vi, int hi) { return { kVLeft[vi] + kRoadW / 2.0, kHTop[hi] + kRoadW / 2.0 }; }
/// Środek pasa (współrzędna poprzeczna) dla kierunku i drogi.
double laneCross(Direction d, int road) {
    switch (d) {
    case Direction::L2R: return kL2R_Y(road) + kCarW / 2.0;
    case Direction::R2L: return kR2L_Y(road) + kCarW / 2.0;
    case Direction::T2B: return kT2B_X(road) + kCarW / 2.0;
    default:             return kB2T_X(road) + kCarW / 2.0;
    }
}
} // namespace

PathPose SampledPath::at(double d) const {
    if (pts.empty()) return { {}, 0.0 };
    if (d <= 0) return pts.front();
    if (d >= length()) return pts.back();
    auto it = std::upper_bound(s.begin(), s.end(), d);
    size_t i = std::max<size_t>(1, size_t(it - s.begin()));
    const double seg = s[i] - s[i - 1];
    const double t = seg > 1e-12 ? (d - s[i - 1]) / seg : 0.0;
    const PathPose& a = pts[i - 1]; const PathPose& b = pts[i];
    return { a.c + (b.c - a.c) * t, a.h + std::remainder(b.h - a.h, 2 * M_PI) * t };
}

double SampledPath::distanceTo(QPointF p) const {
    double best = 1e18;
    for (size_t i = 1; i < pts.size(); ++i) {
        QPointF a = pts[i - 1].c, ab = pts[i].c - a;
        double L2 = ab.x() * ab.x() + ab.y() * ab.y();
        double t = L2 > 1e-12 ? ((p.x() - a.x()) * ab.x() + (p.y() - a.y()) * ab.y()) / L2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        QPointF q = a + ab * t;
        best = std::min(best, std::hypot(p.x() - q.x(), p.y() - q.y()));
    }
    return best;
}

SampledPath SampledPath::arc(QPointF O, double r, double a0, double sweep, int n) {
    SampledPath p;
    const double tangentOff = sweep > 0 ? M_PI / 2 : -M_PI / 2;
    for (int i = 0; i <= n; ++i) {
        const double a = a0 + sweep * double(i) / n;
        p.pts.push_back({ O + QPointF(std::cos(a), std::sin(a)) * r, a + tangentOff });
        p.s.push_back(r * std::abs(sweep) * double(i) / n);
    }
    return p;
}

SampledPath SampledPath::bezier(PathPose from, PathPose to, double k0, double k1, int n) {
    SampledPath p;
    const QPointF P0 = from.c, P1 = from.c + unit(from.h) * k0;
    const QPointF P3 = to.c,   P2 = to.c - unit(to.h) * k1;
    double acc = 0.0; QPointF prev = P0;
    for (int i = 0; i <= n; ++i) {
        const double t = double(i) / n, u = 1 - t;
        QPointF c = P0 * (u * u * u) + P1 * (3 * u * u * t) + P2 * (3 * u * t * t) + P3 * (t * t * t);
        QPointF d = (P1 - P0) * (3 * u * u) + (P2 - P1) * (6 * u * t) + (P3 - P2) * (3 * t * t);
        double h = (std::hypot(d.x(), d.y()) > 1e-9) ? std::atan2(d.y(), d.x()) : (i == 0 ? from.h : to.h);
        if (i == 0) h = from.h;
        if (i == n) h = to.h;
        if (i > 0) acc += std::hypot(c.x() - prev.x(), c.y() - prev.y());
        p.pts.push_back({ c, h }); p.s.push_back(acc); prev = c;
    }
    return p;
}

// ═══════════════════════════════════════════════════════════════════
//  Car — konstrukcja, poza, obrys
// ═══════════════════════════════════════════════════════════════════
Car::Car(QPointF pos, Direction dir, int road)
    : m_dir(dir), m_road(road), m_pos(pos) {}
Car::~Car() = default;

QPointF Car::center() const {
    if (m_man || m_rb) return m_c;
    const bool poziomo = dirIsHoriz(m_dir);
    return m_pos + QPointF(poziomo ? kCarL / 2 : kCarW / 2, poziomo ? kCarW / 2 : kCarL / 2);
}
double Car::headingRad() const { return (m_man || m_rb) ? m_h : dirHeading(m_dir); }

QRectF Car::boundingRect() const {
    if (!m_man && !m_rb) {
        const bool poziomo = dirIsHoriz(m_dir);
        return QRectF(m_pos, QSizeF(poziomo ? kCarL : kCarW, poziomo ? kCarW : kCarL));
    }
    // Obrócony obrys → obejmujący go prostokąt osiowy (konserwatywny).
    const double cs = std::abs(std::cos(m_h)), sn = std::abs(std::sin(m_h));
    const double ex = cs * kCarL / 2 + sn * kCarW / 2, ey = sn * kCarL / 2 + cs * kCarW / 2;
    return QRectF(m_c.x() - ex, m_c.y() - ey, 2 * ex, 2 * ey);
}

double Car::headingAngle() const {
    if (m_man || m_rb) return m_h * 180.0 / M_PI;
    switch (m_dir) {
    case Direction::L2R: return 0;
    case Direction::R2L: return 180;
    case Direction::T2B: return 90;
    default:             return 270;
    }
}

void Car::setCenterLane(QPointF c) {
    // Dokładne ustawienie na osi pasa (eliminuje dryf numeryczny).
    if (dirIsHoriz(m_dir)) c.setY(laneCross(m_dir, m_road)); else c.setX(laneCross(m_dir, m_road));
    const bool poziomo = dirIsHoriz(m_dir);
    m_pos = c - QPointF(poziomo ? kCarL / 2 : kCarW / 2, poziomo ? kCarW / 2 : kCarL / 2);
}

bool Car::isOnRing() const {
    return m_rb != nullptr || (m_man && m_man->kind != Maneuver::Kind::Turn);
}
int Car::ringVI() const {
    if (m_rb) return m_rb->vi;
    if (m_man && m_man->kind != Maneuver::Kind::Turn) return m_man->vi;
    return -1;
}
int Car::ringHI() const {
    if (m_rb) return m_rb->hi;
    if (m_man && m_man->kind != Maneuver::Kind::Turn) return m_man->hi;
    return -1;
}
double Car::ringTheta() const {
    if (m_rb) return m_rb->theta;
    if (m_man && m_man->kind != Maneuver::Kind::Turn) {
        QPointF rc = ringCenterOf(m_man->vi, m_man->hi);
        return std::atan2(m_c.y() - rc.y(), m_c.x() - rc.x());
    }
    return 0.0;
}
double Car::maneuverPathError() const {
    return m_man ? m_man->path.distanceTo(m_c) : 0.0;
}

// ═══════════════════════════════════════════════════════════════════
//  Krok aktualizacji
// ═══════════════════════════════════════════════════════════════════
void Car::integrateSpeed(double dt) {
    double tgt = m_held ? 0.0 : m_target;
    const bool capBinding = m_cap < tgt;
    tgt = std::min(tgt, m_cap);
    if (m_speed < tgt) {
        m_speed = std::min(m_speed + kAccel * dt, tgt);
    } else if (m_speed > tgt) {
        // Hamowanie zwykłe; limit bezpieczeństwa (przeszkoda) może wymagać
        // hamowania awaryjnego, ale nadal w granicach modelu (kMaxBrake).
        const double b = capBinding ? kMaxBrake : kBrake;
        m_speed = std::max(m_speed - b * dt, tgt);
    }
    m_lastCapBy = capBinding ? m_capBy : -1;
    m_cap = std::numeric_limits<double>::infinity();
    m_capBy = -1;
}

void Car::update(double dt) {
    updateBlinker(dt);
    integrateSpeed(dt);
    advance(m_speed * dt);
    m_target = m_max;   // domyślny cel na kolejny krok; silnik nadpisze go w razie potrzeby
}

QPointF Car::ringPoint(double theta) const {
    return QPointF(m_rb->cx + kRingR * std::cos(theta), m_rb->cy + kRingR * std::sin(theta));
}

void Car::advance(double dist) {
    if (dist <= 0) return;
    if (m_man) {
        m_man->pos += dist;
        if (m_man->pos >= m_man->path.length()) { finishManeuver(m_man->pos - m_man->path.length()); return; }
        PathPose p = m_man->path.at(m_man->pos);
        m_c = p.c; m_h = p.h;
        m_pos = boundingRect().topLeft();
        return;
    }
    if (m_rb) {
        const double da = dist / kRingR;
        m_rb->theta    -= da;
        m_rb->traveled += da;
        m_c = ringPoint(m_rb->theta);
        m_h = ringHeading(m_rb->theta);
        m_pos = boundingRect().topLeft();
        return;
    }
    switch (m_dir) {
    case Direction::L2R: m_pos.rx() += dist; break;
    case Direction::R2L: m_pos.rx() -= dist; break;
    case Direction::T2B: m_pos.ry() += dist; break;
    case Direction::B2T: m_pos.ry() -= dist; break;
    }
}

void Car::finishManeuver(double leftover) {
    std::unique_ptr<Maneuver> man = std::move(m_man);
    const PathPose end = man->path.pts.back();
    if (man->kind == Maneuver::Kind::RingEntry) {
        m_rb = std::make_unique<RbState>();
        const QPointF rc = ringCenterOf(man->vi, man->hi);
        m_rb->cx = rc.x(); m_rb->cy = rc.y();
        m_rb->vi = man->vi; m_rb->hi = man->hi;
        m_rb->theta = man->ringTheta;
        m_rb->travelNeeded = man->travelNeeded;
        m_rb->traveled = 0.0;
        m_rb->exitDir = man->exitDir;
        m_rb->exitRoad = man->exitRoad;
        m_c = ringPoint(m_rb->theta); m_h = ringHeading(m_rb->theta);
        m_pos = boundingRect().topLeft();
        advance(leftover);
        return;
    }
    // Turn / RingExit → pas docelowy
    m_dir = man->endDir;
    m_road = man->endRoad;
    setCenterLane(end.c);
    if (man->kind == Maneuver::Kind::Turn) {
        m_decided = true;              // skręt na tym skrzyżowaniu już wykonany
        clearBlinker();
        clearPending();
    } else {
        m_decided = false;
        m_zoneID  = -1;
        clearBlinker();
        clearPending();
    }
    advance(leftover);
}

/// Cofnięcie auta wzdłuż toru — ostateczna siatka bezpieczeństwa.
void Car::pushBack(double dist) {
    if (dist <= 0) return;
    if (m_man) {
        m_man->pos = std::max(0.0, m_man->pos - dist);
        PathPose p = m_man->path.at(m_man->pos); m_c = p.c; m_h = p.h;
        m_pos = boundingRect().topLeft();
        return;
    }
    if (m_rb) {
        const double da = dist / kRingR;
        m_rb->theta += da;
        m_rb->traveled = std::max(0.0, m_rb->traveled - da);
        m_c = ringPoint(m_rb->theta); m_h = ringHeading(m_rb->theta);
        m_pos = boundingRect().topLeft();
        return;
    }
    switch (m_dir) {
    case Direction::L2R: m_pos.rx() -= dist; break;
    case Direction::R2L: m_pos.rx() += dist; break;
    case Direction::T2B: m_pos.ry() -= dist; break;
    case Direction::B2T: m_pos.ry() += dist; break;
    }
}

void Car::updateBlinker(double dt) {
    if (m_blinker == 0) return;
    m_blinkerTimer += dt;
    if (m_blinkerTimer >= kBlinkerHalf) {
        m_blinkerTimer -= kBlinkerHalf;
        m_blinkerOn = !m_blinkerOn;
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Przewidywany tor (wykrywanie przeszkód)
// ═══════════════════════════════════════════════════════════════════
void Car::futurePath(double dist, double step, std::vector<PathPose>& out) const {
    out.clear();
    const int n = std::max(1, int(std::ceil(dist / step)));
    PathPose cur{ center(), headingRad() };
    for (int k = 0; k <= n; ++k) {
        const double d = k * step;
        if (m_man) {
            const double rem = m_man->path.length() - m_man->pos;
            if (d <= rem) { out.push_back(m_man->path.at(m_man->pos + d)); continue; }
            const PathPose e = m_man->path.pts.back();
            if (m_man->kind == Maneuver::Kind::RingEntry) {
                const QPointF rc = ringCenterOf(m_man->vi, m_man->hi);
                const double th = m_man->ringTheta - (d - rem) / kRingR;
                out.push_back({ rc + QPointF(std::cos(th), std::sin(th)) * kRingR, ringHeading(th) });
            } else {
                out.push_back({ e.c + unit(e.h) * (d - rem), e.h });
            }
        } else if (m_rb) {
            const double th = m_rb->theta - d / kRingR;
            out.push_back({ QPointF(m_rb->cx, m_rb->cy) + QPointF(std::cos(th), std::sin(th)) * kRingR, ringHeading(th) });
        } else {
            out.push_back({ cur.c + unit(cur.h) * d, cur.h });
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Skręt na skrzyżowaniu — łuk kołowy styczny do obu pasów
// ═══════════════════════════════════════════════════════════════════
double Car::turnSpeed(double r) {
    return std::clamp(std::sqrt(kTurnLatAccel * std::max(r, 1.0)), kSlowSpeed, kZoneMaxSpd);
}

double Car::turnRadiusFromHere(Direction nd, int nr) const {
    if (m_man || m_rb || dirIsHoriz(nd) == dirIsHoriz(m_dir)) return -1.0;
    const QPointF c = center();
    const double target = laneCross(nd, nr);
    double r = 0;
    switch (m_dir) {
    case Direction::L2R: r = target - c.x(); break;
    case Direction::R2L: r = c.x() - target; break;
    case Direction::T2B: r = target - c.y(); break;
    default:             r = c.y() - target; break;
    }
    return r;
}

bool Car::startTurn(Direction nd, int nr, int vi, int hi) {
    const double r = turnRadiusFromHere(nd, nr);
    if (r < kMinTurnRadius) return false;      // minięto punkt styczności
    const QPointF c = center();
    const double h0 = dirHeading(m_dir), hn = dirHeading(nd);
    const QPointF n = unit(hn);
    const QPointF O = c + n * r;
    const double a0 = std::atan2(-n.y(), -n.x());
    const double sweep = std::remainder(h0 - (a0), 2 * M_PI) > 0 ? M_PI / 2 : -M_PI / 2;
    m_man = std::make_unique<Maneuver>();
    m_man->kind = Maneuver::Kind::Turn;
    m_man->path = SampledPath::arc(O, r, a0, sweep, 24);
    m_man->vi = vi; m_man->hi = hi;
    m_man->endDir = nd; m_man->endRoad = nr;
    m_c = c; m_h = h0;
    m_decided = true;
    m_pos = boundingRect().topLeft();
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Rondo — krzywe wjazdu/zjazdu styczne do okręgu
// ═══════════════════════════════════════════════════════════════════
double Car::laneOffset(Direction d) {
    // Odległość osi pasa od osi drogi, po prawej stronie kierunku jazdy.
    switch (d) {
    case Direction::L2R: return laneCross(d, 0) - (kHTop[0] + kRoadW / 2.0);
    case Direction::R2L: return (kHTop[0] + kRoadW / 2.0) - laneCross(d, 0);
    case Direction::T2B: return (kVLeft[0] + kRoadW / 2.0) - laneCross(d, 0);
    default:             return laneCross(d, 0) - (kVLeft[0] + kRoadW / 2.0);
    }
}
double Car::ringMergeTheta(Direction d, double w) {
    return M_PI - std::asin(w / kRingR) - kRbMergeDelta + dirHeading(d);
}
double Car::ringDivergeTheta(Direction e, double w) {
    return std::asin(w / kRingR) + kRbMergeDelta + dirHeading(e);
}

void Car::enterRoundabout(int vi, int hi, int intent) {
    // Z kierunku wjazdu i intencji (0=prosto,1=prawo,2=lewo) wyznacz zjazd.
    Direction exitDir; int exitRoad;
    switch (intent) {
    case 0: exitDir = m_dir; exitRoad = m_road; break;
    case 1:
        if      (m_dir == Direction::L2R) { exitDir = Direction::T2B; exitRoad = vi; }
        else if (m_dir == Direction::R2L) { exitDir = Direction::B2T; exitRoad = vi; }
        else if (m_dir == Direction::T2B) { exitDir = Direction::R2L; exitRoad = hi; }
        else                              { exitDir = Direction::L2R; exitRoad = hi; }
        break;
    default:
        if      (m_dir == Direction::L2R) { exitDir = Direction::B2T; exitRoad = vi; }
        else if (m_dir == Direction::R2L) { exitDir = Direction::T2B; exitRoad = vi; }
        else if (m_dir == Direction::T2B) { exitDir = Direction::L2R; exitRoad = hi; }
        else                              { exitDir = Direction::R2L; exitRoad = hi; }
        break;
    }
    const QPointF rc = ringCenterOf(vi, hi);
    const double th3 = ringMergeTheta(m_dir, laneOffset(m_dir));
    const double thx = ringDivergeTheta(exitDir, laneOffset(exitDir));
    const QPointF P3 = rc + QPointF(std::cos(th3), std::sin(th3)) * kRingR;
    const PathPose from{ center(), dirHeading(m_dir) }, to{ P3, ringHeading(th3) };
    const double chord = std::hypot(P3.x() - from.c.x(), P3.y() - from.c.y());

    m_man = std::make_unique<Maneuver>();
    m_man->kind = Maneuver::Kind::RingEntry;
    m_man->path = SampledPath::bezier(from, to, 0.45 * chord, 0.45 * chord);
    m_man->vi = vi; m_man->hi = hi;
    m_man->ringTheta = th3;
    m_man->travelNeeded = Rb::mod2pi(th3 - thx);
    m_man->exitDir = exitDir; m_man->exitRoad = exitRoad;
    m_c = from.c; m_h = from.h;
    m_pos = boundingRect().topLeft();
    m_target  = std::min(kRingSpeed, m_max);
    m_decided = true;
    m_zoneID  = vi * kNumRoads + hi;
    clearPending();
}

QPointF Car::ringExitCenter() const {
    int vi = ringVI(), hi = ringHI();
    if (vi < 0) return center();
    const Direction e = ringExitDir();
    const double w = laneOffset(e), h = dirHeading(e);
    const QPointF rc = ringCenterOf(vi, hi);
    return rc + unit(h) * kRbLaneU(w) + QPointF(-std::sin(h), std::cos(h)) * w;
}
QPointF Car::ringExitPos() const {
    const QPointF c = ringExitCenter();
    const bool poziomo = dirIsHoriz(ringExitDir());
    return c - QPointF(poziomo ? kCarL / 2 : kCarW / 2, poziomo ? kCarW / 2 : kCarL / 2);
}
Direction Car::ringExitDir() const {
    if (m_rb) return m_rb->exitDir;
    if (m_man) return m_man->kind == Maneuver::Kind::RingEntry ? m_man->exitDir : m_man->endDir;
    return m_dir;
}
int Car::ringExitRoad() const {
    if (m_rb) return m_rb->exitRoad;
    if (m_man) return m_man->kind == Maneuver::Kind::RingEntry ? m_man->exitRoad : m_man->endRoad;
    return m_road;
}

void Car::startRingExitManeuver() {
    const QPointF P3 = ringExitCenter();
    const Direction e = m_rb->exitDir;
    const PathPose from{ m_c, m_h }, to{ P3, dirHeading(e) };
    const double chord = std::hypot(P3.x() - m_c.x(), P3.y() - m_c.y());
    auto man = std::make_unique<Maneuver>();
    man->kind = Maneuver::Kind::RingExit;
    man->path = SampledPath::bezier(from, to, 0.45 * chord, 0.45 * chord);
    man->vi = m_rb->vi; man->hi = m_rb->hi;
    man->endDir = e; man->endRoad = m_rb->exitRoad;
    m_rb.reset();
    m_man = std::move(man);
}

void Car::doExitRoundabout() { if (m_rb) startRingExitManeuver(); }
