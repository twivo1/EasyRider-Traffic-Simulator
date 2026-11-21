/**
 * @file FlowTests.cpp
 * @brief Testy PŁYNNOŚCI — brak skoków pozycji, płynne skrzyżowania/ronda, hamowanie, ciągłość ruchu.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "FlowTests.h"

// ════════════════════════════════════════════════════════════════════
//  PŁYNNOŚĆ (5) — każda próba obserwuje jedno auto/serię aut
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> flowTests() {
    std::vector<TestCase> t;

    // 1. Brak skoków — 5 prób · każde auto obserwowane 180 klatek
    {
        TestCase tc;
        tc.id = "flow.no_jumps";
        tc.name = "Brak skoków pozycji między klatkami";
        tc.category = "Płynność";
        tc.description = "5 prób · w każdej śledzimy 1 auto przez 180 klatek "
                         "i mierzymy max przemieszczenie/klatkę. Musi być ≤ kMaxSpeed × dt × 1.5.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",   QString::fromUtf8("Auto zespawnowane")),
                MS("observed_60", QString::fromUtf8("Obserwowano przez ≥ 60 klatek")),
                MS("observed_180", QString::fromUtf8("Obserwowano przez ≥ 180 klatek")),
                MS("delta_ok",  QString::fromUtf8("max Δ/klatkę ≤ limit"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.spawnCarsOnMap(8);
            int road = idx % kNumRoads;
            auto cps = buildCheckpoints();
            for (int i = 0; i < (int)cps.size(); ++i)
                if (cps[i].entryDir == Direction::L2R && cps[i].roadIdx == road) {
                    e.spawnSmartCar(cps[i].spawnPos, cps[i].entryDir, cps[i].roadIdx);
                    break;
                }
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static QPolygonF s_trail;
            static QPointF   s_last;
            static double    s_maxD = 0;
            static int       s_obs  = 0;
            if (ctx.frame == 0) { s_trail.clear(); s_last = QPointF(); s_maxD = 0; s_obs = 0; }

            const Car* sc = findSmartCar(*ctx.engine);
            if (!sc) {
                markUnverified(ctx, "SmartCar nie istnieje");
                return true;
            }
            reach(ctx, "spawned");
            QPointF p = sc->boundingRect().center();
            if (s_obs > 0) {
                double d = std::hypot(p.x()-s_last.x(), p.y()-s_last.y());
                if (d > s_maxD) s_maxD = d;
            }
            s_last = p; s_obs++;
            if (ctx.frame % 2 == 0) { s_trail << p; ctx.preview->setTrail(s_trail); }
            double limit = kMaxSpeed / 60.0 * 1.5;
            ctx.preview->setCounters({
                {QString::fromUtf8("Klatki obs."), QString::number(s_obs)},
                {QString::fromUtf8("Max Δ/klatkę"), QString::number(s_maxD, 'f', 2) + " px"},
                {QString::fromUtf8("Limit Δ"), QString::number(limit, 'f', 2) + " px"}
            });
            if (s_obs >= 60)  reach(ctx, "observed_60");
            if (s_obs >= 180) reach(ctx, "observed_180");
            if (s_obs >= 180) {
                if (s_maxD <= limit) reach(ctx, "delta_ok");
                else { markFail(ctx, QString("max Δ = %1 > %2").arg(s_maxD,0,'f',2).arg(limit,0,'f',2)); return true; }
                measure(ctx, QString::fromUtf8("Max Δ/klatkę"), QString::number(s_maxD,'f',2) + " px");
                measure(ctx, QString::fromUtf8("Limit"), QString::number(limit,'f',2) + " px");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 2. Płynne przejście przez skrzyżowanie
    {
        TestCase tc;
        tc.id = "flow.smooth_intersection";
        tc.name = "Płynne przejście przez skrzyżowanie";
        tc.category = "Płynność";
        tc.description = "5 prób · auto przejeżdża przez wybrane skrzyżowanie · "
                         "obserwujemy brak kolizji i ciągłość.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",     QString::fromUtf8("Auto zespawnowane")),
                MS("approaching", QString::fromUtf8("Auto jedzie do skrzyżowania")),
                MS("crossed",     QString::fromUtf8("Auto przejechało skrzyżowanie")),
                MS("no_collision", QString::fromUtf8("Brak kolizji podczas przejazdu"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.setRoundabouts({{0,0},{7,7},{1,7},{7,1}});
            int hi = 3 + (idx % 3);
            // Spawn blisko skrzyżowania (3,hi) — wystarczająco daleko, by miały
            // czas na rozpędzenie, ale wystarczająco blisko by zdążyć w 15s.
            double cx = kVLeft[3] + kRoadW / 2.0;
            e.spawnSmartCar(QPointF(cx - 200, kL2R_Y(hi) - kCarW/2), Direction::L2R, hi);
            // Test sprawdza przejazd NA WPROST przez (3,hi) (kamień milowy „crossed"
            // to x > prawa krawędź strefy). Bez nawigacji intencja jest losowa; dawniej
            // pierwsze skrzyżowanie było zawsze „na wprost" wskutek błędu m_decided=true.
            for (auto& o : e.objects())
                if (auto* c = dynamic_cast<Car*>(o.get())) if (c->isSmartCar()) c->setFixedIntent(0);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) ctx.preview->setHighlight(3, 3);
            const Car* sc = findSmartCar(*ctx.engine);
            if (!sc) { markUnverified(ctx, "SmartCar zniknął"); return true; }
            reach(ctx, "spawned");
            if (sc->speed() > 5) reach(ctx, "approaching");
            int coll = countCollisions(*ctx.engine);
            QPointF p = sc->boundingRect().center();
            if (p.x() > kVLeft[3] + kRoadW) reach(ctx, "crossed");
            if (coll > 0) {
                markFail(ctx, QString("Kolizja podczas przejazdu (%1)").arg(coll));
                return true;
            }
            if (isReached(*ctx.result, "crossed")) reach(ctx, "no_collision");
            ctx.preview->setCounters({
                {QString::fromUtf8("Pozycja X"), QString::number((int)p.x())},
                {QString::fromUtf8("Kolizje"), QString::number(coll)},
                {QString::fromUtf8("Prędkość"), QString::number((int)sc->speed())}
            });
            return false;
        };
        t.push_back(tc);
    }

    // 3. Płynne krążenie po rondzie
    {
        TestCase tc;
        tc.id = "flow.smooth_roundabout";
        tc.name = "Płynne krążenie po rondzie";
        tc.category = "Płynność";
        tc.description = "5 prób · auto wjeżdża na rondo, krąży i zjeżdża · "
                         "trail pokazuje płynność, brak kolizji.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",      QString::fromUtf8("Auto zespawnowane")),
                MS("entered_ring", QString::fromUtf8("Auto wjechało na ring")),
                MS("on_ring_60",   QString::fromUtf8("Płynnie krążyło ≥ 60 klatek")),
                MS("exited_ring",  QString::fromUtf8("Auto opuściło ring")),
                MS("no_collision", QString::fromUtf8("Brak kolizji"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.setRoundabouts({{4,4}});
            Direction dirs[] = {Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T};
            Direction d = dirs[idx % 4];
            // Spawn 220 px od centrum ronda — auto zdąży dojechać i wykonać cykl
            double cx = kVLeft[4] + kRoadW / 2.0;
            double cy = kHTop[4]  + kRoadW / 2.0;
            QPointF p;
            switch (d) {
            case Direction::L2R: p = QPointF(cx - 220,         kL2R_Y(4) - kCarW/2); break;
            case Direction::R2L: p = QPointF(cx + 220 - kCarL, kR2L_Y(4) - kCarW/2); break;
            case Direction::T2B: p = QPointF(kT2B_X(4) - kCarW/2, cy - 220); break;
            default:             p = QPointF(kB2T_X(4) - kCarW/2, cy + 220 - kCarL); break;
            }
            e.spawnSmartCar(p, d, 4);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static QPolygonF s_trail;
            static int s_onRingFrames = 0;
            static bool s_wasOnRing = false;
            if (ctx.frame == 0) {
                s_trail.clear(); s_onRingFrames = 0; s_wasOnRing = false;
                ctx.preview->setFocusRect(intersectionFocus(4, 4, 120));
                ctx.preview->setHighlight(4, 4);
            }
            const Car* sc = findSmartCar(*ctx.engine);
            if (!sc) { markUnverified(ctx, "SmartCar zniknął"); return true; }
            reach(ctx, "spawned");
            if (ctx.frame % 2 == 0) { s_trail << sc->boundingRect().center(); ctx.preview->setTrail(s_trail); }
            if (sc->isOnRing()) {
                reach(ctx, "entered_ring");
                s_wasOnRing = true;
                s_onRingFrames++;
                if (s_onRingFrames >= 60) reach(ctx, "on_ring_60");
            }
            if (s_wasOnRing && !sc->isOnRing()) reach(ctx, "exited_ring");
            int coll = countCollisions(*ctx.engine);
            if (coll == 0 && isReached(*ctx.result, "exited_ring")) reach(ctx, "no_collision");
            ctx.preview->setCounters({
                {QString::fromUtf8("Na ringu?"), sc->isOnRing() ? "TAK" : "NIE"},
                {QString::fromUtf8("Klatek na ringu"), QString::number(s_onRingFrames)},
                {QString::fromUtf8("Kolizje"), QString::number(coll)}
            });
            if (coll > 0) { markFail(ctx, "Kolizja"); return true; }
            return isReached(*ctx.result, "no_collision");
        };
        t.push_back(tc);
    }

    // 4. Płynne hamowanie/przyspieszanie — wymaga obserwacji ≥ 5 cykli zmian
    {
        TestCase tc;
        tc.id = "flow.brake_accel";
        tc.name = "Płynne hamowanie i przyspieszanie";
        tc.category = "Płynność";
        tc.description = "5 prób · obserwujemy SmartCar, w każdej próbie musi "
                         "wystąpić ≥ 2 cykle hamowania i ≥ 2 ruszania.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",   QString::fromUtf8("Auto zespawnowane")),
                MS("braked_2",  QString::fromUtf8("≥ 2 cykle hamowania")),
                MS("accel_2",   QString::fromUtf8("≥ 2 cykle ruszania")),
                MS("no_jump",   QString::fromUtf8("Brak skokowej zmiany prędkości"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            // Dużo aut + ronda → SmartCar musi się zatrzymywać/ruszać
            e.setRoundabouts({{2,2},{5,5},{3,6},{6,3}});
            e.spawnCarsOnMap(40);
            int hi = idx % kNumRoads;
            e.spawnSmartCar(QPointF(20, kL2R_Y(hi) - kCarW/2), Direction::L2R, hi);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static double s_lastV = 0;
            static int s_brakes = 0, s_accels = 0;
            static int s_stateState = 0;   // 0=neutral, -1=braking, +1=accel
            static double s_maxJump = 0;
            if (ctx.frame == 0) { s_lastV = 0; s_brakes = s_accels = 0; s_stateState = 0; s_maxJump = 0; }
            const Car* sc = findSmartCar(*ctx.engine);
            if (!sc) { markUnverified(ctx, "SmartCar zniknął"); return true; }
            reach(ctx, "spawned");
            double v = sc->speed();
            double a = (v - s_lastV) * 60.0;
            s_maxJump = std::max(s_maxJump, std::abs(v - s_lastV));
            s_lastV = v;
            if (a < -30 && s_stateState != -1) { s_brakes++; s_stateState = -1; }
            else if (a > 30 && s_stateState != +1) { s_accels++; s_stateState = +1; }
            if (s_brakes >= 2) reach(ctx, "braked_2");
            if (s_accels >= 2) reach(ctx, "accel_2");
            if (s_maxJump < kMaxSpeed / 60.0 * 1.5) reach(ctx, "no_jump");
            ctx.preview->setCounters({
                {QString::fromUtf8("Prędkość"), QString::number((int)v) + " px/s"},
                {QString::fromUtf8("Przyspieszenie"), QString::number((int)a) + " px/s²"},
                {QString::fromUtf8("Hamowań"), QString::number(s_brakes)},
                {QString::fromUtf8("Ruszań"), QString::number(s_accels)}
            });
            if (isReached(*ctx.result, "braked_2") &&
                isReached(*ctx.result, "accel_2") &&
                isReached(*ctx.result, "no_jump")) {
                measure(ctx, QString::fromUtf8("Hamowań"), QString::number(s_brakes));
                measure(ctx, QString::fromUtf8("Ruszań"), QString::number(s_accels));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 5. Ciągłość ruchu — ≥ 10 aut obserwowanych
    {
        TestCase tc;
        tc.id = "flow.multi_car_continuity";
        tc.name = "Ciągłość ruchu wielu aut";
        tc.category = "Płynność";
        tc.description = "5 prób · w każdej obserwujemy ≥ 10 aut w ruchu, "
                         "0 kolizji przez ≥ 180 klatek.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 360;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_50",   QString::fromUtf8("Zespawnowano 50 aut")),
                MS("moving_10",    QString::fromUtf8("≥ 10 aut w ruchu")),
                MS("observed_180", QString::fromUtf8("Obserwacja przez ≥ 180 klatek")),
                MS("no_collisions", QString::fromUtf8("Zero kolizji przez cały okres"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(50); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0; if (ctx.frame == 0) s_obs = 0;
            int moving = 0, n = 0;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) {
                    n++;
                    if (c->speed() > 10) moving++;
                }
            if (n >= 50) reach(ctx, "spawned_50");
            if (moving >= 10) reach(ctx, "moving_10");
            int coll = countCollisions(*ctx.engine);
            if (coll > 0) {
                markFail(ctx, QString("Kolizja (%1) w trakcie obserwacji").arg(coll));
                return true;
            }
            s_obs++;
            if (s_obs >= 180) {
                reach(ctx, "observed_180");
                reach(ctx, "no_collisions");
                measure(ctx, QString::fromUtf8("Aut żywych"), QString::number(n));
                measure(ctx, QString::fromUtf8("Aut w ruchu"), QString::number(moving));
                measure(ctx, QString::fromUtf8("Klatek obserwacji"), QString::number(s_obs));
                return true;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut żywych"), QString::number(n)},
                {QString::fromUtf8("W ruchu"), QString::number(moving)},
                {QString::fromUtf8("Klatek obs."), QString::number(s_obs)},
                {QString::fromUtf8("Kolizje"), QString::number(coll)}
            });
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
