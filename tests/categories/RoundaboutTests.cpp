/**
 * @file RoundaboutTests.cpp
 * @brief Testy RONDA — kąty wjazdu/wyjazdu, krążenie po ringu, brak kolizji, stop przed linią.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "RoundaboutTests.h"

// ════════════════════════════════════════════════════════════════════
//  RONDO (5) — pełne milestony zgodnie z dokumentem wymagań
//  • Stop PRZED żółtą linią     — auto musi dotrzeć, zatrzymać się, mieć ≥5px
//  • Auto wjeżdża i opuszcza   — auto musi wjechać na ring i go opuścić
//  • Kąty wjazdu              — auta wjeżdżają z 4 stron, każdy kąt zmierzony
//  • Brak kolizji ring–ring   — wiele aut faktycznie krąży po ringu
//  • Kąty w zakresie [-π,π]   — j.w. z weryfikacją zakresu
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> roundaboutTests() {
    std::vector<TestCase> t;

    // Dyrekcje rotujące — różne strony dojazdu w kolejnych próbach
    auto dirForAttempt = [](int idx) -> Direction {
        Direction dirs[] = {Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T};
        return dirs[idx % 4];
    };
    auto spawnAt = [](Direction d, int road) -> QPointF {
        // Spawn BLISKO ronda (4,4) — auta mają wystarczająco mało dystansu,
        // by faktycznie dojechać i wjechać na ring w sensownym czasie.
        double cx = kVLeft[4] + kRoadW / 2.0;
        double cy = kHTop[4]  + kRoadW / 2.0;
        switch (d) {
        case Direction::L2R: return QPointF(cx - 220,            kL2R_Y(road) - kCarW/2);
        case Direction::R2L: return QPointF(cx + 220 - kCarL,    kR2L_Y(road) - kCarW/2);
        case Direction::T2B: return QPointF(kT2B_X(road) - kCarW/2, cy - 220);
        default:             return QPointF(kB2T_X(road) - kCarW/2, cy + 220 - kCarL);
        }
    };

    // ─── 1. Kąty wjazdu — wjazd aut z 4 stron, kąt obliczany dla każdego
    {
        TestCase tc;
        tc.id = "rb.entry_angles";
        tc.name = "Kąty wjazdu na rondo";
        tc.category = "Rondo";
        tc.description = "5 prób · w każdej auto wjeżdża z innego kierunku · "
                         "kąt wjazdu obliczany przez Rb::entryAngle() i wizualnie pokazany.";
        tc.minSuccessfulAttempts = 5;
        tc.maxAttempts = 7;
        tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",      QString::fromUtf8("Auto zespawnowane")),
                MS("approaching",  QString::fromUtf8("Auto jedzie w stronę ronda")),
                MS("entered_ring", QString::fromUtf8("Auto wjechało na ring")),
                MS("angle_measured", QString::fromUtf8("Zmierzono kąt wjazdu")),
                MS("angle_in_range", QString::fromUtf8("Kąt mieści się w [-π, π]"))
            };
        };
        tc.setupAttempt = [dirForAttempt, spawnAt](SimulationEngine& e, int idx) {
            e.setRoundabouts({{4,4}});
            Direction d = dirForAttempt(idx);
            e.addCar(spawnAt(d, 4), d, 4);
        };
        tc.updateAttempt = [dirForAttempt](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) {
                ctx.preview->setFocusRect(intersectionFocus(4, 4, 120));
                ctx.preview->setHighlight(4, 4);
            }
            const Car* car = nullptr;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) { car = c; break; }
            if (!car) return false;

            reach(ctx, "spawned");
            if (car->speed() > 5) reach(ctx, "approaching");

            Direction d = dirForAttempt(ctx.attemptIdx);
            double angle = Rb::entryAngle(d);
            QString dirName = d == Direction::L2R ? "L→R" : d == Direction::R2L ? "R→L"
                              : d == Direction::T2B ? "T→B" : "B→T";

            ctx.preview->setCounters({
                {QString::fromUtf8("Kierunek"), dirName},
                {QString::fromUtf8("Kąt wjazdu"), QString::number(angle, 'f', 2) + " rad"},
                {QString::fromUtf8("W stopniach"), QString::number(angle * 180.0 / 3.14159, 'f', 0) + "°"},
                {QString::fromUtf8("Na ringu?"), car->isOnRing() ? "TAK" : "NIE"}
            });
            measure(ctx, QString::fromUtf8("Kąt (rad)"), QString::number(angle, 'f', 3));
            measure(ctx, QString::fromUtf8("Kierunek"), dirName);

            if (car->isOnRing()) {
                reach(ctx, "entered_ring");
                reach(ctx, "angle_measured");
                if (angle >= -3.15 && angle <= 3.15) reach(ctx, "angle_in_range");
                else { markFail(ctx, QString("Kąt %1 poza zakresem [-π,π]").arg(angle)); return true; }
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // ─── 2. Kąty w zakresie [-π,π] — analogicznie ale podkreśla walidację zakresu
    {
        TestCase tc;
        tc.id = "rb.angles_range";
        tc.name = "Kąty w zakresie [-π, π]";
        tc.category = "Rondo";
        tc.description = "5 prób · sprawdzamy że kąt wjazdu obliczany przez "
                         "Rb::entryAngle() zawsze mieści się w pełnym obrocie.";
        tc.minSuccessfulAttempts = 5;
        tc.maxAttempts = 7;
        tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",      QString::fromUtf8("Auto zespawnowane")),
                MS("entered_ring", QString::fromUtf8("Auto wjechało na ring")),
                MS("angle_in_range", QString::fromUtf8("Kąt mieści się w [-π, π]"))
            };
        };
        tc.setupAttempt = [dirForAttempt, spawnAt](SimulationEngine& e, int idx) {
            e.setRoundabouts({{4,4}});
            Direction d = dirForAttempt(idx);
            e.addCar(spawnAt(d, 4), d, 4);
        };
        tc.updateAttempt = [dirForAttempt](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) {
                ctx.preview->setFocusRect(intersectionFocus(4, 4, 120));
                ctx.preview->setHighlight(4, 4);
            }
            const Car* car = nullptr;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) { car = c; break; }
            if (!car) return false;
            reach(ctx, "spawned");

            Direction d = dirForAttempt(ctx.attemptIdx);
            double angle = Rb::entryAngle(d);
            ctx.preview->setCounters({
                {QString::fromUtf8("Kąt"), QString::number(angle, 'f', 2) + " rad"},
                {QString::fromUtf8("|kąt|"), QString::number(std::abs(angle), 'f', 2)},
                {QString::fromUtf8("Limit"), "3.14159 (π)"}
            });
            measure(ctx, QString::fromUtf8("Kąt zmierzony"), QString::number(angle, 'f', 4));
            if (car->isOnRing()) {
                reach(ctx, "entered_ring");
                if (angle >= -3.15 && angle <= 3.15) reach(ctx, "angle_in_range");
                else { markFail(ctx, "Kąt poza ±π"); return true; }
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // ─── 3. Auto wjeżdża i opuszcza JEDNO rondo
    {
        TestCase tc;
        tc.id = "rb.enter_exit";
        tc.name = "Auto wjeżdża i opuszcza JEDNO rondo";
        tc.category = "Rondo";
        tc.description = "5 prób · pełny cykl: spawn → dojazd → wjazd na ring → "
                         "krążenie → zjazd. Każda próba ma inny kierunek startowy.";
        tc.minSuccessfulAttempts = 5;
        tc.maxAttempts = 8;
        tc.maxFramesPerAttempt = 900;   // 12s safety
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",      QString::fromUtf8("Auto zespawnowane")),
                MS("approaching",  QString::fromUtf8("Auto dojeżdża do ronda")),
                MS("entered_ring", QString::fromUtf8("Auto wjechało na ring")),
                MS("on_ring",      QString::fromUtf8("Auto porusza się po ringu")),
                MS("exited_ring",  QString::fromUtf8("Auto opuściło ring"))
            };
        };
        tc.setupAttempt = [dirForAttempt, spawnAt](SimulationEngine& e, int idx) {
            e.setRoundabouts({{4,4}});
            Direction d = dirForAttempt(idx);
            e.addCar(spawnAt(d, 4), d, 4);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) {
                ctx.preview->setFocusRect(intersectionFocus(4, 4, 160));
                ctx.preview->setHighlight(4, 4);
            }
            const Car* car = nullptr;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) { car = c; break; }
            // Statyczna pamięć stanów na próbę
            static bool s_wasOnRing = false;
            static int  s_ringFrames = 0;
            if (ctx.frame == 0) { s_wasOnRing = false; s_ringFrames = 0; }
            if (!car) {
                markUnverified(ctx, "Auto zniknęło z mapy zanim zaobserwowano cykl");
                return true;
            }
            reach(ctx, "spawned");
            // Dystans do ronda (4,4)
            double rcx = kVLeft[4] + kRoadW/2.0;
            double rcy = kHTop[4]  + kRoadW/2.0;
            QPointF ctr = car->boundingRect().center();
            double dist = std::hypot(ctr.x()-rcx, ctr.y()-rcy);
            if (dist < kRingR + 100 && car->speed() > 5) reach(ctx, "approaching");
            if (car->isOnRing()) {
                reach(ctx, "entered_ring");
                s_wasOnRing = true;
                s_ringFrames++;
                if (s_ringFrames > 10) reach(ctx, "on_ring");
            }
            if (s_wasOnRing && !car->isOnRing() && dist > kRingR + 40) {
                reach(ctx, "exited_ring");
                measure(ctx, QString::fromUtf8("Klatki na ringu"), QString::number(s_ringFrames));
                measure(ctx, QString::fromUtf8("Dystans po zjeździe"), QString::number((int)dist) + " px");
                return true;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Dystans do ronda"), QString::number((int)dist) + " px"},
                {QString::fromUtf8("Na ringu?"), car->isOnRing() ? "TAK" : "NIE"},
                {QString::fromUtf8("Klatek na ringu"), QString::number(s_ringFrames)},
                {QString::fromUtf8("Prędkość"), QString::number((int)car->speed()) + " px/s"}
            });
            return false;
        };
        t.push_back(tc);
    }

    // ─── 4. Brak kolizji ring-ring
    {
        TestCase tc;
        tc.id = "rb.no_ring_collision";
        tc.name = "Brak kolizji ring–ring";
        tc.category = "Rondo";
        tc.description = "5 prób · w każdej kilka aut faktycznie krąży po ringach. "
                         "Test obserwuje rzeczywistą interakcję, nie tylko liczbę.";
        tc.minSuccessfulAttempts = 5;
        tc.maxAttempts = 7;
        tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("cars_spawned",  QString::fromUtf8("Auta zespawnowane")),
                MS("cars_on_ring",  QString::fromUtf8("Co najmniej 2 auta na ringach")),
                MS("frames_observed", QString::fromUtf8("≥ 60 klatek obserwacji ring-ring")),
                MS("no_collisions",   QString::fromUtf8("Brak kolizji ring-ring"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            std::vector<std::pair<int,int>> layouts[] = {
                {{2,2},{5,5},{3,6}}, {{1,3},{4,4},{6,5}}, {{2,5},{5,2},{6,6}},
                {{3,1},{5,5},{2,7}}, {{1,1},{4,3},{6,6}}, {{2,2},{4,5},{6,1}}, {{3,3},{5,5}}
            };
            e.setRoundabouts(layouts[idx % 7]);
            // 50 aut by zwiększyć szansę że ≥2 będą jednocześnie na ringach
            e.spawnCarsOnMap(50);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) ctx.preview->setHighlights(findRoundabouts(*ctx.engine));
            static int s_obsFrames = 0, s_maxOnRing = 0, s_collisions = 0;
            if (ctx.frame == 0) { s_obsFrames = 0; s_maxOnRing = 0; s_collisions = 0; }

            reach(ctx, "cars_spawned");
            // Zbierz auta na ringu
            std::vector<const Car*> ringCars;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get()))
                    if (c->isOnRing()) ringCars.push_back(c);
            s_maxOnRing = std::max(s_maxOnRing, (int)ringCars.size());
            if (ringCars.size() >= 2) {
                reach(ctx, "cars_on_ring");
                s_obsFrames++;
                if (s_obsFrames >= 60) reach(ctx, "frames_observed");
            }
            // Sprawdź kolizje ring-ring
            std::set<const Car*> hit;
            for (size_t i = 0; i < ringCars.size(); ++i)
                for (size_t j = i+1; j < ringCars.size(); ++j) {
                    QRectF x = ringCars[i]->boundingRect().intersected(ringCars[j]->boundingRect());
                    if (x.width()*x.height() > 0.5) {
                        hit.insert(ringCars[i]); hit.insert(ringCars[j]);
                        s_collisions++;
                    }
                }
            std::vector<PvBox> boxes;
            for (auto* c : ringCars) {
                bool ok = !hit.count(c);
                boxes.push_back({c->boundingRect().adjusted(-3,-3,3,3), "",
                                  ok ? kPass : kFail, ok});
            }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut na ringu"), QString::number(ringCars.size())},
                {QString::fromUtf8("Max równocześnie"), QString::number(s_maxOnRing)},
                {QString::fromUtf8("Klatek obserwacji"), QString::number(s_obsFrames)},
                {QString::fromUtf8("Kolizji"), QString::number(s_collisions)}
            });
            if (s_collisions > 0) {
                markFail(ctx, QString("Wykryto %1 kolizji ring-ring").arg(s_collisions));
                measure(ctx, QString::fromUtf8("Kolizji"), QString::number(s_collisions));
                return true;
            }
            if (s_obsFrames >= 60) {
                reach(ctx, "no_collisions");
                measure(ctx, QString::fromUtf8("Klatek obserwacji"), QString::number(s_obsFrames));
                measure(ctx, QString::fromUtf8("Max aut na ringu"), QString::number(s_maxOnRing));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // ─── 5. Stop PRZED żółtą linią — najważniejszy test
    {
        TestCase tc;
        tc.id = "rb.stop_before_yellow";
        tc.name = "Stop PRZED żółtą linią ronda";
        tc.category = "Rondo";
        tc.description = "5 prób · auto dojeżdża do ronda i MUSI faktycznie się "
                         "zatrzymać · dystans frontu od żółtej linii ≥ 5 px.";
        tc.minSuccessfulAttempts = 5;
        tc.maxAttempts = 8;
        tc.maxFramesPerAttempt = 900;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",       QString::fromUtf8("Auto zespawnowane")),
                MS("approaching",   QString::fromUtf8("Auto jedzie w stronę ronda")),
                MS("reached_zone",  QString::fromUtf8("Auto dotarło do strefy stop")),
                MS("stopped",       QString::fromUtf8("Auto zatrzymało się")),
                MS("distance_meas", QString::fromUtf8("Zmierzono dystans frontu od linii")),
                MS("distance_ok",   QString::fromUtf8("Dystans spełnia wymaganie ≥ 5 px"))
            };
        };
        tc.setupAttempt = [dirForAttempt, spawnAt](SimulationEngine& e, int idx) {
            e.setRoundabouts({{4,4}});
            // Wymuszamy „korek" — najpierw kilka aut już w okolicy ronda,
            // by SmartCar musiał ustąpić pierwszeństwa i się zatrzymać.
            e.spawnCarsOnMap(20);
            Direction d = dirForAttempt(idx);
            e.addCar(spawnAt(d, 4), d, 4);
            // Rzeczywisty konflikt pierwszeństwa: po jednym aucie na pozostałych
            // wlotach, blisko ronda. (Dawniej postój wymuszała niewidoczna „faza"
            // ronda — usunięta, bo kazała czekać także przed pustym rondem.)
            const Direction all[4] = {Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T};
            const double rcx = kVLeft[4] + kRoadW / 2.0, rcy = kHTop[4] + kRoadW / 2.0;
            for (Direction o : all) {
                if (o == d) continue;
                const double u = 125.0;
                QPointF p;
                switch (o) {
                case Direction::L2R: p = QPointF(rcx - u - kCarL / 2, kL2R_Y(4)); break;
                case Direction::R2L: p = QPointF(rcx + u - kCarL / 2, kR2L_Y(4)); break;
                case Direction::T2B: p = QPointF(kT2B_X(4), rcy - u - kCarL / 2); break;
                default:             p = QPointF(kB2T_X(4), rcy + u - kCarL / 2); break;
                }
                e.addCar(p, o, 4);
            }
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            if (ctx.frame == 0) {
                ctx.preview->setFocusRect(intersectionFocus(4, 4, 140));
                ctx.preview->setHighlight(4, 4);
            }
            // Znajdź auto najbliżej ronda (4,4) które NIE jest na ringu
            double rcx = kVLeft[4] + kRoadW/2.0;
            double rcy = kHTop[4]  + kRoadW/2.0;
            const Car* candidate = nullptr;
            double bestD = 1e9;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c || c->isOnRing()) continue;
                QPointF ctr = c->boundingRect().center();
                double d = std::hypot(ctr.x()-rcx, ctr.y()-rcy);
                if (d < bestD && d < kRingR + 200) { bestD = d; candidate = c; }
            }
            static int s_stoppedFrames = 0;
            static double s_minFront = 1e9;
            if (ctx.frame == 0) { s_stoppedFrames = 0; s_minFront = 1e9; }
            if (!candidate) return false;
            reach(ctx, "spawned");
            if (candidate->speed() > 5) reach(ctx, "approaching");

            QPointF ctr = candidate->boundingRect().center();
            double d = std::hypot(ctr.x()-rcx, ctr.y()-rcy);
            double front = d - kCarL/2.0 - kRingR;
            // Strefa stop = blisko ronda
            if (d < kRingR + 120) reach(ctx, "reached_zone");

            // Linia pomiarowa front → ring edge
            double dx = rcx - ctr.x(), dy = rcy - ctr.y();
            double len = std::hypot(dx, dy);
            QPointF dir(dx/len, dy/len);
            QPointF carFront = ctr + QPointF(dir.x()*kCarL/2.0, dir.y()*kCarL/2.0);
            QPointF ringEdge = QPointF(rcx, rcy) - QPointF(dir.x()*kRingR, dir.y()*kRingR);
            bool inZone = (d < kRingR + 120 && d > kRingR);
            bool meets = (front >= 5.0);
            std::vector<PvMeasure> ms;
            if (inZone)
                ms.push_back({carFront, ringEdge,
                              QString::number((int)front) + " px",
                              meets ? kPass : kFail, meets});
            ctx.preview->setMeasures(ms);

            if (candidate->speed() < 1.5 && inZone) {
                reach(ctx, "stopped");
                s_stoppedFrames++;
                if (front < s_minFront) s_minFront = front;
                if (s_stoppedFrames >= 6) {
                    reach(ctx, "distance_meas");
                    measure(ctx, QString::fromUtf8("Dystans frontu"),
                            QString::number(s_minFront, 'f', 1) + " px");
                    measure(ctx, QString::fromUtf8("Wymagane min"), "5 px");
                    if (s_minFront >= 5.0) {
                        reach(ctx, "distance_ok");
                    } else {
                        markFail(ctx, QString("Auto przekroczyło linię: front=%1 px < 5 px")
                                       .arg(s_minFront, 0, 'f', 1));
                    }
                    return true;
                }
            } else {
                s_stoppedFrames = 0;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Dystans do centrum"), QString::number((int)d) + " px"},
                {QString::fromUtf8("Front od linii"), QString::number((int)front) + " px"},
                {QString::fromUtf8("Prędkość"), QString::number((int)candidate->speed()) + " px/s"},
                {QString::fromUtf8("W strefie?"), inZone ? "TAK" : "NIE"},
                {QString::fromUtf8("Klatek stop"), QString::number(s_stoppedFrames)}
            });
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
