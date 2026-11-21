/**
 * @file CarTests.cpp
 * @brief Testy SAMOCHODU — ruch w czterech kierunkach, stop(), limit prędkości, poprawność AABB.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "CarTests.h"

// ════════════════════════════════════════════════════════════════════
//  SAMOCHÓD (7) — obserwacja ruchu, hamowania, AABB
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> carTests() {
    std::vector<TestCase> t;

    auto dirName = [](Direction d) -> QString {
        return d == Direction::L2R ? "L→R" : d == Direction::R2L ? "R→L"
             : d == Direction::T2B ? "T→B" : "B→T";
    };
    auto spawnDir = [](Direction d, int road) -> QPointF {
        switch (d) {
        case Direction::L2R: return QPointF(40, kL2R_Y(road) - kCarW/2);
        case Direction::R2L: return QPointF(kCanvasW - 80, kR2L_Y(road) - kCarW/2);
        case Direction::T2B: return QPointF(kT2B_X(road) - kCarW/2, 40);
        default:             return QPointF(kB2T_X(road) - kCarW/2, kCanvasH - 80);
        }
    };

    auto makeDirTest = [&](Direction dir, const QString& arrow, const QString& axis) {
        TestCase tc;
        tc.id = QString("car.dir_") + arrow;
        tc.name = QString("Ruch %1 (kierunek %2)").arg(arrow).arg(axis);
        tc.category = "Samochód";
        tc.description = QString("5 prób · w każdej spawn auta %1 i obserwacja "
                                 "rzeczywistego ruchu po ≥ 30 klatkach.").arg(arrow);
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [arrow](int) {
            return std::vector<Milestone>{
                MS("spawned",   QString::fromUtf8("Auto zespawnowane")),
                MS("moving",    QString::fromUtf8("Auto faktycznie się porusza")),
                MS("direction_ok", QString::fromUtf8("Kierunek zgodny: %1").arg(arrow)),
                MS("displaced", QString::fromUtf8("Auto przemieściło się ≥ 40 px"))
            };
        };
        tc.setupAttempt = [dir, spawnDir](SimulationEngine& e, int idx) {
            e.setRoundabouts({});
            int road = idx % kNumRoads;
            e.addCar(spawnDir(dir, road), dir, road);
        };
        tc.updateAttempt = [dir, dirName, arrow](AttemptCtx& ctx) -> bool {
            static QPointF s_initial;
            if (ctx.frame == 0) s_initial = QPointF(-1, -1);
            const Car* car = nullptr;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) { car = c; break; }
            if (!car) return false;
            reach(ctx, "spawned");
            QPointF p = car->boundingRect().center();
            if (s_initial.x() < 0) s_initial = p;
            if (car->speed() > 5) reach(ctx, "moving");
            if (car->dir() == dir) reach(ctx, "direction_ok");
            double dx = p.x() - s_initial.x(), dy = p.y() - s_initial.y();
            double displacement = std::hypot(dx, dy);
            if (displacement >= 40) reach(ctx, "displaced");
            ctx.preview->setCounters({
                {QString::fromUtf8("Kierunek"), dirName(car->dir())},
                {QString::fromUtf8("Prędkość"), QString::number((int)car->speed()) + " px/s"},
                {QString::fromUtf8("Przemieszczenie"), QString::number((int)displacement) + " px"}
            });
            if (isReached(*ctx.result, "displaced") && isReached(*ctx.result, "direction_ok")) {
                measure(ctx, QString::fromUtf8("Przemieszczenie"), QString::number(displacement, 'f', 1) + " px");
                measure(ctx, QString::fromUtf8("Kierunek końcowy"), dirName(car->dir()));
                return true;
            }
            if (car->dir() != dir && !isReached(*ctx.result, "direction_ok")) {
                // auto skręciło — nie kontynuujemy
                markFail(ctx, QString("Auto zmieniło kierunek na %1 (oczekiwane %2)")
                              .arg(dirName(car->dir())).arg(arrow));
                return true;
            }
            return false;
        };
        return tc;
    };
    t.push_back(makeDirTest(Direction::L2R, "L→R", "+x"));
    t.push_back(makeDirTest(Direction::R2L, "R→L", "-x"));
    t.push_back(makeDirTest(Direction::T2B, "T→B", "+y"));
    t.push_back(makeDirTest(Direction::B2T, "B→T", "-y"));

    // 5. stop() — auto musi RZECZYWIŚCIE się zatrzymać i ruszyć
    {
        TestCase tc;
        tc.id = "car.stop_lights";
        tc.name = "Funkcja stop() — kolejki przed światłami";
        tc.category = "Samochód";
        tc.description = "5 prób · w każdej obserwujemy że auta zatrzymują się "
                         "przy czerwonym świetle (≥1 auto stoi przez ≥30 klatek).";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 8; tc.maxFramesPerAttempt = 480;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",       QString::fromUtf8("Auta zespawnowane")),
                MS("car_stopped",   QString::fromUtf8("Co najmniej 1 auto zatrzymane")),
                MS("stop_observed", QString::fromUtf8("Obserwacja zatrzymania przez ≥ 30 klatek")),
                MS("car_resumed",   QString::fromUtf8("Co najmniej 1 auto ruszyło"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.setRoundabouts({{0,7},{7,0}});
            e.spawnCarsOnMap(30 + idx * 2);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_stopFrames = 0;
            static bool s_sawStopped = false;
            static bool s_sawMoving = false;
            if (ctx.frame == 0) { s_stopFrames = 0; s_sawStopped = false; s_sawMoving = false; }
            reach(ctx, "spawned");
            int stopped = 0, moving = 0;
            std::vector<PvBox> boxes;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                if (c->speed() < 2.0) {
                    stopped++;
                    boxes.push_back({c->boundingRect().adjusted(-2,-2,2,2), "", kFail, false});
                } else moving++;
            }
            ctx.preview->setBoxes(boxes);
            if (stopped >= 1) { s_sawStopped = true; s_stopFrames++; reach(ctx, "car_stopped"); }
            if (s_sawStopped && moving >= 1) s_sawMoving = true;
            if (s_stopFrames >= 30) reach(ctx, "stop_observed");
            if (s_sawMoving) reach(ctx, "car_resumed");
            ctx.preview->setCounters({
                {QString::fromUtf8("Stoi"), QString::number(stopped)},
                {QString::fromUtf8("Jedzie"), QString::number(moving)},
                {QString::fromUtf8("Klatek stop"), QString::number(s_stopFrames)}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Aut zatrzymanych"), QString::number(stopped));
                measure(ctx, QString::fromUtf8("Aut jadących"), QString::number(moving));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 6. Max speed — auto musi osiągnąć prędkość bliską max bez przekroczenia
    {
        TestCase tc;
        tc.id = "car.max_speed";
        tc.name = "Ograniczenie prędkości maksymalnej";
        tc.category = "Samochód";
        tc.description = "5 prób · w każdej obserwujemy ≥ 5 aut · sprawdzamy że "
                         "żadne nie przekracza kMaxSpeed × 1.5.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 360;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_5", QString::fromUtf8("Co najmniej 5 aut zespawnowanych")),
                MS("observed_120", QString::fromUtf8("Obserwacja przez ≥ 120 klatek")),
                MS("max_speed_ok", QString::fromUtf8("Żadne auto nie przekroczyło limitu"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.setRoundabouts({});
            e.spawnCarsOnMap(20 + idx);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0;
            static double s_maxV = 0;
            if (ctx.frame == 0) { s_obs = 0; s_maxV = 0; }
            int n = 0;
            std::vector<PvLabel> labels;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                n++;
                s_maxV = std::max(s_maxV, c->speed());
                if (c->speed() > kMaxSpeed * 1.5) {
                    markFail(ctx, QString("Auto przekracza limit: v=%1 > %2")
                              .arg(c->speed(), 0, 'f', 1).arg(kMaxSpeed * 1.5));
                    return true;
                }
                if (n <= 8) {
                    QPointF lblPos = c->boundingRect().center();
                    lblPos.setY(lblPos.y() - 14);
                    labels.push_back({lblPos, QString::number((int)c->speed()), kInfo, 7});
                }
            }
            ctx.preview->setLabels(labels);
            if (n >= 5) reach(ctx, "spawned_5");
            s_obs++;
            if (s_obs >= 120) reach(ctx, "observed_120");
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Max v"), QString::number((int)s_maxV) + " px/s"},
                {QString::fromUtf8("Limit"), QString::number((int)(kMaxSpeed * 1.5)) + " px/s"},
                {QString::fromUtf8("Klatek obs."), QString::number(s_obs)}
            });
            if (s_obs >= 120 && n >= 5) {
                reach(ctx, "max_speed_ok");
                measure(ctx, QString::fromUtf8("Max zmierzona v"), QString::number(s_maxV, 'f', 1) + " px/s");
                measure(ctx, QString::fromUtf8("Limit"), QString::number(kMaxSpeed * 1.5));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 7. AABB — krokowanie po 4 kierunkach, każde z weryfikacją wymiarów
    {
        TestCase tc;
        tc.id = "car.aabb";
        tc.name = "AABB poprawny dla każdego kierunku";
        tc.category = "Samochód";
        tc.description = "5 prób · w każdej spawn 4 aut (po jednym w kierunku) i "
                         "weryfikacja wymiarów AABB dla każdego kierunku.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_4",   QString::fromUtf8("Zespawnowano 4 auta (4 kierunki)")),
                MS("aabb_l2r_ok", QString::fromUtf8("AABB L→R: kCarL × kCarW")),
                MS("aabb_r2l_ok", QString::fromUtf8("AABB R→L: kCarL × kCarW")),
                MS("aabb_t2b_ok", QString::fromUtf8("AABB T→B: kCarW × kCarL")),
                MS("aabb_b2t_ok", QString::fromUtf8("AABB B→T: kCarW × kCarL"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.setRoundabouts({});
            int o = idx;
            e.addCar(QPointF(80,            kL2R_Y(2 + o%3) - kCarW/2), Direction::L2R, 2 + o%3);
            e.addCar(QPointF(kCanvasW-120,  kR2L_Y(4 + o%3) - kCarW/2), Direction::R2L, 4 + o%3);
            e.addCar(QPointF(kT2B_X(3 + o%3) - kCarW/2, 80),            Direction::T2B, 3 + o%3);
            e.addCar(QPointF(kB2T_X(5 + o%3) - kCarW/2, kCanvasH-120),  Direction::B2T, 5 + o%3);
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int n = countCars(*ctx.engine);
            if (n >= 4) reach(ctx, "spawned_4");
            std::vector<PvBox> boxes;
            int checked = 0;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                QRectF r = c->boundingRect();
                bool isH = (c->dir() == Direction::L2R || c->dir() == Direction::R2L);
                double expW = isH ? kCarL : kCarW;
                double expH = isH ? kCarW : kCarL;
                bool ok = (std::abs(r.width() - expW) < 0.5 && std::abs(r.height() - expH) < 0.5);
                boxes.push_back({r.adjusted(-2,-2,2,2), "", ok ? kPass : kFail, ok});
                if (ok) {
                    if (c->dir() == Direction::L2R) reach(ctx, "aabb_l2r_ok");
                    else if (c->dir() == Direction::R2L) reach(ctx, "aabb_r2l_ok");
                    else if (c->dir() == Direction::T2B) reach(ctx, "aabb_t2b_ok");
                    else reach(ctx, "aabb_b2t_ok");
                    checked++;
                } else {
                    markFail(ctx, QString("AABB zły: %1×%2, oczekiwane %3×%4")
                              .arg(r.width()).arg(r.height()).arg(expW).arg(expH));
                    return true;
                }
            }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("AABB OK"), QString::number(checked) + " / 4"}
            });
            if (checked >= 4 && ctx.frame >= 30) {
                measure(ctx, "kCarL", QString::number(kCarL));
                measure(ctx, "kCarW", QString::number(kCarW));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
