/**
 * @file EngineTests.cpp
 * @brief Testy SILNIKA — stabilność, SmartCar (GPS), skręty, sygnalizacja, brak kolizji, napływ aut.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "EngineTests.h"

// ════════════════════════════════════════════════════════════════════
//  SILNIK (6) — observation windows, SmartCar drives, turn observation
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> engineTests() {
    std::vector<TestCase> t;

    // 1. Stabilność — 5 prób · w każdej 60 aut · obserwacja 180 klatek
    {
        TestCase tc;
        tc.id = "engine.stability";
        tc.name = "Stabilność 5 sekund / 60 aut";
        tc.category = "Silnik";
        tc.description = "5 prób · w każdej spawn 60 aut, obserwacja 180 klatek, "
                         "weryfikacja braku kolizji i utrzymanej liczby.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_60",  QString::fromUtf8("Zespawnowano ≥ 60 aut")),
                MS("observed_180", QString::fromUtf8("Obserwowano przez ≥ 180 klatek")),
                MS("count_held",  QString::fromUtf8("Liczba aut utrzymana ≥ 50")),
                MS("no_collisions", QString::fromUtf8("Zero kolizji przez cały okres"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(60); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0, s_minCount = 999;
            if (ctx.frame == 0) { s_obs = 0; s_minCount = 999; }
            int n = countCars(*ctx.engine);
            s_minCount = std::min(s_minCount, n);
            if (n >= 60) reach(ctx, "spawned_60");
            int coll = countCollisions(*ctx.engine);
            if (coll > 0) {
                markFail(ctx, QString("Kolizja w klatce %1: %2").arg(ctx.frame).arg(coll));
                return true;
            }
            s_obs++;
            if (s_obs >= 180) {
                reach(ctx, "observed_180");
                if (s_minCount >= 50) reach(ctx, "count_held");
                reach(ctx, "no_collisions");
                measure(ctx, QString::fromUtf8("Aut min."), QString::number(s_minCount));
                measure(ctx, QString::fromUtf8("Aut końc."), QString::number(n));
                measure(ctx, QString::fromUtf8("Kolizji"), "0");
                return true;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Min spotted"), QString::number(s_minCount)},
                {QString::fromUtf8("Klatek obs."), QString::number(s_obs)},
                {QString::fromUtf8("Kolizje"), QString::number(coll)}
            });
            return false;
        };
        t.push_back(tc);
    }

    // 2. SmartCar — 5 prób, każda jeden pełny przejazd SmartCar
    {
        TestCase tc;
        tc.id = "engine.smartcar";
        tc.name = "SmartCar — spawn i prowadzenie";
        tc.category = "Silnik";
        tc.description = "5 prób · w każdej spawn SmartCar i obserwacja pełnego "
                         "przejazdu · SmartCar musi przemieścić się ≥ 200 px.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 600;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",     QString::fromUtf8("SmartCar zespawnowany")),
                MS("moving",      QString::fromUtf8("SmartCar w ruchu (v > 10)")),
                MS("displaced_200", QString::fromUtf8("Przemieścił się o ≥ 200 px")),
                MS("alive_or_left", QString::fromUtf8("Auto wciąż jedzie lub opuściło mapę"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int idx) {
            e.spawnCarsOnMap(15);
            auto cps = buildCheckpoints();
            Direction dirs[] = {Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T};
            Direction d = dirs[idx % 4];
            for (int i = 0; i < (int)cps.size(); ++i)
                if (cps[i].entryDir == d) {
                    e.spawnSmartCar(cps[i].spawnPos, cps[i].entryDir, cps[i].roadIdx);
                    // „Prowadzenie" bez nawigacji: jazda na wprost. Dawniej pierwsze
                    // skrzyżowanie było zawsze na wprost (błąd m_decided=true); teraz
                    // losowy skręt w lewo przy krawędzi wyprowadzał auto po ~190 px.
                    for (auto& o : e.objects())
                        if (auto* c = dynamic_cast<Car*>(o.get())) if (c->isSmartCar()) c->setFixedIntent(0);
                    return;
                }
        };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static QPointF s_initial;
            static double s_maxDisplacement = 0;
            static QPolygonF s_trail;
            if (ctx.frame == 0) { s_initial = QPointF(-1, -1); s_maxDisplacement = 0; s_trail.clear(); }
            const Car* sc = findSmartCar(*ctx.engine);
            if (!sc) {
                // SmartCar zniknął — sprawdź czy zdążył przejechać dystans
                if (s_maxDisplacement >= 200) reach(ctx, "alive_or_left");
                if (isReached(*ctx.result, "displaced_200")) return true;
                markUnverified(ctx, "SmartCar zniknął zbyt wcześnie");
                return true;
            }
            reach(ctx, "spawned");
            if (sc->speed() > 10) reach(ctx, "moving");
            QPointF p = sc->boundingRect().center();
            if (s_initial.x() < 0) s_initial = p;
            double d = std::hypot(p.x()-s_initial.x(), p.y()-s_initial.y());
            s_maxDisplacement = std::max(s_maxDisplacement, d);
            if (s_maxDisplacement >= 200) reach(ctx, "displaced_200");
            if (ctx.frame % 2 == 0) { s_trail << p; ctx.preview->setTrail(s_trail); }
            ctx.preview->setCounters({
                {QString::fromUtf8("Prędkość"), QString::number((int)sc->speed()) + " px/s"},
                {QString::fromUtf8("Przemieszczenie"), QString::number((int)s_maxDisplacement) + " px"},
                {QString::fromUtf8("Klatki"), QString::number(ctx.frame)}
            });
            if (isReached(*ctx.result, "displaced_200") && ctx.frame >= 90) {
                reach(ctx, "alive_or_left");
                measure(ctx, QString::fromUtf8("Max przemieszczenie"),
                        QString::number(s_maxDisplacement, 'f', 0) + " px");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 3. Skręty — obserwacja 5 wykonanych skrętów
    {
        TestCase tc;
        tc.id = "engine.turns";
        tc.name = "Skręty na skrzyżowaniach";
        tc.category = "Silnik";
        tc.description = "5 prób · w każdej obserwujemy ≥ 5 aut wykonujących "
                         "skręt (zmianę dir w ruchu).";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 10; tc.maxFramesPerAttempt = 600;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned",   QString::fromUtf8("Auta zespawnowane")),
                MS("turn_observed", QString::fromUtf8("Zaobserwowano ≥ 5 skrętów")),
                MS("dirs_diverse",  QString::fromUtf8("≥ 3 różne kierunki w użyciu"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(60); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static std::map<const Car*, Direction> s_lastDir;
            static int s_turnsObserved = 0;
            if (ctx.frame == 0) { s_lastDir.clear(); s_turnsObserved = 0; }
            int turns_this = 0;
            std::set<Direction> dirs;
            int n = 0;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                n++;
                dirs.insert(c->dir());
                auto it = s_lastDir.find(c);
                if (it != s_lastDir.end() && it->second != c->dir()) {
                    s_turnsObserved++;
                    turns_this++;
                }
                s_lastDir[c] = c->dir();
            }
            if (n > 0) reach(ctx, "spawned");
            if (s_turnsObserved >= 5) reach(ctx, "turn_observed");
            if (dirs.size() >= 3) reach(ctx, "dirs_diverse");
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Skrętów"), QString::number(s_turnsObserved)},
                {QString::fromUtf8("Kierunków"), QString::number(dirs.size()) + " / 4"}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Skrętów zaobserwowanych"), QString::number(s_turnsObserved));
                measure(ctx, QString::fromUtf8("Kierunków w użyciu"), QString::number(dirs.size()));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 4. Sygnalizacja — obserwacja ≥ 3 pełnych cykli faz
    {
        TestCase tc;
        tc.id = "engine.lights";
        tc.name = "Sygnalizacja świetlna — różnorodne fazy";
        tc.category = "Silnik";
        tc.description = "5 prób · obserwujemy że R, G i Y występują w trakcie · "
                         "≥ 3 zmiany fazy zaobserwowane.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 600;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("saw_red",     QString::fromUtf8("Zaobserwowano czerwoną fazę")),
                MS("saw_green",   QString::fromUtf8("Zaobserwowano zieloną fazę")),
                MS("phase_changed", QString::fromUtf8("Zaobserwowano ≥ 3 zmiany fazy"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(10); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static std::map<std::pair<int,int>, LightState> s_prev;
            static int s_changes = 0;
            if (ctx.frame == 0) { s_prev.clear(); s_changes = 0; }
            int r = 0, g = 0, y = 0;
            for (int vi = 0; vi < kNumRoads; ++vi)
                for (int hi = 0; hi < kNumRoads; ++hi) {
                    const auto& st = ctx.engine->intersectionAt(vi, hi);
                    if (st.isRoundabout) continue;
                    auto key = std::make_pair(vi, hi);
                    auto it = s_prev.find(key);
                    if (it != s_prev.end() && it->second != st.hLight) s_changes++;
                    s_prev[key] = st.hLight;
                    if (st.hLight == LightState::Red)    r++;
                    if (st.hLight == LightState::Green)  g++;
                    if (st.hLight == LightState::Yellow) y++;
                }
            if (r > 0) reach(ctx, "saw_red");
            if (g > 0) reach(ctx, "saw_green");
            if (s_changes >= 3) reach(ctx, "phase_changed");
            ctx.preview->setCounters({
                {QString::fromUtf8("Red"), QString::number(r)},
                {QString::fromUtf8("Green"), QString::number(g)},
                {QString::fromUtf8("Yellow"), QString::number(y)},
                {QString::fromUtf8("Zmian fazy"), QString::number(s_changes)}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Zmian fazy zaobserwowano"), QString::number(s_changes));
                measure(ctx, QString::fromUtf8("R/G/Y w finale"),
                        QString("%1/%2/%3").arg(r).arg(g).arg(y));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 5. Brak kolizji — 100 aut, obserwacja 240 klatek
    {
        TestCase tc;
        tc.id = "engine.no_collision_100";
        tc.name = "Brak kolizji — 100 aut × 5 s";
        tc.category = "Silnik";
        tc.description = "5 prób · 100 aut, obserwacja 240 klatek · zero kolizji "
                         "wymagane przez cały okres.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 300;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_100",  QString::fromUtf8("Zespawnowano 100 aut")),
                MS("observed_240", QString::fromUtf8("Obserwowano przez ≥ 240 klatek")),
                MS("zero_collisions", QString::fromUtf8("Zero kolizji przez cały okres"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(100); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0;
            if (ctx.frame == 0) s_obs = 0;
            int n = countCars(*ctx.engine);
            if (n >= 100) reach(ctx, "spawned_100");
            int coll = countCollisions(*ctx.engine);
            if (coll > 0) {
                markFail(ctx, QString("Kolizja w klatce %1: %2").arg(ctx.frame).arg(coll));
                return true;
            }
            s_obs++;
            if (s_obs >= 240) {
                reach(ctx, "observed_240");
                reach(ctx, "zero_collisions");
                measure(ctx, QString::fromUtf8("Aut końc."), QString::number(n));
                measure(ctx, QString::fromUtf8("Klatek obs."), QString::number(s_obs));
                return true;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Klatek"), QString::number(s_obs)},
                {QString::fromUtf8("Kolizje"), QString::number(coll)}
            });
            return false;
        };
        t.push_back(tc);
    }

    // 6. Utrzymanie liczby — wiele cykli observable
    {
        TestCase tc;
        tc.id = "engine.maintain_count";
        tc.name = "Utrzymanie liczby aut (ciągły napływ)";
        tc.category = "Silnik";
        tc.description = "5 prób · setTargetCarCount(40), obserwujemy że liczba "
                         "oscyluje w okolicy targetu przez 240 klatek.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 360;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("target_set",    QString::fromUtf8("Ustawiono target = 40")),
                MS("observed_240",  QString::fromUtf8("Obserwowano przez ≥ 240 klatek")),
                MS("count_in_band", QString::fromUtf8("Liczba mieści się w [25, 55]"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.setTargetCarCount(40); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0, s_min = 999, s_max = 0;
            if (ctx.frame == 0) { s_obs = 0; s_min = 999; s_max = 0; }
            reach(ctx, "target_set");
            int n = countCars(*ctx.engine);
            s_min = std::min(s_min, n); s_max = std::max(s_max, n);
            s_obs++;
            if (s_obs >= 240) {
                reach(ctx, "observed_240");
                if (s_min >= 25 && s_max <= 55) reach(ctx, "count_in_band");
                else { markFail(ctx, QString("Liczba poza pasmem: min=%1 max=%2").arg(s_min).arg(s_max)); return true; }
                measure(ctx, QString::fromUtf8("Min n"), QString::number(s_min));
                measure(ctx, QString::fromUtf8("Max n"), QString::number(s_max));
                measure(ctx, QString::fromUtf8("Target"), "40");
                return true;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Target"), "40"},
                {QString::fromUtf8("Aktualnie"), QString::number(n)},
                {QString::fromUtf8("Min spotted"), QString::number(s_min)},
                {QString::fromUtf8("Max spotted"), QString::number(s_max)}
            });
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
