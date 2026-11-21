/**
 * @file SpawnTests.cpp
 * @brief Testy SPAWNU — limit pojazdów, brak nakładania się aut, poprawność pasów po spawnie.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "SpawnTests.h"

// ════════════════════════════════════════════════════════════════════
//  SPAWN (3) — observation-based, faktyczne spawnowanie i weryfikacja
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> spawnTests() {
    std::vector<TestCase> t;

    // 1. Limit 100 — pełna sekwencja spawnu 200, weryfikacja krokowo
    {
        TestCase tc;
        tc.id = "spawn.limit_100";
        tc.name = "Limit 100 pojazdów (próba 200)";
        tc.category = "Spawn";
        tc.description = "1 próba · spawnujemy 200 aut w 20 partiach po 10. "
                         "Każdy krok widoczny w preview · finalna liczba ≤ 100.";
        tc.isStatic = false;
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 2;
        tc.maxFramesPerAttempt = 20 * 8 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("processed_all", QString::fromUtf8("Przetworzono wszystkie 200 spawnów")),
                MS("limit_respected", QString::fromUtf8("Liczba aut ≤ 100")),
                MS("counter_observed", QString::fromUtf8("Licznik pokazywał odrzucenia"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_attempts = 0;
            if (ctx.frame == 0) s_attempts = 0;
            // Co 8 klatek dosypujemy 10 aut (do 200 łącznie)
            if (ctx.frame % 8 == 0 && s_attempts < 200) {
                int before = countCars(*ctx.engine);
                ctx.engine->spawnCarsOnMap(before + 10);
                s_attempts += 10;
            }
            int n = countCars(*ctx.engine);
            int rejected = std::max(0, s_attempts - n);
            ctx.preview->setCounters({
                {QString::fromUtf8("Próbowano"), QString::number(s_attempts)},
                {QString::fromUtf8("Zaakceptowane"), QString::number(n)},
                {QString::fromUtf8("Odrzucone"), QString::number(rejected)},
                {QString::fromUtf8("Limit"), "100"}
            });
            if (s_attempts >= 200) {
                reach(ctx, "processed_all");
                if (n <= 100) reach(ctx, "limit_respected");
                else { markFail(ctx, QString("Limit przekroczony: n=%1 > 100").arg(n)); return true; }
                if (rejected > 0) reach(ctx, "counter_observed");
                measure(ctx, QString::fromUtf8("Próbowano"), QString::number(s_attempts));
                measure(ctx, QString::fromUtf8("Zaakceptowano"), QString::number(n));
                measure(ctx, QString::fromUtf8("Odrzucono"), QString::number(rejected));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 2. Brak nakładania — sprawdzamy ≥ 100 spawnów, każdy oznaczony
    {
        TestCase tc;
        tc.id = "spawn.no_overlap";
        tc.name = "Brak nakładania się aut przy spawnie";
        tc.category = "Spawn";
        tc.description = "1 próba · spawn 100 aut, każde sprawdzane na nakładanie · "
                         "wszystkie ramki zaznaczone w preview.";
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 2;
        tc.maxFramesPerAttempt = 200;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_100", QString::fromUtf8("Zespawnowano 100 aut")),
                MS("pairs_checked", QString::fromUtf8("Wszystkie pary sprawdzone")),
                MS("no_overlap",  QString::fromUtf8("Żadne dwie się nie nakładają"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(100); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            std::vector<const Car*> cars;
            for (auto& o : ctx.engine->objects())
                if (auto* c = dynamic_cast<const Car*>(o.get())) cars.push_back(c);
            if (cars.size() >= 100) reach(ctx, "spawned_100");
            std::set<const Car*> overlap;
            int pairs = 0;
            for (size_t i = 0; i < cars.size(); ++i)
                for (size_t j = i+1; j < cars.size(); ++j) {
                    pairs++;
                    QRectF x = cars[i]->boundingRect().intersected(cars[j]->boundingRect());
                    if (x.width()*x.height() > 0.5) { overlap.insert(cars[i]); overlap.insert(cars[j]); }
                }
            std::vector<PvBox> boxes;
            for (auto* c : cars) {
                bool ok = !overlap.count(c);
                boxes.push_back({c->boundingRect().adjusted(-1,-1,1,1), "", ok ? kPass : kFail, ok});
            }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(cars.size())},
                {QString::fromUtf8("Par testowanych"), QString::number(pairs)},
                {QString::fromUtf8("Nakładających"), QString::number(overlap.size())}
            });
            if (ctx.frame >= 30 && cars.size() >= 100) {
                reach(ctx, "pairs_checked");
                if (overlap.empty()) reach(ctx, "no_overlap");
                else { markFail(ctx, QString("%1 aut nakłada się").arg(overlap.size())); return true; }
                measure(ctx, QString::fromUtf8("Aut"), QString::number(cars.size()));
                measure(ctx, QString::fromUtf8("Par sprawdzonych"), QString::number(pairs));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 3. Poprawne pasy — sprawdzamy ≥ 50 aut
    {
        TestCase tc;
        tc.id = "spawn.correct_lanes";
        tc.name = "Poprawne pasy ruchu po spawnie";
        tc.category = "Spawn";
        tc.description = "1 próba · spawn 60 aut · każde sprawdzane na zgodność "
                         "z pasem dla swojego kierunku · Δ < 6 px.";
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 2;
        tc.maxFramesPerAttempt = 100;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_50", QString::fromUtf8("Zespawnowano ≥ 50 aut")),
                MS("inspected",  QString::fromUtf8("Wszystkie auta sprawdzone")),
                MS("all_on_lane", QString::fromUtf8("Wszystkie na poprawnym pasie (Δ < 6 px)"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(60); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int n = 0, bad = 0;
            double maxDiff = 0;
            std::vector<PvBox> boxes;
            for (auto& o : ctx.engine->objects()) {
                const Car* c = dynamic_cast<const Car*>(o.get());
                if (!c) continue;
                n++;
                QPointF p = c->boundingRect().center();
                int r = c->road();
                Direction d = c->dir();
                bool horiz = (d == Direction::L2R || d == Direction::R2L);
                double expected, actual;
                if (horiz) {
                    expected = (d == Direction::L2R) ? kL2R_Y(r) : kR2L_Y(r);
                    actual = p.y() - kCarW / 2.0;
                } else {
                    expected = (d == Direction::T2B) ? kT2B_X(r) : kB2T_X(r);
                    actual = p.x() - kCarW / 2.0;
                }
                double diff = std::abs(expected - actual);
                maxDiff = std::max(maxDiff, diff);
                bool ok = diff < 6.0;
                if (!ok) bad++;
                boxes.push_back({c->boundingRect().adjusted(-1,-1,1,1), "", ok ? kPass : kFail, ok});
            }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Złe pasy"), QString::number(bad)},
                {QString::fromUtf8("Max Δ"), QString::number(maxDiff, 'f', 1) + " px"}
            });
            if (n >= 50) reach(ctx, "spawned_50");
            if (ctx.frame >= 30) {
                reach(ctx, "inspected");
                if (bad == 0) reach(ctx, "all_on_lane");
                else { markFail(ctx, QString("%1 aut na złym pasie").arg(bad)); return true; }
                measure(ctx, QString::fromUtf8("Aut sprawdzonych"), QString::number(n));
                measure(ctx, QString::fromUtf8("Max Δ pasa"), QString::number(maxDiff, 'f', 2) + " px");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
