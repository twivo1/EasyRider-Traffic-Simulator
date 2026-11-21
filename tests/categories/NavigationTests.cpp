/**
 * @file NavigationTests.cpp
 * @brief Testy NAWIGACJI — trasy A* z aktualizacjami GPS, zakaz zawracania, symetria tras.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "NavigationTests.h"

// ════════════════════════════════════════════════════════════════════
//  NAWIGACJA (4) — obserwacja prawdziwych A* trasy + GPS updates
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> navigationTests() {
    std::vector<TestCase> t;

    auto routeTest = [](const char* id, const QString& name,
                         Direction fromDir, Direction toDir) {
        TestCase tc;
        tc.id = id;
        tc.name = name;
        tc.category = "Nawigacja";
        tc.description = QString::fromUtf8(
            "5 prób · w każdej generujemy trasę %1 → %2 i obserwujemy ≥ 3 GPS updates.")
            .arg(name.contains("L→R") ? "L→R" : name.contains("T→B") ? "T→B" : "A→B")
            .arg(name.contains("L→R") ? "R→L" : name.contains("T→B") ? "B→T" : "B→A");
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 360;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("route_built",    QString::fromUtf8("Trasę zbudowano")),
                MS("route_nonempty", QString::fromUtf8("Trasa zawiera kroki")),
                MS("gps_updates_3",  QString::fromUtf8("≥ 3 aktualizacje GPS zaobserwowane")),
                MS("cost_finite",    QString::fromUtf8("Koszt trasy jest skończony"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(30); };
        tc.updateAttempt = [fromDir, toDir](AttemptCtx& ctx) -> bool {
            static int s_updates = 0;
            static double s_lastCost = -1;
            if (ctx.frame == 0) { s_updates = 0; s_lastCost = -1; }
            if (ctx.frame % 60 != 0) {
                ctx.preview->setCounters({
                    {QString::fromUtf8("Czas próby"), QString::number(ctx.frame / 60.0, 'f', 1) + " s"},
                    {QString::fromUtf8("GPS updates"), QString::number(s_updates)},
                    {QString::fromUtf8("Ostatni koszt"), s_lastCost >= 0 ? QString::number(s_lastCost, 'f', 1) + " s" : "—"}
                });
                return false;
            }
            TrafficAnalyzer ta(ctx.engine); ta.update();
            DynamicPathfinder pf(ctx.engine, &ta);
            auto cps = buildCheckpoints();
            int from = -1, to = -1;
            for (int i = 0; i < (int)cps.size(); ++i) {
                if (cps[i].entryDir == fromDir && from < 0) from = i;
                if (cps[i].entryDir == toDir   && to   < 0) to   = i;
            }
            if (from < 0 || to < 0) { markUnverified(ctx, "Brak checkpointów"); return true; }
            double cost = 0;
            Route r = pf.findRoute(from, to, cps, &cost);
            ctx.preview->setRoute(routeToPolyline(r));
            ctx.preview->setMarker(cps[from].spawnPos);
            s_updates++;
            s_lastCost = cost;
            reach(ctx, "route_built");
            if (!r.empty())  reach(ctx, "route_nonempty");
            if (cost >= 0 && cost < 1e9) reach(ctx, "cost_finite");
            if (s_updates >= 3) reach(ctx, "gps_updates_3");
            ctx.preview->setCounters({
                {QString::fromUtf8("GPS updates"), QString::number(s_updates)},
                {QString::fromUtf8("Krok trasy"), QString::number(r.size())},
                {QString::fromUtf8("Koszt"), QString::number(cost, 'f', 1) + " s"}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("GPS updates"), QString::number(s_updates));
                measure(ctx, QString::fromUtf8("Kroków trasy"), QString::number(r.size()));
                measure(ctx, QString::fromUtf8("Koszt końcowy"), QString::number(cost, 'f', 1) + " s");
                return true;
            }
            return false;
        };
        return tc;
    };
    t.push_back(routeTest("nav.l2r", "Trasa L→R (z aktualizacjami GPS)",
                          Direction::L2R, Direction::R2L));
    t.push_back(routeTest("nav.t2b", "Trasa T→B (z aktualizacjami GPS)",
                          Direction::T2B, Direction::B2T));

    // 3. Zakaz U-turn — ≥ 5 prób generowanych tras gdzie U-turn byłby możliwy
    {
        TestCase tc;
        tc.id = "nav.no_uturn";
        tc.name = "Zakaz zawracania (U-turn)";
        tc.category = "Nawigacja";
        tc.description = "5 prób · w każdej generujemy 4 trasy między różnymi "
                         "parami checkpointów · żadna nie może zawierać U-turn.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 360;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("routes_4",   QString::fromUtf8("Wygenerowano 4 trasy")),
                MS("no_uturns",  QString::fromUtf8("Żadna trasa nie zawiera U-turn")),
                MS("nonempty",   QString::fromUtf8("Wszystkie trasy niepuste"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(10); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_phase = 0;
            static int s_uturns = 0;
            static int s_empty  = 0;
            if (ctx.frame == 0) { s_phase = 0; s_uturns = 0; s_empty = 0; }
            // Co 50 klatek nowa para
            if (ctx.frame == 50 * s_phase && s_phase < 4) {
                TrafficAnalyzer ta(ctx.engine); ta.update();
                DynamicPathfinder pf(ctx.engine, &ta);
                auto cps = buildCheckpoints();
                int from = (ctx.attemptIdx * 5 + s_phase * 7) % 32;
                int to   = (from + 11 + s_phase) % 32;
                if (from == to) to = (to + 1) % 32;
                Route r = pf.findRoute(from, to, cps);
                if (r.empty()) s_empty++;
                for (int k = 1; k < (int)r.size(); ++k) {
                    Direction p = r[k-1].dir, c = r[k].dir;
                    if ((p == Direction::L2R && c == Direction::R2L) ||
                        (p == Direction::R2L && c == Direction::L2R) ||
                        (p == Direction::T2B && c == Direction::B2T) ||
                        (p == Direction::B2T && c == Direction::T2B)) s_uturns++;
                }
                ctx.preview->setRoute(routeToPolyline(r));
                s_phase++;
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Faza"), QString("%1 / 4").arg(s_phase)},
                {QString::fromUtf8("U-turny"), QString::number(s_uturns)},
                {QString::fromUtf8("Puste trasy"), QString::number(s_empty)}
            });
            if (s_phase >= 4) {
                reach(ctx, "routes_4");
                if (s_uturns == 0) reach(ctx, "no_uturns");
                else { markFail(ctx, QString("Wykryto %1 U-turnów").arg(s_uturns)); return true; }
                if (s_empty == 0) reach(ctx, "nonempty");
                measure(ctx, QString::fromUtf8("Tras wygenerowanych"), "4");
                measure(ctx, QString::fromUtf8("U-turnów"), QString::number(s_uturns));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 4. Symetria — A→B i B→A obie niepuste, koszt podobny
    {
        TestCase tc;
        tc.id = "nav.symmetry";
        tc.name = "Symetria tras (A→B i B→A)";
        tc.category = "Nawigacja";
        tc.description = "5 prób · w każdej generujemy parę A→B i B→A · "
                         "obie muszą być niepuste i mieć porównywalny koszt.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("route_ab",   QString::fromUtf8("Trasa A→B wygenerowana")),
                MS("route_ba",   QString::fromUtf8("Trasa B→A wygenerowana")),
                MS("both_nonempty", QString::fromUtf8("Obie trasy niepuste")),
                MS("costs_comparable", QString::fromUtf8("Koszty porównywalne (różnica < 4×)"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(15); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            if (ctx.frame < 30) {
                // pozwól silnikowi się ustabilizować
                ctx.preview->setCounters({{QString::fromUtf8("Stan"), "Inicjalizacja..."}});
                return false;
            }
            TrafficAnalyzer ta(ctx.engine); ta.update();
            DynamicPathfinder pf(ctx.engine, &ta);
            auto cps = buildCheckpoints();
            int A = (ctx.attemptIdx * 3) % 32;
            int B = (A + 13) % 32;
            double cA = 0, cB = 0;
            Route ab = pf.findRoute(A, B, cps, &cA);
            Route ba = pf.findRoute(B, A, cps, &cB);
            ctx.preview->setRoute(routeToPolyline(ab));
            ctx.preview->setRouteAlt(routeToPolyline(ba));
            if (!ab.empty()) reach(ctx, "route_ab");
            if (!ba.empty()) reach(ctx, "route_ba");
            if (!ab.empty() && !ba.empty()) reach(ctx, "both_nonempty");
            if (!ab.empty() && !ba.empty() && cA > 0 && cB > 0) {
                double ratio = std::max(cA, cB) / std::min(cA, cB);
                if (ratio < 4.0) reach(ctx, "costs_comparable");
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Kroki A→B"), QString::number(ab.size())},
                {QString::fromUtf8("Kroki B→A"), QString::number(ba.size())},
                {QString::fromUtf8("Koszt A→B"), QString::number(cA, 'f', 1)},
                {QString::fromUtf8("Koszt B→A"), QString::number(cB, 'f', 1)}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Koszt A→B"), QString::number(cA, 'f', 1));
                measure(ctx, QString::fromUtf8("Koszt B→A"), QString::number(cB, 'f', 1));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
