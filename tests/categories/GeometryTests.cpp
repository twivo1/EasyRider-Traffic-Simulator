/**
 * @file GeometryTests.cpp
 * @brief Testy GEOMETRII — inspekcyjne sprawdzenie siatki dróg, pasów, checkpointów i stref skrzyżowań.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "GeometryTests.h"

// ════════════════════════════════════════════════════════════════════
//  GEOMETRIA (6) — inspekcyjne: 1 próba, ale stepujemy przez wszystkie
//  obiekty (siatka 8×8, 32 checkpointy, 64 strefy) widocznie w preview.
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> geometryTests() {
    std::vector<TestCase> t;

    // 1. Siatka 8×8 — stepuje przez 64 skrzyżowania, każde podświetlone osobno
    {
        TestCase tc;
        tc.id = "geom.grid";
        tc.name = "Siatka 8×8 dróg";
        tc.category = "Geometria";
        tc.description = "Krokowanie przez wszystkie 64 skrzyżowania — "
                         "każde podświetlone i policzone w panelu inspekcji.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 64 * 6 + 30;   // 6 kl/element + ogon
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("count_8", QString::fromUtf8("Liczba dróg poziomych = 8")),
                MS("count_v8", QString::fromUtf8("Liczba dróg pionowych = 8")),
                MS("inspected_all", QString::fromUtf8("Sprawdzono wszystkie 64 skrzyżowania"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int frame = ctx.frame;
            // Step przez 64 skrzyżowania (6 klatek każde)
            int idx = stepIndex(frame, 64, 6);
            int vi = idx / 8, hi = idx % 8;

            std::vector<PvLabel> labels;
            for (int v = 0; v < kNumRoads; ++v)
                for (int h = 0; h < kNumRoads; ++h) {
                    int id = v * 8 + h;
                    QColor col = (id < idx)  ? kPass : (id == idx ? kAccent : QColor("#a8a29e"));
                    QPointF c(kVLeft[v] + kRoadW/2.0, kHTop[h] + kRoadW/2.0);
                    labels.push_back({c, QString("[%1,%2]").arg(v).arg(h), col, 7});
                }
            ctx.preview->setLabels(labels);
            ctx.preview->setHighlight(vi, hi);
            ctx.preview->setCounters({
                {QString::fromUtf8("Sprawdzono"), QString("%1 / 64").arg(idx + 1)},
                {QString::fromUtf8("Dróg poz."), QString::number(kNumRoads)},
                {QString::fromUtf8("Dróg pion."), QString::number(kNumRoads)}
            });

            if (kNumRoads == 8) {
                reach(ctx, "count_8");
                reach(ctx, "count_v8");
            }
            if (idx >= 63) {
                reach(ctx, "inspected_all");
                measure(ctx, "Skrzyżowań", "64");
                measure(ctx, "kNumRoads", QString::number(kNumRoads));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 2. Wymiary drogi (70 px) — pomiar przez 3 różne drogi
    {
        TestCase tc;
        tc.id = "geom.road_width";
        tc.name = "Wymiary drogi (szerokość 70 px)";
        tc.category = "Geometria";
        tc.description = "Pomiar szerokości na 3 różnych drogach poziomych — "
                         "każda musi mieć dokładnie kRoadW = 70 px.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 3 * 40 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("measured_road_0", QString::fromUtf8("Zmierzono drogę hi=1")),
                MS("measured_road_1", QString::fromUtf8("Zmierzono drogę hi=4")),
                MS("measured_road_2", QString::fromUtf8("Zmierzono drogę hi=7")),
                MS("all_equal_70",    QString::fromUtf8("Wszystkie pomiary = 70 px"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int frame = ctx.frame;
            int roads[] = {1, 4, 7};
            int step = std::min(frame / 40, 2);

            std::vector<PvMeasure> measures;
            for (int i = 0; i <= step; ++i) {
                int hi = roads[i];
                double x = (i + 1) * 200.0;
                QRectF z = kIZone(0, hi);
                bool ok = (z.height() == kRoadW);
                measures.push_back({QPointF(x, kHTop[hi]),
                                     QPointF(x, kHTop[hi] + kRoadW),
                                     QString("hi=%1: %2 px").arg(hi).arg((int)z.height()),
                                     ok ? kPass : kFail, ok});
            }
            ctx.preview->setMeasures(measures);
            ctx.preview->setCounters({
                {"Zmierzono", QString("%1 / 3").arg(step + 1)},
                {"kRoadW", QString::number((int)kRoadW) + " px"}
            });

            if (step >= 0) reach(ctx, "measured_road_0");
            if (step >= 1) reach(ctx, "measured_road_1");
            if (step >= 2) reach(ctx, "measured_road_2");

            if (frame >= 3 * 40) {
                bool allEqual = true;
                for (int hi : roads)
                    if (kIZone(0, hi).height() != kRoadW) allEqual = false;
                if (allEqual) reach(ctx, "all_equal_70");
                else markFail(ctx, "Co najmniej jedna droga nie ma 70 px");
                measure(ctx, "kRoadW", QString::number(kRoadW));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 3. Szerokość pasów — pomiar L2R i R2L na 2 drogach
    {
        TestCase tc;
        tc.id = "geom.lane_width";
        tc.name = "Szerokość pasów ruchu";
        tc.category = "Geometria";
        tc.description = "Pomiar pasa L2R i R2L na drogach hi=2 i hi=5. "
                         "Każdy pas = kCarW + kLaneOff; razem < kRoadW.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 4 * 30 + 20;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("l2r_2", QString::fromUtf8("Zmierzono pas L2R na hi=2")),
                MS("r2l_2", QString::fromUtf8("Zmierzono pas R2L na hi=2")),
                MS("l2r_5", QString::fromUtf8("Zmierzono pas L2R na hi=5")),
                MS("r2l_5", QString::fromUtf8("Zmierzono pas R2L na hi=5")),
                MS("sum_ok", QString::fromUtf8("2·(kLaneOff + kCarW) < kRoadW"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int step = std::min(ctx.frame / 30, 3);
            std::vector<PvMeasure> measures;
            struct Item { int hi; bool l2r; double x; };
            Item items[] = {{2,true,180},{2,false,400},{5,true,180},{5,false,400}};
            for (int i = 0; i <= step; ++i) {
                const Item& it = items[i];
                double y = it.l2r ? kL2R_Y(it.hi) : kR2L_Y(it.hi);
                measures.push_back({QPointF(it.x, y), QPointF(it.x, y + kCarW),
                                     QString("%1 = %2 px").arg(it.l2r ? "L2R" : "R2L").arg((int)kCarW),
                                     kInfo, true});
            }
            ctx.preview->setMeasures(measures);
            double used = 2.0 * (kLaneOff + kCarW);
            ctx.preview->setCounters({
                {QString::fromUtf8("Pomiary"), QString("%1 / 4").arg(step+1)},
                {"kCarW", QString::number((int)kCarW)},
                {"kLaneOff", QString::number((int)kLaneOff)},
                {QString::fromUtf8("Suma użyta"), QString::number((int)used) + " / 70"}
            });
            const char* ids[] = {"l2r_2","r2l_2","l2r_5","r2l_5"};
            for (int i = 0; i <= step; ++i) reach(ctx, ids[i]);

            if (ctx.frame >= 4 * 30) {
                if (used < kRoadW) reach(ctx, "sum_ok");
                else markFail(ctx, "Suma pasów ≥ szerokość drogi");
                measure(ctx, QString::fromUtf8("Suma 2(kLaneOff+kCarW)"),
                        QString::number(used) + " px");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 4. Checkpointy — step przez 32 punkty
    {
        TestCase tc;
        tc.id = "geom.checkpoints";
        tc.name = "Położenie checkpointów";
        tc.category = "Geometria";
        tc.description = "Krokowanie przez wszystkie 32 checkpointy — "
                         "po 8 na każdy bok. Każdy oznaczany kolejno.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 32 * 6 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("count_32", QString::fromUtf8("Znaleziono 32 checkpointy")),
                MS("8_each_side", QString::fromUtf8("Po 8 na każdym boku")),
                MS("inspected_all", QString::fromUtf8("Sprawdzono wszystkie 32"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            auto cps = buildCheckpoints();
            int idx = stepIndex(ctx.frame, (int)cps.size(), 6);
            std::vector<PvLabel> labels;
            std::vector<PvBox>   boxes;
            int top=0, bot=0, lef=0, rig=0;
            for (int i = 0; i < (int)cps.size(); ++i) {
                const auto& cp = cps[i];
                QColor col = (i < idx) ? kPass : (i == idx ? kAccent : QColor("#a8a29e"));
                labels.push_back({cp.spawnPos, QString::number(i), col, 7});
                QRectF r(cp.spawnPos - QPointF(5,5), QSizeF(10,10));
                boxes.push_back({r, "", col, true});
                if (i <= idx) {
                    if (cp.entryDir == Direction::T2B) top++;
                    else if (cp.entryDir == Direction::B2T) bot++;
                    else if (cp.entryDir == Direction::L2R) lef++;
                    else rig++;
                }
            }
            ctx.preview->setLabels(labels);
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Sprawdzono"), QString("%1 / 32").arg(idx + 1)},
                {QString::fromUtf8("Góra/Dół"), QString("%1 / %2").arg(top).arg(bot)},
                {QString::fromUtf8("Lewo/Prawo"), QString("%1 / %2").arg(lef).arg(rig)}
            });
            if (cps.size() == 32) reach(ctx, "count_32");
            if (idx >= 31) {
                int tt=0,bb=0,ll=0,rr=0;
                for (auto& c : cps) {
                    if (c.entryDir == Direction::T2B) tt++;
                    else if (c.entryDir == Direction::B2T) bb++;
                    else if (c.entryDir == Direction::L2R) ll++;
                    else rr++;
                }
                if (tt==8 && bb==8 && ll==8 && rr==8) reach(ctx, "8_each_side");
                else markFail(ctx, QString("Rozkład: T=%1 B=%2 L=%3 R=%4").arg(tt).arg(bb).arg(ll).arg(rr));
                reach(ctx, "inspected_all");
                measure(ctx, "Checkpointów", QString::number(cps.size()));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 5. Strefy niepuste — step po 64
    {
        TestCase tc;
        tc.id = "geom.zones_nonempty";
        tc.name = "Strefy skrzyżowań niepuste";
        tc.category = "Geometria";
        tc.description = "Krokowanie przez 64 strefy — każda musi mieć "
                         "niezerową powierzchnię (kRoadW × kRoadW).";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 64 * 4 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("all_nonzero", QString::fromUtf8("Wszystkie 64 strefy niepuste")),
                MS("inspected_all", QString::fromUtf8("Sprawdzono wszystkie strefy"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int idx = stepIndex(ctx.frame, 64, 4);
            std::vector<PvBox> boxes;
            int empty = 0;
            for (int v = 0; v < kNumRoads; ++v)
                for (int h = 0; h < kNumRoads; ++h) {
                    int id = v * 8 + h;
                    QRectF z = kIZone(v, h);
                    bool ok = (z.width() > 0 && z.height() > 0);
                    if (!ok && id <= idx) empty++;
                    QColor col = (id < idx) ? kPass
                              : (id == idx ? kAccent : QColor("#cbd5e1"));
                    if (!ok) col = kFail;
                    boxes.push_back({z, "", col, ok});
                }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Sprawdzono"), QString("%1 / 64").arg(idx + 1)},
                {QString::fromUtf8("Pustych"), QString::number(empty)}
            });
            if (idx >= 63) {
                reach(ctx, "inspected_all");
                if (empty == 0) reach(ctx, "all_nonzero");
                else markFail(ctx, QString("%1 stref pustych").arg(empty));
                measure(ctx, "Stref", "64");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 6. Brak nakładania — step po parach
    {
        TestCase tc;
        tc.id = "geom.no_overlap";
        tc.name = "Brak nakładających się stref";
        tc.category = "Geometria";
        tc.description = "Sprawdzamy wszystkie pary 64 stref na nakładanie. "
                         "Test krokuje po 64 strefach pokazując wynik.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1;
        tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 64 * 4 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("pairs_tested", QString::fromUtf8("Sprawdzono wszystkie pary")),
                MS("no_overlap", QString::fromUtf8("Żadna para nie nachodzi"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int idx = stepIndex(ctx.frame, 64, 4);
            std::vector<QRectF> zones;
            for (int v = 0; v < kNumRoads; ++v)
                for (int h = 0; h < kNumRoads; ++h)
                    zones.push_back(kIZone(v, h));
            std::set<int> overlap;
            for (size_t i = 0; i < zones.size(); ++i)
                for (size_t j = i+1; j < zones.size(); ++j) {
                    QRectF x = zones[i].intersected(zones[j]);
                    if (x.width()*x.height() > 0.5) {
                        overlap.insert((int)i); overlap.insert((int)j);
                    }
                }
            std::vector<PvBox> boxes;
            for (size_t i = 0; i < zones.size(); ++i) {
                bool ok = !overlap.count((int)i);
                QColor col = ((int)i < idx) ? kPass
                          : ((int)i == idx ? kAccent : QColor("#cbd5e1"));
                if (!ok) col = kFail;
                boxes.push_back({zones[i], "", col, ok});
            }
            ctx.preview->setBoxes(boxes);
            int totalPairs = (int)(zones.size() * (zones.size()-1) / 2);
            ctx.preview->setCounters({
                {QString::fromUtf8("Sprawdzono"), QString("%1 / 64").arg(idx + 1)},
                {QString::fromUtf8("Par testowanych"), QString::number(totalPairs)},
                {QString::fromUtf8("Nachodzących"), QString::number(overlap.size())}
            });
            if (idx >= 63) {
                reach(ctx, "pairs_tested");
                if (overlap.empty()) reach(ctx, "no_overlap");
                else markFail(ctx, QString("%1 stref nachodzi").arg(overlap.size()));
                measure(ctx, "Par testowanych", QString::number(totalPairs));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
