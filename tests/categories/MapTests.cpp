/**
 * @file MapTests.cpp
 * @brief Testy MAPY — liczba rond w zakresie, minimalny odstęp Manhattan, spójność 64 stref.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "MapTests.h"

// ════════════════════════════════════════════════════════════════════
//  MAPA (3) — inspection: visible stepping through items
// ════════════════════════════════════════════════════════════════════
std::vector<std::vector<std::pair<int,int>>> mapPatterns() {
    return {
        {{1,1},{4,2},{6,5},{2,6},{5,7}},
        {{0,3},{3,0},{4,4},{6,6},{2,5},{7,2}},
        {{1,4},{3,2},{5,5},{6,1},{2,7},{0,0},{7,7}},
        {{2,2},{5,1},{1,6},{6,4},{3,7}},
        {{0,5},{2,3},{4,7},{5,0},{7,5},{3,4}}
    };
}

std::vector<TestCase> mapGenTests() {
    std::vector<TestCase> t;

    // 1. Liczba rond — step przez 5 układów
    {
        TestCase tc;
        tc.id = "map.rondas_5_to_10";
        tc.name = "Liczba rond w zakresie 5–10";
        tc.category = "Mapa";
        tc.description = "1 próba · stepujemy przez 5 układów (po 50 klatek) · "
                         "każdy musi mieć 5–10 rond.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 5 * 50 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("inspected_5", QString::fromUtf8("Sprawdzono 5 układów map")),
                MS("all_in_range", QString::fromUtf8("Każdy układ ma 5–10 rond"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.setRoundabouts(mapPatterns()[0]); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int idx = std::min(ctx.frame / 50, 4);
            static int s_lastIdx = -1;
            if (idx != s_lastIdx) {
                ctx.engine->setRoundabouts(mapPatterns()[idx]);
                ctx.preview->setHighlights(findRoundabouts(*ctx.engine));
                s_lastIdx = idx;
            }
            auto rs = findRoundabouts(*ctx.engine);
            ctx.preview->setCounters({
                {QString::fromUtf8("Układ"), QString::number(idx + 1) + " / 5"},
                {QString::fromUtf8("Rond"), QString::number(rs.size())},
                {QString::fromUtf8("Zakres"), "5 – 10"}
            });
            if (ctx.frame >= 5 * 50) {
                bool allOk = true;
                for (auto& p : mapPatterns()) {
                    if (p.size() < 5 || p.size() > 10) { allOk = false; break; }
                }
                reach(ctx, "inspected_5");
                if (allOk) reach(ctx, "all_in_range");
                else { markFail(ctx, "Wzorzec poza zakresem 5-10"); return true; }
                measure(ctx, QString::fromUtf8("Wzorców sprawdzonych"), "5");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 2. Manhattan — inspekcja wszystkich par
    {
        TestCase tc;
        tc.id = "map.manhattan_distance";
        tc.name = "Min. odległość Manhattan między rondami";
        tc.category = "Mapa";
        tc.description = "1 próba · stepujemy przez 5 układów i wszystkie pary "
                         "rond · każda para musi mieć Manhattan ≥ 2.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 5 * 50 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("inspected_5",  QString::fromUtf8("Sprawdzono 5 układów")),
                MS("pairs_tested", QString::fromUtf8("Wszystkie pary rond przetestowane")),
                MS("all_ge_2",     QString::fromUtf8("Każda odległość ≥ 2"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.setRoundabouts(mapPatterns()[0]); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int idx = std::min(ctx.frame / 50, 4);
            static int s_lastIdx = -1;
            static int s_minSeen = 999;
            static int s_pairs = 0;
            if (ctx.frame == 0) { s_lastIdx = -1; s_minSeen = 999; s_pairs = 0; }
            if (idx != s_lastIdx) {
                ctx.engine->setRoundabouts(mapPatterns()[idx]);
                s_lastIdx = idx;
            }
            auto rs = findRoundabouts(*ctx.engine);
            std::vector<PvMeasure> measures;
            for (size_t i = 0; i < rs.size(); ++i)
                for (size_t j = i+1; j < rs.size(); ++j) {
                    int m = std::abs(rs[i].first - rs[j].first) +
                            std::abs(rs[i].second - rs[j].second);
                    s_minSeen = std::min(s_minSeen, m);
                    s_pairs++;
                    if (measures.size() < 6) {
                        QPointF p1(kVLeft[rs[i].first] + kRoadW/2.0, kHTop[rs[i].second] + kRoadW/2.0);
                        QPointF p2(kVLeft[rs[j].first] + kRoadW/2.0, kHTop[rs[j].second] + kRoadW/2.0);
                        measures.push_back({p1, p2, QString("M=%1").arg(m),
                                             m >= 2 ? kPass : kFail, m >= 2});
                    }
                }
            ctx.preview->setMeasures(measures);
            ctx.preview->setCounters({
                {QString::fromUtf8("Układ"), QString::number(idx + 1) + " / 5"},
                {QString::fromUtf8("Par sprawdzonych"), QString::number(s_pairs)},
                {QString::fromUtf8("Min Manhattan"), QString::number(s_minSeen)}
            });
            if (ctx.frame >= 5 * 50) {
                reach(ctx, "inspected_5");
                reach(ctx, "pairs_tested");
                if (s_minSeen >= 2) reach(ctx, "all_ge_2");
                else { markFail(ctx, QString("Min Manhattan = %1 < 2").arg(s_minSeen)); return true; }
                measure(ctx, QString::fromUtf8("Min Manhattan"), QString::number(s_minSeen));
                measure(ctx, QString::fromUtf8("Par sprawdzonych"), QString::number(s_pairs));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 3. Spójność dróg — step po wszystkich 64 strefach
    {
        TestCase tc;
        tc.id = "map.connectivity";
        tc.name = "Spójność dróg — wszystkie 64 strefy OK";
        tc.category = "Mapa";
        tc.description = "1 próba · stepujemy przez 64 strefy · każda musi "
                         "mieć dokładnie kRoadW × kRoadW.";
        tc.isStatic = true;
        tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 64 * 4 + 30;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("inspected_64", QString::fromUtf8("Sprawdzono wszystkie 64 strefy")),
                MS("all_correct",  QString::fromUtf8("Każda strefa = kRoadW × kRoadW"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            int idx = stepIndex(ctx.frame, 64, 4);
            std::vector<PvBox> boxes;
            int bad = 0;
            for (int v = 0; v < kNumRoads; ++v)
                for (int h = 0; h < kNumRoads; ++h) {
                    int id = v * 8 + h;
                    QRectF z = kIZone(v, h);
                    bool ok = (std::abs(z.width() - kRoadW) < 0.5 &&
                                std::abs(z.height() - kRoadW) < 0.5);
                    if (!ok && id <= idx) bad++;
                    QColor col = (id < idx) ? kPass
                              : (id == idx ? kAccent : QColor("#cbd5e1"));
                    if (!ok) col = kFail;
                    boxes.push_back({z, "", col, ok});
                }
            ctx.preview->setBoxes(boxes);
            ctx.preview->setCounters({
                {QString::fromUtf8("Sprawdzono"), QString("%1 / 64").arg(idx + 1)},
                {QString::fromUtf8("OK"), QString::number(idx + 1 - bad)},
                {QString::fromUtf8("Złe"), QString::number(bad)}
            });
            if (idx >= 63) {
                reach(ctx, "inspected_64");
                if (bad == 0) reach(ctx, "all_correct");
                else { markFail(ctx, QString("%1 stref niepoprawnych").arg(bad)); return true; }
                measure(ctx, QString::fromUtf8("Stref OK"), QString::number(64 - bad));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
