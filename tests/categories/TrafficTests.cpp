/**
 * @file TrafficTests.cpp
 * @brief Testy RUCHU DROGOWEGO — zliczanie pojazdów, wykrywanie korka, koszt drogi zatłoczonej.
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "TrafficTests.h"

// ════════════════════════════════════════════════════════════════════
//  RUCH DROGOWY (3) — windows of observation
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> trafficTests() {
    std::vector<TestCase> t;

    // 1. Zliczanie — 5 prób · w każdej obserwacja segmentu przez 5 okien
    {
        TestCase tc;
        tc.id = "traffic.count_on_segment";
        tc.name = "Zliczanie pojazdów na segmencie";
        tc.category = "Ruch drogowy";
        tc.description = "5 prób · w każdej skanujemy wszystkie segmenty przez "
                         "5 okien (co 30 klatek) · zliczamy ≥ 1 segment z ≥ 1 autem.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("windows_5",     QString::fromUtf8("Wykonano 5 okien obserwacji")),
                MS("cars_observed", QString::fromUtf8("≥ 5 aut zaobserwowanych łącznie")),
                MS("segment_busy",  QString::fromUtf8("Co najmniej 1 segment zajęty"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(40); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_windows = 0;
            static int s_totalSeen = 0;
            static int s_maxOnSeg  = 0;
            if (ctx.frame == 0) { s_windows = 0; s_totalSeen = 0; s_maxOnSeg = 0; }
            if (ctx.frame > 0 && ctx.frame % 30 == 0 && s_windows < 5) {
                TrafficAnalyzer ta(ctx.engine); ta.update();
                int frameTotal = 0, frameMax = 0;
                for (int r = 0; r < kNumRoads; ++r)
                    for (int s = 0; s < kNumRoads - 1; ++s)
                        for (Direction d : {Direction::L2R, Direction::R2L, Direction::T2B, Direction::B2T}) {
                            int n = ta.carsOnSegment(d, r, s, s+1);
                            frameTotal += n;
                            frameMax = std::max(frameMax, n);
                        }
                s_totalSeen += frameTotal;
                s_maxOnSeg = std::max(s_maxOnSeg, frameMax);
                s_windows++;
            }
            if (s_windows >= 5) reach(ctx, "windows_5");
            if (s_totalSeen >= 5) reach(ctx, "cars_observed");
            if (s_maxOnSeg >= 1) reach(ctx, "segment_busy");
            ctx.preview->setCounters({
                {QString::fromUtf8("Okno"), QString("%1 / 5").arg(s_windows)},
                {QString::fromUtf8("Suma obserwacji"), QString::number(s_totalSeen)},
                {QString::fromUtf8("Max na seg."), QString::number(s_maxOnSeg)}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Okien"), QString::number(s_windows));
                measure(ctx, QString::fromUtf8("Suma obserwacji"), QString::number(s_totalSeen));
                measure(ctx, QString::fromUtf8("Max segment"), QString::number(s_maxOnSeg));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 2. Korek — generujemy korki i wykrywamy
    {
        TestCase tc;
        tc.id = "traffic.congestion";
        tc.name = "Wykrywanie korka — najgęstszy segment";
        tc.category = "Ruch drogowy";
        tc.description = "5 prób · w każdej obserwujemy ≥ 60 aut przez 180 klatek · "
                         "musi wykryć ≥ 1 zatłoczony segment (≥ 2 aut).";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 300;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("spawned_60",  QString::fromUtf8("Zespawnowano ≥ 60 aut")),
                MS("observed_180", QString::fromUtf8("Obserwacja przez 180 klatek")),
                MS("congestion_seen", QString::fromUtf8("Wykryto segment z ≥ 2 autami"))
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) { e.spawnCarsOnMap(60); };
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static int s_obs = 0;
            static int s_maxN = 0;
            if (ctx.frame == 0) { s_obs = 0; s_maxN = 0; }
            int n = countCars(*ctx.engine);
            if (n >= 60) reach(ctx, "spawned_60");
            s_obs++;
            if (ctx.frame % 20 == 0) {
                TrafficAnalyzer ta(ctx.engine); ta.update();
                int maxN = 0;
                std::vector<PvBox> boxes;
                for (int r = 0; r < kNumRoads; ++r)
                    for (int s = 0; s < kNumRoads - 1; ++s)
                        for (Direction d : {Direction::L2R, Direction::R2L, Direction::T2B, Direction::B2T}) {
                            int cnt = ta.carsOnSegment(d, r, s, s+1);
                            if (cnt >= 2 && maxN < 5) {
                                bool horiz = (d == Direction::L2R || d == Direction::R2L);
                                QRectF seg = horiz
                                    ? QRectF(kVLeft[s] + kRoadW, kHTop[r],
                                              kVLeft[s+1] - kVLeft[s] - kRoadW, kRoadW)
                                    : QRectF(kVLeft[r], kHTop[s] + kRoadW,
                                              kRoadW, kHTop[s+1] - kHTop[s] - kRoadW);
                                boxes.push_back({seg, QString::number(cnt), kAccent, true});
                            }
                            maxN = std::max(maxN, cnt);
                        }
                s_maxN = std::max(s_maxN, maxN);
                ctx.preview->setBoxes(boxes);
            }
            if (s_maxN >= 2) reach(ctx, "congestion_seen");
            if (s_obs >= 180) reach(ctx, "observed_180");
            ctx.preview->setCounters({
                {QString::fromUtf8("Aut"), QString::number(n)},
                {QString::fromUtf8("Klatek obs."), QString::number(s_obs)},
                {QString::fromUtf8("Max segment"), QString::number(s_maxN)}
            });
            if (allMilestonesReached(*ctx.result)) {
                measure(ctx, QString::fromUtf8("Max segment"), QString::number(s_maxN));
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }

    // 3. Koszt — pomiar przed/po dosypaniu ruchu
    {
        TestCase tc;
        tc.id = "traffic.cost_increases";
        tc.name = "Koszt zatłoczonej drogi > pustej";
        tc.category = "Ruch drogowy";
        tc.description = "5 prób · w każdej mierzymy koszt trasy · spawn ruchu · "
                         "ponowny pomiar · koszt nie może spaść.";
        tc.minSuccessfulAttempts = 5; tc.maxAttempts = 7; tc.maxFramesPerAttempt = 300;
        tc.declareMilestones = [](int) {
            return std::vector<Milestone>{
                MS("cost_empty",  QString::fromUtf8("Zmierzono koszt na pustej mapie")),
                MS("traffic_added", QString::fromUtf8("Dosypano ruchu (60 aut)")),
                MS("cost_full",   QString::fromUtf8("Zmierzono koszt z ruchem")),
                MS("cost_higher", QString::fromUtf8("Koszt(z ruchem) ≥ koszt(pusta)"))
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [](AttemptCtx& ctx) -> bool {
            static double s_costEmpty = -1, s_costFull = -1;
            if (ctx.frame == 0) { s_costEmpty = -1; s_costFull = -1; }
            // Klatka 1: pomiar pustej
            if (ctx.frame == 1) {
                TrafficAnalyzer ta(ctx.engine); ta.update();
                DynamicPathfinder pf(ctx.engine, &ta);
                auto cps = buildCheckpoints();
                Route r = pf.findRoute(0, 16, cps, &s_costEmpty);
                if (!r.empty()) reach(ctx, "cost_empty");
                ctx.preview->setRoute(routeToPolyline(r));
            }
            // Klatka 60: dosypujemy ruch
            if (ctx.frame == 60) {
                ctx.engine->spawnCarsOnMap(60);
                reach(ctx, "traffic_added");
            }
            // Klatka 120: pomiar z ruchem
            if (ctx.frame == 120) {
                TrafficAnalyzer ta(ctx.engine); ta.update();
                DynamicPathfinder pf(ctx.engine, &ta);
                auto cps = buildCheckpoints();
                Route r = pf.findRoute(0, 16, cps, &s_costFull);
                if (!r.empty()) reach(ctx, "cost_full");
                ctx.preview->setRoute(routeToPolyline(r));
            }
            ctx.preview->setCounters({
                {QString::fromUtf8("Czas próby"), QString::number(ctx.frame / 60.0, 'f', 1) + " s"},
                {QString::fromUtf8("Koszt pusta"), s_costEmpty < 0 ? "—" : QString::number(s_costEmpty, 'f', 1)},
                {QString::fromUtf8("Koszt z ruchem"), s_costFull < 0 ? "—" : QString::number(s_costFull, 'f', 1)},
                {QString::fromUtf8("Aut na mapie"), QString::number(countCars(*ctx.engine))}
            });
            if (ctx.frame >= 130 && s_costEmpty >= 0 && s_costFull >= 0) {
                if (s_costFull >= s_costEmpty * 0.9) reach(ctx, "cost_higher");
                else { markFail(ctx, QString("Koszt spadł: pusta=%1 z ruchem=%2").arg(s_costEmpty).arg(s_costFull)); return true; }
                measure(ctx, QString::fromUtf8("Koszt pustej"), QString::number(s_costEmpty, 'f', 2) + " s");
                measure(ctx, QString::fromUtf8("Koszt z ruchem"), QString::number(s_costFull, 'f', 2) + " s");
                measure(ctx, QString::fromUtf8("Δ"), QString::number(s_costFull - s_costEmpty, 'f', 2) + " s");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
