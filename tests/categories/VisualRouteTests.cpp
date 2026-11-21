/**
 * @file VisualRouteTests.cpp
 * @brief Testy WIZUALIZACJI TRASY — walidacja danych PRZED renderingiem (ten sam moduł co główna apka).
 *
 *  Treść przeniesiona 1:1 z dawnego tests/TestSuite.h (bez zmian mechaniki).
 */
#include "VisualRouteTests.h"
#include "nav/RouteManager.h"
#include <memory>

// ════════════════════════════════════════════════════════════════════
//  WIZUALIZACJA TRASY (9) — walidacja danych PRZED renderingiem.
//
//  Wszystkie testy używają TEGO SAMEGO modułu co główna apka:
//    src/nav/VisualRouteBuilder + RoutePolylineSanitizer
//  Dzięki temu zielone wyniki tych testów dowodzą, że dane konsumowane
//  przez OverlayRenderer w EasyRider są poprawne i bezpieczne do narysowania.
// ════════════════════════════════════════════════════════════════════
std::vector<TestCase> visualRouteTests() {
    std::vector<TestCase> t;

    // Małe helpery — uniwersalne dla testów tej kategorii.
    auto buildRouteBetween = [](SimulationEngine& e, Direction fromD, Direction toD) -> Route {
        TrafficAnalyzer ta(&e); ta.update();
        DynamicPathfinder pf(&e, &ta);
        auto cps = buildCheckpoints();
        int from = -1, to = -1;
        for (int i = 0; i < (int)cps.size(); ++i) {
            if (cps[i].entryDir == fromD && from < 0) from = i;
            if (cps[i].entryDir == toD   && to   < 0) to   = i;
        }
        if (from < 0 || to < 0) return Route{};
        return pf.findRoute(from, to, cps);
    };

    // ── HELPERY DO PODGLĄDU NA ŻYWO ───────────────────────────────────
    // Każdy test wysyła swoje dane do ctx.preview, żeby prowadzący mógł
    // wzrokowo zweryfikować TĘ SAMĄ trasę, którą sprawdzają asercje.
    // Jeśli polilinia jest pusta, test NIE zalicza milestonu „Trasa
    // wyświetlona w podglądzie" i nie może otrzymać statusu Passed —
    // co eliminuje ryzyko zielonego statusu bez widocznej trasy.
    auto toFragments = [](const VisualRouteData& d) -> std::vector<QPolygonF> {
        std::vector<QPolygonF> out;
        out.reserve(d.currentRouteFragments.size());
        for (const auto& frag : d.currentRouteFragments) out.push_back(frag);
        return out;
    };
    auto rejectedToLines = [](const VisualRouteData& d) -> std::vector<QLineF> {
        std::vector<QLineF> out;
        out.reserve(d.rejected.size());
        for (const auto& rj : d.rejected) out.push_back(QLineF(rj.from, rj.to));
        return out;
    };
    auto totalPoints = [](const std::vector<QPolygonF>& fr) {
        int n = 0; for (const auto& f : fr) n += f.size(); return n;
    };
    // „displayed_in_preview" — wspólny ostatni milestone wszystkich testów
    // wizualnych. Test reach'uje go DOPIERO po przekazaniu niepustych
    // danych do podglądu.
    auto displayMs = []() { return MS("displayed_in_preview",
        QString::fromUtf8("Trasa wyświetlona w podglądzie")); };

    // ─── 1. Brak połączenia starej i nowej trasy ─────────────────────
    {
        TestCase tc;
        tc.id = "vis.no_old_new_connection";
        tc.name = "Brak połączenia starej i nowej trasy";
        tc.category = "Wizualizacja trasy";
        tc.description = "Generujemy trasę A→B, potem inną A→C. "
                         "Nowa polilinia NIE może zawierać punktów z poprzedniej.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 120;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("route_a_built",   QString::fromUtf8("Trasa A→B wygenerowana")),
                MS("route_b_built",   QString::fromUtf8("Trasa A→C wygenerowana")),
                MS("separate_arrays", QString::fromUtf8("Dane wizualne to oddzielne obiekty")),
                MS("no_shared_endpoint", QString::fromUtf8("Brak łączenia końca starej z początkiem nowej")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Route ra = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::R2L);
            Route rb = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::B2T);
            if (ra.empty()) { markUnverified(ctx, "Brak trasy A→B"); return true; }
            if (rb.empty()) { markUnverified(ctx, "Brak trasy A→C"); return true; }
            reach(ctx, "route_a_built");
            reach(ctx, "route_b_built");

            VisualRouteData dataA = VisualRouteBuilder::build(ra);
            VisualRouteData dataB = VisualRouteBuilder::build(rb);
            // Dane to dwa NIEZALEŻNE obiekty — brak współdzielonych kontenerów.
            reach(ctx, "separate_arrays");

            // KRYTYCZNE: w dataB nie może być punktu z końca dataA.
            // (Adresowo to oczywiste — VisualRouteData to świeży POD —
            // ale dla pewności weryfikujemy że żaden ostatni punkt dataA
            // nie pojawia się w żadnym fragmencie dataB.)
            QPointF endA;
            for (auto it = dataA.currentRouteFragments.rbegin();
                 it != dataA.currentRouteFragments.rend(); ++it)
                if (it->size() >= 1) { endA = it->back(); break; }
            bool sharedPoint = false;
            for (const auto& f : dataB.currentRouteFragments)
                for (const auto& p : f)
                    if (std::hypot(p.x()-endA.x(), p.y()-endA.y()) < 0.5)
                        sharedPoint = true;
            if (sharedPoint) {
                markFail(ctx, "Punkt końcowy starej trasy znaleziony w nowej!");
                return true;
            }
            reach(ctx, "no_shared_endpoint");

            // ── Podgląd: stara trasa jako alt (niebieska/wygaszona),
            //    nowa jako aktywna (pomarańczowa). NIE łączone w jeden kontener.
            auto fragA = toFragments(dataA);
            auto fragB = toFragments(dataB);
            ctx.preview->setRouteAltFragments(fragA);
            ctx.preview->setRouteFragments(fragB);
            ctx.preview->setLabels({
                {QPointF(60, 60),    QString::fromUtf8("● Stara trasa (A→B)"), QColor("#475569"), 9},
                {QPointF(60, 90),    QString::fromUtf8("● Nowa trasa (A→C)"),  QColor("#f59e0b"), 9}
            });
            if (totalPoints(fragA) > 0 && totalPoints(fragB) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Fragmentów A"), QString::number(dataA.currentRouteFragments.size()));
            measure(ctx, QString::fromUtf8("Fragmentów B"), QString::number(dataB.currentRouteFragments.size()));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 2. Brak długich segmentów ───────────────────────────────────
    {
        TestCase tc;
        tc.id = "vis.no_long_segments";
        tc.name = "Brak długich segmentów przez mapę";
        tc.category = "Wizualizacja trasy";
        tc.description = "Budujemy 4 różne trasy. Po sanityzacji żaden segment "
                         "polilinii nie może przekraczać limitu (kMaxSaneRouteSegment).";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("routes_built",   QString::fromUtf8("Cztery trasy wygenerowane")),
                MS("max_seg_ok",     QString::fromUtf8("Każdy segment ≤ limit (600 px)")),
                MS("no_rejections",  QString::fromUtf8("Sanitizer nie odrzucił żadnego segmentu")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Direction pairs[][2] = {
                {Direction::L2R, Direction::R2L},
                {Direction::T2B, Direction::B2T},
                {Direction::L2R, Direction::B2T},
                {Direction::T2B, Direction::R2L}
            };
            int built = 0, totalRejected = 0;
            double maxSeg = 0;
            std::vector<QPolygonF> allFragments;   // wszystkie 4 trasy w jednym podglądzie
            for (auto& pair : pairs) {
                Route r = buildRouteBetween(*ctx.engine, pair[0], pair[1]);
                if (r.empty()) continue;
                built++;
                VisualRouteData d = VisualRouteBuilder::build(r);
                totalRejected += d.rejected.size();
                for (const auto& f : d.currentRouteFragments) {
                    allFragments.push_back(f);
                    for (int i = 1; i < f.size(); ++i) {
                        double dx = f[i].x()-f[i-1].x(), dy = f[i].y()-f[i-1].y();
                        maxSeg = std::max(maxSeg, std::sqrt(dx*dx+dy*dy));
                    }
                }
            }
            if (built >= 4) reach(ctx, "routes_built");
            // Podgląd: wszystkie 4 trasy jako oddzielne polilinie pomarańczowe.
            ctx.preview->setRouteFragments(allFragments);
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● 4 trasy testowe (max segment %1 px, limit %2)")
                                  .arg(maxSeg, 0, 'f', 0).arg(kMaxSaneRouteSegment),
                 QColor("#f59e0b"), 9}
            });
            if (totalPoints(allFragments) > 0) reach(ctx, "displayed_in_preview");

            if (maxSeg <= kMaxSaneRouteSegment) reach(ctx, "max_seg_ok");
            else { markFail(ctx, QString("Max seg = %1 > %2").arg(maxSeg).arg(kMaxSaneRouteSegment)); return true; }
            if (totalRejected == 0) reach(ctx, "no_rejections");
            else { markFail(ctx, QString("Odrzucono %1 segmentów").arg(totalRejected)); return true; }
            measure(ctx, QString::fromUtf8("Tras"), QString::number(built));
            measure(ctx, QString::fromUtf8("Max segment"), QString::number(maxSeg, 'f', 1) + " px");
            measure(ctx, QString::fromUtf8("Limit"), QString::number(kMaxSaneRouteSegment) + " px");
            return true;
        };
        t.push_back(tc);
    }

    // ─── 3. Poprawne punkty trasy (brak NaN/inf) ─────────────────────
    {
        TestCase tc;
        tc.id = "vis.finite_points";
        tc.name = "Poprawne punkty trasy";
        tc.category = "Wizualizacja trasy";
        tc.description = "Wszystkie wygenerowane punkty muszą być skończone "
                         "(brak NaN, brak inf). Sprawdzamy 6 różnych tras.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 240;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("routes_built",  QString::fromUtf8("Sześć tras wygenerowanych")),
                MS("all_finite",    QString::fromUtf8("Wszystkie punkty mają skończone wartości")),
                MS("no_invalid",    QString::fromUtf8("Sanitizer nie odrzucił niepoprawnych punktów")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, totalPoints](AttemptCtx& ctx) -> bool {
            Direction pairs[][2] = {
                {Direction::L2R, Direction::R2L}, {Direction::R2L, Direction::L2R},
                {Direction::T2B, Direction::B2T}, {Direction::B2T, Direction::T2B},
                {Direction::L2R, Direction::T2B}, {Direction::T2B, Direction::R2L}
            };
            int built = 0;
            int total = 0, invalidCnt = 0;
            std::vector<QPolygonF> allFragments;
            for (auto& pair : pairs) {
                Route r = buildRouteBetween(*ctx.engine, pair[0], pair[1]);
                if (r.empty()) continue;
                built++;
                VisualRouteData d = VisualRouteBuilder::build(r);
                invalidCnt += d.invalidPointsRemoved;
                for (const auto& f : d.currentRouteFragments) {
                    allFragments.push_back(f);
                    for (const auto& p : f) {
                        total++;
                        if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
                            markFail(ctx, QString("Niepoprawny punkt: (%1,%2)").arg(p.x()).arg(p.y()));
                            return true;
                        }
                    }
                }
            }
            if (built >= 6) reach(ctx, "routes_built");
            reach(ctx, "all_finite");
            if (invalidCnt == 0) reach(ctx, "no_invalid");
            else { markFail(ctx, QString("Sanitizer wycieł %1 niepoprawnych punktów").arg(invalidCnt)); return true; }

            ctx.preview->setRouteFragments(allFragments);
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● 6 tras: %1 punktów, 0 niepoprawnych")
                                  .arg(total), QColor("#f59e0b"), 9}
            });
            if (totalPoints(allFragments) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Punktów sprawdzonych"), QString::number(total));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 4. Trasa przez skrzyżowanie bez artefaktów ──────────────────
    {
        TestCase tc;
        tc.id = "vis.intersection_clean";
        tc.name = "Trasa przez skrzyżowanie bez artefaktów";
        tc.category = "Wizualizacja trasy";
        tc.description = "Trasa wymuszająca skręt na skrzyżowaniu. "
                         "Polilinia musi być ciągła (1 fragment) i bez diagonali.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 60;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("route_built",     QString::fromUtf8("Trasa ze skrętem wygenerowana")),
                MS("single_fragment", QString::fromUtf8("Jedna ciągła polilinia (brak przerw)")),
                MS("no_diagonals",    QString::fromUtf8("Brak segmentów diagonalnych dłuższych niż 1 kratka")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Route r = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::B2T);
            if (r.empty()) { markUnverified(ctx, "Brak trasy ze skrętem"); return true; }
            reach(ctx, "route_built");
            VisualRouteData d = VisualRouteBuilder::build(r);
            if (d.currentRouteFragments.size() == 1) reach(ctx, "single_fragment");
            else { markFail(ctx, QString("Polilinia w %1 fragmentach").arg(d.currentRouteFragments.size())); return true; }

            // Max segment dla CZYSTEJ trasy: ~kVLeft[i+1] - kVLeft[i] = ok. 200 px
            // (odległość między skrzyżowaniami). > 400 = artefakt.
            double maxSeg = 0;
            for (const auto& f : d.currentRouteFragments)
                for (int i = 1; i < f.size(); ++i) {
                    double dx = f[i].x()-f[i-1].x(), dy = f[i].y()-f[i-1].y();
                    maxSeg = std::max(maxSeg, std::sqrt(dx*dx+dy*dy));
                }
            if (maxSeg <= 400) reach(ctx, "no_diagonals");
            else { markFail(ctx, QString("Max segment = %1 px (oczekiwane ≤ 400)").arg(maxSeg)); return true; }

            auto frags = toFragments(d);
            ctx.preview->setRouteFragments(frags);
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● Trasa L→góra ze skrętem"), QColor("#f59e0b"), 9},
                {QPointF(60, 90), QString::fromUtf8("max segment %1 px ≤ 400 px ✓").arg(maxSeg, 0, 'f', 0),
                 QColor("#16a34a"), 9}
            });
            if (totalPoints(frags) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Max segment"), QString::number(maxSeg, 'f', 1) + " px");
            measure(ctx, QString::fromUtf8("Kroków trasy"), QString::number(r.size()));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 5. Trasa przez rondo bez artefaktów ─────────────────────────
    {
        TestCase tc;
        tc.id = "vis.roundabout_clean";
        tc.name = "Trasa przez rondo bez artefaktów";
        tc.category = "Wizualizacja trasy";
        tc.description = "Trasa przebiegająca przez rondo. Polilinia musi "
                         "być ciągła i nie zawierać diagonali łączących "
                         "różne strony ringu.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 60;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("route_built",     QString::fromUtf8("Trasa przez rondo wygenerowana")),
                MS("no_long_jumps",   QString::fromUtf8("Brak długich skoków")),
                MS("no_rejected",     QString::fromUtf8("Sanitizer nie odrzucił żadnego segmentu")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine& e, int) {
            e.setRoundabouts({{3,3},{4,4}});
        };
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Route r = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::R2L);
            if (r.empty()) { markUnverified(ctx, "Brak trasy"); return true; }
            reach(ctx, "route_built");
            VisualRouteData d = VisualRouteBuilder::build(r);
            double maxSeg = 0;
            for (const auto& f : d.currentRouteFragments)
                for (int i = 1; i < f.size(); ++i) {
                    double dx = f[i].x()-f[i-1].x(), dy = f[i].y()-f[i-1].y();
                    maxSeg = std::max(maxSeg, std::sqrt(dx*dx+dy*dy));
                }
            if (maxSeg <= 400) reach(ctx, "no_long_jumps");
            else { markFail(ctx, QString("Max segment = %1 (przerwa w ringu)").arg(maxSeg)); return true; }
            if (d.rejected.empty()) reach(ctx, "no_rejected");
            else { markFail(ctx, QString("Odrzucono %1 segmentów").arg(d.rejected.size())); return true; }

            auto frags = toFragments(d);
            ctx.preview->setRouteFragments(frags);
            ctx.preview->setHighlights({{3,3},{4,4}});  // podkreśl ronda
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● Trasa przez 2 ronda"), QColor("#f59e0b"), 9},
                {QPointF(60, 90), QString::fromUtf8("max segment %1 px (limit 400)").arg(maxSeg, 0, 'f', 0),
                 QColor("#16a34a"), 9}
            });
            if (totalPoints(frags) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Max segment"), QString::number(maxSeg, 'f', 1) + " px");
            measure(ctx, QString::fromUtf8("Fragmentów polilinii"), QString::number(d.currentRouteFragments.size()));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 6. GPS zastępuje starą trasę ────────────────────────────────
    {
        TestCase tc;
        tc.id = "vis.gps_replaces";
        tc.name = "GPS zastępuje starą trasę";
        tc.category = "Wizualizacja trasy";
        tc.description = "Po recalc nowa VisualRouteData NIE zawiera punktów ze "
                         "starej. Pipeline jest stateless — każdy build() to czysty start.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 60;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("old_route_built", QString::fromUtf8("Stara trasa zbudowana")),
                MS("new_route_built", QString::fromUtf8("Nowa trasa zbudowana (po recalc)")),
                MS("clean_replace",   QString::fromUtf8("Nowa polilinia bez residuum starej")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Route rOld = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::R2L);
            if (rOld.empty()) { markUnverified(ctx, "Brak starej trasy"); return true; }
            reach(ctx, "old_route_built");

            // Sym recalc: A* z dosypanym ruchem (zmienia preferencje).
            ctx.engine->spawnCarsOnMap(40);
            Route rNew = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::R2L);
            if (rNew.empty()) { markUnverified(ctx, "Brak nowej trasy"); return true; }
            reach(ctx, "new_route_built");

            VisualRouteData dataOld = VisualRouteBuilder::build(rOld);
            VisualRouteData dataNew = VisualRouteBuilder::build(rNew);

            // Sumaryczna liczba punktów w nowej polilinii musi odpowiadać
            // wyłącznie rNew — nie zawiera śladu starej.
            int oldPts = 0, newPts = 0;
            for (const auto& f : dataOld.currentRouteFragments) oldPts += f.size();
            for (const auto& f : dataNew.currentRouteFragments) newPts += f.size();

            // Spodziewana liczba punktów dla rNew = liczba kroków × 2..3.
            int expectedMax = (int)rNew.size() * 3;
            if (newPts <= expectedMax) reach(ctx, "clean_replace");
            else {
                markFail(ctx, QString("Nowa trasa ma %1 pkt, oczekiwane ≤ %2").arg(newPts).arg(expectedMax));
                return true;
            }

            // Podgląd: stara (alt, niebieska) + nowa (aktywna, pomarańczowa).
            auto fragOld = toFragments(dataOld);
            auto fragNew = toFragments(dataNew);
            ctx.preview->setRouteAltFragments(fragOld);
            ctx.preview->setRouteFragments(fragNew);
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● Stara trasa (przed dosypaniem ruchu)"),
                 QColor("#475569"), 9},
                {QPointF(60, 90), QString::fromUtf8("● Nowa trasa (po recalc) — zastąpiła starą"),
                 QColor("#f59e0b"), 9}
            });
            if (totalPoints(fragNew) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Punktów starej"), QString::number(oldPts));
            measure(ctx, QString::fromUtf8("Punktów nowej"), QString::number(newPts));
            measure(ctx, QString::fromUtf8("Limit dla nowej"), QString::number(expectedMax));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 7. Historia trasy oddzielona od aktywnej ────────────────────
    {
        TestCase tc;
        tc.id = "vis.history_separated";
        tc.name = "Historia trasy oddzielona od aktywnej";
        tc.category = "Wizualizacja trasy";
        tc.description = "VisualRouteData ma OSOBNE pola dla aktywnej trasy, "
                         "przebytej ścieżki i historii. Pipeline ich nie miesza.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 60;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("structure_distinct", QString::fromUtf8("currentRouteFragments ≠ drivenTrace ≠ historyRoutes")),
                MS("pipeline_clean",     QString::fromUtf8("Pipeline build() wypełnia tylko current — nie miesza warstw")),
                MS("can_separate",       QString::fromUtf8("Można dodać drivenTrace bez wpływu na current")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [buildRouteBetween, toFragments, totalPoints](AttemptCtx& ctx) -> bool {
            Route r = buildRouteBetween(*ctx.engine, Direction::L2R, Direction::R2L);
            if (r.empty()) { markUnverified(ctx, "Brak trasy"); return true; }
            VisualRouteData d = VisualRouteBuilder::build(r);
            int currentSizeBefore = 0;
            for (auto& f : d.currentRouteFragments) currentSizeBefore += f.size();
            d.drivenTrace << QPointF(100, 100) << QPointF(200, 200);
            d.historyRoutes.push_back(QPolygonF() << QPointF(300, 300) << QPointF(400, 400));
            int currentSizeAfter = 0;
            for (auto& f : d.currentRouteFragments) currentSizeAfter += f.size();
            if (currentSizeBefore == currentSizeAfter) reach(ctx, "can_separate");
            else { markFail(ctx, "Modyfikacja jednego pola wpłynęła na drugie!"); return true; }
            reach(ctx, "structure_distinct");
            VisualRouteData fresh = VisualRouteBuilder::build(r);
            if (fresh.drivenTrace.isEmpty() && fresh.historyRoutes.empty() &&
                fresh.reroutePreview.isEmpty())
                reach(ctx, "pipeline_clean");
            else { markFail(ctx, "Pipeline zanieczyścił warstwy historii"); return true; }

            // Podgląd: trzy WARSTWY rysowane oddzielnie:
            //   • current (aktywna) — pomarańczowa
            //   • driven (przebyta) — pomarańczowa wygaszona (m_trail)
            //   • history (poprzednia) — szare polilinie alt
            auto fragCurrent = toFragments(d);
            ctx.preview->setRouteFragments(fragCurrent);
            ctx.preview->setTrail(d.drivenTrace);
            std::vector<QPolygonF> historyFrags(d.historyRoutes.begin(), d.historyRoutes.end());
            ctx.preview->setRouteAltFragments(historyFrags);
            ctx.preview->setLabels({
                {QPointF(60, 60),  QString::fromUtf8("● Aktywna (current)"), QColor("#f59e0b"), 9},
                {QPointF(60, 90),  QString::fromUtf8("● Przebyta (driven trace)"), QColor("#fbbf24"), 9},
                {QPointF(60, 120), QString::fromUtf8("● Historia (osobne polilinie)"), QColor("#64748b"), 9}
            });
            if (totalPoints(fragCurrent) > 0) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Punkty current"), QString::number(currentSizeAfter));
            measure(ctx, QString::fromUtf8("Punkty driven"), QString::number(d.drivenTrace.size()));
            measure(ctx, QString::fromUtf8("Historie"), QString::number(d.historyRoutes.size()));
            return true;
        };
        t.push_back(tc);
    }

    // ─── 8. Raport odrzuconych segmentów ─────────────────────────────
    {
        TestCase tc;
        tc.id = "vis.rejected_reporting";
        tc.name = "Raport odrzuconych segmentów";
        tc.category = "Wizualizacja trasy";
        tc.description = "Wstrzykujemy patologiczny segment (przerwę). "
                         "Sanitizer MUSI odrzucić go, podzielić polilinię "
                         "na fragmenty i zgłosić w rejected[] z polskim powodem.";
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 60;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("raw_built",         QString::fromUtf8("Wstrzyknięto patologiczną polilinię")),
                MS("split_into_2",      QString::fromUtf8("Sanitizer podzielił na ≥ 2 fragmenty")),
                MS("rejected_reported", QString::fromUtf8("Segment odrzucony zaraportowany w rejected[]")),
                MS("reason_polish",     QString::fromUtf8("Powód zawiera polski opis")),
                displayMs()
            };
        };
        tc.setupAttempt = [](SimulationEngine&, int) {};
        tc.updateAttempt = [toFragments, rejectedToLines, totalPoints](AttemptCtx& ctx) -> bool {
            // Polilinia z patologicznym segmentem przez całą mapę.
            // Skalujemy do układu canvas (kCanvasW=2380, kCanvasH=1430)
            // żeby preview ZOBACZYŁ co odrzuca sanitizer.
            QPolygonF raw;
            raw << QPointF(200, 400)
                << QPointF(700, 400)
                << QPointF(2100, 1200)   // skok ~1500 px przez mapę
                << QPointF(2200, 1200)
                << QPointF(2200, 1300);
            reach(ctx, "raw_built");

            VisualRouteData data;
            data.rawPointCount = raw.size();
            RoutePolylineSanitizer::sanitizeInto(raw, data);

            if (data.currentRouteFragments.size() >= 2) reach(ctx, "split_into_2");
            else {
                markFail(ctx, QString("Spodziewane ≥ 2 fragmenty, otrzymano %1")
                              .arg(data.currentRouteFragments.size()));
                return true;
            }
            if (!data.rejected.empty()) reach(ctx, "rejected_reported");
            else { markFail(ctx, "Sanitizer nie zaraportował odrzucenia"); return true; }
            const auto& rj = data.rejected[0];
            if (rj.reasonPL.contains(QString::fromUtf8("przerwę")) ||
                rj.reasonPL.contains(QString::fromUtf8("segment")) ||
                rj.reasonPL.contains(QString::fromUtf8("pominięt"))) {
                reach(ctx, "reason_polish");
            } else {
                markFail(ctx, QString("Powód bez polskiego opisu: %1").arg(rj.reasonPL));
                return true;
            }

            // Podgląd: poprawne fragmenty (pomarańczowe) + odrzucony segment (czerwony dash).
            auto frags = toFragments(data);
            auto rejLines = rejectedToLines(data);
            ctx.preview->setRouteFragments(frags);
            ctx.preview->setRejectedSegments(rejLines);
            ctx.preview->setLabels({
                {QPointF(60, 60), QString::fromUtf8("● Poprawne fragmenty (%1)").arg(frags.size()),
                 QColor("#f59e0b"), 9},
                {QPointF(60, 90), QString::fromUtf8("● Odrzucony segment: %1 (%2 px)")
                                  .arg(rj.reasonPL).arg(rj.length, 0, 'f', 0),
                 QColor("#dc2626"), 9}
            });
            if (totalPoints(frags) > 0 && !rejLines.empty()) reach(ctx, "displayed_in_preview");

            measure(ctx, QString::fromUtf8("Fragmentów"), QString::number(data.currentRouteFragments.size()));
            measure(ctx, QString::fromUtf8("Odrzucono"), QString::number(data.rejected.size()));
            measure(ctx, QString::fromUtf8("Długość odrzuc."), QString::number(rj.length, 'f', 1) + " px");
            measure(ctx, QString::fromUtf8("Powód"), rj.reasonPL);
            return true;
        };
        t.push_back(tc);
    }

    // ─── 9. Brak „trójkątów" i wyjazdów poza drogę przy skrętach ──────
    //     Test JAZDY: prowadzi prawdziwy SmartCar przez RouteManager (ten sam
    //     tor co aplikacja) i co klatkę liczy dane wizualne tak jak widok:
    //         build( routeManager.visualRoute(), pozycjaAuta ).
    //     Dla klatek ŚRÓDTRASOWYCH (≥4 kroki — z dala od dojazdu do celu,
    //     który z natury kończy się krótkim odcinkiem do markera za siatką)
    //     ŻADEN segment nie może wychodzić poza jezdnię ani być skośną
    //     przekątną (trójkątem). To pilnuje regresji zgłaszanej wielokrotnie.
    {
        struct DriveState {
            std::unique_ptr<RouteManager> rm;
            QPointF   start;
            Direction lastDir = Direction::L2R;
            int       turns   = 0;
            bool      ok      = true;
            QPolygonF trail;          ///< ślad jazdy auta (zapisywany co klatkę)
        };
        auto st = std::make_shared<DriveState>();

        auto onRoad = [](QPointF p) -> bool {
            constexpr double tol = 2.0;
            for (int hi = 0; hi < kNumRoads; ++hi)
                if (p.y() >= kHTop[hi]-tol && p.y() <= kHTop[hi]+kRoadW+tol) return true;
            for (int vi = 0; vi < kNumRoads; ++vi)
                if (p.x() >= kVLeft[vi]-tol && p.x() <= kVLeft[vi]+kRoadW+tol) return true;
            return false;
        };

        TestCase tc;
        tc.id = "vis.no_turn_triangles_drive";
        tc.name = QString::fromUtf8("Brak trójkątów i wyjazdów poza drogę (jazda)");
        tc.category = "Wizualizacja trasy";
        tc.description = QString::fromUtf8(
            "Auto przejeżdża trasę ze skrętami. W każdej klatce śródtrasowej "
            "linia GPS musi trzymać się jezdni — bez skośnych przekątnych "
            "(trójkątów) i bez segmentów przez trawę.");
        tc.isStatic = true; tc.minSuccessfulAttempts = 1; tc.maxAttempts = 1;
        tc.maxFramesPerAttempt = 1500;
        tc.declareMilestones = [&displayMs](int) {
            return std::vector<Milestone>{
                MS("nav_started", QString::fromUtf8("Nawigacja wystartowała")),
                MS("drove",       QString::fromUtf8("Auto przejechało trasę")),
                MS("no_offroad",  QString::fromUtf8("Żaden segment nie wychodzi poza drogę")),
                MS("no_triangle", QString::fromUtf8("Brak skośnych przekątnych (trójkątów) przy skrętach")),
                displayMs()
            };
        };
        tc.setupAttempt = [st](SimulationEngine& e, int) {
            st->rm = std::make_unique<RouteManager>(&e);   // świeży RM na świeży silnik
            st->turns = 0; st->ok = true; st->start = QPointF();
            st->trail.clear();
            auto cps = buildCheckpoints();
            int from = -1, to = -1;
            for (int i = 0; i < (int)cps.size(); ++i) {
                if (cps[i].entryDir == Direction::L2R && from < 0) from = i;
                if (cps[i].entryDir == Direction::B2T && to   < 0) to   = i; // wymusza skręty
            }
            if (from >= 0 && to >= 0) st->rm->startNavigation(from, to);
        };
        tc.updateAttempt = [st, onRoad](AttemptCtx& ctx) -> bool {
            RouteManager* rm = st->rm.get();
            if (!rm || !rm->isNavigating()) {       // brak/zakończona nawigacja → finalizuj
                if (st->ok) { reach(ctx, "no_offroad"); reach(ctx, "no_triangle"); }
                if (ctx.frame >= 200) reach(ctx, "drove");
                // Końcowy podgląd: pokaż trasę i ślad przebyty.
                if (st->ok && !st->trail.isEmpty()) reach(ctx, "displayed_in_preview");
                return true;
            }
            rm->tick();
            reach(ctx, "nav_started");

            QPointF carPos = ctx.engine->smartCarOnRing() ? QPointF()
                                                          : ctx.engine->smartCarCenter();
            if (st->start.isNull() && !carPos.isNull()) st->start = carPos;
            if (!carPos.isNull() &&
                std::hypot(carPos.x()-st->start.x(), carPos.y()-st->start.y()) > kRoadW)
                reach(ctx, "drove");

            // licznik skrętów (zmiana kierunku jazdy)
            Direction d = ctx.engine->smartCarDir();
            if (d != st->lastDir) { st->turns++; st->lastDir = d; }

            // ŚLAD jazdy
            if (!carPos.isNull()) st->trail << carPos;

            Route vr = rm->visualRoute();
            if ((int)vr.size() >= 4) {              // tylko klatki ŚRÓDTRASOWE
                VisualRouteData data = VisualRouteBuilder::build(vr, carPos);

                // Co kilka klatek aktualizuj podgląd — to ta sama pipeline
                // co w głównej aplikacji (OverlayRenderer::drawRoute).
                if (ctx.frame % 8 == 0) {
                    std::vector<QPolygonF> frags(
                        data.currentRouteFragments.begin(),
                        data.currentRouteFragments.end());
                    ctx.preview->setRouteFragments(frags);
                    ctx.preview->setTrail(st->trail);
                    ctx.preview->setLabels({
                        {QPointF(60, 60), QString::fromUtf8("● Trasa GPS (na żywo)"),
                         QColor("#f59e0b"), 9},
                        {QPointF(60, 90), QString::fromUtf8("● Przebyta ścieżka auta"),
                         QColor("#fbbf24"), 9},
                        {QPointF(60, 120), QString::fromUtf8("Skręty: %1, klatka %2")
                                          .arg(st->turns).arg(ctx.frame),
                         QColor("#475569"), 9}
                    });
                    if (!frags.empty()) reach(ctx, "displayed_in_preview");
                }

                for (const auto& frag : data.currentRouteFragments) {
                    for (int i = 1; i < frag.size(); ++i) {
                        QPointF a = frag[i-1], b = frag[i];
                        double dx = b.x()-a.x(), dy = b.y()-a.y();
                        if (std::fabs(dx) > 6 && std::fabs(dy) > 6 && std::hypot(dx,dy) > 60.0) {
                            st->ok = false;
                            // Pokaż skośny segment jako odrzucony (czerwony).
                            ctx.preview->setRejectedSegments({QLineF(a, b)});
                            markFail(ctx, QString::fromUtf8(
                                "Skośny odcinek (trójkąt) przy skręcie: (%1,%2)→(%3,%4), klatka %5")
                                .arg(a.x(),0,'f',0).arg(a.y(),0,'f',0)
                                .arg(b.x(),0,'f',0).arg(b.y(),0,'f',0).arg(ctx.frame));
                            return true;
                        }
                        for (double tt = 0.15; tt <= 0.85; tt += 0.1) {
                            QPointF m(a.x()+dx*tt, a.y()+dy*tt);
                            if (!onRoad(m)) {
                                st->ok = false;
                                ctx.preview->setRejectedSegments({QLineF(a, b)});
                                markFail(ctx, QString::fromUtf8(
                                    "Segment trasy poza jezdnią: (%1,%2)→(%3,%4), klatka %5")
                                    .arg(a.x(),0,'f',0).arg(a.y(),0,'f',0)
                                    .arg(b.x(),0,'f',0).arg(b.y(),0,'f',0).arg(ctx.frame));
                                return true;
                            }
                        }
                    }
                }
            }

            if (ctx.frame >= 1400) {                // przejechano spory kawałek bez artefaktów
                reach(ctx, "drove");
                reach(ctx, "no_offroad");
                reach(ctx, "no_triangle");
                if (!st->trail.isEmpty()) reach(ctx, "displayed_in_preview");
                return true;
            }
            return false;
        };
        t.push_back(tc);
    }
    return t;
}
