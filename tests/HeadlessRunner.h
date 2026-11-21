#pragma once
/**
 * @file HeadlessRunner.h
 * @brief Bezgłowy (konsolowy) wykonawca testów EasyRider.
 *
 *  Uruchamia DOKŁADNIE te same testy co interfejs graficzny
 *  (@ref buildAllCategories) i stosuje IDENTYCZNĄ regułę werdyktu co
 *  metoda TestWindow::runSingleTest():
 *
 *      próba zaliczona  ⇔ wszystkie milestony osiągnięte lub markPass(),
 *      test  zaliczony  ⇔ liczba udanych prób ≥ minSuccessfulAttempts.
 *
 *  Różnice względem GUI są wyłącznie INFRASTRUKTURALNE (nie wpływają na
 *  mechanikę): brak okna, brak ograniczenia do 60 fps oraz brak pauz
 *  między próbami. Dzięki temu pełny przebieg 51 testów trwa sekundy (audyt mechaniki: ~2 min),
 *  co umożliwia uruchamianie wsadowe (CI) oraz wielokrotną weryfikację.
 *
 *  @warning Pętla prób poniżej jest lustrzanym odbiciem pętli z
 *           TestWindow::runSingleTest. Przy modyfikacji JEDNEJ należy
 *           zaktualizować DRUGĄ, aby werdykt headless == werdykt GUI.
 */
#include "TestFramework.h"
#include "TestPreviewView.h"
#include "TestSuite.h"
#include "../src/core/SimulationEngine.h"

#include <QString>
#include <cstdio>
#include <vector>

namespace HeadlessRunner {

/// Zwięzły wynik pojedynczego testu (do wydruku w konsoli).
struct TestOutcome {
    QString    name;
    QString    category;
    TestStatus status     = TestStatus::Pending;
    int        attempts   = 0;
    int        passed     = 0;
    int        failed     = 0;
    int        unverified = 0;
};

/**
 * @brief Wykonuje pojedynczy test attempt-based bez UI i bez rate-limitera.
 * @param tc       Definicja testu (mutowalna — lambdy mogą trzymać stan).
 * @param preview  Offscreenowy podgląd (NIGDY nie pokazany) — istnieje tylko
 *                 po to, by settery @c ctx.preview->... działały identycznie
 *                 jak w GUI. Może być @c nullptr.
 */
inline TestOutcome runOne(TestCase& tc, TestPreviewView* preview) {
    int passed = 0, failed = 0, unverified = 0;
    std::vector<AttemptResult> attempts;

    for (int attempt = 0; attempt < tc.maxAttempts; ++attempt) {
        if (passed >= tc.minSuccessfulAttempts) break;

        AttemptResult result;
        result.seed = attempt * 17 + 31;   // metadana — identyczna jak w GUI
        if (tc.declareMilestones)
            result.milestones = tc.declareMilestones(attempt);

        // Nowy engine dla każdej próby — tak samo jak w runSingleTest.
        SimulationEngine engine;
        engine.setSeed(static_cast<unsigned>(result.seed));   // jak w GUI (TestWindow)
        if (preview) preview->setEngine(&engine);

        try {
            if (tc.setupAttempt) tc.setupAttempt(engine, attempt);
        } catch (...) {
            result.status   = AttemptStatus::Failed;
            result.reasonPL = QString::fromUtf8("Wyjątek podczas setupAttempt()");
        }

        const double secPerStep = 1.0 / 60.0;   // stały dt — determinizm
        bool done = (result.status != AttemptStatus::Running);

        for (int frame = 0; frame < tc.maxFramesPerAttempt && !done; ++frame) {
            engine.update(secPerStep);
            AttemptCtx ctx{&engine, attempt, frame, &result, preview};
            try {
                if (tc.updateAttempt) done = tc.updateAttempt(ctx);
            } catch (...) {
                markFail(ctx, QString::fromUtf8("Wyjątek podczas updateAttempt()"));
                done = true;
            }
            // Auto-pass gdy wszystkie milestony osiągnięte (jak w GUI).
            if (!done && allMilestonesReached(result) &&
                result.status == AttemptStatus::Running) {
                result.status = AttemptStatus::Passed;
                done = true;
            }
        }

        // Brak werdyktu po pętli == timeout → Niezweryfikowana (jak w GUI).
        if (result.status == AttemptStatus::Running) {
            result.status = allMilestonesReached(result)
                                ? AttemptStatus::Passed
                                : AttemptStatus::Unverified;
        }

        switch (result.status) {
        case AttemptStatus::Passed:     passed++;     break;
        case AttemptStatus::Failed:     failed++;     break;
        case AttemptStatus::Unverified:
        case AttemptStatus::Timeout:    unverified++; break;
        default: break;
        }
        attempts.push_back(result);

        if (preview) preview->setEngine(nullptr);   // bez wiszącego wskaźnika
    }

    TestOutcome o;
    o.name       = tc.name;
    o.category   = tc.category;
    o.attempts   = static_cast<int>(attempts.size());
    o.passed     = passed;
    o.failed     = failed;
    o.unverified = unverified;
    o.status     = (passed >= tc.minSuccessfulAttempts) ? TestStatus::Pass
                                                        : TestStatus::Fail;
    return o;
}

/**
 * @brief Uruchamia WSZYSTKIE testy @p repeats razy i wypisuje raport.
 * @return 0 gdy w każdym przebiegu wszystkie testy zaliczone, inaczej 1.
 */
inline int runAll(int repeats) {
    int overallFail = 0;

    for (int r = 1; r <= repeats; ++r) {
        std::vector<TestCategory> cats = buildAllCategories();
        TestPreviewView preview;           // offscreen, nigdy nie pokazany

        int total = 0, pass = 0, fail = 0;
        std::printf("\n========== PRZEBIEG %d / %d ==========\n", r, repeats);

        for (auto& cat : cats) {
            for (auto& tc : cat.tests) {
                TestOutcome o = runOne(tc, &preview);
                ++total;
                const bool ok = (o.status == TestStatus::Pass);
                if (ok) ++pass; else { ++fail; ++overallFail; }

                const QString line = cat.name + " / " + o.name;
                std::printf("[%s] %-46s  proby:%d  ok:%d  zle:%d  nzw:%d\n",
                            ok ? "ZAL" : "NIE",
                            line.toUtf8().constData(),
                            o.attempts, o.passed, o.failed, o.unverified);
            }
        }
        std::printf("---------- Przebieg %d: %d / %d zaliczone ----------\n",
                    r, pass, total);
    }

    std::printf("\n=== PODSUMOWANIE: %s ===\n",
                overallFail == 0 ? "WSZYSTKIE PRZEBIEGI ZALICZONE"
                                 : "WYKRYTO NIEPOWODZENIA");
    return overallFail == 0 ? 0 : 1;
}

} // namespace HeadlessRunner
