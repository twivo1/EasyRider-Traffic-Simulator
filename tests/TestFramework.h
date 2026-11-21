/**
 * @file TestFramework.h
 * @brief Typy i model testów: TestCase, AttemptResult, Milestone, EvidenceReport, statusy.
 * @ingroup tests_grp
 */
#pragma once
#include <QString>
#include <QPointF>
#include <QList>
#include <QPair>
#include <QStringList>
#include <functional>
#include <vector>
#include <map>

class SimulationEngine;
class TestPreviewView;

enum class TestStatus { Pending, Running, Pass, Fail };

// ──────────────────────────────────────────────────────────────────────
//   Attempt-based model — test musi rzeczywiście OBSERWOWAĆ wymagane
//   zachowanie w wielu próbach. Czas nie kończy testu; kończy go
//   osiągnięcie wymaganej liczby udanych prób (każda potwierdzona
//   przez realne milestony z silnika).
// ──────────────────────────────────────────────────────────────────────

enum class AttemptStatus {
    Running,       ///< próba w trakcie
    Passed,        ///< wszystkie milestony osiągnięte
    Failed,        ///< zaobserwowano naruszenie warunku
    Unverified,    ///< nie udało się zaobserwować wymaganego zachowania (timeout)
    Timeout        ///< traktowane jak Unverified, ale rozróżnione dla raportu
};

struct Milestone {
    QString id;              ///< stabilny identyfikator (np. "spawned")
    QString labelPL;         ///< polski opis wyświetlany w UI
    bool    reached = false;
    int     frameReached = -1;
};

struct AttemptResult {
    AttemptStatus status = AttemptStatus::Running;
    QString reasonPL;
    std::vector<Milestone> milestones;
    QList<QPair<QString,QString>> measured;   ///< klucz→wartość
    QStringList events;                       ///< chronologiczny log zdarzeń
    int seed = 0;
    int startFrame = 0;
    int endFrame = -1;
};

struct EvidenceReport {
    QString testName;
    QString category;
    int     totalAttempts = 0;
    int     passed = 0;
    int     failed = 0;
    int     unverified = 0;
    std::vector<AttemptResult> attempts;
    TestStatus finalStatus = TestStatus::Pending;
};

/// Kontekst próby — testy odczytują silnik i mutują AttemptResult.
struct AttemptCtx {
    SimulationEngine* engine;
    int               attemptIdx;     ///< 0-based
    int               frame;
    AttemptResult*    result;
    TestPreviewView*  preview;
};

/// Definicja przypadku testowego (attempt-based).
struct TestCase {
    QString id;
    QString name;
    QString category;
    QString description;

    int  minSuccessfulAttempts = 5;   ///< ile MUSI się powieść
    int  maxAttempts           = 8;   ///< limit twardy (z marginesem na niepowodzenia)
    int  maxFramesPerAttempt   = 600; ///< 10 s @ 60 fps — twardy safety timeout
    bool isStatic              = false; ///< test inspekcyjny (1 próba, krokowanie)

    /// Deklaruje listę milestonów wymaganych dla każdej próby
    /// (otrzymuje numer próby — może się zmieniać między próbami).
    std::function<std::vector<Milestone>(int attemptIdx)> declareMilestones;

    /// Buduje scenariusz dla danej próby (deterministycznie po attemptIdx).
    std::function<void(SimulationEngine&, int attemptIdx)> setupAttempt;

    /// Wywoływana co klatkę. Test:
    ///   • odczytuje stan silnika,
    ///   • zaznacza osiągnięte milestony przez reach(),
    ///   • aktualizuje nakładki podglądu (preview),
    ///   • zwraca TRUE gdy próba zakończona (status w ctx.result musi być
    ///     ustawiony przez markPass/markFail/markUnverified, lub
    ///     wszystkie milestony osiągnięte → runner ustawi Passed).
    std::function<bool(AttemptCtx&)> updateAttempt;
};

struct TestCategory {
    QString               name;
    QString               accent;
    std::vector<TestCase> tests;
};

// ────────── HELPERY ────────────────────────────────────────────────
inline Milestone MS(const char* id, const QString& label) {
    return Milestone{QString::fromUtf8(id), label, false, -1};
}

inline bool isReached(const AttemptResult& r, const char* id) {
    QString qid = QString::fromUtf8(id);
    for (auto& m : r.milestones) if (m.id == qid) return m.reached;
    return false;
}

inline void reach(AttemptCtx& ctx, const char* id) {
    QString qid = QString::fromUtf8(id);
    for (auto& m : ctx.result->milestones)
        if (m.id == qid && !m.reached) {
            m.reached = true;
            m.frameReached = ctx.frame;
            ctx.result->events << QString("[%1] %2")
                .arg(ctx.frame, 4)
                .arg(m.labelPL);
            return;
        }
}

inline bool allMilestonesReached(const AttemptResult& r) {
    for (auto& m : r.milestones) if (!m.reached) return false;
    return true;
}

inline void markPass(AttemptCtx& ctx, const QString& reason = QString()) {
    ctx.result->status = AttemptStatus::Passed;
    if (!reason.isEmpty()) ctx.result->reasonPL = reason;
}
inline void markFail(AttemptCtx& ctx, const QString& reason) {
    ctx.result->status = AttemptStatus::Failed;
    ctx.result->reasonPL = reason;
}
inline void markUnverified(AttemptCtx& ctx, const QString& reason) {
    ctx.result->status = AttemptStatus::Unverified;
    ctx.result->reasonPL = reason;
}

inline void measure(AttemptCtx& ctx, const QString& key, const QString& value) {
    // Zastąp jeśli już istnieje (zwykle chcemy ostatnią/finalną wartość)
    for (auto& p : ctx.result->measured)
        if (p.first == key) { p.second = value; return; }
    ctx.result->measured.append({key, value});
}
