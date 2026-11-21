#include "TestWindow.h"
#include "ui_test_window.h"      // wygenerowane przez uic z ui/test_window.ui
#include "TestSuite.h"
#include "TestPreviewView.h"
#include "TestCategoryCard.h"
#include "TestRow.h"
#include "../src/core/SimulationEngine.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QScrollArea>
#include <QApplication>
#include <QFrame>
#include <QElapsedTimer>
#include <QEvent>
#include <QMouseEvent>
#include <QThread>

// ─────────────────────────────────────────────────────────────────────
//   POMOCNICZE — fabryki małych elementów UI w jednolitym stylu
// ─────────────────────────────────────────────────────────────────────
//  Karty podsumowania (ZALICZONE/NIEZALICZONE/DO URUCHOMIENIA) są w
//  ui/test_window.ui; karta kategorii i wiersz testu mają własne szablony
//  Qt Designer: ui/widgets/test_category_card.ui i ui/widgets/test_row.ui.

// ─────────────────────────────────────────────────────────────────────
//   STAN WYŚWIETLANIA (TYLKO UI — bez wpływu na mechanikę/werdykt)
// ─────────────────────────────────────────────────────────────────────
//  Werdykt wykonawcy testów to wyłącznie Pass/Fail (patrz runSingleTest:
//  finalPass = passed >= minSuccessfulAttempts). Poniżej wzbogacamy go o
//  etykietę „Niezweryfikowany": jest to Fail, w którym NIE zaobserwowano
//  żadnego twardego naruszenia (failed == 0), a zabrakło jedynie
//  potwierdzonych prób (unverified > 0). Rozróżnienie jest CZYSTO wizualne —
//  NIE zmienia m_status, zliczeń kart podsumowania ani raportu dowodowego.
enum class DisplayState { Pending, Running, Passed, Unverified, Failed };

static DisplayState displayStateFor(TestStatus s, const EvidenceReport& r) {
    switch (s) {
    case TestStatus::Pending: return DisplayState::Pending;
    case TestStatus::Running: return DisplayState::Running;
    case TestStatus::Pass:    return DisplayState::Passed;
    case TestStatus::Fail:
        if (r.totalAttempts > 0 && r.failed == 0 && r.unverified > 0)
            return DisplayState::Unverified;
        return DisplayState::Failed;
    }
    return DisplayState::Pending;
}

/// Glif + styl ikony i etykiety wiersza listy dla danego stanu wyświetlania.
struct RowStyle { QString glyph; QString iconCss; QString labelCss; };
static RowStyle rowStyleFor(DisplayState d) {
    switch (d) {
    case DisplayState::Running:
        return {QString::fromUtf8("◐"),
                "color:#f59e0b; font-size:16px; font-weight:700; background:transparent;",
                "color:#1c1917; font-size:12px; font-weight:600; background:transparent;"};
    case DisplayState::Passed:
        return {QString::fromUtf8("✓"),
                "color:#16a34a; font-size:15px; font-weight:800; background:transparent;",
                "color:#1c1917; font-size:12px; font-weight:500; background:transparent;"};
    case DisplayState::Unverified:
        return {QString::fromUtf8("◌"),
                "color:#b45309; font-size:16px; font-weight:800; background:transparent;",
                "color:#92400e; font-size:12px; font-weight:600; background:transparent;"};
    case DisplayState::Failed:
        return {QString::fromUtf8("✗"),
                "color:#dc2626; font-size:15px; font-weight:800; background:transparent;",
                "color:#dc2626; font-size:12px; font-weight:700; background:transparent;"};
    case DisplayState::Pending:
    default:
        return {QString::fromUtf8("•"),
                "color:#d6d3d1; font-size:18px; font-weight:700; background:transparent;",
                "color:#44403c; font-size:12px; font-weight:500; background:transparent;"};
    }
}

/// Tekst + styl „pigułki" statusu w panelu szczegółów (4 znaczniki + oczekuje).
struct PillStyle { QString text; QString css; };
static PillStyle pillStyleFor(DisplayState d) {
    const QString base =
        "padding:4px 12px; border-radius:2px; font-size:11px; font-weight:700;";
    switch (d) {
    case DisplayState::Running:
        return {QString::fromUtf8("◐  Trwa"),
                "color:#92400e; background:#fef3c7; border:1px solid #fde68a;" + base};
    case DisplayState::Passed:
        return {QString::fromUtf8("✓  Zaliczony"),
                "color:#166534; background:#dcfce7; border:1px solid #bbf7d0;" + base};
    case DisplayState::Unverified:
        return {QString::fromUtf8("◌  Niezweryfikowany"),
                "color:#92400e; background:#fffbeb; border:1px solid #fde68a;" + base};
    case DisplayState::Failed:
        return {QString::fromUtf8("✗  Niezaliczony"),
                "color:#991b1b; background:#fee2e2; border:1px solid #fecaca;" + base};
    case DisplayState::Pending:
    default:
        return {QString::fromUtf8("⏸  Oczekuje"),
                "color:#78716c; background:#f5f5f4; border:1px solid #e7e5e4;" + base};
    }
}

// ─────────────────────────────────────────────────────────────────────
//   KONSTRUKTOR
// ─────────────────────────────────────────────────────────────────────

TestWindow::TestWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::TestWindow) {
    // Tytuł okna, rozmiar startowy i arkusz stylów (QSS) pochodzą teraz
    // z ui/test_window.ui — patrz buildUI()/setupUi().

    m_categories = buildAllCategories();
    for (int ci = 0; ci < (int)m_categories.size(); ++ci)
        for (int ti = 0; ti < (int)m_categories[ci].tests.size(); ++ti) {
            m_flat.append({ci, ti});
            m_status.append(TestStatus::Pending);
            m_detail.append("");
            m_evidence.append(EvidenceReport{});
        }
    buildUI();
}

TestWindow::~TestWindow() {
    // Odłącz preview od silnika ZANIM unique_ptr go usunie (wisi pointer).
    if (m_preview) m_preview->setEngine(nullptr);
    delete ui;
}

// ─────────────────────────────────────────────────────────────────────
//   BUDOWA UI
// ─────────────────────────────────────────────────────────────────────

void TestWindow::buildUI() {
    // Całe drzewo widżetów (układy, karty, panele, przyciski, pasek postępu,
    // promowany podgląd TestPreviewView) pochodzi z Qt Designer:
    ui->setupUi(this);
    bindFromUi();

    // Zakres paska postępu zależy od liczby testów (dynamiczny).
    m_progress->setRange(0, m_flat.size());

    connect(m_runBtn,    &QPushButton::clicked, this, &TestWindow::runAllTests);
    connect(m_cancelBtn, &QPushButton::clicked, this, &TestWindow::cancelTests);

    buildTestList();              // dynamiczne karty kategorii do listScroll z .ui
    updateSummary();              // policz karty podsumowania + pasek postępu
    setCurrentTest(-1, nullptr);  // stan początkowy panelu szczegółów
}

// Powiązanie wskaźników składowych z widżetami utworzonymi przez setupUi().
// Dzięki temu cała dotychczasowa logika (renderTestRow, updateSummary,
// runSingleTest, eventFilter…) działa bez zmian na tych samych obiektach.
void TestWindow::bindFromUi() {
    m_passCard      = ui->passCard;
    m_failCard      = ui->failCard;
    m_pendCard      = ui->pendCard;
    m_runBtn        = ui->runBtn;
    m_cancelBtn     = ui->cancelBtn;
    m_progress      = ui->progress;
    m_listScroll    = ui->listScroll;
    m_preview       = ui->preview;
    m_curCat        = ui->curCat;
    m_curTitle      = ui->curTitle;
    m_curDesc       = ui->curDesc;
    m_curStatus     = ui->curStatus;
    m_curDetail     = ui->curDetail;
    m_evidenceTitle = ui->evidenceTitle;
    m_evidenceText  = ui->evidenceText;
}

// ─────────────────────────────────────────────────────────────────────
//   Lewy panel — DYNAMICZNE karty kategorii i wiersze testów.
//   Wstrzykiwane do gotowego QScrollArea „listScroll" zdefiniowanego w .ui
//   (treść listy jest generowana z danych, więc pozostaje w kodzie).
// ─────────────────────────────────────────────────────────────────────
void TestWindow::buildTestList() {
    auto* container = new QWidget();
    container->setStyleSheet("background:transparent;");
    auto* lay = new QVBoxLayout(container);
    lay->setContentsMargins(0, 0, 6, 0);
    lay->setSpacing(6);

    int globalIdx = 0;
    for (int ci = 0; ci < (int)m_categories.size(); ++ci) {
        const TestCategory& cat = m_categories[ci];

        // Karta kategorii — instancja szablonu z Qt Designer
        // (ui/widgets/test_category_card.ui); kod podaje tylko dane.
        auto* catCard = new TestCategoryCard(
            cat.name, cat.accent, (int)cat.tests.size());

        // Wiersze testów — instancje szablonu ui/widgets/test_row.ui.
        // KAŻDY klikalny: kliknięcie uruchamia test w podglądzie.
        for (int ti = 0; ti < (int)cat.tests.size(); ++ti) {
            const TestCase& tc = cat.tests[ti];

            auto* row = new TestRow(tc.name);
            row->setProperty("testIdx", globalIdx);
            row->installEventFilter(this);
            catCard->addRow(row);

            m_rowIcons.append(row->icon());
            m_rowLabels.append(row->label());
            globalIdx++;
        }

        lay->addWidget(catCard);
    }

    lay->addStretch();
    m_listScroll->setWidget(container);   // QScrollArea pochodzi z .ui
}


// ─────────────────────────────────────────────────────────────────────
//   Aktualizacja UI dla pojedynczego wiersza i podsumowania
// ─────────────────────────────────────────────────────────────────────

void TestWindow::renderTestRow(int globalIdx) {
    if (globalIdx < 0 || globalIdx >= m_rowIcons.size()) return;
    // Styl wiersza wynika ze stanu WYŚWIETLANIA (nie zmienia werdyktu);
    // dla testu zakończonego bez twardego niepowodzenia, lecz bez kompletu
    // potwierdzeń, pokaże bursztynowy znacznik „niezweryfikowany".
    const RowStyle rs = rowStyleFor(displayStateFor(m_status[globalIdx],
                                                     m_evidence[globalIdx]));
    m_rowIcons[globalIdx]->setText(rs.glyph);
    m_rowIcons[globalIdx]->setStyleSheet(rs.iconCss);
    m_rowLabels[globalIdx]->setStyleSheet(rs.labelCss);
}

void TestWindow::updateSummary() {
    int pass = 0, fail = 0, pend = 0;
    for (TestStatus s : m_status) {
        if (s == TestStatus::Pass) pass++;
        else if (s == TestStatus::Fail) fail++;
        else if (s == TestStatus::Pending) pend++;
    }
    auto setCard = [](QLabel* card, const QString& big, const QString& sub,
                      const QString& accent) {
        card->setText(QString(
            "<div style='line-height:1.15'>"
            "<span style='font-size:14px;font-weight:700;color:%1'>%2</span>  "
            "<span style='font-size:11px;color:#78716c;letter-spacing:0.4px'>%3</span>"
            "</div>").arg(accent, big, sub));
    };
    setCard(m_passCard, QString::number(pass),
            QString::fromUtf8("ZALICZONE"),     "#16a34a");
    setCard(m_failCard, QString::number(fail),
            QString::fromUtf8("NIEZALICZONE"),  "#dc2626");
    setCard(m_pendCard, QString::number(pend),
            QString::fromUtf8("DO URUCHOMIENIA"),"#92400e");

    int done = pass + fail;
    m_progress->setValue(done);
    auto* pLbl = findChild<QLabel*>("progressLabel");
    if (pLbl) pLbl->setText(QString("%1 / %2").arg(done).arg(m_flat.size()));
}

void TestWindow::setCurrentTest(int globalIdx, const TestCase* tc) {
    if (!tc) {
        m_curCat->setText(QString::fromUtf8("PRZED URUCHOMIENIEM"));
        m_curTitle->setText(QString::fromUtf8("Wybierz test lub uruchom wszystkie"));
        m_curDesc->setText(QString::fromUtf8(
            "Każdy test pokazany zostanie tu wraz z opisem oraz wynikiem."));
        m_curStatus->setText(QString::fromUtf8("⏸  Oczekuje"));
        m_curStatus->setStyleSheet(
            "color:#78716c; background:#f5f5f4; border:1px solid #e7e5e4;"
            "padding:4px 12px; border-radius:2px; font-size:11px; font-weight:700;");
        m_curDetail->setText("");
        return;
    }
    int catIdx = m_flat[globalIdx].catIdx;
    const auto& cat = m_categories[catIdx];
    m_curCat->setText(cat.name.toUpper());
    m_curCat->setStyleSheet(QString(
        "color:#666660; font-size:12px; font-weight:600;")
        .arg(cat.accent));
    m_curTitle->setText(tc->name);
    m_curDesc->setText(tc->description);

    // Pigułka statusu — spójna z ikonami listy i legendą (stan wyświetlania).
    const PillStyle ps = pillStyleFor(displayStateFor(m_status[globalIdx],
                                                       m_evidence[globalIdx]));
    m_curStatus->setText(ps.text);
    m_curStatus->setStyleSheet(ps.css);
    m_curDetail->setText(m_detail[globalIdx]);

    // ── Raport dowodowy ─────────────────────────────────────────────
    const EvidenceReport& rep = m_evidence[globalIdx];
    if (rep.totalAttempts == 0) {
        m_evidenceTitle->setVisible(false);
        m_evidenceText->setVisible(false);
        m_evidenceText->setText("");
    } else {
        m_evidenceTitle->setVisible(true);
        m_evidenceText->setVisible(true);
        auto statusToPL = [](AttemptStatus s) -> QString {
            switch (s) {
            case AttemptStatus::Passed:     return "ZALICZONA";
            case AttemptStatus::Failed:     return "NIEZALICZONA";
            case AttemptStatus::Unverified: return "NIEZWERYFIKOWANA";
            case AttemptStatus::Timeout:    return "TIMEOUT";
            default: return "—";
            }
        };
        QString out;
        out += QString::fromUtf8("Test: %1\n").arg(rep.testName);
        out += QString::fromUtf8("Kategoria: %1\n").arg(rep.category);
        out += QString::fromUtf8("Próby łącznie: %1\n").arg(rep.totalAttempts);
        out += QString::fromUtf8("Udane: %1   Nieudane: %2   Niezweryfikowane: %3\n\n")
                   .arg(rep.passed).arg(rep.failed).arg(rep.unverified);

        int show = std::min<int>(rep.attempts.size(), 8);
        for (int i = 0; i < show; ++i) {
            const auto& a = rep.attempts[i];
            out += QString::fromUtf8("── Próba %1 ── seed=%2\n").arg(i+1).arg(a.seed);
            out += QString::fromUtf8("Wynik: %1\n").arg(statusToPL(a.status));
            if (!a.reasonPL.isEmpty())
                out += QString::fromUtf8("Powód: %1\n").arg(a.reasonPL);
            // Milestony
            for (const auto& m : a.milestones) {
                QString mk = m.reached ? QString::fromUtf8("[✓]") : "[✗]";
                out += QString("  %1 %2\n").arg(mk).arg(m.labelPL);
            }
            // Pomiary
            for (const auto& p : a.measured)
                out += QString::fromUtf8("  → %1: %2\n").arg(p.first).arg(p.second);
            out += "\n";
        }
        if (rep.attempts.size() > show)
            out += QString::fromUtf8("… i %1 więcej prób …\n")
                   .arg(rep.attempts.size() - show);
        m_evidenceText->setText(out);
    }
}

// ─────────────────────────────────────────────────────────────────────
//   Wykonanie testów — z faktycznym podglądem symulacji
// ─────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────
//   eventFilter — przechwytuje kliknięcie na wiersz testu
// ─────────────────────────────────────────────────────────────────────

bool TestWindow::eventFilter(QObject* obj, QEvent* event) {
    if (event->type() == QEvent::MouseButtonRelease) {
        auto* me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::LeftButton) {
            QVariant prop = obj->property("testIdx");
            if (prop.isValid()) {
                int idx = prop.toInt();
                if (!m_runningTest) runSingleTest(idx);
                return true;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

// ─────────────────────────────────────────────────────────────────────
//   runSingleTest — uruchamia POJEDYNCZY test (kliknięcie wiersza)
//
//   Po zakończeniu engine pozostaje żywy (member unique_ptr), dzięki
//   czemu podgląd dalej pokazuje STAN KOŃCOWY symulacji — można obejrzeć
//   sytuację która spowodowała ewentualny błąd.
// ─────────────────────────────────────────────────────────────────────

void TestWindow::runSingleTest(int idx) {
    if (idx < 0 || idx >= m_flat.size()) return;
    m_runningTest = true;

    const auto& ref = m_flat[idx];
    TestCase& tc = m_categories[ref.catIdx].tests[ref.testIdx];

    m_preview->setEngine(nullptr);
    m_preview->resetForNewTest();

    m_status[idx] = TestStatus::Running;
    m_detail[idx] = "";
    renderTestRow(idx);
    setCurrentTest(idx, &tc);
    QApplication::processEvents();

    // ── Pętla prób (attempt-based) ─────────────────────────────────
    EvidenceReport report;
    report.testName = tc.name;
    report.category = tc.category;
    int passed = 0, failed = 0, unverified = 0;

    auto statusToPL = [](AttemptStatus s) -> QString {
        switch (s) {
        case AttemptStatus::Passed:     return "Zaliczona";
        case AttemptStatus::Failed:     return "Niezaliczona";
        case AttemptStatus::Unverified: return "Niezweryfikowana";
        case AttemptStatus::Timeout:    return "Timeout";
        default: return "Trwa";
        }
    };
    auto statusColor = [](AttemptStatus s) -> QColor {
        switch (s) {
        case AttemptStatus::Passed:     return QColor("#16a34a");
        case AttemptStatus::Failed:     return QColor("#dc2626");
        case AttemptStatus::Unverified: return QColor("#92400e");
        case AttemptStatus::Timeout:    return QColor("#92400e");
        default: return QColor("#0891b2");
        }
    };

    for (int attempt = 0; attempt < tc.maxAttempts; ++attempt) {
        if (m_cancelRequested.load()) break;
        if (passed >= tc.minSuccessfulAttempts) break;

        int seed = attempt * 17 + 31;
        AttemptResult result;
        result.seed = seed;
        if (tc.declareMilestones) result.milestones = tc.declareMilestones(attempt);

        // Nowy engine dla każdej próby.
        // KOLEJNOŚĆ KRYTYCZNA: najpierw odłączamy preview od STAREGO enginu,
        // potem niszczymy stary i tworzymy nowy, potem podłączamy nowy.
        // Bez tego pomiędzy zniszczeniem starego a podłączeniem nowego
        // paint canvasu mógłby przejść po danglingu (gdy expose event
        // dojdzie z X-servera w GUI).
        m_preview->setEngine(nullptr);
        m_currentEngine = std::make_unique<SimulationEngine>();
        // Ziarno próby faktycznie steruje losowością silnika (dawniej tylko metadana;
        // generator był globalny, więc próby różniły się przypadkowo).
        m_currentEngine->setSeed(static_cast<unsigned>(seed));
        m_preview->setEngine(m_currentEngine.get());
        // Świeży test = świeży podgląd (poprzednie polilinie, ramki, etykiety).
        m_preview->clearRoute();
        m_preview->clearRouteAlt();
        m_preview->clearRouteFragments();
        m_preview->clearRouteAltFragments();
        m_preview->clearRejectedSegments();
        m_preview->clearTrail();
        m_preview->setBoxes({});
        m_preview->setArrows({});
        m_preview->setLabels({});
        m_preview->setMeasures({});
        // Wstępna informacja o próbie (zanim test cokolwiek narysuje)
        PvAttemptInfo ai;
        ai.currentAttempt = attempt + 1;
        ai.requiredAttempts = tc.minSuccessfulAttempts;
        ai.passed = passed; ai.failed = failed; ai.unverified = unverified;
        ai.statusPL = "Trwa"; ai.statusColor = QColor("#0891b2"); ai.visible = true;
        m_preview->setAttemptInfo(ai);

        std::vector<PvMilestone> pvms;
        for (auto& m : result.milestones) pvms.push_back({m.labelPL, m.reached});
        m_preview->setMilestones(pvms);
        m_preview->setCaption(tc.description);

        try { if (tc.setupAttempt) tc.setupAttempt(*m_currentEngine, attempt); }
        catch (...) {
            result.status = AttemptStatus::Failed;
            result.reasonPL = "Wyjątek podczas setupAttempt()";
        }

        // ── Pętla klatek dla TEJ próby (60 fps real-time) ──────────
        QElapsedTimer wallTimer; wallTimer.start();
        const double secPerStep = 1.0 / 60.0;
        bool done = (result.status != AttemptStatus::Running);

        for (int frame = 0; frame < tc.maxFramesPerAttempt && !done; ++frame) {
            if (m_cancelRequested.load()) { done = true; break; }
            m_currentEngine->update(secPerStep);

            AttemptCtx ctx{m_currentEngine.get(), attempt, frame, &result, m_preview};
            try {
                if (tc.updateAttempt) done = tc.updateAttempt(ctx);
            }
            catch (...) {
                markFail(ctx, "Wyjątek podczas updateAttempt()");
                done = true;
            }

            // Aktualizuj milestones na podglądzie (mogły się zmienić)
            if (frame % 4 == 0) {
                std::vector<PvMilestone> pvms2;
                for (auto& m : result.milestones) pvms2.push_back({m.labelPL, m.reached});
                m_preview->setMilestones(pvms2);
                m_preview->update();
                QApplication::processEvents();
            }

            // Rate limit do 60 fps wall-clock
            const double targetSec = (frame + 1) * secPerStep;
            while (wallTimer.nsecsElapsed() / 1.0e9 < targetSec) {
                QApplication::processEvents(QEventLoop::AllEvents, 4);
                QThread::usleep(500);
            }

            // Auto-pass jeśli wszystkie milestony osiągnięte
            if (!done && allMilestonesReached(result) && result.status == AttemptStatus::Running) {
                result.status = AttemptStatus::Passed;
                done = true;
            }
        }

        // Jeśli pętla skończyła się bez markPass/markFail/markUnverified, to timeout
        if (result.status == AttemptStatus::Running) {
            if (allMilestonesReached(result)) {
                result.status = AttemptStatus::Passed;
            } else {
                result.status = AttemptStatus::Unverified;
                int reached = 0;
                for (auto& m : result.milestones) if (m.reached) reached++;
                result.reasonPL = QString::fromUtf8(
                    "Timeout: zaobserwowano %1 z %2 wymaganych milestonów")
                    .arg(reached).arg(result.milestones.size());
            }
        }

        switch (result.status) {
        case AttemptStatus::Passed:     passed++;     break;
        case AttemptStatus::Failed:     failed++;     break;
        case AttemptStatus::Unverified:
        case AttemptStatus::Timeout:    unverified++; break;
        default: break;
        }
        report.attempts.push_back(result);

        // Aktualizuj UI po próbie
        ai.passed = passed; ai.failed = failed; ai.unverified = unverified;
        ai.statusPL = statusToPL(result.status);
        ai.statusColor = statusColor(result.status);
        m_preview->setAttemptInfo(ai);

        std::vector<PvMilestone> pvms3;
        for (auto& m : result.milestones) pvms3.push_back({m.labelPL, m.reached});
        m_preview->setMilestones(pvms3);

        if (m_cancelRequested.load()) break;

        // Małe okno na podejrzenie wyniku próby zanim startuje następna
        if (attempt + 1 < tc.maxAttempts && passed < tc.minSuccessfulAttempts) {
            QElapsedTimer pause; pause.start();
            while (pause.elapsed() < 400) {
                if (m_cancelRequested.load()) break;
                QApplication::processEvents(QEventLoop::AllEvents, 4);
                QThread::usleep(2000);
            }
        }
    }

    // Jeśli anulowano w trakcie, traktuj test jako NIE uruchomiony — bez raportu,
    // bez werdyktu. Status pozostaje Pending, by nie kłamać że został zaliczony
    // przy częściowych próbach.
    if (m_cancelRequested.load()) {
        m_status[idx] = TestStatus::Pending;
        m_detail[idx] = "";
        m_evidence[idx] = EvidenceReport{};
        renderTestRow(idx);
        m_preview->clearVerdict();
        m_runningTest = false;
        return;
    }

    // ── Finalny werdykt: pass jeśli zebraliśmy wymagane sukcesy ────
    bool finalPass = (passed >= tc.minSuccessfulAttempts);
    report.totalAttempts = (int)report.attempts.size();
    report.passed = passed;
    report.failed = failed;
    report.unverified = unverified;
    report.finalStatus = finalPass ? TestStatus::Pass : TestStatus::Fail;

    m_status[idx]   = report.finalStatus;
    m_evidence[idx] = report;
    m_detail[idx]   = QString::fromUtf8(
        "Próby: %1 · Udane: %2 · Nieudane: %3 · Niezweryfikowane: %4")
        .arg(report.totalAttempts).arg(passed).arg(failed).arg(unverified);

    m_preview->setVerdict(finalPass, m_detail[idx]);
    renderTestRow(idx);
    setCurrentTest(idx, &tc);
    updateSummary();
    m_runningTest = false;
}

// ─────────────────────────────────────────────────────────────────────
//   runAllTests — uruchamia po kolei wszystkie 51 testów
// ─────────────────────────────────────────────────────────────────────

void TestWindow::updateRunControls() {
    if (m_runningSuite) {
        m_runBtn->setVisible(false);
        m_cancelBtn->setVisible(true);
        m_cancelBtn->setEnabled(true);
    } else {
        m_runBtn->setVisible(true);
        m_cancelBtn->setVisible(false);
        m_runBtn->setEnabled(true);
        m_runBtn->setText(QString::fromUtf8("▶  Uruchom wszystkie testy"));
    }
}

void TestWindow::cancelTests() {
    if (!m_runningSuite) return;
    // Flaga jest atomicznie sprawdzana przez pętlę prób runSingleTest.
    // Bezpieczne anulowanie: po bieżącej klatce wracamy z pętli.
    m_cancelRequested.store(true);
    m_cancelBtn->setEnabled(false);
    m_cancelBtn->setText(QString::fromUtf8("⏹  Anulowanie..."));
}

void TestWindow::runAllTests() {
    if (m_runningTest || m_runningSuite) return;

    m_runningSuite = true;
    m_cancelRequested.store(false);
    updateRunControls();

    // Reset stanów
    for (int i = 0; i < m_flat.size(); ++i) {
        m_status[i] = TestStatus::Pending;
        m_detail[i] = "";
        m_evidence[i] = EvidenceReport{};
        renderTestRow(i);
    }
    updateSummary();
    m_preview->resetForNewTest();

    int executed = 0;
    for (int idx = 0; idx < m_flat.size(); ++idx) {
        if (m_cancelRequested.load()) break;
        runSingleTest(idx);
        executed++;
        if (m_cancelRequested.load()) break;
    }

    bool wasCanceled = m_cancelRequested.load();
    m_runningSuite = false;
    m_cancelRequested.store(false);
    updateRunControls();

    if (wasCanceled) {
        // Pozostałe testy zostają jako Pending — czyste prawdziwe stany.
        // Tylko sygnalizujemy że suite został zatrzymany przez użytkownika.
        m_curStatus->setText(QString::fromUtf8("✦  Testy anulowane"));
        m_curStatus->setStyleSheet(
            "background:#fef3c7; color:#92400e; "
            "padding:4px 12px; border-radius:2px; font-size:11px; font-weight:700;");
        m_curDetail->setText(
            QString::fromUtf8("Anulowano wykonywanie testów — uruchomiono %1 z %2.")
                .arg(executed).arg(m_flat.size()));
        m_preview->setVerdict(false, QString::fromUtf8("Anulowano wykonywanie testów"));
    } else {
        m_runBtn->setText(QString::fromUtf8("▶  Uruchom ponownie"));
    }
}
