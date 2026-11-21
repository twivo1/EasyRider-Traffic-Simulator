/**
 * @file TestWindow.h
 * @brief Okno runnera testów — ładuje ui/test_window.ui, prowadzi pętlę prób i anulowanie.
 * @ingroup tests_grp
 */
#pragma once
#include <QMainWindow>
#include <QVector>
#include <atomic>
#include <memory>
#include "TestFramework.h"

class QScrollArea;
class QLabel;
class QPushButton;
class QProgressBar;
class QVBoxLayout;
class TestPreviewView;
class QFrame;
class SimulationEngine;

namespace Ui { class TestWindow; }   ///< Klasa generowana przez uic z ui/test_window.ui

/**
 * @brief Okno testów z białym/beżowym motywem i podglądem symulacji na żywo.
 *
 * Lewy panel: lista kategorii (10) i ich testy (50 łącznie) z ikonami stanu
 * (oczekuje / trwa / zaliczony / niezweryfikowany / niezaliczony). KAŻDY
 * wiersz testu jest KLIKALNY — kliknięcie uruchamia ten test, animując
 * symulację w prawym panelu, a po zakończeniu pozostawia jej stan końcowy
 * do obejrzenia. Stan „niezweryfikowany" jest etykietą WYŁĄCZNIE wizualną
 * (patrz displayStateFor w TestWindow.cpp) — nie zmienia werdyktu Pass/Fail.
 *
 * Układ okna pochodzi z Qt Designer (@c ui/test_window.ui) i jest budowany
 * przez @c setupUi(); @ref bindFromUi() wiąże wskaźniki składowe z widżetami
 * z .ui, a @ref buildTestList() dosypuje DYNAMICZNĄ listę kategorii/testów do
 * gotowego @c QScrollArea. Podgląd @ref TestPreviewView jest widżetem
 * PROMOWANYM. Logika wykonania testów pozostaje bez zmian.
 */
class TestWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit TestWindow(QWidget* parent = nullptr);
    ~TestWindow() override;

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    void runAllTests();
    void cancelTests();

private:
    void buildUI();
    void bindFromUi();                 ///< Powiązanie wskaźników z widżetami z .ui
    void buildTestList();              ///< Dynamiczne karty kategorii + wiersze testów
    void renderTestRow(int globalIdx);
    void updateSummary();
    void setCurrentTest(int globalIdx, const TestCase* tc);
    void runSingleTest(int globalIdx);
    void updateRunControls();          ///< Pokazuje run/cancel zależnie od stanu.

    std::vector<TestCategory> m_categories;

    Ui::TestWindow* ui = nullptr;      ///< Widżety z ui/test_window.ui

    struct FlatRef { int catIdx; int testIdx; };
    QVector<FlatRef>        m_flat;
    QVector<TestStatus>     m_status;
    QVector<QString>        m_detail;
    QVector<EvidenceReport> m_evidence;
    QVector<QLabel*>        m_rowIcons;
    QVector<QLabel*>        m_rowLabels;

    std::unique_ptr<SimulationEngine> m_currentEngine;

    bool              m_runningTest = false;   ///< Re-entrancy guard.
    bool              m_runningSuite = false;  ///< Cały batch (runAllTests).
    std::atomic<bool> m_cancelRequested{false};

    QLabel*           m_passCard   = nullptr;
    QLabel*           m_failCard   = nullptr;
    QLabel*           m_pendCard   = nullptr;
    QPushButton*      m_runBtn     = nullptr;
    QPushButton*      m_cancelBtn  = nullptr;
    QProgressBar*     m_progress   = nullptr;
    QScrollArea*      m_listScroll = nullptr;
    TestPreviewView*  m_preview    = nullptr;
    QLabel*           m_curTitle   = nullptr;
    QLabel*           m_curCat     = nullptr;
    QLabel*           m_curDesc    = nullptr;
    QLabel*           m_curStatus  = nullptr;
    QLabel*           m_curDetail  = nullptr;
    QLabel*           m_evidenceTitle = nullptr;
    QLabel*           m_evidenceText  = nullptr;
};
