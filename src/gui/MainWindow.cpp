#include "MainWindow.h"
#include "ui_main_window.h"      // wygenerowane przez uic z ui/main_window.ui
#include "../core/SimulationEngine.h"
#include "../core/RoadGeometry.h"
#include "SimulationView.h"
#include "SideMenu.h"
#include "StartScreen.h"
#include "../nav/RouteManager.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QString>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // ── Core objects ─────────────────────────────────────────────────
    m_engine       = new SimulationEngine();
    m_routeManager = new RouteManager(m_engine, this);

    // ── View (promowany widżet z .ui) ────────────────────────────────
    m_view = ui->view;
    m_view->setEngine(m_engine);

    QWidget* central = centralWidget();   // utworzony przez setupUi

    // ── SideMenu (layout column) ─────────────────────────
    m_sideMenu = ui->sideMenu;
    m_sideMenu->hide();
    m_sideMenu->raise();

    m_timer.setTimerType(Qt::PreciseTimer);

    // ── Connections ───────────────────────────────────────────────────
    connect(&m_timer, &QTimer::timeout,   this, &MainWindow::tick);
    connect(m_sideMenu, &SideMenu::speedChanged,    this, &MainWindow::onSpeedChanged);
    connect(m_sideMenu, &SideMenu::routeRequested,  this, &MainWindow::onRouteRequested);
    connect(m_sideMenu, &SideMenu::heatmapToggled,  m_view, &SimulationView::setHeatmapEnabled);
    connect(m_sideMenu, &SideMenu::themeChanged,    m_view, &SimulationView::setTheme);
    connect(m_sideMenu, &SideMenu::cancelRouteRequested, this, &MainWindow::onCancelRoute);
    connect(m_sideMenu, &SideMenu::backToMenuRequested, this, [this]() {
        // ── Powrót do menu startowego ───────────────────────────────
        // 1. Zatrzymaj symulację, 2. anuluj nawigację, 3. wyzeruj silnik,
        // 4. zwiń panel boczny, 5. pokaż ponownie ekran startowy.
        m_timer.stop();
        m_routeManager->stopNavigation();
        m_navState = NavState::Idle;
        m_view->clearRoute();
        m_view->setStatsText("");
        m_view->setRouteSelectionMode(false);
        m_sideMenu->setRouteSelectionMode(false);
        m_sideMenu->setNavStats(0, 0, 0, 0);
        if (m_sideMenu->isOpen()) m_sideMenu->toggle();
        m_engine->reset();
        m_view->resetPresentation();
        m_sideMenu->hide();
        m_view->update();
        m_startScreen->setPage(StartScreen::Page::Title);
        m_startScreen->setGeometry(centralWidget()->rect());
        // Przywróć przyjmowanie zdarzeń myszy (ustawione na transparent
        // przy startSimulation jako zabezpieczenie przed blokowaniem klików).
        m_startScreen->setAttribute(Qt::WA_TransparentForMouseEvents, false);
        m_startScreen->show();
        m_startScreen->raise();
        updateBottomStatus(QString::fromUtf8("  Symulacja zatrzymana"),
                           QString::fromUtf8("  Trasa: —"));
    });

    connect(m_view, &SimulationView::checkpointClicked, this, &MainWindow::onCheckpointClicked);

    connect(m_routeManager, &RouteManager::routeChanged,      this, &MainWindow::onRouteChanged);
    connect(m_routeManager, &RouteManager::navigationFinished,this, &MainWindow::onNavigationFinished);
    connect(m_routeManager, &RouteManager::navigationFailed,  this, [this](){
        m_navState = NavState::Idle;
        m_view->clearRoute();
        m_view->setStatsText(QString::fromUtf8(" Samochód zgubił trasę"));
        m_sideMenu->setNavStats(0,0,0,0);
        updateBottomStatus(ui->labelSimStatus->text(),
                           QString::fromUtf8("  Trasa: zgubiona"));
        QTimer::singleShot(3000, this, [this](){
            m_view->setStatsText("");
            updateBottomStatus(ui->labelSimStatus->text(),
                               QString::fromUtf8("  Trasa: —"));
        });
    });

    // Timer rusza dopiero po "Akceptuj" — symulacja jest wstrzymana w menu.
    // m_timer.start(16) wywoływane jest po zaakceptowaniu parametrów.

    // Odroczone przeliczenie geometrii menu (po pierwszym ułożeniu układu).
    QTimer::singleShot(0, this, [this]() { m_sideMenu->reposition(); });


    // ── Start screen (configuration pages) ─────────────────
    m_startScreen = new StartScreen(m_engine, m_view, central);
    m_startScreen->raise();
    connect(m_startScreen, &StartScreen::startSimulation, this,
            [this](int nCars, MapTheme theme){
        m_view->setTheme(theme);          // zastosuj wybrany motyw
        m_sideMenu->setSelectedTheme(theme);
        m_sideMenu->show();
        if (!m_sideMenu->isOpen()) m_sideMenu->toggle();
        m_view->fitMap();
        m_engine->spawnCarsOnMap(nCars);  // wygeneruj ruch
        m_timer.start(16);                // uruchom symulację
        m_sideMenu->reposition();
        int rounds = 0;
        for (int vi=0; vi<kNumRoads; ++vi)
            for (int hi=0; hi<kNumRoads; ++hi)
                if (m_engine->intersectionAt(vi, hi).isRoundabout) ++rounds;
        ui->statsLabel->setText(QString("Ronda: %1 · Sygnalizacja: %2").arg(rounds).arg(kNumRoads*kNumRoads-rounds));
        updateBottomStatus(QString("  Ruch aktywny · Pojazdy: %1").arg(nCars),
                           QString::fromUtf8("  Trasa: —"));
    });
}

MainWindow::~MainWindow() {
    delete ui;            // zwolnij obiekt widżetów z .ui
}

// ── Zmiana rozmiaru: przelicz położenie nakładki menu bocznego ──────
void MainWindow::resizeEvent(QResizeEvent* e) {
    QMainWindow::resizeEvent(e);
    if (m_sideMenu && centralWidget())
        m_sideMenu->reposition();
    if (m_startScreen && centralWidget())
        m_startScreen->setGeometry(centralWidget()->rect());
}

// ── Main tick ────────────────────────────────────────────────────────
void MainWindow::tick() {
    // RouteManager przekazuje SmartCar intencję skrętu PRZED krokiem silnika.
    m_routeManager->tick();

    m_engine->update(0.016 * m_speedMult);
    m_view->captureFrame();

    int n = (int)m_engine->objects().size();
    m_sideMenu->setCarCount(n);

    // Aktualizacja labelSimStatus w infoBar (z .ui): bieżąca liczba pojazdów.
    if (ui && ui->labelSimStatus) {
        ui->labelSimStatus->setText(QString("  Ruch aktywny · Pojazdy: %1").arg(n));
    }

    // ── Aktualizacja trasy wizualnej co klatkę (GPS-style) ───────────
    // Trasa jest przycinana do aktualnej pozycji SmartCar: minięte
    // skrzyżowania znikają, linia zaczyna się od auta.
    if (m_routeManager->isNavigating()) {
        m_view->setRoute(m_routeManager->visualRoute());

        QString hud = QString::fromUtf8(
                          "Nawigacja GPS\n"
                          "Szacowany czas\t%1 s\n"
                          "Koszt trasy\t%2 s\n"
                          "Przeliczenia\t%3\n"
                          "Wykryte korki\t%4")
            .arg((int)m_routeManager->estTimeSeconds())
            .arg(m_routeManager->routeCost(), 0,'f',1)
            .arg(m_routeManager->recalcCount())
            .arg(m_routeManager->congestionCount());
        m_view->setStatsText(hud);
        m_sideMenu->setNavStats(
            m_routeManager->routeCost(),
            m_routeManager->recalcCount(),
            m_routeManager->congestionCount(),
            m_routeManager->estTimeSeconds());
    }

    // Presentation timer owns repaint cadence; engine ticks only publish snapshots.
}

// ── Controls ──────────────────────────────────────────────────────────
void MainWindow::onSpeedChanged(double mult) { m_speedMult = mult; }

// ── Etykiety statusu w dolnym pasku (infoBar z .ui) ───────────────────
// Te etykiety są zdefiniowane w ui/main_window.ui i są REALNIE
// aktualizowane przez aplikację (nie są design-time-only).
void MainWindow::updateBottomStatus(const QString& simStatus, const QString& routeStatus) {
    if (ui && ui->labelSimStatus)   ui->labelSimStatus->setText(simStatus);
    if (ui && ui->labelRouteStatus) ui->labelRouteStatus->setText(routeStatus);
}

// ── Route selection flow ──────────────────────────────────────────────
void MainWindow::onRouteRequested() {
    if (m_navState == NavState::Navigating) {
        // Najpierw przerwij trwającą nawigację.
        m_routeManager->stopNavigation();
        m_view->clearRoute();
        m_view->setStatsText("");
        m_sideMenu->setNavStats(0,0,0,0);
    }

    m_navState   = NavState::SelectStart;
    m_startCpId  = -1;

    // Pokaż checkpointy na mapie.
    m_view->setCheckpoints(m_routeManager->checkpoints());
    m_view->setRouteSelectionMode(true);
    m_sideMenu->setRouteSelectionMode(true);
    // Zamknij menu, by odsłonić mapę do wyboru checkpointów.
    if (m_sideMenu->isOpen()) m_sideMenu->toggle();

    m_view->setStatsText("Kliknij punkt STARTOWY na krawędzi mapy");
    updateBottomStatus(ui->labelSimStatus->text(),
                       QString::fromUtf8("  Trasa: wybór punktu startowego"));
}

void MainWindow::onCancelRoute() {
    cancelRouteSelection();
}

void MainWindow::cancelRouteSelection() {
    m_routeManager->stopNavigation();
    m_navState  = NavState::Idle;
    m_startCpId = -1;
    m_view->clearCheckpoints();
    m_view->clearRoute();
    m_view->setRouteSelectionMode(false);
    m_view->setStatsText("");
    m_sideMenu->setRouteSelectionMode(false);
    m_sideMenu->setNavStats(0,0,0,0);
    updateBottomStatus(ui->labelSimStatus->text(),
                       QString::fromUtf8("  Trasa: —"));
}

void MainWindow::onCheckpointClicked(int cpId) {
    if (m_navState == NavState::SelectStart) {
        m_startCpId = cpId;
        m_navState  = NavState::SelectEnd;

        // Oznacz ten checkpoint wizualnie jako START.
        m_view->setCheckpointState(cpId, Checkpoint::UIState::IsStart);
        m_view->setStatsText("Kliknij punkt KOŃCOWY na krawędzi mapy");
        updateBottomStatus(ui->labelSimStatus->text(),
                           QString::fromUtf8("  Trasa: wybór punktu końcowego"));

    } else if (m_navState == NavState::SelectEnd) {
        if (cpId == m_startCpId) return; // cannot pick same point

        m_view->setCheckpointState(cpId, Checkpoint::UIState::IsEnd);
        m_view->setStatsText(" Wyznaczam trasę…");

        // Ukryj checkpointy i rozpocznij nawigację.
        m_view->setRouteSelectionMode(false);
        m_view->clearCheckpoints();
        m_sideMenu->setRouteSelectionMode(false);

        bool ok = m_routeManager->startNavigation(m_startCpId, cpId);
        if (ok) {
            m_navState = NavState::Navigating;
            updateBottomStatus(ui->labelSimStatus->text(),
                               QString::fromUtf8("  Trasa: nawigacja aktywna"));
        } else {
            m_view->setStatsText(" Nie znaleziono trasy");
            m_navState = NavState::Idle;
            updateBottomStatus(ui->labelSimStatus->text(),
                               QString::fromUtf8("  Trasa: nie znaleziono"));
        }
    }
}

void MainWindow::onRouteChanged(Route newRoute) {
    m_view->setRoute(newRoute);
}

void MainWindow::onNavigationFinished() {
    m_navState = NavState::Idle;
    m_view->clearRoute();
    m_view->setStatsText(" Samochód dotarł do celu!");
    m_sideMenu->setNavStats(0,0,0,0);
    updateBottomStatus(ui->labelSimStatus->text(),
                       QString::fromUtf8("  Trasa: dotarł do celu"));
    // Wyczyść statystyki po krótkiej chwili (timer jednorazowy).
    QTimer::singleShot(3000, this, [this](){
        m_view->setStatsText("");
        updateBottomStatus(ui->labelSimStatus->text(),
                           QString::fromUtf8("  Trasa: —"));
    });
}

// ── Key events ────────────────────────────────────────────────────────
void MainWindow::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Escape) {
        if (m_navState != NavState::Idle) {
            cancelRouteSelection();
        } else {
            close();
        }
    } else {
        QMainWindow::keyPressEvent(e);
    }
}
