/**
 * @file MainWindow.h
 * @brief Główne okno aplikacji EasyRider — ładuje ui/main_window.ui i spina warstwy GUI ↔ silnik ↔ nawigacja.
 * @ingroup gui_grp
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "nav/RouteManager.h"   // dołącza NavTypes.h → Checkpoint, Route

class SimulationEngine;
class SimulationView;
class SideMenu;
class StartScreen;
class QLabel;
class QKeyEvent;

namespace Ui { class MainWindow; }   ///< Klasa generowana przez uic z ui/main_window.ui

/**
 * @brief Główne okno aplikacji — układ wczytywany z Qt Designer.
 *
 * Struktura widżetów (centralny widget, pionowy layout, dolny pasek
 * informacyjny) pochodzi z pliku @c ui/main_window.ui i jest budowana
 * przez @c Ui::MainWindow::setupUi(). Widok symulacji @ref SimulationView
 * jest w .ui WIDŻETEM PROMOWANYM (placeholder QWidget → SimulationView).
 * Nakładki @ref SideMenu i @ref StartScreen pozostają tworzone w kodzie
 * (są pozycjonowane ręcznie w @ref resizeEvent — Qt Designer nie wyraża
 * nakładek). Cała logika (timer, nawigacja, wybór trasy) jest bez zmian.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void resizeEvent(QResizeEvent* e) override;
    void keyPressEvent(QKeyEvent* e)  override;

private slots:
    void tick();
    void onSpeedChanged(double mult);

    // Route selection flow
    void onRouteRequested();
    void onCancelRoute();
    void onCheckpointClicked(int cpId);

    // RouteManager signals
    void onRouteChanged(Route newRoute);
    void onNavigationFinished();

private:
    Ui::MainWindow*   ui            = nullptr;   ///< Widżety z ui/main_window.ui
    SimulationEngine* m_engine       = nullptr;
    SimulationView*   m_view         = nullptr;
    SideMenu*         m_sideMenu     = nullptr;
    RouteManager*     m_routeManager = nullptr;
    StartScreen*      m_startScreen  = nullptr;
    QTimer            m_timer;

    double  m_speedMult = 1.0;

    // Route selection state
    enum class NavState { Idle, SelectStart, SelectEnd, Navigating };
    NavState m_navState  = NavState::Idle;
    int      m_startCpId = -1;

    void updateStatusLabel();
    void cancelRouteSelection();
    void updateBottomStatus(const QString& simStatus, const QString& routeStatus); ///< etykiety w infoBar (ui->labelSimStatus / ui->labelRouteStatus)
};

#endif
