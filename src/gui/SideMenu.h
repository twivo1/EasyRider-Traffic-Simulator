/// Compact, collapsible control column. Widgets are defined in side_menu.ui.
#pragma once
#include <QWidget>
#include "Theme.h"

class QPushButton;
class QLabel;
class QSlider;
class QComboBox;

namespace Ui { class SideMenu; }   ///< Klasa generowana przez uic z ui/side_menu.ui

class SideMenu : public QWidget {
    Q_OBJECT
public:
    explicit SideMenu(QWidget* parent = nullptr);
    ~SideMenu() override;

    /// Przelicza geometrię panelu względem rodzica (po zmianie rozmiaru).
    void reposition();
    /// Aktualizuje wyświetlaną liczbę pojazdów.
    void setCarCount(int n);
    void setSelectedTheme(const MapTheme& theme);
    /// Aktualizuje statystyki nawigacji (koszt, przeliczenia, korki, czas).
    void setNavStats(double cost, int recalcs, int hotspots, double estTime);
    /// Włącza/wyłącza tryb wyboru trasy (zmienia podpis przycisku).
    void setRouteSelectionMode(bool active);
    bool isOpen() const { return m_open; }

    static constexpr int kPanelWidth = 244;  ///< Szerokość rozwiniętego panelu.
    static constexpr int kHandleW    = 24;   ///< Szerokość uchwytu.

signals:
    void speedChanged(double multiplier);   ///< Zmiana mnożnika prędkości.
    void routeRequested();                  ///< Żądanie wyznaczenia trasy.
    void cancelRouteRequested();            ///< Anulowanie trasy.
    void heatmapToggled(bool enabled);      ///< Przełączenie mapy korków.
    void themeChanged(MapTheme theme);      ///< Zmiana motywu wizualnego.
    void backToMenuRequested();             ///< Powrót do ekranu startowego.

public slots:
    /// Otwiera/zamyka kolumnę sterowania.
    void toggle();

private:
    void buildUI();      ///< setupUi z .ui + wiązanie wskaźników + sygnały
    void bindFromUi();   ///< Powiązanie wskaźników składowych z widżetami z .ui

    Ui::SideMenu* ui = nullptr;   ///< Widżety z ui/side_menu.ui
    QWidget*     m_panel     = nullptr;
    QPushButton* m_handleBtn = nullptr;
    QPushButton* m_routeBtn  = nullptr;
    QPushButton* m_cancelBtn = nullptr;
    QLabel*      m_carCount  = nullptr;
    QLabel*      m_stats     = nullptr;
    QSlider*     m_speedSlider = nullptr;
    QComboBox*   m_themeCombo  = nullptr;

    std::vector<MapTheme> m_themes;
    bool m_open = true;
};
