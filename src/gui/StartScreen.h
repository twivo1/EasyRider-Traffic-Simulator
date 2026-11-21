#pragma once
#include <QWidget>
#include <vector>
#include "Theme.h"
class SimulationEngine;
class SimulationView;
class QStackedWidget;
class QLabel;
class StartScreen : public QWidget {
    Q_OBJECT
public:
    explicit StartScreen(SimulationEngine*, SimulationView*, QWidget* parent = nullptr);
    enum class Page { Title, ThemePick, MapPreview, CarCount };
    void setPage(Page);
signals:
    void startSimulation(int carCount, MapTheme theme);
protected:
    void paintEvent(QPaintEvent*) override;
private:
    SimulationEngine* m_engine;
    SimulationView* m_view;
    Page m_page = Page::Title;
    int m_carCount = 30;
    int m_themeIdx = 0;
    std::vector<MapTheme> m_themes;
    QStackedWidget* m_pages;
    QLabel* m_mapInfo;
    void generateNewMap();
};
