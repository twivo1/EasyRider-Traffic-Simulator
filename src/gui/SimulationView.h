/**
 * @file SimulationView.h
 * @brief Promowany widżet symulacji: rysuje mapę i auta, obsługuje zoom/pan, klikanie checkpointów i drag HUD-a.
 * @ingroup gui_grp
 */
#pragma once
#include <QWidget>
#include <QTimer>
#include <QPixmap>
#include <QPointF>
#include <vector>
#include "../nav/NavTypes.h"
#include "MapRenderer.h"
#include "CarRenderer.h"
#include "OverlayRenderer.h"
#include "Theme.h"
#include "WorldView.h"
#include "RoutePicker.h"

class SimulationEngine;

/**
 * @brief Widok symulacji — komponuje renderery i obsługuje pan/zoom.
 *
 * Klasa nie zawiera logiki rysowania: deleguje ją do trzech rendererów
 * (@ref MapRenderer, @ref CarRenderer, @ref OverlayRenderer), przekazując
 * im aktywny motyw (@ref MapTheme). Sama zarządza tylko stanem widoku:
 * przesunięciem, powiększeniem, trasą GPS, checkpointami i obsługą myszy.
 *
 * @note Widżet PROMOWANY w Qt Designer: w @c ui/main_window.ui występuje jako
 *       placeholder QWidget podniesiony do klasy SimulationView (nagłówek
 *       @c gui/SimulationView.h). Rysowanie pozostaje w kodzie.
 */
class SimulationView : public QWidget {
    Q_OBJECT
public:
    explicit SimulationView(QWidget* parent = nullptr);

    /// Podłącza silnik symulacji, z którego czytane są obiekty.
    void setEngine(SimulationEngine* e);
    void captureFrame();
    void resetPresentation();
    void fitMap();

    /// Ustawia checkpointy (punkty wjazdu/wyjazdu na krawędziach mapy).
    void setCheckpoints(const std::vector<Checkpoint>& cps);
    /// Usuwa wszystkie checkpointy.
    void clearCheckpoints();
    /// Zmienia stan wizualny pojedynczego checkpointu (np. start/cel).
    void setCheckpointState(int cpId, Checkpoint::UIState st);
    /// Ustawia trasę GPS do narysowania.
    void setRoute(const Route& route);
    /// Usuwa narysowaną trasę.
    void clearRoute();

    /// Włącza/wyłącza tryb wyboru trasy (pokazuje checkpointy).
    void setRouteSelectionMode(bool active);
    /// Etap wyboru trasy: które punkty są wybieralne i który jest wybranym STARTEM.
    void setRoutePickStage(RoutePicker::Stage stage, const std::vector<int>& selectable, int startId = -1);
    RoutePicker::Stage routePickStage() const { return m_pickStage; }
    /// Bieżący układ znaczników (ta sama funkcja co rysunek i hit-test).
    RoutePicker::Layout pickerLayout() const;
    /// Przesuwa kamerę do skrajnego położenia przy danej stronie widoku.
    void panToSide(int side);
    double zoom() const { return m_zoom; }
    double minZoom() const;
    static constexpr double kMaxZoom = 4.0;
    bool isRouteSelectionMode() const { return m_routeSelectMode; }
    /// Włącza/wyłącza nakładkę mapy korków (heatmapę).
    void setHeatmapEnabled(bool on);
    bool isHeatmapEnabled() const { return m_heatmapEnabled; }
    /// Ustawia tekst statystyk nawigacji (rysowany w HUD).
    void setStatsText(const QString& t);

    /// Ustawia aktywny motyw wizualny i odświeża widok.
    void setTheme(const MapTheme& th) { m_theme = th; m_staticMap = {}; update(); }
    const MapTheme& theme() const { return m_theme; }
    QTransform worldTransform() const { return WorldView::camera(m_zoom,m_pan); }
    /// Dostęp tylko do odczytu (narzędzie weryfikacji GUI: stan silnika i pozy rysowane).
    SimulationEngine*  engine() const      { return m_engine; }
    const CarRenderer& carRenderer() const { return m_carRenderer; }

signals:
    /// Emitowany po kliknięciu checkpointu w trybie wyboru trasy.
    void checkpointClicked(int cpId);

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private:
    void clampPan();                          ///< Ogranicza przesunięcie do granic mapy.
    void updateHover(QPointF pos);            ///< Hover znaczników/zakładek, kursor, podpowiedź.
    void setZoomAt(double zoom, QPointF screenPos); ///< Zoom pod kursorem + ograniczenia.

    SimulationEngine*        m_engine = nullptr;
    std::vector<Checkpoint>  m_checkpoints;
    Route                    m_route;
    bool                     m_routeSelectMode = false;
    RoutePicker::Stage       m_pickStage = RoutePicker::Stage::None;
    std::vector<bool>        m_pickSelectable;
    int                      m_pickStart = -1;
    int                      m_hoverCp = -1, m_hoverTab = -1;
    bool                     m_heatmapEnabled  = false;
    QString                  m_statsText;
    MapTheme                 m_theme = defaultTheme();

    // Przesunięcie i powiększenie widoku
    double  m_zoom    = 1.0;
    QPointF m_pan     = {0, 0};
    bool    m_panning = false;
    QPointF m_panStart;

    // Przeciąganie HUD-a nawigacji GPS myszką
    QPointF m_hudOffset{0, 0};       ///< Przesunięcie HUD-a względem (18, 18).
    QRectF  m_hudScreenRect{};       ///< Bounding rect HUD-a w przestrzeni ekranu (do hit-testu).
    bool    m_hudDragging = false;
    QPointF m_hudDragStart;          ///< Pozycja myszy w chwili rozpoczęcia drag-a.
    QPointF m_hudOffsetAtDragStart;  ///< Offset HUD-a w chwili rozpoczęcia drag-a.

    // Tryb diagnostyczny checkpointów (F3) — rysuje hitboxy + IDki i loguje kliki.
    bool m_debugCheckpoints = false;

    // Renderery (kompozycja zamiast dziedziczenia)
    QTimer m_presentTimer;
    QPixmap m_staticMap;
    QTransform m_cachedTransform;
    quint64 m_cachedRoundabouts = 0;
    MapRenderer     m_mapRenderer;
    CarRenderer     m_carRenderer;
    OverlayRenderer m_overlayRenderer;
};
