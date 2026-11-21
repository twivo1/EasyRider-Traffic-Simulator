#pragma once
#include <QPainter>
#include <vector>
#include "../nav/NavTypes.h"
#include "../nav/VisualRouteTypes.h"
#include "Theme.h"

class SimulationEngine;

/**
 * @brief Renderer nakładek: trasa GPS, checkpointy, mapa korków, panel statystyk.
 *
 * Elementy rysowane przez tę klasę leżą "nad" mapą i autami. Panel statystyk
 * (HUD) rysowany jest w przestrzeni ekranu, pozostałe — w przestrzeni świata.
 */
class OverlayRenderer {
public:
    /// Convenience: Route → wbudowany VisualRouteData → rysunek.
    /// Cała logika polilinii w VisualRouteBuilder + RoutePolylineSanitizer.
    void drawRoute(QPainter& p, const Route& route, const MapTheme& th,
                   QPointF carPos = {}) const;

    /// Rysuje gotowe dane wizualne trasy. Renderer NIE decyduje co rysować —
    /// dostaje sanityzowane fragmenty i tylko je maluje.
    void drawVisualRouteData(QPainter& p, const VisualRouteData& d) const;

    /// Rysuje checkpointy (widoczne tylko w trybie wyboru trasy).
    void drawCheckpoints(QPainter& p, const std::vector<Checkpoint>& cps,
                         bool selectionMode) const;
    /// Rysuje nakładkę natężenia ruchu (zielony→czerwony).
    void drawHeatmap(QPainter& p, const SimulationEngine* engine) const;
    /// Rysuje panel statystyk nawigacji w rogu ekranu.
    /**
     * @brief HUD ze statystykami w lewym górnym rogu.
     * @param p painter (transformacja zostanie zresetowana wewnątrz).
     * @param text wielowierszowy tekst (pierwsza linia = nagłówek; kolejne „etykieta\\tWartość").
     * @param viewport prostokąt widoku (nieużywany — pozostawiony dla kompatybilności).
     * @param topLeft pozycja lewego-górnego rogu HUD-a w przestrzeni ekranu (pozwala go przesuwać).
     * @param outRect jeżeli != nullptr, otrzyma faktyczny prostokąt zajęty przez HUD
     *                (do hit-testingu kliknięć i drag-and-drop).
     */
    void drawStatsHUD(QPainter& p, const QString& text, const QRectF& viewport,
                      QPointF topLeft = QPointF(18, 18),
                      QRectF* outRect = nullptr) const;
};
