#pragma once
#include <QPainter>
#include "Theme.h"
#include <vector>
#include <QRectF>
class SimulationEngine;

/**
 * @brief Renderer statycznej mapy: jezdnie, oznaczenia, skrzyżowania, ronda.
 *
 * Wszystkie metody rysują w przestrzeni świata (przed transformacją widoku)
 * i pobierają kolory z przekazanego motywu @ref MapTheme.
 */
class MapRenderer {
public:
    /// Rysuje asfalt wszystkich jezdni (poziomych i pionowych).
    void drawRoads(QPainter& p, const MapTheme& th) const;
    /// Rysuje linie pasów i linie stopu.
    void drawMarkings(QPainter& p, const MapTheme& th) const;
    /// Rysuje skrzyżowania: ronda lub sygnalizację świetlną.
    void drawIntersections(QPainter& p, const SimulationEngine* engine,
                           const MapTheme& th) const;
    void drawRoundabouts(QPainter& p, const SimulationEngine* engine, const MapTheme& th) const;
    void drawSignals(QPainter& p, const SimulationEngine* engine, const MapTheme& th) const;
    /// Prostokąty (świat) obudów wszystkich sygnalizatorów — jedno źródło prawdy
    /// dla rysowania i dla układu znaczników wyboru trasy (nie mogą ich zasłaniać).
    static std::vector<QRectF> signalHeadRects(const SimulationEngine* engine);
    static constexpr double kSignalHalf = 5.0;  ///< Połowa boku obudowy sygnalizatora.
private:
    /// Rysuje pojedyncze rondo (asfalt, wyspa, strzałki kierunku).
    void drawRoundabout(QPainter& p, double cx, double cy, const MapTheme& th) const;
    /// Rysuje komplet sygnalizatorów dla jednego skrzyżowania.
    void drawTrafficLight(QPainter& p, int vi, int hi,
                          const SimulationEngine* engine, const MapTheme& th) const;
};
