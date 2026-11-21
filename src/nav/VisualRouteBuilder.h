#pragma once
/**
 * @file VisualRouteBuilder.h
 * @brief Buduje surową polilinię trasy z sekwencji RouteStep.
 *
 *  Konwertuje semantyczne kroki trasy (RouteStep z NavTypes) na konkretne
 *  punkty świata (QPointF) wzdłuż pasów ruchu na drogach i skrzyżowaniach.
 *  Logika dawniej dublowana w:
 *    • src/gui/OverlayRenderer.cpp::%drawRoute()   (główna apka)
 *    • tests/TestSuite.h::routeToPolyline()        (runner testowy)
 *  Teraz obie strony używają TEGO SAMEGO kodu — testy weryfikują dokładnie
 *  to co rysuje główna apka.
 */
#include "VisualRouteTypes.h"
#include "NavTypes.h"

namespace VisualRouteBuilder {

/**
 * @brief Buduje surową polilinię z kroków trasy (bez sanityzacji).
 *
 * @param route   Sekwencja RouteStep z DynamicPathfinder/RouteManager.
 * @param carPos  Opcjonalna pozycja auta. Jeśli niepusta — polilinia jest
 *                przycięta tak, by ZACZYNAŁA SIĘ od carPos (GTA-style),
 *                a punkt-rzut wybierany jest defensywnie (kara dla rzutów
 *                poza segmentem, by trim nie sięgał po drugiej stronie
 *                skrzyżowania → orange tail).
 * @return        QPolygonF z punktami pasów ruchu (laneEntry/laneCross/laneExit).
 *                MOŻE zawierać niepoprawne punkty lub długie segmenty —
 *                ZAWSZE należy następnie przepuścić przez sanitizer.
 */
QPolygonF buildRawPolyline(const Route& route, QPointF carPos = QPointF());

/**
 * @brief Pełny pipeline: route → raw polyline → sanityzacja → VisualRouteData.
 *
 * Jedyny entry-point używany przez OverlayRenderer w głównej apce.
 * Testy mogą też używać tego, lub osobno buildRawPolyline() + sanitizer.
 */
VisualRouteData build(const Route& route, QPointF carPos = QPointF());

} // namespace VisualRouteBuilder
