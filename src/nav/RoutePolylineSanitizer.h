#pragma once
/**
 * @file RoutePolylineSanitizer.h
 * @brief Sanityzator polilinii trasy — ostatnia linia obrony przed
 *        artefaktami przed renderingiem.
 *
 *  Wycina:
 *    • niepoprawne punkty (NaN, inf),
 *    • duplikaty (punkty bliższe sobie niż 0.5 px),
 *    • nienormalnie długie segmenty (> kMaxSaneRouteSegment).
 *
 *  Przy długim segmencie: NIE łączy a→b. Dotychczasowy fragment zamyka
 *  jako osobną polilinię, segment trafia na listę rejected (z polskim
 *  opisem przyczyny), a następny fragment zaczyna się od b.
 */
#include "VisualRouteTypes.h"
#include <QPolygonF>

namespace RoutePolylineSanitizer {

/**
 * @brief Czyści surową polilinię i zapisuje wynik do gotowego pola
 *        VisualRouteData::currentRouteFragments + diagnostyki.
 *
 * @param raw       Wejściowa surowa polilinia (z VisualRouteBuilder).
 * @param outData   Wynikowa struktura — pola fragments/rejected/diagnostyka
 *                  zostaną wypełnione. POZOSTAŁE pola (drivenTrace, history)
 *                  NIE są dotykane — sanitizer nie miesza warstw.
 */
void sanitizeInto(const QPolygonF& raw, VisualRouteData& outData);

} // namespace RoutePolylineSanitizer
