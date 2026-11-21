#pragma once
/**
 * @file TestHelpers.h
 * @brief Wspólne helpery testów EasyRider — wyłącznie nagłówkowe (inline).
 *
 *  Ten nagłówek jest JEDYNYM, który dołączają pliki kategorii
 *  w katalogu tests/categories (po jednej kategorii w pliku). Udostępnia komplet narzędzi używanych przez
 *  testy:
 *
 *    • Helpery WYNIKU PRÓBY pochodzą z TestFramework.h (dołączonego niżej):
 *        - MS(...)              — tworzy Milestone,
 *        - reach(ctx, id)       — oznacza osiągnięty milestone,
 *        - measure(ctx, k, v)   — zapisuje zmierzoną wartość,
 *        - markPass / markFail / markUnverified — ustawiają werdykt próby.
 *
 *    • Helpery SCENY/GEOMETRII (zdefiniowane w tym pliku):
 *        - kPass/kFail/kInfo/kAccent/kNeutral — wspólna paleta nakładek,
 *        - countCollisions / countCars / findSmartCar / findRoundabouts,
 *        - intersectionFocus    — prostokąt kadrowania skrzyżowania,
 *        - stepIndex            — krokowanie testów inspekcyjnych,
 *        - routeToPolyline(route) — cienki wrapper na WSPÓLNY moduł
 *          VisualRouteBuilder::buildRawPolyline (ten sam kod co główna apka).
 *
 *  Wszystkie symbole są `inline`/`constexpr`, więc nagłówek może być
 *  dołączany w wielu jednostkach translacji bez naruszania ODR.
 */
#include "../TestFramework.h"
#include "../TestPreviewView.h"
#include "core/RoadGeometry.h"
#include "core/PhysicsConstants.h"
#include "core/Roundabout.h"
#include "core/Car.h"
#include "core/SimulationEngine.h"
#include "nav/NavTypes.h"
#include "nav/TrafficAnalyzer.h"
#include "nav/DynamicPathfinder.h"
#include "nav/VisualRouteBuilder.h"
#include "nav/RoutePolylineSanitizer.h"
#include "nav/VisualRouteTypes.h"

#include <QColor>
#include <QPolygonF>
#include <QRectF>
#include <QPointF>
#include <cmath>
#include <set>
#include <map>
#include <vector>
#include <utility>

// ──────────────────────────────────────────────────────────────────────
//   POMOCNICZE — wspólne dla wielu testów
// ──────────────────────────────────────────────────────────────────────

inline const QColor kPass    = QColor("#16a34a");
inline const QColor kFail    = QColor("#dc2626");
inline const QColor kInfo    = QColor("#0891b2");
inline const QColor kAccent  = QColor("#f59e0b");
inline const QColor kNeutral = QColor("#475569");

inline int countCollisions(const SimulationEngine& e) {
    const auto& o = e.objects(); int c = 0;
    for (size_t i = 0; i < o.size(); ++i) {
        auto* a = dynamic_cast<const Car*>(o[i].get()); if (!a) continue;
        for (size_t j = i+1; j < o.size(); ++j) {
            auto* b = dynamic_cast<const Car*>(o[j].get()); if (!b) continue;
            QRectF x = a->boundingRect().intersected(b->boundingRect());
            if (x.width() * x.height() > 0.5) c++;
        }
    }
    return c;
}

inline std::vector<std::pair<int,int>> findRoundabouts(const SimulationEngine& e) {
    std::vector<std::pair<int,int>> r;
    for (int vi = 0; vi < kNumRoads; ++vi)
        for (int hi = 0; hi < kNumRoads; ++hi)
            if (e.intersectionAt(vi, hi).isRoundabout) r.push_back({vi, hi});
    return r;
}

// Cienki wrapper na WSPÓLNĄ logikę z VisualRouteBuilder. Dawniej był tu
// duplikat ~45 linii kodu identycznych z OverlayRenderer.cpp; teraz testy
// i główna apka korzystają z TEGO SAMEGO modułu (src/nav/VisualRouteBuilder).
inline QPolygonF routeToPolyline(const Route& r) {
    return VisualRouteBuilder::buildRawPolyline(r);
}

inline QRectF intersectionFocus(int vi, int hi, double extra = 80) {
    return QRectF(kVLeft[vi] - extra, kHTop[hi] - extra,
                  kRoadW + 2*extra, kRoadW + 2*extra);
}

inline const Car* findSmartCar(const SimulationEngine& e) {
    for (auto& o : e.objects())
        if (auto* c = dynamic_cast<const Car*>(o.get()))
            if (c->isSmartCar()) return c;
    return nullptr;
}

inline int countCars(const SimulationEngine& e) {
    int n = 0;
    for (auto& o : e.objects()) if (dynamic_cast<const Car*>(o.get())) n++;
    return n;
}

// Krokowanie po liście elementów — dla testów inspekcyjnych (statycznych).
// Zwraca indeks bieżącego elementu do "podświetlenia" na podstawie klatki.
inline int stepIndex(int frame, int totalItems, int framesPerItem = 6) {
    if (totalItems <= 0) return -1;
    int idx = frame / framesPerItem;
    return std::min(idx, totalItems - 1);
}
