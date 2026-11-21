#pragma once
#include "../core/RoadGeometry.h"
#include <QTransform>
#include <algorithm>
// Roads terminate at canvas edges. Include vehicle overhang, markers and strokes.
namespace WorldView {
inline QRectF bounds() {
    static const QRectF extent = [] {
        QRectF b(0,0,kCanvasW,kCanvasH);
        for (int vi=0;vi<kNumRoads;++vi) {
            b = b.united(QRectF(kVLeft[vi]-2,0,kRoadW+4,kCanvasH));
            b = b.united(QRectF(0,kHTop[vi]-2,kCanvasW,kRoadW+4));
            for (int hi=0;hi<kNumRoads;++hi) {
                const QPointF c(kVLeft[vi]+kRoadW/2,kHTop[hi]+kRoadW/2);
                // Any intersection can become a ring; include its 65-unit surface and edge.
                b = b.united(QRectF(c-QPointF(66,66), QSizeF(132,132)));
            }
        }
        return b.adjusted(-24,-24,24,24); // checkpoint hit/paint radius
    }();
    return extent;
}
inline double fit(QSizeF viewport, QRectF b=bounds()) {
    return std::max(0.01,std::min(std::max(1.0,viewport.width()-24)/b.width(),std::max(1.0,viewport.height()-24)/b.height()));
}
inline QTransform fitted(QRectF viewport, QRectF world=bounds()) {
    double z=fit(viewport.size(), world);
    QTransform t;t.translate(viewport.center().x(),viewport.center().y());t.scale(z,z);t.translate(-world.center().x(),-world.center().y());return t;
}
/// Logiczna kanwa świata — drogi kończą się dokładnie na jej krawędziach.
inline QRectF canvas() { return QRectF(0, 0, kCanvasW, kCanvasH); }
/// Skala COVER: najmniejsza, przy której kanwa pokrywa cały viewport (bez pustych pasów).
inline double cover(QSizeF viewport) {
    if (viewport.width() <= 0 || viewport.height() <= 0) return 1.0;
    return std::max(viewport.width() / kCanvasW, viewport.height() / kCanvasH);
}
inline QTransform camera(double zoom,QPointF pan) { QTransform t;t.scale(zoom,zoom);t.translate(-pan.x(),-pan.y());return t; }
}
