#include "MapRenderer.h"
#include "RoadVisuals.h"
#include "../core/SimulationEngine.h"
#include "../core/RoadGeometry.h"
#include "../core/Roundabout.h"

#include <cmath>
#include <QPainterPath>

/**
 * @brief Rysuje asfalt wszystkich jezdni.
 *
 * Płaski asfalt i cienka, kontrastowa krawędź jezdni.
 */
void MapRenderer::drawRoads(QPainter& p, const MapTheme& th) const {
    p.setPen(Qt::NoPen);
    // Inset blocks provide spatial structure without decorative objects.
    p.setBrush(th.island);
    for(int x=0;x<kNumRoads-1;++x) for(int y=0;y<kNumRoads-1;++y)
        p.drawRoundedRect(QRectF(kVRight(x)+10,kHBot(y)+10,kVLeft[x+1]-kVRight(x)-20,kHTop[y+1]-kHBot(y)-20),8,8);
    // Road boundary
    p.setBrush(th.roadEdge);
    for (int i=0;i<kNumRoads;i++){
        p.drawRect(QRectF(0, kHTop[i]-2, kCanvasW, kRoadW+4));
        p.drawRect(QRectF(kVLeft[i]-2, 0, kRoadW+4, kCanvasH));
    }
    // Właściwy asfalt
    p.setBrush(th.road);
    for (int i=0;i<kNumRoads;i++){
        p.drawRect(QRectF(0, kHTop[i], kCanvasW, kRoadW));
        p.drawRect(QRectF(kVLeft[i], 0, kRoadW, kCanvasH));
    }
}

/**
 * @brief Rysuje przerywane linie pasów oraz linie stopu na skrzyżowaniach.
 */
void MapRenderer::drawMarkings(QPainter& p, const MapTheme& th) const {
    // Divider lines stop at intersection boundaries.
    QPen dash(th.laneMark, 1.5, Qt::DashLine);
    dash.setDashPattern({12, 12});
    p.setPen(dash);
    static const QVector<QLineF> dividers = [] {
    QVector<QLineF> lines;
    lines.reserve(2*kNumRoads*(kNumRoads+1));
    for (int road = 0; road < kNumRoads; ++road) {
        for (int segment = 0; segment <= kNumRoads; ++segment) {
            const double x0 = segment == 0 ? 0 : kVRight(segment-1);
            const double x1 = segment == kNumRoads ? kCanvasW : kVLeft[segment];
            const double y0 = segment == 0 ? 0 : kHBot(segment-1);
            const double y1 = segment == kNumRoads ? kCanvasH : kHTop[segment];
            lines.append(QLineF(QPointF(x0, kHTop[road]+kLaneW), QPointF(x1, kHTop[road]+kLaneW)));
            lines.append(QLineF(QPointF(kVLeft[road]+kLaneW, y0), QPointF(kVLeft[road]+kLaneW, y1)));
        }
    }
    return lines;
    }();
    p.drawLines(dividers);
    p.setPen(Qt::NoPen);
    p.setBrush(th.stopLine);
    for (int vi=0; vi<kNumRoads; ++vi) {
        for (int hi=0; hi<kNumRoads; ++hi) {
            const double ix=kVLeft[vi], iy=kHTop[hi];
            p.drawRect(QRectF(kSL_L2R(vi), iy+kLaneW+2, 2, kLaneW-4));
            p.drawRect(QRectF(kSL_R2L(vi), iy+2, 2, kLaneW-4));
            p.drawRect(QRectF(ix+2, kSL_T2B(hi), kLaneW-4, 2));
            p.drawRect(QRectF(ix+kLaneW+2, kSL_B2T(hi), kLaneW-4, 2));
        }
    }
}

/**
 * @brief Rysuje wszystkie skrzyżowania — ronda albo sygnalizację świetlną.
 */
void MapRenderer::drawIntersections(QPainter& p, const SimulationEngine* engine,
                                     const MapTheme& th) const {
    drawRoundabouts(p, engine, th);
    drawSignals(p, engine, th);
}

void MapRenderer::drawRoundabouts(QPainter& p, const SimulationEngine* engine,
                                  const MapTheme& th) const {
    if (!engine) return;
    for (int vi=0;vi<kNumRoads;vi++){
        for (int hi=0;hi<kNumRoads;hi++){
            bool rb = engine->intersectionAt(vi,hi).isRoundabout;
            if (rb) {
                double cx=kVLeft[vi]+kRoadW/2.0, cy=kHTop[hi]+kRoadW/2.0;
                drawRoundabout(p,cx,cy,th);
            }
        }
    }
}

void MapRenderer::drawSignals(QPainter& p, const SimulationEngine* engine,
                              const MapTheme& th) const {
    if (!engine) return;
    for (int vi=0; vi<kNumRoads; ++vi)
        for (int hi=0; hi<kNumRoads; ++hi)
            if (!engine->intersectionAt(vi,hi).isRoundabout)
                drawTrafficLight(p,vi,hi,engine,th);
}

/**
 * @brief Rysuje pojedyncze rondo: warstwy asfaltu, wyspę i strzałki kierunku.
 *
 * Rondo jest celowo większe niż strefa skrzyżowania — "wybrzusza się" w rogi
 * trawnika, dzięki czemu wizualnie obejmuje auta jadące po promieniu kRingR.
 * Strzałki układają się stycznie do okręgu, wskazując ruch przeciwny do
 * wskazówek zegara (zgodnie z ruchem prawostronnym w Polsce).
 */
void MapRenderer::drawRoundabout(QPainter& p, double cx, double cy,
                                  const MapTheme& th) const {
    constexpr double kOuterR  = RoadVisuals::outerRadius;
    const double kIslandR = RoadVisuals::islandRadius();
    constexpr double kLaneR   = kRingR;  // linia pasa (gdzie jadą auta)

    // Roundabout perimeter is visible only between its four road entrances.
    p.setPen(Qt::NoPen);p.setBrush(th.road);
    p.drawEllipse(QPointF(cx,cy),kOuterR,kOuterR);
    p.save();
    QPainterPath corners;corners.addRect(QRectF(cx-70,cy-70,140,140));
    QPainterPath entrances;entrances.addRect(QRectF(cx-35,cy-70,70,140));entrances.addRect(QRectF(cx-70,cy-35,140,70));
    p.setClipPath(corners.subtracted(entrances),Qt::IntersectClip);
    p.setPen(QPen(th.roadEdge,2));p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(cx,cy),kOuterR,kOuterR);p.restore();
    p.setPen(QPen(th.islandEdge,2));p.setBrush(th.island);
    p.drawEllipse(QPointF(cx,cy),kIslandR,kIslandR);
    p.setPen(QPen(th.roadEdge,1));p.setBrush(Qt::NoBrush);
    p.drawEllipse(QPointF(cx,cy),kIslandR-6,kIslandR-6);

    // Strzałki kierunku jazdy wokół wyspy (CCW)
    p.setPen(Qt::NoPen);
    p.setBrush(th.laneMark);
    constexpr int kNumArrows = 4;
    double arrowR = (kIslandR + kLaneR) / 2.0;
    for (int i = 0; i < kNumArrows; i++) {
        double a = 2.0 * M_PI * i / kNumArrows;
        double ax = cx + arrowR * std::cos(a);
        double ay = cy + arrowR * std::sin(a);
        double ta = a - M_PI / 2.0;  // styczna (kierunek CCW)
        double cosT = std::cos(ta), sinT = std::sin(ta);
        QPolygonF tri;
        tri << QPointF(ax + 7*cosT,          ay + 7*sinT)
            << QPointF(ax - 4*cosT + 3*sinT, ay - 4*sinT - 3*cosT)
            << QPointF(ax - 4*cosT - 3*sinT, ay - 4*sinT + 3*cosT);
        p.drawPolygon(tri);
    }
}

/**
 * @brief Rysuje sygnalizator przy każdym pasie wjazdowym skrzyżowania.
 *
 * Sygnalizatory poziome (lewo/prawo) i pionowe (góra/dół) świecą zgodnie ze
 * stanem świateł skrzyżowania. Kolory nie zależą od palety mapy.
 */
namespace {
/// Środki 4 sygnalizatorów skrzyżowania (vi,hi) — po jednym na pas wjazdowy.
void signalCenters(int vi, int hi, QPointF out[4]) {
    const double ix = kVLeft[vi], iy = kHTop[hi];
    out[0] = QPointF(ix - 8, iy + kRoadW * 0.75);          // wjazd L2R (poziome)
    out[1] = QPointF(ix + kRoadW + 8, iy + kRoadW * 0.25); // wjazd R2L (poziome)
    out[2] = QPointF(ix + kRoadW * 0.25, iy - 8);          // wjazd T2B (pionowe)
    out[3] = QPointF(ix + kRoadW * 0.75, iy + kRoadW + 8); // wjazd B2T (pionowe)
}
} // namespace

std::vector<QRectF> MapRenderer::signalHeadRects(const SimulationEngine* engine) {
    std::vector<QRectF> r;
    if (!engine) return r;
    for (int vi = 0; vi < kNumRoads; ++vi)
        for (int hi = 0; hi < kNumRoads; ++hi) {
            if (engine->intersectionAt(vi, hi).isRoundabout) continue;
            QPointF c[4]; signalCenters(vi, hi, c);
            for (const QPointF& q : c)
                r.push_back(QRectF(q.x() - kSignalHalf, q.y() - kSignalHalf, 2 * kSignalHalf, 2 * kSignalHalf));
        }
    return r;
}

void MapRenderer::drawTrafficLight(QPainter& p, int vi, int hi,
                                    const SimulationEngine* engine,
                                    const MapTheme& th) const {
    auto col=[](LightState s)->QColor{
        switch(s){case LightState::Green:return{65,196,114};
                  case LightState::Yellow:return{255,200,0};
                  default:return{240,80,68};}};

    const auto& st=engine->intersectionAt(vi,hi);

    auto drawOne=[&](double x,double y,LightState s){
        // Obudowa
        p.setPen(QPen(QColor("#E0E1D9"), 0.8));
        p.setBrush(QColor(28,28,32));
        p.drawRoundedRect(QRectF(x-kSignalHalf,y-kSignalHalf,2*kSignalHalf,2*kSignalHalf),2,2);
        // Światło
        p.setPen(Qt::NoPen);
        p.setBrush(col(s));
        p.drawEllipse(QPointF(x,y),3.5,3.5);
    };
    // One signal per incoming lane; outgoing lanes need no duplicate signal.
    QPointF c[4]; signalCenters(vi, hi, c);
    drawOne(c[0].x(), c[0].y(), st.hLight);
    drawOne(c[1].x(), c[1].y(), st.hLight);
    drawOne(c[2].x(), c[2].y(), st.vLight);
    drawOne(c[3].x(), c[3].y(), st.vLight);
}
