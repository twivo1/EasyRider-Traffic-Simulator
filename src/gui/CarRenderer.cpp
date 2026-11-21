#include "CarRenderer.h"
#include <QPainterPath>
#include "../core/Car.h"
#include "../core/SimulationEngine.h"
#include "../core/SimulationObject.h"
#include "../core/RoadGeometry.h"

void CarRenderer::capture(const SimulationEngine* engine) {
    std::unordered_map<const void*,Frames> next;
    for(const auto& object:engine->objects()) {
        DisplayPose pose;pose.center=object->boundingRect().center();pose.angle=object->headingAngle();
        const auto* car=dynamic_cast<const Car*>(object.get());
        if(car && car->circling()) {   // krzywe wjazdu/zjazdu nie leżą na okręgu
            pose.ring=true;pose.ringCenter=QPointF(kVLeft[car->ringVI()]+kRoadW/2,kHTop[car->ringHI()]+kRoadW/2);
        }
        auto it=m_frames.find(object.get());
        next[object.get()]={it!=m_frames.end() && frameAge()<100 ? it->second.after:pose,pose};
    }
    m_frames.swap(next);m_clock.restart();
}
DisplayPose CarRenderer::displayedPose(const SimulationObject* object, double alpha) const {
    const auto found=m_frames.find(object);
    if(found!=m_frames.end()) return betweenPoses(found->second.before,found->second.after,alpha);
    DisplayPose pose;
    pose.center=object->boundingRect().center();pose.angle=object->headingAngle();
    return pose;
}
QPointF CarRenderer::displayedSmartCarCenter(const SimulationEngine* engine, double alpha) const {
    for(const auto& object:engine->objects())
        if(object->isSmartCar()) return displayedPose(object.get(),alpha).center;
    return {};
}
// Pure presentation: body dimensions and sampled poses come from the engine.
void CarRenderer::drawCars(QPainter& p, const SimulationEngine* engine,
                           const MapTheme& th, double alpha) const {
    if (!engine) return;
    constexpr double L = kCarL, W = kCarW;
    if(alpha<0) alpha=presentationAlpha();
    std::unordered_map<const void*,int> liveColours;
    const QColor colours[]={th.carBody,QColor("#D6CBB5"),QColor("#B6C5AE"),QColor("#C99983"),QColor("#E0E4DF")};
    for (const auto& obj : engine->objects()) {
        auto colour=m_colourIds.find(obj.get());
        int colourId=colour==m_colourIds.end() ? (m_nextColour++%5) : colour->second;
        liveColours[obj.get()]=colourId;
        p.save();
        const auto pose=displayedPose(obj.get(),alpha);
        p.translate(pose.center);
        p.rotate(pose.angle);
        const QRectF body(-L/2, -W/2, L, W);
        p.setPen(Qt::NoPen);p.setBrush(QColor(12,18,21,85));
        p.drawRoundedRect(body.translated(1,1.5),5,5);
        p.setPen(QPen(QColor("#20282C"), 1.1));
        p.setBrush(obj->isSmartCar() ? th.smartCar : colours[colourId]);
        QPainterPath shell;
        shell.moveTo(-L/2+4,-W/2);shell.lineTo(L/2-6,-W/2);
        shell.quadTo(L/2,-W/2,L/2,-W/2+5);shell.lineTo(L/2,W/2-5);
        shell.quadTo(L/2,W/2,L/2-6,W/2);shell.lineTo(-L/2+4,W/2);
        shell.quadTo(-L/2,W/2,-L/2,W/2-4);shell.lineTo(-L/2,-W/2+4);
        shell.quadTo(-L/2,-W/2,-L/2+4,-W/2);p.drawPath(shell);
        // Broad front windscreen and smaller rear window show orientation.
        p.setPen(Qt::NoPen);
        p.setBrush(th.carAccent);
        p.drawRoundedRect(QRectF(L*0.08, -W/2+3, L*0.20, W-6),2,2);
        p.drawRoundedRect(QRectF(-L*0.34, -W/2+3, L*0.14, W-6),1,1);
        p.setPen(QPen(QColor(255,255,255,95),0.8));p.drawLine(QPointF(-4,-7),QPointF(2,-7));p.setPen(Qt::NoPen);
        p.setBrush(QColor("#FFF2CB"));
        p.drawRect(QRectF(L/2-3, -W/2+2, 2, 4));
        p.drawRect(QRectF(L/2-3, W/2-6, 2, 4));
        p.setBrush(QColor("#A53730"));
        p.drawRect(QRectF(-L/2+1, -W/2+2, 2, 4));
        p.drawRect(QRectF(-L/2+1, W/2-6, 2, 4));
        if (obj->isSmartCar()) {
            p.setBrush(QColor("#503B28"));
            p.drawRect(QRectF(-5, -2, 7, 4));
        }
        const int bl = obj->blinkerSide();
        if (bl != 0 && obj->blinkerFlash()) {
            p.setBrush(QColor("#FFD65C"));
            const double sideY = (bl == 1) ? W/2-2 : -(W/2-2);
            p.drawEllipse(QPointF(L/2-4, sideY), 2.2, 2.2);
        }
        p.restore();
    }
    m_colourIds.swap(liveColours);
}
