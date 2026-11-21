#pragma once
#include <QPainter>
#include <QElapsedTimer>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "Theme.h"
class SimulationEngine;
class SimulationObject;

/**
 * @brief Renderer pojazdów.
 *
 * Rysuje wszystkie auta obrócone zgodnie z kierunkiem jazdy (także podczas
 * jazdy po rondzie). Kolory karoserii pobiera z motywu @ref MapTheme,
 * a auto GPS (SmartCar) wyróżnia osobnym kolorem i pasem na dachu.
 */
struct DisplayPose {
    QPointF center, ringCenter;
    double angle=0;
    bool ring=false;
};
inline DisplayPose betweenPoses(DisplayPose a,DisplayPose b,double t) {
    t=std::clamp(t,0.0,1.0);
    if(QLineF(a.center,b.center).length()>40) return b;   // spawn / usunięcie — bez interpolacji
    if(a.ring && b.ring && a.ringCenter==b.ringCenter) {
        double start=std::atan2(a.center.y()-a.ringCenter.y(),a.center.x()-a.ringCenter.x());
        double end=std::atan2(b.center.y()-b.ringCenter.y(),b.center.x()-b.ringCenter.x());
        double theta=start+std::remainder(end-start,2*3.141592653589793)*t;
        double r=QLineF(b.center,b.ringCenter).length();
        b.center=b.ringCenter+QPointF(std::cos(theta)*r,std::sin(theta)*r);
        b.angle=a.angle+std::remainder(b.angle-a.angle,360.0)*t;
    } else if(std::abs(std::remainder(b.angle-a.angle,360.0))<30.0) {
        // Ciągły ruch (łuk skrętu, krzywa wjazdu/zjazdu, przejście krzywa↔okrąg):
        // interpolacja pozycji i kursu. Dawniej przy zmianie trybu (ring) zwracano
        // od razu stan docelowy — przeskok o cały krok silnika (3,5 px przy ×4).
        b.center=a.center+(b.center-a.center)*t;
        b.angle=a.angle+std::remainder(b.angle-a.angle,360.0)*t;
    }
    return b; // Discontinuous turns/spawns are never interpolated through scenery.
}
class CarRenderer {
    struct Frames { DisplayPose before,after; };
    std::unordered_map<const void*,Frames> m_frames;
    QElapsedTimer m_clock;
    mutable std::unordered_map<const void*,int> m_colourIds;
    mutable int m_nextColour=0;
public:
    /// Poza rysowana dla obiektu przy danym alpha (diagnostyka: zgodność render–silnik).
    DisplayPose displayedPose(const SimulationObject* object, double alpha) const;
    void capture(const SimulationEngine* engine);
    qint64 frameAge() const { return m_clock.isValid()?m_clock.elapsed():1000; }
    double presentationAlpha() const { return std::min(1.0,frameAge()/16.0); }
    QPointF displayedSmartCarCenter(const SimulationEngine* engine, double alpha) const;

    /// Rysuje wszystkie pojazdy z silnika z uwzględnieniem motywu.
    void drawCars(QPainter& p, const SimulationEngine* engine, const MapTheme& th, double alpha=-1) const;
};
