#include "gui/SimulationView.h"
#include "core/SimulationEngine.h"
#include "core/SimulationObject.h"
#if __has_include("gui/RoadVisuals.h")
#include "gui/RoadVisuals.h"
#endif
#include <QApplication>
#include <QCryptographicHash>
#include <QDataStream>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QDir>
#include <QFile>
#include <QPainterPath>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QWheelEvent>
#include <QMouseEvent>
#include <cmath>

class PaintCounter : public QObject {
public:
    int frames=0;
    bool eventFilter(QObject*, QEvent* event) override {
        if(event->type()==QEvent::Paint)++frames;
        return false;
    }
};

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM","offscreen");
    QApplication app(argc,argv);
    SimulationEngine engine;
    engine.spawnCarsOnMap(app.arguments().contains("--benchmark") ? 100 : 30);
    SimulationView view; view.setEngine(&engine);view.resize(1280,768);view.show();app.processEvents();
    QImage image(view.size(),QImage::Format_ARGB32_Premultiplied);
    auto capture=[&] {
#if __has_include("gui/WorldView.h")
        view.captureFrame();
#else
        view.update();
#endif
    };
    if(app.arguments().contains("--timing")) {
        PaintCounter painted;view.installEventFilter(&painted);
        QEventLoop loop;QTimer logic;logic.setTimerType(Qt::PreciseTimer);
        int steps=0;const double speed=app.arguments().contains("--fast")?4:1;
        QObject::connect(&logic,&QTimer::timeout,[&] {engine.update(.016*speed);capture();++steps;});
        QElapsedTimer elapsed;elapsed.start();logic.start(16);
        QTimer::singleShot(6000,&loop,&QEventLoop::quit);loop.exec();
        QTextStream(stdout)<<"Offscreen timer sample "<<speed<<"x: "<<steps<<" engine steps, "<<painted.frames
            <<" paint events, "<<elapsed.elapsed()<<" ms; no PNG writes\n";
        return 0;
    }
    if(app.arguments().contains("--benchmark")) {
        for(int i=0;i<180;++i)engine.update(1.0/60.0);
        for(int i=0;i<10;++i)view.render(&image);
        QElapsedTimer clock;clock.start();
        for(int i=0;i<300;++i)view.render(&image);
        QTextStream(stdout)<<"Widget render, 100 cars, 1280x768: "<<clock.nsecsElapsed()/300.0/1e6<<" ms/frame\n";
        if(app.arguments().contains("--save")) image.save(app.arguments().last());
        return 0;
    }
    if(app.arguments().contains("--record")) {
        const QString folder=app.arguments().last();QDir().mkpath(folder);
        for(int i=0;i<180;++i)engine.update(.016);
#if __has_include("gui/WorldView.h")
        if(app.arguments().contains("--detail")) {
            const QPointF target(kVLeft[6]+kRoadW/2,kHTop[3]+kRoadW/2);
            for(int i=0;i<10;++i) {
                const QPointF local=view.worldTransform().map(target);
                QWheelEvent wheel(local,view.mapToGlobal(local.toPoint()),QPoint(),QPoint(0,120),Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
                QApplication::sendEvent(&view,&wheel);
            }
            const QPointF local=view.worldTransform().map(target), centre=view.rect().center();
            QMouseEvent down(QEvent::MouseButtonPress,local,view.mapToGlobal(local.toPoint()),Qt::LeftButton,Qt::LeftButton,Qt::ControlModifier);
            QMouseEvent move(QEvent::MouseMove,centre,view.mapToGlobal(centre.toPoint()),Qt::NoButton,Qt::LeftButton,Qt::ControlModifier);
            QMouseEvent up(QEvent::MouseButtonRelease,centre,view.mapToGlobal(centre.toPoint()),Qt::LeftButton,Qt::NoButton,Qt::ControlModifier);
            QApplication::sendEvent(&view,&down);QApplication::sendEvent(&view,&move);QApplication::sendEvent(&view,&up);
        }
#endif
        QTimer logic, frames;logic.setTimerType(Qt::PreciseTimer);frames.setTimerType(Qt::PreciseTimer);
        // Keep modest 800px frames in memory; PNG encoding must not stall logic.
        std::vector<QImage> recorded;recorded.reserve(150);
        QEventLoop loop;int count=0,steps=0;QElapsedTimer elapsed;elapsed.start();
        QObject::connect(&logic,&QTimer::timeout,[&] {engine.update(.016);capture();++steps;});
        QObject::connect(&frames,&QTimer::timeout,[&] {
            view.render(&image);recorded.push_back(image.scaledToWidth(800,Qt::SmoothTransformation));++count;
            if(count==150)loop.quit();
        });
        logic.start(16);frames.start(40);loop.exec();
        const auto duration=elapsed.elapsed();logic.stop();frames.stop();
        for(int i=0;i<count;++i)recorded[i].save(folder+QString("/frame-%1.png").arg(i,3,10,QChar('0')));
        QTextStream(stdout)<<"Recorded "<<count<<" frames / "<<steps<<" engine steps / "<<duration<<" ms (PNG encoding deferred)\n";
        return 0;
    }
    // Audit the actual world poses against the road surface, including islands.
    QPainterPath road;road.setFillRule(Qt::WindingFill);
    for(int i=0;i<kNumRoads;++i) {
        road.addRect(QRectF(0,kHTop[i],kCanvasW,kRoadW));
        road.addRect(QRectF(kVLeft[i],0,kRoadW,kCanvasH));
    }
    QPainterPath islands;
    for(int v=0;v<kNumRoads;++v)for(int h=0;h<kNumRoads;++h)
        if(engine.intersectionAt(v,h).isRoundabout) {
            QPointF c(kVLeft[v]+kRoadW/2,kHTop[h]+kRoadW/2);
            road.addEllipse(c,65,65);
#if __has_include("gui/RoadVisuals.h")
            islands.addEllipse(c,RoadVisuals::islandRadius()+1,RoadVisuals::islandRadius()+1);
#else
            islands.addEllipse(c,32,32);
#endif
        }
    road=road.subtracted(islands);
    QCryptographicHash digest(QCryptographicHash::Sha256);
    int outside=0, samples=0, edgeObjects=0;
    for(int frame=0;frame<900;++frame) {
        const double dt=.016*(frame<300 ? .25 : frame<600 ? 1.0 : 4.0);
        engine.update(dt);
        if(app.arguments().contains("--render")) {capture();view.render(&image);}
        QByteArray state;QDataStream stream(&state,QIODevice::WriteOnly);
        stream << frame << dt << quint32(engine.objects().size());
        for(const auto& car:engine.objects()) {
            stream<<car->position()<<car->boundingRect()<<car->headingAngle()<<car->speed()
                  <<qint32(car->dir())<<car->road()<<car->isOnRing()<<car->ringVI()<<car->ringHI()
                  <<car->blinkerSide()<<car->blinkerFlash();
            QTransform body;auto c=car->boundingRect().center();body.translate(c.x(),c.y());body.rotate(car->headingAngle());
            if(!QRectF(0,0,kCanvasW,kCanvasH).contains(c)) {++edgeObjects;continue;}
            for(QPointF corner:{QPointF(-20,-10),QPointF(20,-10),QPointF(20,10),QPointF(-20,10)}) {
                QPointF point=body.map(corner);
                if(!QRectF(0,0,kCanvasW,kCanvasH).contains(point))continue;
                ++samples;if(!road.contains(point)) {
                    if(outside<8) QTextStream(stdout)<<"Off-road world corner at step "<<frame
                        <<": centre=("<<c.x()<<","<<c.y()<<"), corner=("<<point.x()<<","<<point.y()
                        <<"), heading="<<car->headingAngle()<<", ring="<<car->isOnRing()<<"\n";
                    ++outside;
                }
            }
        }
        for(int v=0;v<kNumRoads;++v)for(int h=0;h<kNumRoads;++h) {
            const auto& light=engine.intersectionAt(v,h);
            stream<<qint32(light.hLight)<<qint32(light.vLight)<<light.isRoundabout;
        }
        digest.addData(state);
    }
    QTextStream(stdout)<<"Trace SHA256: "<<digest.result().toHex()<<"\nWorld body corner samples: "<<samples
        <<", off road: "<<outside<<", centres outside canvas: "<<edgeObjects<<"\n";
#if __has_include("gui/RoadVisuals.h")
    if(outside) return 1;
#endif
    return 0;
}

