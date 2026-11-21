// Offscreen Qt rendering and interaction checks; never automates the desktop.
#include "gui/MainWindow.h"
#include "gui/SimulationView.h"
#include "gui/StartScreen.h"
#include "gui/SideMenu.h"
#include "gui/MapRenderer.h"
#include "gui/CarRenderer.h"
#include "gui/RoadVisuals.h"
#include "core/SimulationEngine.h"
#include "core/SimulationObject.h"
#include "core/Car.h"
#include "TestWindow.h"
#include "TestRow.h"
#include "TestPreviewView.h"
#if __has_include("gui/UiStyle.h")
#include "gui/UiStyle.h"
#define REDESIGN 1
#endif
#include <QApplication>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QComboBox>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QElapsedTimer>
#include <QDir>
#include <QTimer>
#include <QTextStream>
#include <QFontDatabase>
#include <QEventLoop>
#include <stdexcept>

void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void click(QWidget* w, QPointF pos) {
    QMouseEvent down(QEvent::MouseButtonPress,pos,w->mapToGlobal(pos.toPoint()),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent up(QEvent::MouseButtonRelease,pos,w->mapToGlobal(pos.toPoint()),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(w,&down); QApplication::sendEvent(w,&up);
    QApplication::processEvents();
}
void benchmark(const QString& out) {
        // Fixed engine scene, independent of startup's randomized roundabout layout.
        SimulationEngine engine;engine.spawnCarsOnMap(100);
        for(int i=0;i<180;++i)engine.update(1.0/60.0);
        QImage scene(1280,768,QImage::Format_ARGB32_Premultiplied);
        const auto theme=defaultTheme();MapRenderer map;CarRenderer cars;
        const auto before=engine.objects().front()->boundingRect();
        QElapsedTimer clock;clock.start();
        for(int i=0;i<300;++i) {
            scene.fill(theme.grassTop);
            QPainter p(&scene);p.setRenderHint(QPainter::Antialiasing);
            p.scale(1280.0/kCanvasW,768.0/kCanvasH);
            map.drawRoads(p,theme);map.drawMarkings(p,theme);map.drawIntersections(p,&engine,theme);cars.drawCars(p,&engine,theme);
        }
        QTextStream(stdout)<<"Render 100 cars / 1280x768: "<<clock.elapsed()/300.0<<" ms/frame\n";
        check(before==engine.objects().front()->boundingRect(),"Rendering mutated position");
        check(scene.save(out+"/13-fixed-scene.png"),"Scene save failed");
}
int main(int argc,char** argv) {
    qputenv("QT_QPA_PLATFORM","offscreen");
    QApplication app(argc,argv);
    // The Windows offscreen platform does not enumerate installed system fonts.
    for (const auto& file : {"segoeui.ttf", "segoeuib.ttf", "segoeuisl.ttf", "seguisym.ttf"})
        QFontDatabase::addApplicationFont(QString("C:/Windows/Fonts/")+file);
    app.setFont(QFont("Segoe UI",10));
#ifdef REDESIGN
    UiStyle::apply(app);
#endif
    const QString out=argc>1 ? argv[1] : "visual-output";
    QDir().mkpath(out);
    if(app.arguments().contains("--benchmark")) { benchmark(out); return 0; }
    auto save=[&](QWidget& w,const QString& name) {
        QApplication::processEvents();
        check(w.grab().save(out+"/"+name+".png"),"PNG save failed");
    };
    try {
        for(QSizeF size:{QSizeF(640,480),QSizeF(960,600),QSizeF(1920,1080)}) {
            auto transform=WorldView::fitted(QRectF(QPointF(),size));
            check(QRectF(QPointF(),size).contains(transform.mapRect(WorldView::bounds())),"World bounds clipped");
            check(std::abs(transform.m11()-transform.m22())<1e-12,"Anisotropic scale");
            for(QPointF point:{QPointF(0,0),QPointF(kCanvasW,kCanvasH),QPointF(100,200)})
                check(QLineF(point,transform.inverted().map(transform.map(point))).length()<1e-8,"Inverse camera mismatch");
        }
        DisplayPose a,b;a.ring=b.ring=true;a.ringCenter=b.ringCenter=QPointF(100,100);
        a.center=QPointF(150,100);b.center=QPointF(100+50*std::cos(-0.2),100+50*std::sin(-0.2));a.angle=-90;b.angle=-101.4591559;
        check(QLineF(betweenPoses(a,b,0.5).center,b.center).length()>1,"Ring interpolation skipped");
        for(int i=0;i<=100;++i) {
            auto pose=betweenPoses(a,b,i/100.0);
            check(std::abs(QLineF(pose.center,pose.ringCenter).length()-kRingR)<1e-8,"Interpolated ring cuts island");
            check(pose.angle<=-90 && pose.angle>=-180,"Angle leaves shortest arc");
        }
        a.ring=b.ring=false;a.angle=0;b.angle=90;
        check(betweenPoses(a,b,0.5).center==b.center,"Discontinuous turn was invented");
        for(double offset:{Rb::southOff,Rb::northMag}) {
            const QPointF corner(std::sqrt(kRingR*kRingR-offset*offset)-kCarL/2,offset-kCarW/2);
            check(QLineF(QPointF(),corner).length()>RoadVisuals::islandRadius()+1,"Island overlaps transition hull");
        }
        a.ring=b.ring=true;a.angle=179;b.angle=-179;
        check(std::abs(betweenPoses(a,b,.5).angle-180)<1e-8,"Heading wraps along long arc");
        for(Direction direction:{Direction::L2R,Direction::R2L,Direction::T2B,Direction::B2T}) {
            Car car(QPointF(100,200),direction,0);
            QTransform body;body.translate(car.boundingRect().center().x(),car.boundingRect().center().y());body.rotate(car.headingAngle());
            check(body.mapRect(QRectF(-kCarL/2,-kCarW/2,kCarL,kCarW))==car.boundingRect(),"Car rotation/centre disagrees with logical bounds");
        }
        // Sample actual engine motion and its rendered hull, not just screenshots.
        {
            SimulationEngine sample;sample.spawnCarsOnMap(30);
            int ringFrames=0, outside=0;
            for(int frame=0;frame<900;++frame) {
                sample.update(0.016);
                for(const auto& car:sample.objects()) {
                    if(!car->isOnRing()) continue;
                    ++ringFrames;
                    QPointF rc(kVLeft[car->ringVI()]+kRoadW/2,kHTop[car->ringHI()]+kRoadW/2);
                    QTransform body;body.translate(car->boundingRect().center().x(),car->boundingRect().center().y());body.rotate(car->headingAngle());
                    for(QPointF corner:{QPointF(-20,-10),QPointF(20,-10),QPointF(20,10),QPointF(-20,10)})
                        if(QLineF(body.map(corner),rc).length()>65.01) ++outside;
                }
            }
            QTextStream(stdout)<<"Ring frames: "<<ringFrames<<", corners outside road: "<<outside<<"\n";
            check(ringFrames>0 && outside==0,"Ring body exceeds road surface");
        }
        MainWindow w; w.resize(1280,800); w.show(); QApplication::processEvents();
        auto* start=w.findChild<StartScreen*>();
        auto* view=w.findChild<SimulationView*>();
        auto* menu=w.findChild<SideMenu*>();
        auto* manager=w.findChild<RouteManager*>();
        check(start&&view&&menu&&manager,"Missing main widgets");
        save(w,"01-title");
#ifdef REDESIGN
        auto press=[&](const char* name) {
            auto* b=w.findChild<QPushButton*>(name); check(b&&b->isVisible()&&b->isEnabled(),name);
            b->click(); QApplication::processEvents();
        };
        press("beginButton"); save(w,"02-themes");
        press("themeNext"); save(w,"03-map-preview");
        press("newMap"); press("mapBack"); press("themeNext"); press("acceptMap");
        auto* count=w.findChild<QSpinBox*>("initialCarCount"); count->setValue(30);
        check(w.findChild<QSlider*>("initialCarSlider")->value()==30,"Count controls disagree");
        save(w,"04-configuration"); press("startButton");
#else
        start->setPage(StartScreen::Page::ThemePick); save(w,"02-themes");
        start->setPage(StartScreen::Page::MapPreview); save(w,"03-map-preview");
        start->setPage(StartScreen::Page::CarCount); save(w,"04-configuration");
        start->hide(); start->startSimulation(100,defaultTheme());
        if(!menu->isOpen()) menu->toggle();
        QEventLoop settle;
        QTimer::singleShot(350,&settle,&QEventLoop::quit);
        settle.exec();
#endif
        for(int i=0;i<150;++i) QMetaObject::invokeMethod(&w,"tick",Qt::DirectConnection);
        save(w,"05-simulation");
#ifdef REDESIGN
        check(!view->geometry().intersects(menu->geometry()),"Sidebar overlaps map");
        double speed=0;
        QObject::connect(menu,&SideMenu::speedChanged,[&](double v){speed=v;});
        auto* slider=w.findChild<QSlider*>("speedSlider");slider->setValue(8);check(speed==4,"Speed mapping changed");slider->setValue(4);check(speed==1,"Speed reset failed");
        press("heatBtn");save(w,"06-heatmap");press("heatBtn");
        auto* combo=w.findChild<QComboBox*>("themeCombo");
        for(int i=0;i<combo->count();++i) {combo->setCurrentIndex(i);save(w,QString("theme-%1").arg(i));}
        combo->setCurrentIndex(0);
        press("routeBtn");save(w,"07-checkpoints");
        auto checkpointClick=[&](int id) {
            click(view,view->worldTransform().map(manager->checkpoints()[id].worldPos));
        };
        checkpointClick(0);checkpointClick(15);
        check(manager->isNavigating(),"Checkpoint clicks did not start GPS");
        for(int i=0;i<120;++i)QMetaObject::invokeMethod(&w,"tick",Qt::DirectConnection);
        save(w,"08-navigation");
        QWheelEvent zoom(QPointF(600,400),view->mapToGlobal(QPoint(600,400)),QPoint(),QPoint(0,480),Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
        QApplication::sendEvent(view,&zoom);save(w,"09-zoom");
        for(int i=0;i<8;++i)QApplication::sendEvent(view,&zoom);
        const auto beforePan=view->worldTransform();
        QMouseEvent panDown(QEvent::MouseButtonPress,QPointF(400,300),view->mapToGlobal(QPoint(400,300)),Qt::LeftButton,Qt::LeftButton,Qt::ControlModifier);
        QMouseEvent panMove(QEvent::MouseMove,QPointF(430,320),view->mapToGlobal(QPoint(430,320)),Qt::NoButton,Qt::LeftButton,Qt::ControlModifier);
        QMouseEvent panUp(QEvent::MouseButtonRelease,QPointF(430,320),view->mapToGlobal(QPoint(430,320)),Qt::LeftButton,Qt::NoButton,Qt::ControlModifier);
        QApplication::sendEvent(view,&panDown);QApplication::sendEvent(view,&panMove);QApplication::sendEvent(view,&panUp);
        check(view->worldTransform()!=beforePan,"Pan did not move camera");
        check(std::abs(view->worldTransform().m11()-beforePan.m11())<1e-9,"Pan altered zoom");
        save(w,"09-detail");
        QKeyEvent escape(QEvent::KeyPress,Qt::Key_Escape,Qt::NoModifier);
        QApplication::sendEvent(&w,&escape);check(!manager->isNavigating(),"Esc did not cancel GPS");
        if(!menu->isOpen())menu->toggle();
        press("routeBtn");if(!menu->isOpen())menu->toggle();press("cancelBtn");
        for(QSize sz:{QSize(960,600),QSize(1366,768),QSize(1920,1080)}) {
            w.resize(sz);save(w,QString("size-%1x%2").arg(sz.width()).arg(sz.height()));
            check(w.size()==sz,"Main layout cannot fit requested size");
            check(!view->geometry().intersects(menu->geometry()),"Resized sidebar overlaps map");
        }
        press("backBtn");check(start->isVisible(),"Return to menu failed");
        press("beginButton");press("themeNext");press("acceptMap");press("startButton");
        check(std::abs(view->worldTransform().m11()-WorldView::fit(view->size()))<1e-8,"New simulation retained previous zoom");
        press("backBtn");
#endif
        w.hide();
        TestWindow tests;tests.resize(1366,768);tests.show();save(tests,"10-tests");
        auto rows=tests.findChildren<TestRow*>();check(rows.size()==51,"Missing test rows");
        click(rows.front(),rows.front()->rect().center());save(tests,"11-test-result");
#ifdef REDESIGN
        check(tests.height()<=768,"Test report forces window taller than laptop screen");
        auto* run=tests.findChild<QPushButton*>("runBtn");auto* cancel=tests.findChild<QPushButton*>("cancelBtn");
        QTimer::singleShot(100,cancel,&QPushButton::click);run->click();
        check(run->isEnabled(),"Test cancellation did not restore run control");
        save(tests,"12-test-cancel");
#endif
        tests.hide();

        benchmark(out);
        QTextStream(stdout)<<"PASS: visual checks completed\n";
    } catch(const std::exception& e) { QTextStream(stderr)<<"FAIL: "<<e.what()<<"\n";return 1; }
    return 0;
}
