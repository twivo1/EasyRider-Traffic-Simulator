/**
 * @file gui_check.cpp
 * @brief Skryptowana weryfikacja RZECZYWISTEGO okna EasyRidera (MainWindow).
 *
 *  Uruchamia produkcyjne MainWindow w widocznym oknie (platforma systemowa,
 *  prawdziwy QTimer 16 ms, RouteManager, SimulationView::paintEvent, interpolacja
 *  CarRenderer) i steruje nim zdarzeniami wejścia: kliknięcia przycisków ekranu
 *  startowego i menu, suwak tempa, przycisk mapy korków, kliknięcia myszą w
 *  checkpointy (przez hit-test widoku). To NIE jest obsługa przez człowieka —
 *  to automatyczna sesja na prawdziwym GUI z zapisem zrzutów do oceny.
 *
 *  Po każdym kroku silnika:
 *   • SimMonitor — inwarianty silnika (obrócone obrysy, skoki, legalność),
 *   • pozy RYSOWANE przez CarRenderer dla α ∈ {0, ¼, ½, ¾, 1} (wszystkie stany,
 *     jakie paintEvent może pokazać między krokami): zgodność z silnikiem,
 *     ciągłość, skoki kursu, nakładanie obrysów — liczone NIEZALEŻNIE metodą
 *     przycinania wielokątów (Sutherland–Hodgman), a nie SAT.
 *
 *  Użycie: EasyRiderGuiCheck <katalog_wyników>
 */
#include <QApplication>
#include <QTest>
#include <QScreen>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QElapsedTimer>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QImage>
#include <QPainter>
#include <QWindow>
#include <cmath>
#include <map>
#include <unordered_map>

#include "gui/MainWindow.h"
#include "gui/SimulationView.h"
#include "gui/StartScreen.h"
#include "gui/SideMenu.h"
#include "gui/Theme.h"
#include "nav/RouteManager.h"
#include "nav/DynamicPathfinder.h"
#include "nav/TrafficAnalyzer.h"
#include "core/SimulationEngine.h"
#include "core/Car.h"
#include "core/SimMonitor.h"

namespace {

using Poly = std::vector<QPointF>;

Poly quad(QPointF c, double hDeg, double L = kCarL, double W = kCarW) {
    const double h = hDeg * M_PI / 180.0, cs = std::cos(h), sn = std::sin(h);
    const QPointF ax(cs * L / 2, sn * L / 2), ay(-sn * W / 2, cs * W / 2);
    return { c + ax + ay, c - ax + ay, c - ax - ay, c + ax - ay };
}
double area(const Poly& p) {
    double a = 0; for (size_t i = 0; i < p.size(); ++i) { const QPointF& u = p[i]; const QPointF& v = p[(i + 1) % p.size()]; a += u.x() * v.y() - v.x() * u.y(); }
    return std::abs(a) / 2;
}
/// Przecięcie dwóch wypukłych wielokątów (Sutherland–Hodgman) — pole części wspólnej.
double clipArea(Poly subject, const Poly& clip) {
    // orientacja przycinającego
    double s = 0; for (size_t i = 0; i < clip.size(); ++i) { const QPointF& u = clip[i]; const QPointF& v = clip[(i + 1) % clip.size()]; s += u.x() * v.y() - v.x() * u.y(); }
    const double sign = s >= 0 ? 1 : -1;
    for (size_t i = 0; i < clip.size() && !subject.empty(); ++i) {
        const QPointF A = clip[i], B = clip[(i + 1) % clip.size()];
        auto inside = [&](QPointF P) { return sign * ((B.x() - A.x()) * (P.y() - A.y()) - (B.y() - A.y()) * (P.x() - A.x())) >= 0; };
        auto inter = [&](QPointF P, QPointF Q) {
            const double a1 = (B.x() - A.x()) * (P.y() - A.y()) - (B.y() - A.y()) * (P.x() - A.x());
            const double a2 = (B.x() - A.x()) * (Q.y() - A.y()) - (B.y() - A.y()) * (Q.x() - A.x());
            const double t = a1 / (a1 - a2); return P + (Q - P) * t;
        };
        Poly out;
        for (size_t k = 0; k < subject.size(); ++k) {
            const QPointF P = subject[k], Q = subject[(k + 1) % subject.size()];
            const bool pi = inside(P), qi = inside(Q);
            if (pi && qi) out.push_back(Q);
            else if (pi && !qi) out.push_back(inter(P, Q));
            else if (!pi && qi) { out.push_back(inter(P, Q)); out.push_back(Q); }
        }
        subject = out;
    }
    return subject.size() >= 3 ? area(subject) : 0.0;
}

struct PhaseStats {
    QString name; double simStart = 0, simEnd = 0; long frames = 0; int maxCars = 0;
    long renderMismatch = 0, renderOverlaps = 0, renderJumps = 0, renderHeadingJumps = 0, renderDiscont = 0;
    long engineClipOverlaps = 0, missed = 0; double maxRenderStepExcess = 0, maxRenderHeadingStep = 0;
    MonitorStats mon; QStringList log, monLog;
};

class Checker : public QObject {
public:
    MainWindow* w = nullptr; SimulationView* view = nullptr; SimulationEngine* eng = nullptr;
    QTimer sampler; double simTime = 0, prevPos = -1; int refVi = -1, refHi = -1;
    std::unique_ptr<SimMonitor> mon; PhaseStats cur; std::vector<PhaseStats> done;
    struct Prev { QPointF c; double a; QPointF ec; };
    std::unordered_map<int, Prev> prev;          // klucz: id auta (adresy są ponownie używane)
    double nominalDt = 0.016;                     // 0,016 × tempo — krok jednego ticka MainWindow
    long missedTicks = 0;
    QStringList globalLog;

    void pickRef() {
        refVi = refHi = -1;
        for (int v = 0; v < kNumRoads && refVi < 0; ++v) for (int h = 0; h < kNumRoads; ++h)
            if (!eng->intersectionAt(v, h).isRoundabout) { refVi = v; refHi = h; break; }
        prevPos = -1;
    }
    double cyclePos() const {
        static const double dur[4] = { 8, 2, 8, 2 };
        const auto& s = eng->intersectionAt(refVi, refHi);
        double p = s.timer; for (int i = 0; i < s.phase; ++i) p += dur[i];
        return p;
    }
    void begin(const QString& name) {
        cur = PhaseStats(); cur.name = name; cur.simStart = simTime;
        mon = std::make_unique<SimMonitor>(1.0); prev.clear();
    }
    void end() {
        cur.simEnd = simTime; cur.mon = mon->stats(); cur.monLog = mon->violations().mid(0, 8);
        done.push_back(cur);
    }
    void start() {
        sampler.setTimerType(Qt::PreciseTimer);
        connect(&sampler, &QTimer::timeout, this, [this] { sample(); });
        sampler.start(2);
    }
    void sample() {
        if (!eng || refVi < 0) return;
        const double pos = cyclePos();
        if (prevPos < 0) { prevPos = pos; return; }
        double dt = pos - prevPos; if (dt < 0) dt += 20.0;
        if (dt < 1e-9) return;                    // brak nowego kroku silnika
        prevPos = pos; simTime += dt; cur.frames++;
        mon->observe(*eng, dt);
        // Pominięty tick (próbkowanie nie nadążyło): ciągłości α=0 nie da się
        // ocenić — poprzednio zapamiętany stan jest starszy niż „before" renderera.
        const bool missed = dt > nominalDt * 1.5;
        if (missed) { cur.missed++; missedTicks++; }
        checkRender(dt, missed);
    }
    void checkRender(double dt, bool missed) {
        const CarRenderer& cr = view->carRenderer();
        struct R { const Car* car; QPointF ec; double ea; DisplayPose p[5]; };
        std::vector<R> rs;
        const double alphas[5] = { 0.0, 0.25, 0.5, 0.75, 1.0 };
        for (const auto& o : eng->objects()) {
            const Car* c = dynamic_cast<const Car*>(o.get()); if (!c) continue;
            R r{ c, c->center(), c->headingAngle(), {} };
            for (int k = 0; k < 5; ++k) r.p[k] = cr.displayedPose(o.get(), alphas[k]);
            rs.push_back(r);
        }
        cur.maxCars = std::max<int>(cur.maxCars, (int)rs.size());
        for (const R& r : rs) {
            // α=1 musi być DOKŁADNIE stanem silnika
            const double dev = QLineF(r.p[4].center, r.ec).length();
            const double adev = std::abs(std::remainder(r.p[4].angle - r.ea, 360.0));
            if (dev > 0.01 || adev > 0.01) {
                cur.renderMismatch++;
                if (cur.log.size() < 20) cur.log << QString("t=%1 RENDER≠ENGINE car#%2 Δ=%3 px Δkąt=%4°").arg(simTime, 0, 'f', 2).arg(r.car->id()).arg(dev, 0, 'f', 3).arg(adev, 0, 'f', 2);
            }
            auto it = prev.find(r.car->id());
            if (it != prev.end() && !missed) {
                // α=0 musi kontynuować poprzednio narysowany stan końcowy (brak skoku przy nowym kroku)
                const double disc = QLineF(r.p[0].center, it->second.c).length();
                if (disc > 0.01) {
                    cur.renderDiscont++;
                    if (cur.log.size() < 20) cur.log << QString("t=%1 RENDER-DISCONT car#%2 %3 px").arg(simTime, 0, 'f', 2).arg(r.car->id()).arg(disc, 0, 'f', 2);
                }
                const double engStep = QLineF(r.ec, it->second.ec).length();
                for (int k = 1; k < 5; ++k) {
                    const double st = QLineF(r.p[k].center, r.p[k - 1].center).length();
                    const double excess = st - (engStep * 0.25 + 0.3);
                    cur.maxRenderStepExcess = std::max(cur.maxRenderStepExcess, excess);
                    if (excess > 0) {
                        cur.renderJumps++;
                        if (cur.log.size() < 20) cur.log << QString("t=%1 RENDER-JUMP car#%2 krok %3 px (silnik %4 px/klatkę)").arg(simTime, 0, 'f', 2).arg(r.car->id()).arg(st, 0, 'f', 2).arg(engStep, 0, 'f', 2);
                    }
                    const double ha = std::abs(std::remainder(r.p[k].angle - r.p[k - 1].angle, 360.0));
                    cur.maxRenderHeadingStep = std::max(cur.maxRenderHeadingStep, ha);
                    if (ha > 10.0) {
                        cur.renderHeadingJumps++;
                        if (cur.log.size() < 20) cur.log << QString("t=%1 RENDER-HEADING car#%2 %3°").arg(simTime, 0, 'f', 2).arg(r.car->id()).arg(ha, 0, 'f', 1);
                    }
                }
            }
            prev[r.car->id()] = { r.p[4].center, r.p[4].angle, r.ec };
        }
        // nakładanie: rysowane (każde α) i silnik — przycinanie wielokątów
        for (size_t i = 0; i < rs.size(); ++i) for (size_t j = i + 1; j < rs.size(); ++j) {
            if (QLineF(rs[i].ec, rs[j].ec).length() > 60) continue;
            if (clipArea(quad(rs[i].ec, rs[i].ea), quad(rs[j].ec, rs[j].ea)) > 0.05) {
                cur.engineClipOverlaps++;
                if (cur.log.size() < 20) cur.log << QString("t=%1 ENGINE-OVERLAP car#%2/#%3").arg(simTime, 0, 'f', 2).arg(rs[i].car->id()).arg(rs[j].car->id());
            }
            for (int k = 0; k < 4; ++k) {
                if (clipArea(quad(rs[i].p[k].center, rs[i].p[k].angle), quad(rs[j].p[k].center, rs[j].p[k].angle)) > 0.05) {
                    cur.renderOverlaps++;
                    if (cur.log.size() < 20) cur.log << QString("t=%1 RENDER-OVERLAP α=%2 car#%3/#%4").arg(simTime, 0, 'f', 2).arg(k * 0.25).arg(rs[i].car->id()).arg(rs[j].car->id());
                }
            }
        }
    }
    bool waitSim(double s, double wallMaxSec, std::function<bool()> stop = nullptr) {
        const double target = simTime + s; QElapsedTimer t; t.start();
        while (simTime < target && t.elapsed() < wallMaxSec * 1000) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
            if (stop && stop()) return true;
        }
        return simTime >= target;
    }
    void waitWall(int ms) { QElapsedTimer t; t.start(); while (t.elapsed() < ms) QCoreApplication::processEvents(QEventLoop::AllEvents, 5); }
};

QString outDir;
void shot(MainWindow& w, const QString& name) {
    // Rzeczywiste piksele z ekranu (kompozytor systemowy), nie render pozaekranowy.
    QPixmap px = w.screen()->grabWindow(w.winId());
    px.save(outDir + "/" + name + ".png");
}
/// Nakładanie kolejnych klatek prawdziwego paintEvent widoku (wycinek wokół punktu świata).
struct Onion {
    QImage acc; QRectF world; int n = 0;
    void add(SimulationView* v) {
        const QRect r = v->worldTransform().mapRect(world).toAlignedRect().intersected(v->rect());
        QImage f = v->grab(r).toImage().convertToFormat(QImage::Format_ARGB32);
        if (acc.isNull()) { acc = f; n = 1; return; }
        QPainter p(&acc); p.setCompositionMode(QPainter::CompositionMode_Lighten); p.drawImage(0, 0, f); n++;
    }
    void save(const QString& name) { if (!acc.isNull()) acc.save(outDir + "/" + name + ".png"); }
};

Car* placeCar(SimulationEngine* e, Direction d, int road, double along, int intent) {
    QPointF tl;
    switch (d) {
    case Direction::L2R: tl = { along - kCarL / 2, kL2R_Y(road) }; break;
    case Direction::R2L: tl = { along - kCarL / 2, kR2L_Y(road) }; break;
    case Direction::T2B: tl = { kT2B_X(road), along - kCarL / 2 }; break;
    default:             tl = { kB2T_X(road), along - kCarL / 2 }; break;
    }
    e->addCar(tl, d, road);
    Car* c = dynamic_cast<Car*>(e->objects().back().get());
    if (intent >= 0) c->setFixedIntent(intent);
    return c;
}

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    outDir = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QDir::currentPath() + "/gui_check_out";
    QDir().mkpath(outDir);
    QFile repf(outDir + "/report.txt"); repf.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream rep(&repf);
    auto say = [&](const QString& s) { rep << s << "\n"; rep.flush(); std::printf("%s\n", s.toUtf8().constData()); std::fflush(stdout); };

    MainWindow w; w.resize(1600, 960); w.show();
    if (!QTest::qWaitForWindowExposed(&w, 5000)) { say("BŁĄD: okno nie zostało wyświetlone"); return 2; }
    say(QString("Platforma Qt: %1, ekran %2x%3").arg(QGuiApplication::platformName())
            .arg(w.screen()->geometry().width()).arg(w.screen()->geometry().height()));

    auto* view = w.findChild<SimulationView*>();
    auto* rm   = w.findChild<RouteManager*>();
    auto click = [&](const char* name) {
        auto* b = w.findChild<QPushButton*>(name);
        if (!b) { say(QString("BRAK przycisku %1").arg(name)); return false; }
        if (b->isVisible()) QTest::mouseClick(b, Qt::LeftButton); else b->click();
        QTest::qWait(250); return true;
    };
    Checker ck; ck.w = &w; ck.view = view; ck.eng = view->engine();

    // ── 1. Ekran startowy: rzeczywiste przyciski ───────────────────────
    shot(w, "00-title");
    click("beginButton"); click("themeNext"); shot(w, "01-map-preview"); click("acceptMap");
    if (auto* s = w.findChild<QSlider*>("initialCarSlider")) s->setValue(45);
    click("startButton");
    QTest::qWait(500);
    ck.pickRef(); ck.start();
    say(QString("Start: aut=%1").arg(ck.eng->objects().size()));

    // ── 2. Ruch ogólny ×1 na mapie wygenerowanej przez ekran startowy ─
    ck.begin("Ruch ogólny ×1 (mapa z ekranu startowego, 45 aut)");
    ck.eng->setTargetCarCount(45);
    ck.waitSim(40, 70);
    shot(w, "02-traffic-x1");
    click("heatBtn"); QTest::qWait(400); shot(w, "03-heatmap");
    ck.end();

    // ── 3. ×4 suwakiem tempa ────────────────────────────────────────────
    auto* speed = w.findChild<QSlider*>("speedSlider");
    QObject::connect(speed, &QSlider::valueChanged, [&](int v) {
        static const double tbl[] = { 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0 };
        ck.nominalDt = 0.016 * tbl[std::clamp(v - 1, 0, 7)];
    });
    ck.begin("Ruch ogólny ×4 (suwak), 45 aut");
    if (speed) speed->setValue(8);
    ck.waitSim(80, 40);
    shot(w, "04-traffic-x4");
    ck.end();
    if (speed) speed->setValue(4);   // ×1

    // ── 4. Nawigacja przez GUI: przycisk trasy + kliknięcia myszą w checkpointy ──
    struct Trip { int a, b; };
    const Trip trips[] = { {0,13},{24,7},{16,27},{11,5},{3,30},{20,9},{28,2},{14,22},{6,18},{31,12} };
    if (speed) speed->setValue(8);   // ×4, by przejazdy trwały krócej w czasie rzeczywistym
    int arrived = 0, failed = 0, firstTurnOk = 0, firstTurnChecked = 0, ringPasses = 0;
    double sumPred = 0, sumAct = 0;
    int tripNo = 0;
    for (const Trip& tp : trips) {
        ++tripNo;
        ck.begin(QString("Nawigacja GUI %1→%2 (×4)").arg(tp.a).arg(tp.b));
        bool fin = false, fail = false;
        auto c1 = QObject::connect(rm, &RouteManager::navigationFinished, [&] { fin = true; });
        auto c2 = QObject::connect(rm, &RouteManager::navigationFailed, [&] { fail = true; });
        click("routeBtn");
        const auto& cps = rm->checkpoints();
        auto clickCp = [&](int id) {
            const QPoint p = view->worldTransform().map(cps[id].worldPos).toPoint();
            QTest::mouseClick(view, Qt::LeftButton, Qt::NoModifier, p); QTest::qWait(120);
        };
        clickCp(tp.a); clickCp(tp.b);
        if (!rm->isNavigating()) { say(QString("  trasa %1→%2: nawigacja NIE wystartowała").arg(tp.a).arg(tp.b)); ck.end(); QObject::disconnect(c1); QObject::disconnect(c2); continue; }
        const double pred = rm->routeCost(); const double t0 = ck.simTime;
        // Pierwszy węzeł trasy i planowany wyjazd z niego
        const Route r0 = rm->currentRoute();
        int fVi = -1, fHi = -1; Direction plannedExit = Direction::L2R; bool haveFirst = false;
        if (r0.size() >= 3 && r0[1].vi >= 0) { fVi = r0[1].vi; fHi = r0[1].hi; plannedExit = r0[2].dir; haveFirst = true; }
        bool wasAtFirst = false, firstJudged = false, wasOnRing = false; int ringsHere = 0;
        bool shotTaken = false;
        ck.waitSim(240, 120, [&] {
            const Car* sc = ck.eng->smartCar();
            if (sc) {
                const bool onRing = sc->isOnRing();
                if (onRing && !wasOnRing) ringsHere++;
                wasOnRing = onRing;
                if (haveFirst && !firstJudged) {
                    const bool atFirst = (sc->isOnRing() && sc->ringVI() == fVi && sc->ringHI() == fHi)
                                      || (sc->inManeuver() && sc->maneuverVI() == fVi && sc->maneuverHI() == fHi)
                                      || kIZone(fVi, fHi).contains(sc->center());
                    if (atFirst) {
                        wasAtFirst = true;
                        // planowany wyjazd mógł się zmienić przeliczeniem przed węzłem
                        for (size_t k = 0; k + 1 < rm->currentRoute().size(); ++k)
                            if (rm->currentRoute()[k].vi == fVi && rm->currentRoute()[k].hi == fHi) plannedExit = rm->currentRoute()[k + 1].dir;
                    } else if (wasAtFirst && !sc->inManeuver() && !sc->isOnRing()) {
                        firstJudged = true; firstTurnChecked++;
                        if (sc->dir() == plannedExit) firstTurnOk++;
                        else say(QString("  %1→%2: pierwszy węzeł (%3,%4) wyjazd %5 ≠ plan %6").arg(tp.a).arg(tp.b).arg(fVi).arg(fHi).arg((int)sc->dir()).arg((int)plannedExit));
                    }
                }
                if (!shotTaken && tripNo == 1 && ck.simTime - t0 > 12) { shot(w, "05-navigation"); shotTaken = true; }
            }
            return fin || fail;
        });
        const double act = ck.simTime - t0;
        ringPasses += ringsHere;
        if (fin) { arrived++; sumPred += pred; sumAct += act; }
        if (fail) failed++;
        say(QString("  trasa %1→%2: %3, przewid. %4 s, rzecz. %5 s, przeliczeń %6, przejazdów przez ronda %7")
                .arg(tp.a).arg(tp.b).arg(fin ? "DOTARŁ" : fail ? "ZGUBIŁ TRASĘ" : "TIMEOUT")
                .arg(pred, 0, 'f', 1).arg(act, 0, 'f', 1).arg(rm->recalcCount()).arg(ringsHere));
        QObject::disconnect(c1); QObject::disconnect(c2);
        ck.end();
        QTest::qWait(300);
    }
    say(QString("Nawigacja GUI: dotarło %1/10, zgubiło %2, pierwszy skręt zgodny %3/%4, przejazdów przez ronda %5, ETA Σprzewid. %6 s vs Σrzecz. %7 s")
            .arg(arrived).arg(failed).arg(firstTurnOk).arg(firstTurnChecked).arg(ringPasses).arg(sumPred, 0, 'f', 0).arg(sumAct, 0, 'f', 0));

    // ── 5. Reakcja na zatłoczenie: ciężki zator na trasie na wprost, trasa przez GUI ──
    auto resetScene = [&](std::vector<std::pair<int,int>> rbs) {
        ck.eng->reset(); ck.eng->setRoundabouts(rbs); view->setTheme(defaultTheme()); ck.pickRef(); QTest::qWait(100);
    };
    {
        resetScene({ {1,2},{2,5},{6,6} });          // brak ronda na korytarzu hi=3
        ck.eng->spawnCarsOnMap(30); ck.eng->setTargetCarCount(30);
        ck.begin("Zator vs wybór trasy (GUI, ×4)");
        ck.waitSim(6, 10);
        auto* e = ck.eng;
        auto freeAt = [&](QPointF topLeft, Direction d) {
            const bool h = dirIsHoriz(d);
            const QRectF r(topLeft, QSizeF(h ? kCarL : kCarW, h ? kCarW : kCarL));
            for (const auto& o : e->objects()) if (o->boundingRect().intersects(r.adjusted(-8, -8, 8, 8))) return false;
            return true;
        };
        // Koszt trasy na wprost 3→11 PRZED zatorem (ten sam model co RouteManager).
        TrafficAnalyzer ta(e); ta.update(); DynamicPathfinder pf(e, &ta);
        const auto cps = buildCheckpoints();
        Route straight; { DynamicPathfinder p2(e, &ta);
            p2.setCostOverride([](Direction d, int, int from, int to, int, int) { return segmentLength(d, from, to) / kMaxSpeed + (dirIsHoriz(d) ? 0 : 99); });
            straight = p2.findRoute(3, 11, cps); }
        const double costBefore = pf.evaluateRoute(straight, cps);
        int placed = 0, skipped = 0; std::vector<Car*> jam;
        for (int seg : { 4, 5 }) for (int i = 0; i < 6; ++i) {
            const double x = kVLeft[seg] - 30 - i * 47;
            const QPointF tl(x - kCarL / 2, kL2R_Y(3));
            if (!freeAt(tl, Direction::L2R)) { skipped++; continue; }
            Car* c = placeCar(e, Direction::L2R, 3, x, 0); c->setHeld(true); jam.push_back(c); placed++;
        }
        ck.waitSim(3, 10);
        ta.update();
        const double costAfter = pf.evaluateRoute(straight, cps);
        say(QString("Zator: postawiono %1 aut (pominięto zajęte miejsca: %2); koszt trasy na wprost 3→11: przed %3 s, po %4 s")
                .arg(placed).arg(skipped).arg(costBefore, 0, 'f', 1).arg(costAfter, 0, 'f', 1));
        bool fin = false; auto c1 = QObject::connect(rm, &RouteManager::navigationFinished, [&] { fin = true; });
        click("routeBtn");
        const auto& rcps = rm->checkpoints();
        for (int id : { 3, 11 }) { QTest::mouseClick(view, Qt::LeftButton, Qt::NoModifier, view->worldTransform().map(rcps[id].worldPos).toPoint()); QTest::qWait(120); }
        bool throughJam = false;
        for (size_t k = 0; k + 1 < rm->currentRoute().size(); ++k) {
            const auto& A = rm->currentRoute()[k]; const auto& B = rm->currentRoute()[k + 1];
            if (A.hi == 3 && B.hi == 3 && (B.vi == 4 || B.vi == 5) && B.dir == Direction::L2R) throughJam = true;
        }
        QTest::qWait(600); shot(w, "06-jam-route-heatmap");
        say(QString("  trasa wybrana przez GUI omija zator: %1 (przewidywany czas %2 s, trasa na wprost %3 s)")
                .arg(throughJam ? "NIE" : "TAK").arg(rm->routeCost(), 0, 'f', 1).arg(costAfter, 0, 'f', 1));
        const double t0 = ck.simTime; bool released = false;
        ck.waitSim(200, 90, [&] {
            if (!released && ck.simTime - t0 > 25) { for (Car* c : jam) c->setHeld(false); released = true; }
            return fin;
        });
        say(QString("  przejazd: %1 po %2 s").arg(fin ? "DOTARŁ" : "NIE DOTARŁ").arg(ck.simTime - t0, 0, 'f', 1));
        QObject::disconnect(c1);
        ck.end();
    }
    if (speed) speed->setValue(4);   // ×1

    // ── 6. Scenariusze mechaniki w prawdziwym oknie (×1) ─────────────────
    auto mech = [&](const QString& name, const QString& file, QRectF crop, std::vector<std::pair<int,int>> rbs,
                    std::function<void(SimulationEngine*)> setup, double secs, std::function<void(SimulationEngine*, double)> during = nullptr) {
        resetScene(rbs);
        ck.begin(name);
        setup(ck.eng);
        Onion on; on.world = crop; long lastFrame = -1; const double t0 = ck.simTime;
        ck.waitSim(secs, secs * 3 + 10, [&] {
            if (ck.cur.frames != lastFrame && ck.cur.frames % 4 == 0) { on.add(view); lastFrame = ck.cur.frames; }
            if (during) during(ck.eng, ck.simTime - t0);
            return false;
        });
        on.save(file);
        ck.end();
    };
    const QPointF rc(kVLeft[4] + kRoadW / 2, kHTop[4] + kRoadW / 2);
    const QRectF rbCrop(rc.x() - 200, rc.y() - 180, 400, 360);
    const QRectF xCrop(kVLeft[3] - 190, kHTop[3] - 170, 450, 410);
    mech("Skrzyżowanie z sygnalizacją: 8 aut, skręty i prosto", "10-intersection", xCrop, {}, [&](SimulationEngine* e) {
        const Direction ds[4] = { Direction::L2R, Direction::T2B, Direction::R2L, Direction::B2T };
        int k = 0; const QPointF xc(kVLeft[3] + kRoadW / 2, kHTop[3] + kRoadW / 2);
        for (int wv = 0; wv < 2; ++wv) for (Direction d : ds) {
            const double u = 120 + wv * 60; double along = 0;
            switch (d) { case Direction::L2R: along = xc.x() - u; break; case Direction::R2L: along = xc.x() + u; break;
                         case Direction::T2B: along = xc.y() - u; break; default: along = xc.y() + u; }
            placeCar(e, d, 3, along, (k++) % 3);
        }
    }, 30);
    mech("Kolejka na czerwonym i ruszanie: 7 aut", "11-queue", xCrop, {}, [&](SimulationEngine* e) {
        e->setIntersectionPhase(3, 3, 2, 0.0);
        for (int i = 0; i < 7; ++i) placeCar(e, Direction::L2R, 3, kVLeft[3] - 60 - i * 70, 0);
    }, 22);
    mech("Rondo: jeden pojazd na wprost", "12-rb-single", rbCrop, { {4,4} }, [&](SimulationEngine* e) {
        placeCar(e, Direction::L2R, 4, rc.x() - 190, 0);
    }, 12);
    mech("Rondo: 7 aut z 4 wlotów (różne zjazdy)", "13-rb-multi", rbCrop, { {4,4} }, [&](SimulationEngine* e) {
        placeCar(e, Direction::L2R, 4, rc.x() - 180, 0); placeCar(e, Direction::L2R, 4, rc.x() - 250, 2);
        placeCar(e, Direction::R2L, 4, rc.x() + 180, 1); placeCar(e, Direction::R2L, 4, rc.x() + 250, 0);
        placeCar(e, Direction::T2B, 4, rc.y() - 125, 2); placeCar(e, Direction::B2T, 4, rc.y() + 125, 1);
        placeCar(e, Direction::B2T, 4, rc.y() + 175, 0);
    }, 30);
    Car* blocker = nullptr;
    mech("Rondo: zablokowany zjazd (awaria 10 s) + 4 auta", "14-rb-blocked", rbCrop, { {4,4} }, [&](SimulationEngine* e) {
        blocker = placeCar(e, Direction::L2R, 4, rc.x() + 110, 0); blocker->setHeld(true);
        for (int i = 0; i < 3; ++i) placeCar(e, Direction::L2R, 4, rc.x() - 180 - i * 60, 0);
        placeCar(e, Direction::T2B, 4, rc.y() - 125, 2);
    }, 30, [&](SimulationEngine* e, double t) {
        if (t > 10) for (auto& o : e->objects()) if (auto* c = dynamic_cast<Car*>(o.get())) c->setHeld(false);
    });
    resetScene({ {1,2},{2,5},{4,4},{6,3},{5,6} });
    ck.eng->spawnCarsOnMap(45); ck.eng->setTargetCarCount(45);
    shot(w, "15-final-state");

    // ── Raport ───────────────────────────────────────────────────────────
    say("\nFAZA | czas sym. [s] | klatek | max aut | silnik: nakł.(SAT) / nakł.(clip) / styki / teleport. / skoki kursu / korekty / zjazd z toru | render: ≠silnik / nieciągł. / skoki / skoki kursu / nakładania | max Δkurs render");
    long bad = 0;
    for (const auto& p : ck.done) {
        const auto& m = p.mon;
        const long eng = m.overlaps + m.sweptHits + m.contacts + p.engineClipOverlaps + m.teleports + m.reverseJumps + m.headingJumps
                       + m.engineCorrections + m.offPath + m.illegalTransitions + m.uturns + m.nonFinite + m.vanished;
        const long ren = p.renderMismatch + p.renderDiscont + p.renderJumps + p.renderHeadingJumps + p.renderOverlaps;
        bad += eng + ren;
        say(QString("%1 | %2 | %3 | %4 | %5 / %6 / %7 / %8 / %9 / %10 / %11 | %12 / %13 / %14 / %15 / %16 | %17° | %18")
                .arg(p.name).arg(p.simEnd - p.simStart, 0, 'f', 1).arg(p.frames).arg(p.maxCars)
                .arg(m.overlaps + m.sweptHits).arg(p.engineClipOverlaps).arg(m.contacts).arg(m.teleports + m.reverseJumps)
                .arg(m.headingJumps).arg(m.engineCorrections).arg(m.offPath + m.illegalTransitions)
                .arg(p.renderMismatch).arg(p.renderDiscont).arg(p.renderJumps).arg(p.renderHeadingJumps).arg(p.renderOverlaps)
                .arg(p.maxRenderHeadingStep, 0, 'f', 1).arg(eng + ren == 0 ? "OK" : "BŁĘDY"));
        say(QString("     (pominięte ticki próbkowania: %1 z %2)").arg(p.missed).arg(p.frames));
        for (const auto& l : p.monLog.mid(0, 6)) say("     ↳ silnik: " + l);
        for (const auto& l : p.log.mid(0, 6)) say("     ↳ render: " + l);
    }
    say(QString("\nRAZEM naruszeń: %1").arg(bad));
    QTest::qWait(500);
    return bad == 0 && failed == 0 ? 0 : 1;
}
