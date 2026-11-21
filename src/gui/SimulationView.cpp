#include "SimulationView.h"
#include "WorldView.h"
#include "../core/SimulationEngine.h"
#include "../core/RoadGeometry.h"

#include <QPainter>
#include <QPainterPath>


#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QKeyEvent>
#include <QDebug>
#include <QToolTip>
#include "MapRenderer.h"
#include <cmath>

// ═══════════════════════════════════════════════════════════════════
//  Zarządzanie stanem widoku
// ═══════════════════════════════════════════════════════════════════
SimulationView::SimulationView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(400, 300);
    m_presentTimer.setTimerType(Qt::PreciseTimer);
    m_presentTimer.setInterval(8);
    connect(&m_presentTimer,&QTimer::timeout,this,[this] {
        update(); if(m_carRenderer.frameAge()>=32) m_presentTimer.stop();
    });
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);   // potrzebne do podpowiedzi kursora nad HUD-em
}

void SimulationView::captureFrame() {
    if(!m_engine) return;
    m_carRenderer.capture(m_engine);
    if(!m_presentTimer.isActive()) m_presentTimer.start();
}
void SimulationView::resetPresentation() {
    m_presentTimer.stop();
    m_carRenderer = CarRenderer();
    m_staticMap = {};
    update();
}
/// Widok domyślny: skala minimalna (COVER), środek kanwy.
void SimulationView::fitMap() {
    m_zoom = minZoom();
    m_pan = WorldView::canvas().center()-QPointF(width(),height())/(2*m_zoom);
    clampPan();
    update();
}
void SimulationView::setEngine(SimulationEngine* e) { m_engine = e; resetPresentation(); }

void SimulationView::setCheckpoints(const std::vector<Checkpoint>& cps) {
    m_checkpoints = cps; update();
}
void SimulationView::clearCheckpoints() { m_checkpoints.clear(); update(); }
void SimulationView::setCheckpointState(int cpId, Checkpoint::UIState st) {
    if (cpId >= 0 && cpId < (int)m_checkpoints.size())
        m_checkpoints[cpId].uiState = st;
    update();
}
void SimulationView::setRoute(const Route& route) {
    m_route = route;
    if (!m_presentTimer.isActive()) update();
}
void SimulationView::clearRoute()                  { m_route.clear(); update(); }
void SimulationView::setRouteSelectionMode(bool a) {
    m_routeSelectMode = a;
    if (!a) { m_pickStage = RoutePicker::Stage::None; m_pickSelectable.clear(); m_pickStart = -1;
              m_hoverCp = m_hoverTab = -1; QToolTip::hideText(); setCursor(Qt::ArrowCursor); }
    update();
}
void SimulationView::setRoutePickStage(RoutePicker::Stage stage, const std::vector<int>& selectable, int startId) {
    m_pickStage = stage;
    m_pickSelectable.assign(m_checkpoints.size(), false);
    for (int id : selectable) if (id >= 0 && id < (int)m_pickSelectable.size()) m_pickSelectable[id] = true;
    m_pickStart = startId;
    m_hoverCp = m_hoverTab = -1;
    update();
}
RoutePicker::Layout SimulationView::pickerLayout() const {
    RoutePicker::Input in;
    in.viewport = QSizeF(width(), height());
    in.world = worldTransform();
    in.cps = &m_checkpoints;
    in.stage = m_routeSelectMode ? m_pickStage : RoutePicker::Stage::None;
    in.selectable = m_pickSelectable;
    in.startId = m_pickStart;
    in.signalWorldRects = MapRenderer::signalHeadRects(m_engine);
    in.instructionRect = RoutePicker::instructionRect(in.viewport);
    if (m_engine)
        for (int vi = 0; vi < kNumRoads; ++vi)
            for (int hi = 0; hi < kNumRoads; ++hi)
                if (m_engine->intersectionAt(vi, hi).isRoundabout) in.roundabouts |= quint64(1) << (vi * kNumRoads + hi);
    return RoutePicker::layout(in);
}
/// Zakładka „punkty poza widokiem": przesuwa kamerę tak, by NAJBLIŻSZY ukryty punkt
/// z tej strony znalazł się w widoku (kolejne kliknięcia — kolejne punkty).
/// Przesunięcie do skrajnego położenia pomijałoby punkty w środku krawędzi.
void SimulationView::panToSide(int side) {
    const RoutePicker::Layout l = pickerLayout();
    const QRectF vr(0, 0, width(), height());
    const RoutePicker::Marker* best = nullptr; double bestOver = 1e18;
    for (const auto& m : l.markers) {
        if (!m.selectable || m.offSide != side) continue;
        const double over = std::max({ vr.left() - m.anchor.x(), m.anchor.x() - vr.right(),
                                       vr.top() - m.anchor.y(), m.anchor.y() - vr.bottom() });
        if (over < bestOver) { bestOver = over; best = &m; }
    }
    if (!best) return;
    const QPointF w = worldTransform().inverted().map(best->anchor);
    const double margin = RoutePicker::kHit + 24;   // px — miejsce na znacznik i odstęp
    auto fitAxis = [&](double worldCoord, double screenCoord, double extent, double& pan) {
        if (screenCoord < margin) pan = worldCoord - margin / m_zoom;
        else if (screenCoord > extent - margin) pan = worldCoord - (extent - margin) / m_zoom;
    };
    double px = m_pan.x(), py = m_pan.y();
    fitAxis(w.x(), best->anchor.x(), width(), px);
    fitAxis(w.y(), best->anchor.y(), height(), py);
    m_pan = QPointF(px, py);
    clampPan(); update();
}
void SimulationView::setHeatmapEnabled(bool on)    { m_heatmapEnabled = on; update(); }
void SimulationView::setStatsText(const QString& t){
    if (m_statsText == t) return;
    m_statsText = t;
    if (!m_presentTimer.isActive()) update();
}

// ═══════════════════════════════════════════════════════════════════
//  Przesunięcie i powiększenie
// ═══════════════════════════════════════════════════════════════════

/// Minimalny zoom (COVER): kanwa pokrywa cały viewport w obu osiach.
///   minZoom = max(szerokośćWidoku / szerokośćKanwy, wysokośćWidoku / wysokośćKanwy)
/// (m_zoom = piksele logiczne ekranu na jednostkę świata; obrót brak, skala jednakowa).
double SimulationView::minZoom() const { return WorldView::cover(QSizeF(width(), height())); }

/// Ograniczenie kamery: widoczny wycinek świata [pan, pan + rozmiar/zoom] musi leżeć
/// w całości w kanwie [0, W] × [0, H]. Ponieważ zoom ≥ minZoom (COVER), szerokość
/// i wysokość wycinka nie przekraczają kanwy, więc przedział pan jest zawsze niepusty.
/// Stosowane po KAŻDEJ zmianie: pan, zoom, rozmiar okna, panel boczny, reset.
void SimulationView::clampPan() {
    const double zmin = minZoom();
    m_zoom = std::clamp(m_zoom, zmin, std::max(kMaxZoom, zmin));
    const double vw = width() / m_zoom, vh = height() / m_zoom;
    m_pan.setX(std::clamp(m_pan.x(), 0.0, std::max(0.0, kCanvasW - vw)));
    m_pan.setY(std::clamp(m_pan.y(), 0.0, std::max(0.0, kCanvasH - vh)));
}
void SimulationView::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    // Zachowaj stopień przybliżenia względem skali minimalnej i środek widoku.
    const QSizeF old = e->oldSize();
    const bool hadOld = old.width() > 0 && old.height() > 0;
    const double ratio = hadOld ? std::max(1.0, m_zoom / WorldView::cover(old)) : 1.0;
    const QPointF center = hadOld ? m_pan + QPointF(old.width(), old.height()) / (2 * m_zoom)
                                  : WorldView::canvas().center();
    m_zoom = minZoom() * ratio;
    m_pan = center - QPointF(width(), height()) / (2 * m_zoom);
    clampPan();
}

void SimulationView::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        bool ctrl = e->modifiers() & Qt::ControlModifier;

        // (1) Tryb wyboru trasy — najwyższy priorytet: LPM bez Ctrl = klik
        //     checkpointu. HUD instruktażowy NIE może blokować klików w
        //     checkpointy pod nim — dlatego HUD-drag jest tu wyłączony.
        if (m_routeSelectMode && !ctrl) {
            // Ten sam układ, który jest rysowany — hitbox = znacznik.
            const RoutePicker::Layout l = pickerLayout();
            const int cp = RoutePicker::hitMarker(l, e->position());
            if (cp >= 0) {
                if (m_debugCheckpoints) qDebug() << "[CP] HIT id=" << cp << "screen=" << e->position();
                emit checkpointClicked(cp);
                e->accept(); return;
            }
            const int tab = RoutePicker::hitTab(l, e->position());
            if (tab >= 0) { panToSide(tab); updateHover(e->position()); e->accept(); return; }
            // Klik poza znacznikami: przeciąganie przesuwa mapę (dojazd do punktów).
        }

        // (2) Drag HUD-a (panel „Nawigacja GPS") — działa tylko POZA trybem
        //     wyboru trasy, więc nie koliduje z klikiem w checkpoint.
        if (!m_routeSelectMode && !m_statsText.isEmpty() && m_hudScreenRect.contains(e->position())) {
            m_hudDragging         = true;
            m_hudDragStart        = e->position();
            m_hudOffsetAtDragStart = m_hudOffset;
            setCursor(Qt::ClosedHandCursor);
            e->accept(); return;
        }

        // (3) Pan mapy (LPM z Ctrl, lub LPM poza trybem wyboru i poza HUDem).
        m_panning = true;
        m_panStart = e->position();
        setCursor(Qt::ClosedHandCursor);
        e->accept(); return;
    }
    QWidget::mousePressEvent(e);
}

void SimulationView::mouseMoveEvent(QMouseEvent* e) {
    if (m_hudDragging) {
        // Przesuwanie HUD-a — clamp do prostokąta widżetu, aby zawsze
        // pozostawał w pełni widoczny.
        QPointF d = e->position() - m_hudDragStart;
        QPointF newOffset = m_hudOffsetAtDragStart + d;
        // Wyliczamy dopuszczalny zakres tak, by m_hudScreenRect mieścił się w widoku.
        const QPointF base(18, 18);
        double minX = -base.x() + 4;
        double maxX = width()  - base.x() - m_hudScreenRect.width()  - 4;
        double minY = -base.y() + 4;
        double maxY = height() - base.y() - m_hudScreenRect.height() - 4;
        if (maxX < minX) maxX = minX;
        if (maxY < minY) maxY = minY;
        newOffset.setX(std::clamp(newOffset.x(), minX, maxX));
        newOffset.setY(std::clamp(newOffset.y(), minY, maxY));
        m_hudOffset = newOffset;
        update();
        return;
    }
    if (m_panning) {
        QPointF d = (e->position() - m_panStart) / m_zoom;
        m_pan -= QPointF(d.x(), d.y());
        m_panStart = e->position();
        clampPan(); update();
        return;
    }
    if (m_routeSelectMode) { updateHover(e->position()); return; }
    // Kursor „grab" gdy najechany na HUD (sugestia, że można go przesunąć).
    // Wyłączamy ten hint w trybie wyboru trasy — wówczas klik w HUD trafia
    // w checkpoint pod spodem (nie w sam HUD).
    if (!m_routeSelectMode && !m_statsText.isEmpty()
        && m_hudScreenRect.contains(e->position())) {
        setCursor(Qt::OpenHandCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void SimulationView::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        if (m_hudDragging) {
            m_hudDragging = false;
            setCursor(m_hudScreenRect.contains(e->position())
                       ? Qt::OpenHandCursor : Qt::ArrowCursor);
            return;
        }
        if (m_panning) {
            m_panning = false;
            setCursor(Qt::ArrowCursor);
        }
    }
}

void SimulationView::setZoomAt(double zoom, QPointF cursor) {
    // Punkt świata pod kursorem pozostaje pod kursorem; potem ograniczenie kamery
    // (przy krawędzi kanwy ograniczenie ma pierwszeństwo — brak pustych pasów).
    const double oldZoom = m_zoom;
    m_zoom = std::clamp(zoom, minZoom(), std::max(kMaxZoom, minZoom()));
    const QPointF worldBefore = cursor / oldZoom + m_pan;
    const QPointF worldAfter  = cursor / m_zoom + m_pan;
    m_pan += (worldBefore - worldAfter);
    clampPan(); update();
    if (m_routeSelectMode) updateHover(cursor);
}

void SimulationView::wheelEvent(QWheelEvent* e) {
    const double factor = (e->angleDelta().y() > 0) ? 1.15 : (1.0/1.15);
    setZoomAt(m_zoom * factor, e->position());
    e->accept();
}

void SimulationView::updateHover(QPointF pos) {
    const RoutePicker::Layout l = pickerLayout();
    const int cp = RoutePicker::hitMarker(l, pos);
    const int tab = cp < 0 ? RoutePicker::hitTab(l, pos) : -1;
    if (cp != m_hoverCp || tab != m_hoverTab) {
        m_hoverCp = cp; m_hoverTab = tab;
        if (cp >= 0)
            QToolTip::showText(mapToGlobal(pos.toPoint()),
                m_pickStage == RoutePicker::Stage::Start ? QString::fromUtf8("Wybierz punkt startowy")
                                                         : QString::fromUtf8("Wybierz punkt docelowy"), this);
        else if (tab >= 0)
            QToolTip::showText(mapToGlobal(pos.toPoint()), QString::fromUtf8("Przesuń widok do punktów poza ekranem"), this);
        else
            QToolTip::hideText();
        update();
    }
    setCursor((cp >= 0 || tab >= 0) ? Qt::PointingHandCursor : Qt::ArrowCursor);
}

void SimulationView::keyPressEvent(QKeyEvent* e) {
    // F3 — przełącz tryb diagnostyczny checkpointów.
    // Rysuje hitboxy (okręgi o promieniu 24), IDki obok markerów
    // i loguje każdy klik w konsoli aplikacji.
    if (e->key() == Qt::Key_F3) {
        m_debugCheckpoints = !m_debugCheckpoints;
        qDebug() << "[CP] tryb diagnostyczny:"
                 << (m_debugCheckpoints ? "WŁ" : "WYŁ")
                 << " — F3 przełącza";
        update();
        e->accept(); return;
    }
    QWidget::keyPressEvent(e);
}

// ═══════════════════════════════════════════════════════════════════
//  Rysowanie — delegacja do rendererów z aktywnym motywem
// ═══════════════════════════════════════════════════════════════════
void SimulationView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    if (!m_engine) { p.fillRect(rect(), m_theme.grassTop); return; }
    // Cache only static geometry, at the exact viewport resolution and DPR.
    // Light states, traffic overlays and cars are always drawn live.
    quint64 roundabouts = 0;
    for (int vi=0; vi<kNumRoads; ++vi)
        for (int hi=0; hi<kNumRoads; ++hi)
            if (m_engine->intersectionAt(vi,hi).isRoundabout)
                roundabouts |= quint64(1) << (vi*kNumRoads+hi);
    const qreal dpr = devicePixelRatioF();
    const QSize pixels(qCeil(width()*dpr), qCeil(height()*dpr));
    if (m_staticMap.size()!=pixels || m_staticMap.devicePixelRatio()!=dpr ||
        m_cachedTransform!=worldTransform() || m_cachedRoundabouts!=roundabouts) {
        m_staticMap = QPixmap(pixels);
        m_staticMap.setDevicePixelRatio(dpr);
        m_staticMap.fill(m_theme.grassTop);
        QPainter background(&m_staticMap);
        background.setRenderHint(QPainter::Antialiasing);
        background.setTransform(worldTransform());
        m_mapRenderer.drawRoads(background, m_theme);
        m_mapRenderer.drawMarkings(background, m_theme);
        m_mapRenderer.drawRoundabouts(background, m_engine, m_theme);
        m_cachedTransform = worldTransform();
        m_cachedRoundabouts = roundabouts;
    }
    p.drawPixmap(0, 0, m_staticMap);

    // Transformacja zoom + pan
    p.save();
    p.setTransform(worldTransform());

    // Renderery (kolejność: od tła do pierwszego planu)
    if (m_heatmapEnabled)
        m_overlayRenderer.drawHeatmap(p, m_engine);
    // Pass SmartCar position so the GPS line is trimmed to start AT the car
    // (GTA-style — minięte odcinki znikają). Na rondzie: NIE przekazujemy
    // pozycji, by linia była STATYCZNA i nie "wymachiwała" przy okrążeniach.
    QPointF carPos;
    const double alpha=m_carRenderer.presentationAlpha();
    if (m_engine->hasSmartCar() && !m_engine->smartCarOnRing())
        carPos = m_carRenderer.displayedSmartCarCenter(m_engine,alpha);
    m_overlayRenderer.drawRoute(p, m_route, m_theme, carPos);
    m_mapRenderer.drawSignals(p, m_engine, m_theme); // GPS must not obscure signal states.
    m_carRenderer.drawCars(p, m_engine, m_theme,alpha);

    p.restore();

    // ── Wybór trasy: znaczniki START/CEL i instrukcja (przestrzeń ekranu) ──
    if (m_routeSelectMode && m_pickStage != RoutePicker::Stage::None) {
        const RoutePicker::Layout l = pickerLayout();
        RoutePicker::draw(p, l, m_pickStage, m_hoverCp, m_hoverTab);
        RoutePicker::drawInstruction(p, RoutePicker::instructionRect(QSizeF(width(), height())), m_pickStage);
        if (m_debugCheckpoints) {   // F3: hitboxy i id — dokładnie te, które obsługują klik
            p.save();
            p.setPen(QPen(QColor(0, 255, 200, 220), 1.0)); p.setBrush(QColor(0, 255, 200, 30));
            for (const auto& m : l.markers) if (m.visible && m.selectable) {
                p.drawRect(m.hit);
                p.drawText(m.hit.adjusted(2, 0, 0, 0), Qt::AlignLeft | Qt::AlignTop, QString::number(m.cpId));
            }
            p.setPen(QPen(QColor(255, 0, 200, 220), 1.0)); p.setBrush(Qt::NoBrush);
            for (const QRectF& r : l.signalScreenRects) p.drawRect(r);
            p.restore();
        }
    }

    // HUD statystyk w przestrzeni ekranu — pozycja = (18, 18) + offset od użytkownika
    QPointF hudTopLeft(18 + m_hudOffset.x(), 18 + m_hudOffset.y());
    QRectF  hudRect;
    m_overlayRenderer.drawStatsHUD(p, m_statsText, rect(), hudTopLeft, &hudRect);
    m_hudScreenRect = hudRect;   // cache do hit-testu w mousePressEvent
}
