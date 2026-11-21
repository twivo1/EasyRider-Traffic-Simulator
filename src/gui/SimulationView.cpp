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
void SimulationView::fitMap() {
    m_zoom = WorldView::fit(size());
    m_pan = WorldView::bounds().center()-QPointF(width(),height())/(2*m_zoom);
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
void SimulationView::setRouteSelectionMode(bool a) { m_routeSelectMode = a; update(); }
void SimulationView::setHeatmapEnabled(bool on)    { m_heatmapEnabled = on; update(); }
void SimulationView::setStatsText(const QString& t){
    if (m_statsText == t) return;
    m_statsText = t;
    if (!m_presentTimer.isActive()) update();
}

// ═══════════════════════════════════════════════════════════════════
//  Przesunięcie i powiększenie
// ═══════════════════════════════════════════════════════════════════

/// Ogranicza przesunięcie tak, aby mapa nie wyjechała poza widok.
/// Bounding box widoku to PEŁNA kanwa (0..kCanvasW, 0..kCanvasH) — drogi
/// rozciągają się przez całą kanwę i są wjazdami do/wyjazdami z mapy.
/// Checkpointy znajdują się na krawędziach kanwy, więc widok musi je obejmować.
void SimulationView::clampPan() {
    const auto b=WorldView::bounds();
    double vw=width()/m_zoom, vh=height()/m_zoom;
    m_pan.setX(vw>=b.width() ? b.center().x()-vw/2 : std::clamp(m_pan.x(),b.left(),b.right()-vw));
    m_pan.setY(vh>=b.height() ? b.center().y()-vh/2 : std::clamp(m_pan.y(),b.top(),b.bottom()-vh));
}
void SimulationView::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    double previousFit=WorldView::fit(e->oldSize());
    double ratio=e->oldSize().isValid() ? std::max(1.0,m_zoom/previousFit) : 1.0;
    QPointF center=m_pan+QPointF(e->oldSize().width(),e->oldSize().height())/(2*m_zoom);
    m_zoom=WorldView::fit(size())*ratio;
    m_pan=center-QPointF(width(),height())/(2*m_zoom);
    clampPan();
}

void SimulationView::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        bool ctrl = e->modifiers() & Qt::ControlModifier;

        // (1) Tryb wyboru trasy — najwyższy priorytet: LPM bez Ctrl = klik
        //     checkpointu. HUD instruktażowy NIE może blokować klików w
        //     checkpointy pod nim — dlatego HUD-drag jest tu wyłączony.
        if (m_routeSelectMode && !ctrl) {
            QPointF world = worldTransform().inverted().map(e->position());
            int cp = hitCheckpoint(world);
            if (cp >= 0) {
                emit checkpointClicked(cp);
                if (m_debugCheckpoints)
                    qDebug() << "[CP] HIT id=" << cp
                             << "screen=" << e->position()
                             << "world=" << world;
            } else {
                // Klik chybiony — zaloguj najbliższy checkpoint i odległość.
                // Pomaga zdiagnozować przyczynę: za mały hitbox? Zły zoom?
                // Złe pozycjonowanie? Zasłonięcie przez overlay?
                int nearestId = -1;
                double nearestDist2 = 1e18;
                for (int i = 0; i < (int)m_checkpoints.size(); i++) {
                    QPointF d = world - m_checkpoints[i].worldPos;
                    double d2 = d.x()*d.x() + d.y()*d.y();
                    if (d2 < nearestDist2) { nearestDist2 = d2; nearestId = i; }
                }
                qDebug() << "[CP] MISS screen=" << e->position()
                         << "world=" << world
                         << "zoom=" << m_zoom << "pan=" << m_pan
                         << "→ najbliższy cp.id=" << nearestId
                         << "dist=" << std::sqrt(nearestDist2) << "px (hit radius 24)";
            }
            e->accept(); return;
        }

        // (2) Drag HUD-a (panel „Nawigacja GPS") — działa tylko POZA trybem
        //     wyboru trasy, więc nie koliduje z klikiem w checkpoint.
        if (!m_statsText.isEmpty() && m_hudScreenRect.contains(e->position())) {
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

void SimulationView::wheelEvent(QWheelEvent* e) {
    // Zoom względem pozycji kursora.
    // Minimalny zoom = taki, by mapa wypełniała viewport (cała kanwa widoczna).
    double oldZoom = m_zoom;
    double factor = (e->angleDelta().y() > 0) ? 1.15 : (1.0/1.15);
    double minZoom = WorldView::fit(size());
    m_zoom = std::clamp(m_zoom * factor, minZoom, 4.0);
    QPointF cursor = e->position();
    QPointF worldBefore = cursor / oldZoom + m_pan;
    QPointF worldAfter  = cursor / m_zoom + m_pan;
    m_pan += (worldBefore - worldAfter);
    clampPan(); update();
    e->accept();
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

/// Zwraca indeks checkpointu pod podanym punktem świata lub -1.
/// Promień łapania = 24 px (większy niż wcześniej 18) — checkpointy są
/// rysowane jako prostokąty 32×32, więc 24 px daje wygodny margines kliku.
int SimulationView::hitCheckpoint(QPointF worldPos) const {
    for (int i = 0; i < (int)m_checkpoints.size(); i++) {
        QPointF d = worldPos - m_checkpoints[i].worldPos;
        if (d.x()*d.x() + d.y()*d.y() < 24*24) return i;
    }
    return -1;
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
    m_overlayRenderer.drawCheckpoints(p, m_checkpoints, m_routeSelectMode);
    m_mapRenderer.drawSignals(p, m_engine, m_theme); // GPS must not obscure signal states.
    m_carRenderer.drawCars(p, m_engine, m_theme,alpha);

    // ── Diagnostyka checkpointów (F3) ─────────────────────────────────
    // W trybie debug rysujemy obwódkę hitboxa i ID przy każdym checkpoincie,
    // żeby było widać, gdzie dokładnie sięga klikalna strefa.
    if (m_debugCheckpoints && !m_checkpoints.empty()) {
        p.save();
        p.setPen(QPen(QColor(0, 255, 200, 220), 2.0 / m_zoom));
        p.setBrush(QColor(0, 255, 200, 30));
        QFont f("Segoe UI", 9, QFont::Bold);
        f.setPointSizeF(11.0 / m_zoom);
        p.setFont(f);
        for (auto& cp : m_checkpoints) {
            p.drawEllipse(cp.worldPos, 24, 24);          // promień hitboxa
            p.setPen(QPen(QColor(0, 0, 0, 200), 3.0 / m_zoom));
            p.drawText(QRectF(cp.worldPos.x()+18, cp.worldPos.y()-18, 60, 20),
                       Qt::AlignLeft | Qt::AlignVCenter, QString::number(cp.id));
            p.setPen(QPen(QColor(0, 255, 200, 255), 1.0 / m_zoom));
            p.drawText(QRectF(cp.worldPos.x()+18, cp.worldPos.y()-18, 60, 20),
                       Qt::AlignLeft | Qt::AlignVCenter, QString::number(cp.id));
            p.setPen(QPen(QColor(0, 255, 200, 220), 2.0 / m_zoom));
        }
        p.restore();
    }

    p.restore();

    // HUD statystyk w przestrzeni ekranu — pozycja = (18, 18) + offset od użytkownika
    QPointF hudTopLeft(18 + m_hudOffset.x(), 18 + m_hudOffset.y());
    QRectF  hudRect;
    m_overlayRenderer.drawStatsHUD(p, m_statsText, rect(), hudTopLeft, &hudRect);
    m_hudScreenRect = hudRect;   // cache do hit-testu w mousePressEvent
}
