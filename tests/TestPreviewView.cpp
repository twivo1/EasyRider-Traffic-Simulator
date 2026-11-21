#include "TestPreviewView.h"
#include "ui_test_preview_view.h"  // wygenerowane przez uic z ui/widgets/test_preview_view.ui
#include "../src/core/SimulationEngine.h"
#include "../src/core/Car.h"
#include "../src/core/RoadGeometry.h"
#include <QPainter>
#include <QPaintEvent>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QScrollArea>
#include <cmath>
#include "gui/MapRenderer.h"
#include "gui/WorldView.h"
#include "gui/CarRenderer.h"

// ════════════════════════════════════════════════════════════════════
//  TestSimulationCanvas — czysta scena symulacji
// ════════════════════════════════════════════════════════════════════
TestSimulationCanvas::TestSimulationCanvas(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 220);
}

void TestSimulationCanvas::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), defaultTheme().grassTop);
    p.setPen(QPen(QColor("#e7e5e4"), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 8, 8);

    if (!m_engine) {
        p.setPen(QColor("#a8a29e"));
        QFont f = font(); f.setPointSize(10); p.setFont(f);
        p.drawText(rect(), Qt::AlignCenter,
                   QString::fromUtf8("Wybierz test, by zobaczyć dedykowany podgląd"));
        return;
    }

    // Kadr kamery
    QRectF view = m_hasFocus ? m_focusRect.adjusted(-40, -40, 40, 40)
                              : WorldView::bounds();
    p.save();
    p.setTransform(WorldView::fitted(QRectF(0,0,width(),height()-46), view));

    const MapTheme theme = defaultTheme();
    MapRenderer map;
    map.drawRoads(p, theme);
    map.drawMarkings(p, theme);
    map.drawIntersections(p, m_engine, theme);

    // Trail
    if (m_trail.size() >= 2) {
        p.setPen(QPen(QColor(245,158,11,140), 3, Qt::SolidLine, Qt::RoundCap));
        p.setBrush(Qt::NoBrush);
        p.drawPolyline(m_trail);
    }

    // Trasy GPS
    auto drawPoly = [&](const QPolygonF& pts, QColor outer, QColor inner) {
        if (pts.size() < 2) return;
        p.setPen(QPen(outer, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.setBrush(Qt::NoBrush);
        p.drawPolyline(pts);
        p.setPen(QPen(inner, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(pts);
    };
    drawPoly(m_route,    QColor("#30383B"), QColor("#F0AE57"));
    drawPoly(m_routeAlt, QColor( 37, 99, 235), QColor(147, 197, 253));

    // Wieloodcinkowe trasy — każdy fragment rysowany OSOBNO,
    // bez łączenia końców (kluczowe dla testów rozdzielania stara/nowa).
    for (const auto& frag : m_routeFragments)
        drawPoly(frag, QColor("#30383B"), QColor("#F0AE57"));
    for (const auto& frag : m_routeAltFragments)
        drawPoly(frag, QColor(100, 116, 139), QColor(203, 213, 225));

    // Odrzucone segmenty (np. zbyt długie, NaN, duplikaty) — czerwony dash.
    if (!m_rejectedSegs.empty()) {
        QPen rejPen(QColor(220, 38, 38), 3, Qt::DashLine, Qt::RoundCap);
        p.setPen(rejPen);
        p.setBrush(Qt::NoBrush);
        for (const auto& s : m_rejectedSegs) p.drawLine(s);
    }

    m_cars.drawCars(p, m_engine, theme);

    // Ramki (world-space overlay)
    for (const auto& b : m_boxes) {
        QColor c = b.color; if (!b.pass) c = QColor("#dc2626");
        QColor fill = c; fill.setAlpha(40);
        p.setPen(QPen(c, 1.8, Qt::DashLine));
        p.setBrush(fill);
        p.drawRoundedRect(b.rect, 4, 4);
        if (!b.label.isEmpty()) {
            p.setPen(c);
            QFont f = font(); f.setPointSize(8); f.setBold(true); p.setFont(f);
            p.drawText(b.rect.adjusted(2, -14, 0, 0),
                       Qt::AlignLeft | Qt::AlignTop, b.label);
        }
    }

    // Pomiary
    for (const auto& m : m_measures) {
        QColor c = m.color; if (!m.pass) c = QColor("#dc2626");
        p.setPen(QPen(c, 1.8, Qt::SolidLine));
        p.setBrush(c);
        p.drawLine(m.p1, m.p2);
        QPointF d = m.p2 - m.p1;
        double len = std::hypot(d.x(), d.y());
        if (len > 1) {
            QPointF perp(-d.y()/len, d.x()/len);
            p.drawLine(m.p1 - perp*4, m.p1 + perp*4);
            p.drawLine(m.p2 - perp*4, m.p2 + perp*4);
        }
        if (!m.label.isEmpty()) {
            QPointF mid = (m.p1 + m.p2) * 0.5;
            p.setPen(c);
            QFont f = font(); f.setPointSize(8); f.setBold(true); p.setFont(f);
            p.drawText(QRectF(mid.x()-40, mid.y()-18, 80, 14),
                       Qt::AlignCenter, m.label);
        }
    }

    // Strzałki
    for (const auto& a : m_arrows) {
        p.setPen(QPen(a.color, 2));
        p.setBrush(a.color);
        p.drawLine(a.from, a.to);
        QPointF d = a.to - a.from;
        double len = std::hypot(d.x(), d.y());
        if (len > 1) {
            QPointF u(d.x()/len, d.y()/len);
            QPointF perp(-u.y(), u.x());
            QPolygonF tri;
            tri << a.to << (a.to - u*8 + perp*4) << (a.to - u*8 - perp*4);
            p.drawPolygon(tri);
        }
        if (!a.label.isEmpty()) {
            p.setPen(a.color);
            QFont f = font(); f.setPointSize(8); p.setFont(f);
            p.drawText(QRectF(a.to.x()+6, a.to.y()-12, 80, 14),
                       Qt::AlignLeft | Qt::AlignVCenter, a.label);
        }
    }

    // Etykiety w przestrzeni świata
    for (const auto& l : m_labels) {
        p.setPen(l.color);
        QFont f = font(); f.setPointSize(l.size); f.setBold(true); p.setFont(f);
        QColor bg = QColor("#ffffff"); bg.setAlpha(220);
        p.setBrush(bg);
        QRectF tight = p.fontMetrics().boundingRect(l.text).adjusted(-3, -1, 3, 1);
        tight.moveCenter(l.pos);
        p.setPen(QPen(l.color, 1));
        p.drawRoundedRect(tight, 3, 3);
        p.setPen(l.color);
        p.drawText(tight, Qt::AlignCenter, l.text);
    }

    // Podświetlenia skrzyżowań
    auto highlightZone = [&](int vi, int hi) {
        QRectF z = kIZone(vi, hi).adjusted(-6, -6, 6, 6);
        p.setPen(QPen(QColor("#f59e0b"), 2.5, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(z, 4, 4);
    };
    if (m_hVi >= 0 && m_hHi >= 0) highlightZone(m_hVi, m_hHi);
    for (auto [vi, hi] : m_highlights) highlightZone(vi, hi);

    if (m_hasMarker) {
        p.setPen(QPen(QColor("#dc2626"), 2));
        p.setBrush(QColor("#fecaca"));
        p.drawEllipse(m_marker, 8, 8);
    }
    p.restore();

    // Werdykt — pasek dolny
    if (m_hasVerdict) {
        QColor bg = m_verdict ? QColor("#dcfce7") : QColor("#fee2e2");
        QColor brd = m_verdict ? QColor("#bbf7d0") : QColor("#fecaca");
        QColor txt = m_verdict ? QColor("#166534") : QColor("#991b1b");
        QString icon = m_verdict ? QString::fromUtf8("✓") : QString::fromUtf8("✗");
        QRectF vbox(12, height() - 46, width() - 24, 30);
        p.setBrush(bg); p.setPen(QPen(brd, 1));
        p.drawRoundedRect(vbox, 6, 6);
        QFont fv = font(); fv.setPointSize(10); fv.setBold(true); p.setFont(fv);
        p.setPen(txt);
        p.drawText(vbox.adjusted(12, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft,
                   icon + "  " + m_verdictMsg);
    }

    // Caption — sam dół
    if (!m_caption.isEmpty()) {
        p.setPen(QColor("#CBD2D2"));
        QFont fc = font(); fc.setPointSize(9); p.setFont(fc);
        int y = m_hasVerdict ? height() - 14 : height() - 22;
        p.drawText(QRect(12, y - 6, width() - 24, 16),
                   Qt::AlignLeft | Qt::AlignVCenter, m_caption);
    }
}

// ════════════════════════════════════════════════════════════════════
//  TestPreviewView — fasada: lewa kolumna [diag, układ] + canvas
// ════════════════════════════════════════════════════════════════════
TestPreviewView::TestPreviewView(QWidget* parent)
    : QWidget(parent), ui(new Ui::TestPreviewView) {
    // Fasada podglądu pochodzi z Qt Designer (ui/widgets/test_preview_view.ui):
    // lewa kolumna z kartami „TRYB DIAGNOSTYCZNY" i „UKŁAD" oraz po prawej
    // PROMOWANY canvas symulacji (TestSimulationCanvas — rysowanie w kodzie).
    ui->setupUi(this);

    m_diagPanel     = ui->diagBody;
    m_countersPanel = ui->countersBody;
    m_canvas        = ui->canvas;

    renderDiag();
    renderCounters();
}

TestPreviewView::~TestPreviewView() { delete ui; }

void TestPreviewView::setCounters(const std::vector<PvCounter>& v) {
    m_counters = v;
    renderCounters();
}

void TestPreviewView::setMilestones(const std::vector<PvMilestone>& v) {
    m_milestones = v;
    renderDiag();
}

void TestPreviewView::setDiagnostic(const QString& s) {
    m_diagnostic = s;
    renderDiag();
}

void TestPreviewView::setAttemptInfo(const PvAttemptInfo& a) {
    m_attempt = a;
    renderDiag();
}

void TestPreviewView::clearAttemptInfo() {
    m_attempt = PvAttemptInfo{};
    renderDiag();
}

void TestPreviewView::clearMilestones() {
    m_milestones.clear();
    renderDiag();
}

void TestPreviewView::resetForNewTest() {
    m_canvas->reset();
    m_attempt = PvAttemptInfo{};
    m_milestones.clear();
    m_diagnostic.clear();
    m_counters.clear();
    renderDiag();
    renderCounters();
}

void TestPreviewView::renderDiag() {
    QString html;
    bool anything = m_attempt.visible || !m_milestones.empty() || !m_diagnostic.isEmpty();
    if (!anything) {
        m_diagPanel->setText(QString::fromUtf8(
            "<span style='color:#a8a29e;'>Wybierz test, by zobaczyć szczegóły wykonania.</span>"));
        return;
    }

    if (m_attempt.visible) {
        html += QString("<div style='font-weight:bold; color:#1c1917;'>"
                        "Próba %1 / %2</div>")
                    .arg(m_attempt.currentAttempt).arg(m_attempt.requiredAttempts);
        html += QString("<div style='color:#57534e; font-size:10px; margin-top:2px;'>"
                        "Udane: %1 &middot; Nieudane: %2 &middot; Niezweryf.: %3</div>")
                    .arg(m_attempt.passed).arg(m_attempt.failed).arg(m_attempt.unverified);
        html += QString("<div style='margin-top:6px; font-weight:bold; color:%1;'>"
                        "Status: %2</div>")
                    .arg(m_attempt.statusColor.name()).arg(m_attempt.statusPL.toHtmlEscaped());
    }

    if (!m_milestones.empty()) {
        html += QString::fromUtf8(
            "<div style='margin-top:10px; color:#92400e; font-size:9px; "
            "font-weight:800; letter-spacing:1.4px;'>KAMIENIE MILOWE</div>");
        for (const auto& ms : m_milestones) {
            QString mark = ms.reached ? QString::fromUtf8("[✓]") : QString::fromUtf8("[&nbsp;&nbsp;]");
            QString col  = ms.reached ? "#16a34a" : "#a8a29e";
            html += QString("<div style='margin-top:2px; color:%1; font-family:Segoe UI;'>"
                            "%2&nbsp;&nbsp;<span style='color:#1c1917;'>%3</span></div>")
                        .arg(col).arg(mark).arg(ms.labelPL.toHtmlEscaped());
        }
    }

    if (!m_diagnostic.isEmpty()) {
        html += QString::fromUtf8(
            "<div style='margin-top:10px; color:#92400e; font-size:9px; "
            "font-weight:800; letter-spacing:1.4px;'>POMIARY</div>");
        for (auto& ln : m_diagnostic.split('\n')) {
            html += QString("<div style='color:#292524; font-family:Segoe UI; "
                            "font-size:10px; margin-top:1px;'>%1</div>")
                        .arg(ln.toHtmlEscaped());
        }
    }

    m_diagPanel->setText(html);
}

void TestPreviewView::renderCounters() {
    if (m_counters.empty()) {
        m_countersPanel->setText(QString::fromUtf8(
            "<span style='color:#a8a29e;'>Brak liczników do wyświetlenia.</span>"));
        return;
    }
    QString html = "<table cellspacing='0' cellpadding='0' style='width:100%;'>";
    for (const auto& c : m_counters) {
        html += QString(
            "<tr>"
            "<td style='color:#78716c; font-size:9px; font-weight:600; "
            "    letter-spacing:0.8px; padding:6px 0 0 0; vertical-align:top;'>%1</td>"
            "</tr>"
            "<tr>"
            "<td style='color:%2; font-size:13px; font-weight:700; "
            "    font-family:Segoe UI; padding:1px 0 4px 0; "
            "    border-bottom:1px solid #f5f5f4;'>%3</td>"
            "</tr>")
            .arg(c.key.toUpper().toHtmlEscaped())
            .arg(c.color.name())
            .arg(c.value.toHtmlEscaped());
    }
    html += "</table>";
    m_countersPanel->setText(html);
}
