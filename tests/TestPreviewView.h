/**
 * @file TestPreviewView.h
 * @brief Podgląd na żywo w aplikacji testowej (karty diagnostyki + promowany canvas symulacji).
 * @ingroup tests_grp
 */
#pragma once
#include <QWidget>
#include "gui/CarRenderer.h"
#include <QPointF>
#include <QPolygonF>
#include <QLineF>
#include <QRectF>
#include <QString>
#include <QColor>
#include <QLabel>
#include <vector>

class SimulationEngine;
class QFrame;

// ──────────────────────────────────────────────────────────────────────
//   Prymitywy nakładkowe — każdy test wypełnia własny zestaw, dzięki
//   czemu „Podgląd na żywo" odzwierciedla DOKŁADNIE to co weryfikuje.
// ──────────────────────────────────────────────────────────────────────
struct PvLabel   { QPointF pos; QString text; QColor color  = QColor("#1c1917"); int size = 9; };
struct PvArrow   { QPointF from, to; QString label; QColor color = QColor("#0891b2"); };
struct PvBox     { QRectF  rect;  QString label; QColor color = QColor("#22c55e"); bool pass = true; };
struct PvMeasure { QPointF p1, p2; QString label; QColor color = QColor("#0891b2"); bool pass = true; };
struct PvCounter { QString key;   QString value; QColor color = QColor("#0891b2"); };
struct PvMilestone { QString labelPL; bool reached = false; };

struct PvAttemptInfo {
    int currentAttempt = 0;
    int requiredAttempts = 0;
    int passed = 0;
    int failed = 0;
    int unverified = 0;
    QString statusPL;
    QColor  statusColor = QColor("#0891b2");
    bool    visible = false;
};

/**
 * @brief Sam canvas symulacji — bez nakładek diagnostycznych.
 *        Panele TRYB DIAGNOSTYCZNY / UKŁAD żyją POZA tym widgetem
 *        (lewa kolumna w TestPreviewView), więc canvas pozostaje pełny.
 */
class TestSimulationCanvas : public QWidget {
    Q_OBJECT
public:
    explicit TestSimulationCanvas(QWidget* parent = nullptr);

    void setEngine(SimulationEngine* eng) { m_engine = eng; update(); }
    void setCaption(const QString& s)     { m_caption = s; update(); }

    void setRoute   (const QPolygonF& p) { m_route    = p; update(); }
    void setRouteAlt(const QPolygonF& p) { m_routeAlt = p; update(); }
    void clearRoute()    { m_route.clear();    update(); }
    void clearRouteAlt() { m_routeAlt.clear(); update(); }

    // ── Wieloodcinkowe trasy (każdy fragment to OSOBNA polilinia) ─────
    // Sanitizer rozdziela trasę na niezależne kawałki, gdy wykryje
    // nieciągłość (np. zbyt długi segment). Preview rysuje je BEZ łączenia,
    // co dokładnie odpowiada temu, co weryfikują testy WIZUALIZACJA TRASY.
    void setRouteFragments(const std::vector<QPolygonF>& f)    { m_routeFragments    = f; update(); }
    void setRouteAltFragments(const std::vector<QPolygonF>& f) { m_routeAltFragments = f; update(); }
    void clearRouteFragments()    { m_routeFragments.clear();    update(); }
    void clearRouteAltFragments() { m_routeAltFragments.clear(); update(); }

    // ── Odrzucone segmenty (rysowane na czerwono, przerywaną linią) ───
    void setRejectedSegments(const std::vector<QLineF>& s) { m_rejectedSegs = s; update(); }
    void clearRejectedSegments()                           { m_rejectedSegs.clear(); update(); }

    void setHighlight(int vi, int hi) { m_hVi = vi; m_hHi = hi; update(); }
    void setHighlights(const std::vector<std::pair<int,int>>& list) { m_highlights = list; update(); }
    void clearHighlights() { m_highlights.clear(); update(); }

    void setMarker(const QPointF& p, bool show = true) { m_marker = p; m_hasMarker = show; update(); }
    void setTrail(const QPolygonF& t) { m_trail = t; update(); }
    void clearTrail() { m_trail.clear(); update(); }

    void setLabels  (const std::vector<PvLabel>&   v) { m_labels   = v; update(); }
    void setArrows  (const std::vector<PvArrow>&   v) { m_arrows   = v; update(); }
    void setBoxes   (const std::vector<PvBox>&     v) { m_boxes    = v; update(); }
    void setMeasures(const std::vector<PvMeasure>& v) { m_measures = v; update(); }

    void setVerdict(bool pass, const QString& msg) {
        m_hasVerdict = true; m_verdict = pass; m_verdictMsg = msg; update();
    }
    void clearVerdict() { m_hasVerdict = false; update(); }

    void setFocusRect(const QRectF& r) { m_focusRect = r; m_hasFocus = true; update(); }
    void clearFocusRect() { m_hasFocus = false; update(); }

    void reset() {
        m_labels.clear(); m_arrows.clear(); m_boxes.clear(); m_measures.clear();
        m_caption.clear(); m_route.clear(); m_routeAlt.clear(); m_trail.clear();
        m_routeFragments.clear(); m_routeAltFragments.clear();
        m_rejectedSegs.clear();
        m_highlights.clear(); m_hVi = m_hHi = -1;
        m_hasMarker = false; m_hasFocus = false; m_hasVerdict = false;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    CarRenderer m_cars;
    SimulationEngine* m_engine = nullptr;
    QString  m_caption;
    QPolygonF m_route, m_routeAlt, m_trail;
    std::vector<QPolygonF> m_routeFragments;     ///< aktywne fragmenty trasy (osobne polilinie)
    std::vector<QPolygonF> m_routeAltFragments;  ///< stara/historia (osobne polilinie)
    std::vector<QLineF>    m_rejectedSegs;        ///< odrzucone segmenty (czerwone, dashed)
    int      m_hVi = -1, m_hHi = -1;
    std::vector<std::pair<int,int>> m_highlights;
    QPointF  m_marker; bool m_hasMarker = false;

    std::vector<PvLabel>   m_labels;
    std::vector<PvArrow>   m_arrows;
    std::vector<PvBox>     m_boxes;
    std::vector<PvMeasure> m_measures;

    QRectF   m_focusRect; bool m_hasFocus = false;
    bool     m_hasVerdict = false;
    bool     m_verdict = true;
    QString  m_verdictMsg;
};

/**
 * @brief „Podgląd na żywo" — kompozyt z LEWĄ KOLUMNĄ paneli informacyjnych
 *        i prawą kolumną w pełni widocznego canvasu symulacji.
 *
 *  Stara konstrukcja rysowała panele TRYB DIAGNOSTYCZNY i licznikowe
 *  W POPRZEK obszaru symulacji, zasłaniając drogi/ronda/auta. Po refaktorze:
 *    • lewa kolumna — wertykalnie złożone karty TRYB DIAGNOSTYCZNY i UKŁAD,
 *    • prawa kolumna — symulacja zajmująca cały dostępny obszar.
 *  Publiczne API setterów pozostało bez zmian — wewnętrznie kierujemy dane
 *  do właściwego dziecka.
 *
 *  @note Widżet PROMOWANY w Qt Designer: w @c ui/test_window.ui występuje jako
 *        placeholder QWidget podniesiony do klasy TestPreviewView (nagłówek
 *        @c TestPreviewView.h). Logika rysowania pozostaje w kodzie.
 */
namespace Ui { class TestPreviewView; }  ///< Generowane z ui/widgets/test_preview_view.ui

class TestPreviewView : public QWidget {
    Q_OBJECT
public:
    explicit TestPreviewView(QWidget* parent = nullptr);
    ~TestPreviewView() override;

    // ── Forward na canvas ─────────────────────────────────────────
    void setEngine(SimulationEngine* eng)            { m_canvas->setEngine(eng); }
    void setCaption(const QString& s)                { m_canvas->setCaption(s); }
    void setRoute(const QPolygonF& p)                { m_canvas->setRoute(p); }
    void setRouteAlt(const QPolygonF& p)             { m_canvas->setRouteAlt(p); }
    void clearRoute()                                { m_canvas->clearRoute(); }
    void clearRouteAlt()                             { m_canvas->clearRouteAlt(); }
    void setRouteFragments(const std::vector<QPolygonF>& f)    { m_canvas->setRouteFragments(f); }
    void setRouteAltFragments(const std::vector<QPolygonF>& f) { m_canvas->setRouteAltFragments(f); }
    void clearRouteFragments()                       { m_canvas->clearRouteFragments(); }
    void clearRouteAltFragments()                    { m_canvas->clearRouteAltFragments(); }
    void setRejectedSegments(const std::vector<QLineF>& s) { m_canvas->setRejectedSegments(s); }
    void clearRejectedSegments()                     { m_canvas->clearRejectedSegments(); }
    void setHighlight(int vi, int hi)                { m_canvas->setHighlight(vi, hi); }
    void setHighlights(const std::vector<std::pair<int,int>>& l) { m_canvas->setHighlights(l); }
    void clearHighlights()                           { m_canvas->clearHighlights(); }
    void setMarker(const QPointF& p, bool s = true)  { m_canvas->setMarker(p, s); }
    void setTrail(const QPolygonF& t)                { m_canvas->setTrail(t); }
    void clearTrail()                                { m_canvas->clearTrail(); }
    void setLabels  (const std::vector<PvLabel>&   v){ m_canvas->setLabels(v); }
    void setArrows  (const std::vector<PvArrow>&   v){ m_canvas->setArrows(v); }
    void setBoxes   (const std::vector<PvBox>&     v){ m_canvas->setBoxes(v); }
    void setMeasures(const std::vector<PvMeasure>& v){ m_canvas->setMeasures(v); }
    void setVerdict(bool pass, const QString& msg)   { m_canvas->setVerdict(pass, msg); }
    void clearVerdict()                              { m_canvas->clearVerdict(); }
    void setFocusRect(const QRectF& r)               { m_canvas->setFocusRect(r); }
    void clearFocusRect()                            { m_canvas->clearFocusRect(); }

    // ── Forward na lewą kolumnę (panele info) ──────────────────────
    void setCounters  (const std::vector<PvCounter>& v);
    void setDiagnostic(const QString& s);
    void setAttemptInfo(const PvAttemptInfo& a);
    void setMilestones (const std::vector<PvMilestone>& m);
    void clearAttemptInfo();
    void clearMilestones();

    // Reset całego widoku między testami
    void resetForNewTest();

private:
    void renderDiag();
    void renderCounters();

    Ui::TestPreviewView*  ui = nullptr;        ///< Widżety z ui/widgets/test_preview_view.ui
    TestSimulationCanvas* m_canvas = nullptr;
    QLabel*               m_diagPanel = nullptr;     ///< body karty TRYB DIAGNOSTYCZNY
    QLabel*               m_countersPanel = nullptr; ///< body karty UKŁAD

    // Stan danych do renderowania w panelach bocznych
    std::vector<PvCounter>   m_counters;
    std::vector<PvMilestone> m_milestones;
    QString                  m_diagnostic;
    PvAttemptInfo            m_attempt;
};
