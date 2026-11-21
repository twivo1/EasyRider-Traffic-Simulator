#include "OverlayRenderer.h"
#include "../core/SimulationEngine.h"
#include "../core/SimulationObject.h"
#include "../core/RoadGeometry.h"
#include "../nav/VisualRouteBuilder.h"
#include "../nav/VisualRouteTypes.h"
#include <QFont>
#include <QPainterPath>
#include <cmath>
#include <algorithm>

/**
 * @brief Rysuje POMARAŃCZOWĄ linię trasy GPS.
 *
 *  Logika budowania polilinii została WYEKSTRAHOWANA do współdzielonego
 *  modułu src/nav/VisualRouteBuilder + RoutePolylineSanitizer. Ten sam
 *  kod używany jest przez EasyRiderTests do walidacji danych wizualnych
 *  PRZED renderingiem. Tutaj zostało tylko rysowanie.
 */
void OverlayRenderer::drawRoute(QPainter& p, const Route& route,
                                 const MapTheme& th, QPointF carPos) const {
    (void)th;
    VisualRouteData data = VisualRouteBuilder::build(route, carPos);
    drawVisualRouteData(p, data);
}

/**
 * @brief Rysuje gotowe dane wizualne trasy (VisualRouteData).
 *
 *  Renderer NIE decyduje co rysować — dostaje już zdiagnozowane i
 *  zsanityzowane fragmenty. Brak fragmentów = nic nie rysujemy.
 *
 *  Każdy fragment rysowany NIEZALEŻNIE (osobny stroke): polilinia
 *  z RoundJoin, dwie warstwy (cienka ciemna obwódka + bursztynowy
 *  rdzeń). Nigdy fill — to polilinia, nie kształt zamknięty.
 */
void OverlayRenderer::drawVisualRouteData(QPainter& p, const VisualRouteData& d) const {
    if (!d.hasAnythingToDraw()) return;
    p.setBrush(Qt::NoBrush);
    for (const QPolygonF& frag : d.currentRouteFragments) {
        if (frag.size() < 2) continue;
        p.setPen(QPen(QColor("#30383B"), 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(frag);
        p.setPen(QPen(QColor("#F0AE57"), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(frag);
    }
    // (drivenTrace / historyRoutes / reroutePreview — celowo nieobsługiwane
    //  w trybie podstawowym; tryb debug podłączymy gdy zajdzie potrzeba.)
}


/**
 * @brief Rysuje checkpointy z obwódką, strzałką kierunku i kolorem stanu.
 *
 * Widoczne tylko w trybie wyboru trasy. Stan (dostępny / start / cel)
 * decyduje o kolorze wypełnienia.
 */
void OverlayRenderer::drawCheckpoints(QPainter& p,
                                       const std::vector<Checkpoint>& cps,
                                       bool selectionMode) const {
    if (!selectionMode || cps.empty()) return;

    for (auto& cp : cps) {
        QColor fill, border;
        switch (cp.uiState) {
        case Checkpoint::UIState::Available:
            fill = QColor("#D96535"); border = QColor(200,120,0); break;
        case Checkpoint::UIState::IsStart:
            fill = QColor("#358255");  border = QColor(0,160,60);  break;
        case Checkpoint::UIState::IsEnd:
            fill = QColor("#B84235"); border = QColor(180,30,30); break;
        }

        // Ciemne tło pod ikoną
        p.setPen(Qt::NoPen);
        p.setBrush(QColor("#F8F7F4"));
        p.drawRoundedRect(QRectF(cp.worldPos.x()-16, cp.worldPos.y()-16, 32, 32), 2, 2);

        // Okrąg
        p.setBrush(fill);
        p.setPen(QPen(border, 2));
        p.drawEllipse(cp.worldPos, 12, 12);

        // Strzałka kierunku wjazdu
        double ax=0, ay=0;
        switch(cp.entryDir){
        case Direction::L2R:ax= 5;break;case Direction::R2L:ax=-5;break;
        case Direction::T2B:ay= 5;break;case Direction::B2T:ay=-5;break;}
        p.setPen(QPen(Qt::white, 2.5, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(cp.worldPos + QPointF(-ax*1.2,-ay*1.2),
                   cp.worldPos + QPointF(ax*1.2, ay*1.2));
        double px=-ay*0.6, py=ax*0.6;
        p.drawLine(cp.worldPos + QPointF(ax*1.2, ay*1.2),
                   cp.worldPos + QPointF(ax*0.3+px, ay*0.3+py));
        p.drawLine(cp.worldPos + QPointF(ax*1.2, ay*1.2),
                   cp.worldPos + QPointF(ax*0.3-px, ay*0.3-py));
    }
}

/**
 * @brief Rysuje nakładkę natężenia ruchu (heatmapę) na odcinkach jezdni.
 *
 * Dla każdego odcinka między skrzyżowaniami liczona jest liczba aut, a kolor
 * przechodzi od zieleni (pusto) przez żółć do czerwieni (zator).
 */
void OverlayRenderer::drawHeatmap(QPainter& p, const SimulationEngine* engine) const {
    if (!engine) return;

    int counts[4][kNumRoads][kNumRoads+1] = {};
    for (const auto& obj : engine->objects()) {
        Direction d = obj->dir();
        int r = obj->road();
        QPointF c = obj->boundingRect().center();
        int di = (int)d, seg = -1;

        if (d==Direction::L2R || d==Direction::R2L) {
            double cx = c.x();
            if (cx < kVLeft[0]) seg = 0;
            else {
                for (int vi=0;vi<kNumRoads-1;vi++)
                    if (cx>=kVLeft[vi]+kRoadW && cx<kVLeft[vi+1]) { seg=vi+1; break; }
                if (seg<0 && cx>=kVLeft[kNumRoads-1]+kRoadW) seg=kNumRoads;
            }
        } else {
            double cy = c.y();
            if (cy < kHTop[0]) seg = 0;
            else {
                for (int hi=0;hi<kNumRoads-1;hi++)
                    if (cy>=kHTop[hi]+kRoadW && cy<kHTop[hi+1]) { seg=hi+1; break; }
                if (seg<0 && cy>=kHTop[kNumRoads-1]+kRoadW) seg=kNumRoads;
            }
        }
        if (seg>=0 && r>=0 && r<kNumRoads) counts[di][r][seg]++;
    }

    p.setPen(Qt::NoPen);
    // Mapowanie liczby aut na kolor (zielony→czerwony)
    auto heatColor = [](int n) -> QColor {
        if (n<=0) return {34,197,94,50};
        if (n==1) return {74,222,128,65};
        if (n==2) return {250,204,21,75};
        if (n==3) return {251,146,60,85};
        if (n==4) return {239,68,68,95};
        return {185,28,28,110};
    };

    for (int hi=0;hi<kNumRoads;hi++)
        for (int seg=1;seg<kNumRoads;seg++) {
            double x0=kVLeft[seg-1]+kRoadW, x1=kVLeft[seg];
            if (x1<=x0) continue;
            int total=counts[0][hi][seg]+counts[1][hi][seg];
            p.setBrush(heatColor(total));
            p.drawRoundedRect(QRectF(x0,kHTop[hi]+2,x1-x0,kRoadW-4),3,3);
        }
    for (int vi=0;vi<kNumRoads;vi++)
        for (int seg=1;seg<kNumRoads;seg++) {
            double y0=kHTop[seg-1]+kRoadW, y1=kHTop[seg];
            if (y1<=y0) continue;
            int total=counts[2][vi][seg]+counts[3][vi][seg];
            p.setBrush(heatColor(total));
            p.drawRoundedRect(QRectF(kVLeft[vi]+2,y0,kRoadW-4,y1-y0),3,3);
        }
}

/**
 * @brief Rysuje elegancki panel statystyk nawigacji w lewym górnym rogu.
 *
 * Panel celowo jest po lewej stronie — menu boczne i jego uchwyt zajmują
 * prawą krawędź, więc rysowanie HUD po prawej powodowało nakładanie się
 * elementów. Szerokość panelu dopasowuje się do najdłuższego wiersza tekstu,
 * dzięki czemu komunikaty (np. „Kliknij punkt startowy…") nie są ucinane,
 * a panel nie zasłania niepotrzebnie dużej części planszy.
 *
 * Format wejścia: linie „etykieta\\twartość" rozdzielone znakiem nowej linii.
 * Pierwsza linia jest nagłówkiem. Panel ma płaskie jasne tło i cienkie obramowanie.
 */
void OverlayRenderer::drawStatsHUD(QPainter& p, const QString& text,
                                    const QRectF& viewport,
                                    QPointF topLeft, QRectF* outRect) const {

    if (text.isEmpty()) { if (outRect) *outRect = QRectF(); return; }
    p.resetTransform();

    QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty()) { if (outRect) *outRect = QRectF(); return; }

    QFont titleF("Segoe UI", 10, QFont::Bold);
    QFont rowF("Segoe UI", 10);
    QFontMetrics tfm(titleF), rfm(rowF);

    // Wymiary panelu — szerokość dopasowana do najszerszego wiersza.
    const int pad = 8, rowH = 20, titleH = 22, accent = 0;
    const int colGap = 22;   // minimalny odstęp między etykietą a wartością
    int contentW = tfm.horizontalAdvance(lines[0]);
    for (int i = 1; i < lines.size(); ++i) {
        int tab = lines[i].indexOf('\t');
        if (tab >= 0) {
            int w = rfm.horizontalAdvance(lines[i].left(tab)) + colGap
                  + rfm.horizontalAdvance(lines[i].mid(tab + 1));
            contentW = std::max(contentW, w);
        } else {
            contentW = std::max(contentW, rfm.horizontalAdvance(lines[i]));
        }
    }
    // Cap szerokości podniesiony do 480 px, żeby dłuższe komunikaty
    // („Kliknij punkt KOŃCOWY na krawędzi mapy") nie były obcinane.
    contentW = std::clamp(contentW, 180, 480);

    int bh = titleH + (lines.size() - 1) * rowH + pad * 2;
    int bw = contentW + pad * 2 + accent;
    int bx = std::clamp((int)topLeft.x(), 2, std::max(2, (int)viewport.width()-bw-2));
    int by = std::clamp((int)topLeft.y(), 2, std::max(2, (int)viewport.height()-bh-2));

    if (outRect) *outRect = QRectF(bx, by, bw, bh);

    p.setPen(QPen(QColor("#D0CEC8"), 1));
    p.setBrush(QColor("#F8F7F4"));
    p.drawRect(QRectF(bx, by, bw, bh));

    int tx = bx + accent + pad;
    int y  = by + pad;

    // Nagłówek (pierwsza linia)
    p.setFont(titleF);
    p.setPen(QColor("#242424"));
    p.drawText(QRectF(tx, y, contentW, titleH), Qt::AlignVCenter | Qt::AlignLeft, lines[0]);
    y += titleH;

    // Separator
    p.setPen(QPen(QColor("#D0CEC8"), 1));
    p.drawLine(QPointF(tx, y - 2), QPointF(tx + contentW, y - 2));

    // Wiersze „etykieta : wartość"
    p.setFont(rowF);
    for (int i = 1; i < lines.size(); ++i) {
        QString line = lines[i];
        int tab = line.indexOf('\t');
        QString label = (tab >= 0) ? line.left(tab) : line;
        QString value = (tab >= 0) ? line.mid(tab + 1) : QString();

        p.setPen(QColor("#666660"));
        p.drawText(QRectF(tx, y, contentW, rowH), Qt::AlignVCenter | Qt::AlignLeft, label);
        p.setPen(QColor("#242424"));
        p.drawText(QRectF(tx, y, contentW, rowH), Qt::AlignVCenter | Qt::AlignRight, value);
        y += rowH;
    }
}
