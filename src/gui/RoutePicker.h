#pragma once
/**
 * @file RoutePicker.h
 * @brief Znaczniki wyboru punktów START / CEL trasy (warstwa wizualna + hit-test).
 *
 * Znaczniki są rysowane w PRZESTRZENI EKRANU (stały rozmiar w pikselach
 * logicznych, niezależny od zoomu i DPI), zakotwiczone w rzeczywistym końcu
 * pasa na krawędzi mapy: w etapie START — pas wjazdowy, w etapie CEL — pas
 * wyjazdowy tej samej drogi. Pozycja wizualna jest wybierana tak, by znacznik:
 *   • w całości mieścił się w widoku,
 *   • nie nachodził na żaden sygnalizator (z odstępem),
 *   • nie nachodził na inne znaczniki, kartę instrukcji ani zakładki krawędzi.
 * Jeśli znacznik musi odsunąć się od kotwicy, rysowana jest cienka linia
 * łącząca go z końcem pasa. Fizyczne punkty świata nie są przesuwane.
 *
 * Hitbox = kwadrat kHit × kHit wokół środka znacznika — ta sama geometria,
 * która jest rysowana; obliczana tą samą funkcją layout() co rysunek.
 * Punkty, których kotwica jest poza widokiem, NIE są klikalne — zamiast nich
 * przy krawędzi widoku pojawia się klikalna zakładka, która przesuwa kamerę.
 */
#include <QPainter>
#include <QRectF>
#include <QTransform>
#include <vector>
#include "../nav/NavTypes.h"

class RoutePicker {
public:
    enum class Stage { None, Start, End };
    /// Strona widoku dla zakładek z punktami poza ekranem.
    enum Side { Left = 0, Right = 1, Top = 2, Bottom = 3 };

    static constexpr double kRadius = 13.0;   ///< Promień znacznika START (px logiczne).
    static constexpr double kSquare = 24.0;   ///< Bok znacznika CEL (px logiczne).
    static constexpr double kHit    = 40.0;   ///< Bok kwadratowego hitboxa.
    static constexpr double kGap    = 4.0;    ///< Minimalny odstęp od sygnalizatora.

    struct Marker {
        int     cpId = -1;
        QPointF anchor;          ///< Koniec pasa na krawędzi mapy (ekran).
        QPointF center;          ///< Środek narysowanego znacznika (ekran).
        QRectF  hit;             ///< Hitbox (ekran) — pusty, gdy niewidoczny.
        bool    visible = false; ///< Kotwica w widoku i znaleziono wolne miejsce.
        bool    selectable = false;
        bool    isStart = false; ///< Wybrany START (etap CEL).
        bool    leader = false;  ///< Znacznik odsunięty — rysuj linię do kotwicy.
        bool    conflict = false;///< Nie znaleziono pozycji bez kolizji (diagnostyka).
        int     offSide = -1;    ///< Strona widoku, za którą leży kotwica.
        Direction dir = Direction::L2R; ///< Kierunek strzałki (wjazd/wyjazd).
        QRectF  label;           ///< Etykieta START (ekran), pusta dla pozostałych.
    };
    struct Tab { int side = -1; int count = 0; QRectF rect; };

    struct Input {
        QSizeF viewport;
        QTransform world;                     ///< Świat → ekran.
        const std::vector<Checkpoint>* cps = nullptr;
        Stage stage = Stage::None;
        std::vector<bool> selectable;         ///< Indeks = cpId.
        int startId = -1;
        std::vector<QRectF> signalWorldRects; ///< Obudowy sygnalizatorów (świat).
        QRectF instructionRect;               ///< Karta instrukcji (ekran).
        quint64 roundabouts = 0;              ///< Bit (vi*kNumRoads+hi) = rondo.
    };
    struct Layout {
        std::vector<Marker> markers;
        std::vector<Tab> tabs;
        std::vector<QRectF> signalScreenRects; ///< Sygnalizatory w widoku (diagnostyka/testy).
    };

    static Layout layout(const Input& in);
    static void draw(QPainter& p, const Layout& l, Stage stage, int hoverCp, int hoverTab);
    /// Karta instrukcji (ekran) dla danego rozmiaru widoku.
    static QRectF instructionRect(QSizeF viewport);
    static void drawInstruction(QPainter& p, const QRectF& r, Stage stage);
    /// Hit-test: zwraca cpId klikalnego znacznika lub −1.
    static int hitMarker(const Layout& l, QPointF pos);
    /// Hit-test zakładek: zwraca stronę lub −1.
    static int hitTab(const Layout& l, QPointF pos);
};
