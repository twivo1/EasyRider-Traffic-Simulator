#pragma once
/**
 * @file VisualRouteTypes.h
 * @brief Typy danych wizualizacji trasy — używane PRZEZ OBA binaria
 *        (EasyRider — główna aplikacja, EasyRiderTests — runner testowy).
 *
 *  Te struktury reprezentują GOTOWE-DO-RYSOWANIA dane trasy. Nie zawierają
 *  żadnej logiki QPainter — to czyste POD-y. OverlayRenderer (główna apka)
 *  oraz testy w TestSuite obie konsumują dokładnie te same dane.
 */
#include <QPolygonF>
#include <QPointF>
#include <QString>
#include <vector>

/**
 * @brief Segment trasy odrzucony przez sanitizer wraz z przyczyną.
 *
 *  Generowane przez RoutePolylineSanitizer, gdy zbyt długi/niepoprawny
 *  segment zostaje wycięty z polilinii. Pozwala testom potwierdzić, że
 *  konkretny artefakt został rzeczywiście wykryty.
 */
struct RejectedSegment {
    QPointF from;             ///< Punkt początkowy odrzuconego segmentu
    QPointF to;               ///< Punkt końcowy odrzuconego segmentu
    QString reasonPL;         ///< Polski opis ('Segment dłuższy niż 600 px')
    double  length = 0.0;     ///< Rzeczywista długość zmierzona (px)
    double  maxAllowed = 0.0; ///< Próg po którym segment uznawany za artefakt
};

/**
 * @brief Pełny zestaw danych wizualnych trasy — to co główna apka rysuje
 *        + cała diagnostyka pozwalająca testom zweryfikować poprawność.
 *
 *  ZASADA SEPARACJI: każde z poniższych pól reprezentuje INNE pojęcie.
 *  Nigdy nie należy ich łączyć w jedną tablicę punktów — m.in. „driven
 *  trace" (przebyta ścieżka) NIE łączy się z „current route" (planowaną
 *  bieżącą trasą).
 */
struct VisualRouteData {
    /// AKTYWNA planowana trasa — POMARAŃCZOWA linia w głównej apce.
    /// Zawsze złożona z 0..N fragmentów. Po sanityzacji każdy fragment
    /// jest gwarantowanie ciągły (brak segmentów dłuższych niż próg).
    /// Pusta lista = brak trasy do narysowania.
    std::vector<QPolygonF> currentRouteFragments;

    /// HISTORYCZNA ścieżka — gdzie auto faktycznie pojechało.
    /// Renderowana NIEZALEŻNIE; nigdy nie połączona z currentRouteFragments.
    QPolygonF drivenTrace;

    /// STARE/HISTORYCZNE trasy (np. po przeliczeniach). Każda osobny obiekt.
    /// Renderowane tylko w trybie debug.
    std::vector<QPolygonF> historyRoutes;

    /// PODGLĄD pendingowej re-route'a — gdy A* zbudował alternatywę, ale
    /// RouteManager jeszcze jej nie zaakceptował (próg gain).
    QPolygonF reroutePreview;

    /// Segmenty odrzucone w trakcie sanityzacji — do diagnostyki/testów.
    std::vector<RejectedSegment> rejected;

    // ── Diagnostyka liczbowa (do testów) ───────────────────────────
    int rawPointCount        = 0;  ///< Surowych punktów PRZED sanityzacją
    int finalPointCount      = 0;  ///< Punktów w currentRouteFragments
    int duplicatesRemoved    = 0;  ///< Liczba zignorowanych duplikatów
    int invalidPointsRemoved = 0;  ///< Liczba odrzuconych NaN/inf
    int discontinuitiesFound = 0;  ///< Liczba przerw w polilinii

    /// Zwraca true gdy zbiór danych zawiera jakiekolwiek dane do narysowania.
    bool hasAnythingToDraw() const {
        if (!currentRouteFragments.empty()) {
            for (const auto& f : currentRouteFragments)
                if (f.size() >= 2) return true;
        }
        if (drivenTrace.size() >= 2) return true;
        if (reroutePreview.size() >= 2) return true;
        for (const auto& h : historyRoutes)
            if (h.size() >= 2) return true;
        return false;
    }
};

/// Próg maksymalnej rozsądnej długości pojedynczego segmentu polilinii.
/// Większy = artefakt (np. linia łącząca starą trasę z nową po recalc,
/// lub diagonalna „tail" przy wyjeździe z ronda).
constexpr double kMaxSaneRouteSegment = 600.0;

/// Próg duplikatu — punkty bliżej siebie niż to są kompresowane.
constexpr double kRoutePointDuplicateThreshold = 0.5;
