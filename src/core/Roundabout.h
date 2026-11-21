#pragma once
#include "RoadGeometry.h"
#include <cmath>

/**
 * @file Roundabout.h
 * @brief Geometria ronda: kąty wjazdu/zjazdu oraz dystanse dojazdu.
 *
 * Auto na rondzie jedzie po okręgu o promieniu kRingR ze stałą prędkością
 * @ref kRingSpeed. Pozycję na rondzie opisuje kąt @c theta (radiany), a postęp
 * jazdy — przebyty łuk. Funkcje w przestrzeni nazw @ref Rb wyliczają kąty
 * wjazdu i zjazdu dla każdego kierunku oraz potrzebny łuk między nimi.
 */
inline constexpr double kRingSpeed = 55.0;  ///< Stała prędkość jazdy po rondzie (px/s).
/// Łuk okręgu (rad) między przecięciem osi pasa z okręgiem a punktem styczności
/// krzywej wjazdu/zjazdu. Ograniczony przez odstęp wjazdu i pierwszego zjazdu.
inline constexpr double kRbMergeDelta = 0.35;
/// Promień linii ustąpienia pierwszeństwa. Prostokąt auta jadącego po okręgu
/// zakreśla narożnikami promień √((R+W/2)² + (L/2+s₀)²) ≈ 65,4 px (z odstępem
/// postoju s₀ uwzględnianym przy wykrywaniu przeszkód). Czekające auto musi
/// stać POZA tym obszarem, inaczej auta na rondzie (poprawnie) widzą je na
/// swoim torze i powstaje wzajemne czekanie (zakleszczenie).
inline const double kRbYieldR = std::sqrt((kRingR + kCarW / 2.0) * (kRingR + kCarW / 2.0)
                                          + (kCarL / 2.0 + 6.0) * (kCarL / 2.0 + 6.0)) + 2.0;
/// Odległość (wzdłuż pasa) środka auta od środka ronda, gdy przód stoi na linii
/// ustąpienia — punkt startu wjazdu i (symetrycznie) koniec zjazdu.
inline double kRbLaneU(double laneOffset) {
    return std::sqrt(kRbYieldR * kRbYieldR - laneOffset * laneOffset) + kCarL / 2.0;
}

/// Funkcje pomocnicze geometrii ronda (kąty wjazdu/zjazdu, dystanse dojazdu).
namespace Rb {
    static const double southOff = kLaneW + kLaneOff + kCarW/2.0 - kRoadW/2.0;
    static const double northMag = kRoadW/2.0 - kLaneOff - kCarW/2.0;

    static const double entL2R = M_PI - std::asin(southOff / kRingR);
    static const double entR2L = -std::asin(northMag / kRingR);
    static const double entT2B = -(M_PI - std::acos(northMag / kRingR));
    static const double entB2T = std::acos(southOff / kRingR);

    static const double extL2R = std::asin(southOff / kRingR);
    static const double extR2L = M_PI + std::asin(northMag / kRingR);
    static const double extT2B = M_PI - std::acos(northMag / kRingR);
    static const double extB2T = -std::acos(southOff / kRingR);

    static const double exL2R = kRingR * std::cos(entL2R);
    static const double eyL2R = kRingR * std::sin(entL2R);
    static const double exR2L = kRingR * std::cos(entR2L);
    static const double eyR2L = kRingR * std::sin(entR2L);
    static const double exT2B = kRingR * std::cos(entT2B);
    static const double eyT2B = kRingR * std::sin(entT2B);
    static const double exB2T = kRingR * std::cos(entB2T);
    static const double eyB2T = kRingR * std::sin(entB2T);

    inline double mod2pi(double t) {
        t = std::fmod(t, 2.0*M_PI);
        if (t <= 0) t += 2.0*M_PI;
        return t;
    }
    inline double travelNeeded(double entry, double exit) {
        return mod2pi(entry - exit);
    }
    inline double entryAngle(Direction d) {
        switch(d){case Direction::L2R:return entL2R;case Direction::R2L:return entR2L;
                  case Direction::T2B:return entT2B;default:return entB2T;}
    }
    inline double exitAngle(Direction d) {
        switch(d){case Direction::L2R:return extL2R;case Direction::R2L:return extR2L;
                  case Direction::T2B:return extT2B;default:return extB2T;}
    }
    inline double approachDist(const QPointF& ctr, Direction dir, double cx, double cy) {
        double ex,ey;
        switch(dir){case Direction::L2R:ex=cx+exL2R;ey=cy+eyL2R;break;
                    case Direction::R2L:ex=cx+exR2L;ey=cy+eyR2L;break;
                    case Direction::T2B:ex=cx+exT2B;ey=cy+eyT2B;break;
                    default:           ex=cx+exB2T;ey=cy+eyB2T;break;}
        switch(dir){case Direction::L2R:return ex-ctr.x();
                    case Direction::R2L:return ctr.x()-ex;
                    case Direction::T2B:return ey-ctr.y();
                    default:           return ctr.y()-ey;}
    }
}

// ── Stan jazdy po rondzie (per auto) ────────────────────────────────
/**
 * @brief Stan auta poruszającego się po rondzie.
 *
 * Trzymany w aucie tylko gdy jest ono na rondzie (wskaźnik @c m_rb w Car).
 */
struct RbState {
    double cx, cy;          ///< Środek ronda w przestrzeni świata.
    int    vi, hi;          ///< Indeksy skrzyżowania, na którym jest rondo.
    double theta;           ///< Aktualny kąt położenia auta na okręgu (radiany).
    double travelNeeded;    ///< Łuk (radiany), który trzeba pokonać do zjazdu.
    double traveled;        ///< Łuk już pokonany (radiany).
    Direction exitDir;      ///< Kierunek jazdy po zjeździe z ronda.
    int    exitRoad;        ///< Numer drogi, na którą auto zjeżdża.
};
