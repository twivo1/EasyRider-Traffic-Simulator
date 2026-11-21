#pragma once
#include <QRectF>

/**
 * @file RoadGeometry.h
 * @brief Geometria siatki dróg i skrzyżowań.
 *
 * Plansza to siatka 8×8 (64 skrzyżowania). Stałe i funkcje pomocnicze
 * opisują pozycje jezdni, pasów ruchu, linii stopu oraz stref skrzyżowań.
 * Nagłówek jest niezależny od silnika, więc moduły nav/ i gui/ mogą z niego
 * korzystać bez dołączania pełnego SimulationEngine.h.
 *
 * Odstępy między drogami dobrano tak, aby jezdnie wizualnie się nie zlewały,
 * a proporcje planszy (≈1.66:1) wypełniały ekran przy powiększeniu 1.0.
 */

inline constexpr int    kNumRoads = 8;     ///< Liczba dróg w każdym kierunku.

inline constexpr double kRoadW   = 70.0;   ///< Szerokość jezdni (oba pasy).
inline constexpr double kLaneW   = 35.0;   ///< Szerokość jednego pasa.
inline constexpr double kCarL    = 40.0;   ///< Długość auta.
inline constexpr double kCarW    = 20.0;   ///< Szerokość auta.
inline constexpr double kLaneOff =  8.0;   ///< Odsunięcie auta od krawędzi pasa.
inline constexpr double kRingR   = 50.0;   ///< Promień toru jazdy na rondzie.

inline constexpr double kCanvasW = 2380.0; ///< Szerokość całej planszy (px).
inline constexpr double kCanvasH = 1430.0; ///< Wysokość całej planszy (px).

// ── Pozycje dróg ─────────────────────────────────────────────────────
/// Górne krawędzie dróg poziomych (odstęp 180 px).
inline constexpr double kHTop[kNumRoads] = {
     40, 220, 400, 580, 760, 940, 1120, 1300
};
/// Lewe krawędzie dróg pionowych (odstęp 310 px).
inline constexpr double kVLeft[kNumRoads] = {
     70, 380, 690, 1000, 1310, 1620, 1930, 2240
};

inline double kHBot  (int hi) { return kHTop [hi] + kRoadW; } ///< Dolna krawędź drogi poziomej.
inline double kVRight(int vi) { return kVLeft[vi] + kRoadW; } ///< Prawa krawędź drogi pionowej.

// ── Kierunek jazdy ───────────────────────────────────────────────────
/**
 * @brief Kierunek poruszania się obiektu.
 * L2R = w prawo (wschód), R2L = w lewo (zachód),
 * T2B = w dół (południe), B2T = w górę (północ).
 */
enum class Direction { L2R, R2L, T2B, B2T };

/// Czy kierunek jest poziomy (L2R lub R2L).
inline bool dirIsHoriz(Direction d){ return d==Direction::L2R||d==Direction::R2L; }
/// Zwraca kierunek przeciwny.
inline Direction dirOpposite(Direction d){
    switch(d){
    case Direction::L2R:return Direction::R2L; case Direction::R2L:return Direction::L2R;
    case Direction::T2B:return Direction::B2T; default:           return Direction::T2B;
    }
}

// ── Pozycje pasów (ruch prawostronny: L2R dołem, R2L górą) ───────────
inline double kL2R_Y(int hi) { return kHTop [hi] + kLaneW + kLaneOff; } ///< Y środka pasa L2R.
inline double kR2L_Y(int hi) { return kHTop [hi] + kLaneOff;          } ///< Y środka pasa R2L.
inline double kT2B_X(int vi) { return kVLeft[vi] + kLaneOff;          } ///< X środka pasa T2B.
inline double kB2T_X(int vi) { return kVLeft[vi] + kLaneW + kLaneOff; } ///< X środka pasa B2T.

/// Prostokąt strefy skrzyżowania (vi, hi).
inline QRectF kIZone(int vi, int hi) {
    return { kVLeft[vi], kHTop[hi], kRoadW, kRoadW };
}

inline double kSL_L2R(int vi) { return kVLeft[vi]          -  6.0; } ///< Linia stopu dla L2R.
inline double kSL_R2L(int vi) { return kVLeft[vi] + kRoadW +  6.0; } ///< Linia stopu dla R2L.
inline double kSL_T2B(int hi) { return kHTop [hi]          -  6.0; } ///< Linia stopu dla T2B.
inline double kSL_B2T(int hi) { return kHTop [hi] + kRoadW +  6.0; } ///< Linia stopu dla B2T.

/**
 * @brief Domyślny układ 8 rond na siatce (używany bez generatora map).
 * @return true jeśli skrzyżowanie (vi, hi) jest rondem.
 */
inline bool kIsRoundabout(int vi, int hi) {
    return (vi==1&&hi==2)||(vi==2&&hi==1)||
           (vi==5&&hi==2)||(vi==2&&hi==5)||
           (vi==4&&hi==4)||
           (vi==6&&hi==3)||(vi==3&&hi==6)||
           (vi==6&&hi==6);
}
