#pragma once

/**
 * @file PhysicsConstants.h
 * @brief Stałe fizyczne ruchu, wspólne dla Car, SimulationEngine i zasad ruchu.
 *
 * Wartości dobrano empirycznie tak, aby ruch wyglądał naturalnie przy kroku
 * czasowym ~16 ms (60 FPS) i skali planszy zdefiniowanej w RoadGeometry.h.
 * Wszystkie prędkości są w px/s, a przyspieszenia w px/s².
 */

// ── Prędkości ────────────────────────────────────────────────────────
inline constexpr double kMaxSpeed   = 110.0;  ///< Domyślna prędkość przelotowa.
inline constexpr double kSlowSpeed  =  22.0;  ///< Prędkość ostrożna (dojazd do stopu).
inline constexpr double kBoostSpeed = 140.0;  ///< Prędkość przy zielonym na skrzyżowaniu.
inline constexpr double kZoneMaxSpd =  75.0;  ///< Limit prędkości w strefie skrzyżowania.

// ── Przyspieszenia / hamowanie ───────────────────────────────────────
inline constexpr double kAccel     =  90.0;   ///< Przyspieszanie.
inline constexpr double kBrake     = 160.0;   ///< Hamowanie podstawowe.
inline constexpr double kFarBrake  =  90.0;   ///< Hamowanie z daleka (zwalnianie).
inline constexpr double kStopBrake =  10.0;   ///< Hamowanie tuż przed linią stopu.
/// Maksymalne (awaryjne) opóźnienie, gdy limit bezpieczeństwa wymaga ostrzejszego
/// hamowania niż kBrake (np. auto nagle pojawia się na torze). Planowanie odstępu
/// zakłada łagodniejsze kPlanBrake, więc w normalnym ruchu ten limit nie jest osiągany.
inline constexpr double kMaxBrake  = 320.0;
/// Opóźnienie przyjmowane przy planowaniu bezpiecznej prędkości (Gipps).
inline constexpr double kPlanBrake = 120.0;
/// Minimalny odstęp między obrysami aut w postoju (zderzak–zderzak).
inline constexpr double kStandGap  =   6.0;
/// Czas reakcji modelu przy planowaniu bezpiecznej prędkości [s].
inline constexpr double kReactTime =   0.05;

// ── Odległości bezpieczeństwa ────────────────────────────────────────
inline constexpr double kClearance =  12.0;   ///< Margines dystansu w strefie.
inline constexpr double kSafeGap   =  45.0;   ///< Bezpieczny odstęp między autami w kolejce.
inline constexpr double kTurnGap   =  30.0;   ///< Wymagany wolny dystans, by wykonać skręt.

// ── Skręty po łuku ───────────────────────────────────────────────────
inline constexpr double kTurnLatAccel  = 140.0; ///< Dopuszczalne przyspieszenie boczne na łuku [px/s²].
inline constexpr double kMinTurnRadius =   8.0; ///< Poniżej — skręt geometrycznie niemożliwy (auto jedzie prosto).

// ── Ronda: dystanse ustępowania pierwszeństwa ────────────────────────
inline constexpr double kRbFarBrake  = 200.0; ///< Zwalnianie przed rondem.
inline constexpr double kRbStopBrake = 160.0; ///< Zatrzymanie przed wjazdem na rondo.

/// Minimalny odstęp kątowy (radiany) między autami na rondzie przy wjeździe.
/// Auto o długości kCarL≈40px na promieniu kRingR=50 zajmuje ≈0,8 rad, więc
/// 0,9 dawało praktycznie zero zapasu → nakładki ring-ring. 1,45 rad (≈83°)
/// gwarantuje pełny odstęp bezpieczeństwa z każdej strony punktu wjazdu.
inline constexpr double kRbEntryGap = 1.45;

// ── Kierunkowskazy / decyzje o skręcie ───────────────────────────────
inline constexpr double kSignalDist = 130.0;  ///< Dystans, na którym auto włącza kierunkowskaz.
inline constexpr double kBlinkerHalf =  0.3;  ///< Połowa okresu migania kierunkowskazu (s).
