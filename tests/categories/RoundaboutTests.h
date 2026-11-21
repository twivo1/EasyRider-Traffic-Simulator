#pragma once
/**
 * @file RoundaboutTests.h
 * @brief Testy RONDA — kąty wjazdu/wyjazdu, krążenie po ringu, brak kolizji, stop przed linią.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w RoundaboutTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> roundaboutTests();
