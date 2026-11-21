#pragma once
/**
 * @file MapTests.h
 * @brief Testy MAPY — liczba rond w zakresie, minimalny odstęp Manhattan, spójność 64 stref.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w MapTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> mapGenTests();
