#pragma once
/**
 * @file NavigationTests.h
 * @brief Testy NAWIGACJI — trasy A* z aktualizacjami GPS, zakaz zawracania, symetria tras.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w NavigationTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> navigationTests();
