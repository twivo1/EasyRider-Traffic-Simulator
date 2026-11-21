#pragma once
/**
 * @file EngineTests.h
 * @brief Testy SILNIKA — stabilność, SmartCar (GPS), skręty, sygnalizacja, brak kolizji, napływ aut.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w EngineTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> engineTests();
