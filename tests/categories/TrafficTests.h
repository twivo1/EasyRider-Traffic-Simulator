#pragma once
/**
 * @file TrafficTests.h
 * @brief Testy RUCHU DROGOWEGO — zliczanie pojazdów, wykrywanie korka, koszt drogi zatłoczonej.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w TrafficTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> trafficTests();
