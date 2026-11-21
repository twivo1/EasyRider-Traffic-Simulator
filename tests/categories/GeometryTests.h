#pragma once
/**
 * @file GeometryTests.h
 * @brief Testy GEOMETRII — inspekcyjne sprawdzenie siatki dróg, pasów, checkpointów i stref skrzyżowań.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w GeometryTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> geometryTests();
