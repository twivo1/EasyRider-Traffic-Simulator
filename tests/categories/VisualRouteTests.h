#pragma once
/**
 * @file VisualRouteTests.h
 * @brief Testy WIZUALIZACJI TRASY — walidacja danych PRZED renderingiem (ten sam moduł co główna apka).
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w VisualRouteTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> visualRouteTests();
