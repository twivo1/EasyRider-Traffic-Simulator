#pragma once
/**
 * @file FlowTests.h
 * @brief Testy PŁYNNOŚCI — brak skoków pozycji, płynne skrzyżowania/ronda, hamowanie, ciągłość ruchu.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w FlowTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> flowTests();
