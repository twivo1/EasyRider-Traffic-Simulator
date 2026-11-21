#pragma once
/**
 * @file SpawnTests.h
 * @brief Testy SPAWNU — limit pojazdów, brak nakładania się aut, poprawność pasów po spawnie.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w SpawnTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> spawnTests();
