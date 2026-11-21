#pragma once
/**
 * @file CarTests.h
 * @brief Testy SAMOCHODU — ruch w czterech kierunkach, stop(), limit prędkości, poprawność AABB.
 *
 *  Wydzielone z dawnego monolitu tests/TestSuite.h. Definicje testów żyją
 *  w CarTests.cpp; tutaj jest tylko deklaracja fabryki kategorii.
 */
#include "../utils/TestHelpers.h"

/// Buduje i zwraca komplet testów tej kategorii (attempt-based).
std::vector<TestCase> carTests();
