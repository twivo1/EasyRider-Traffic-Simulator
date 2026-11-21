#pragma once
/**
 * @file TestSuite.h
 * @brief Cienki AGREGATOR testów attempt-based (10 kategorii + audyt mechaniki).
 *
 *  Dawniej monolit ~2950 linii. Po refaktorze treść testów mieszka w
 *  plikach katalogu tests/categories (po jednej kategorii w pliku), a wspólne helpery
 *  w tests/utils/TestHelpers.h. Ten plik jedynie:
 *    • re-eksportuje helpery (zgodność wstecz dla kodu dołączającego TestSuite.h),
 *    • dołącza nagłówki wszystkich kategorii,
 *    • udostępnia buildAllCategories() spinające je w jeden zestaw.
 *
 *  ATTEMPT-BASED: czas nie kończy testu — kończy go zebranie wymaganej
 *  liczby udanych prób (każda potwierdzona realnymi milestonami z silnika)
 *  lub wyczerpanie maxAttempts.
 *
 *  PODZIAŁ KATEGORII (51 testów + kategoria „Audyt mechaniki", patrz docs/MECHANICS_AUDIT.md):
 *    Geometria 6 · Płynność 5 · Samochód 7 · Rondo 5 · Nawigacja 4 ·
 *    Wizualizacja trasy 9 · Ruch drogowy 3 · Mapa 3 · Spawn 3 · Silnik 6.
 */
#include "utils/TestHelpers.h"

#include "categories/GeometryTests.h"
#include "categories/FlowTests.h"
#include "categories/CarTests.h"
#include "categories/RoundaboutTests.h"
#include "categories/NavigationTests.h"
#include "categories/VisualRouteTests.h"
#include "categories/TrafficTests.h"
#include "categories/MapTests.h"
#include "categories/SpawnTests.h"
#include "categories/EngineTests.h"
#include "audit/MechanicsAudit.h"

// ════════════════════════════════════════════════════════════════════
//  ZBIORCZA LISTA
// ════════════════════════════════════════════════════════════════════
inline std::vector<TestCategory> buildAllCategories() {
    std::vector<TestCategory> c;
    c.push_back({"Geometria",         "#92400e", geometryTests()});
    c.push_back({"Płynność",          "#7c3aed", flowTests()});
    c.push_back({"Samochód",          "#0891b2", carTests()});
    c.push_back({"Rondo",             "#16a34a", roundaboutTests()});
    c.push_back({"Nawigacja",         "#ea580c", navigationTests()});
    c.push_back({"Wizualizacja trasy","#d97706", visualRouteTests()});
    c.push_back({"Ruch drogowy",      "#be185d", trafficTests()});
    c.push_back({"Mapa",              "#0369a1", mapGenTests()});
    c.push_back({"Spawn",             "#65a30d", spawnTests()});
    c.push_back({"Silnik",            "#525252", engineTests()});
    // Audyt mechaniki: deterministyczne scenariusze A–E z niezależnym walidatorem
    // (obrócone obrysy, skoki, legalność odcinków, wyrocznia Dijkstry). ~2 min.
    c.push_back({"Audyt mechaniki",   "#b91c1c", Audit::auditTests()});
    return c;
}
