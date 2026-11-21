#pragma once
/**
 * @file MechanicsAudit.h
 * @brief Audyt mechaniki symulacji — deterministyczne scenariusze A–E.
 *
 *  Każdy scenariusz uruchamia świeży SimulationEngine z ustalonym ziarnem,
 *  krokuje go stałym dt i po każdym kroku przekazuje stan do niezależnego
 *  walidatora @ref SimMonitor (obrócone obrysy, skoki, legalność odcinków).
 *  Grupa A (trasowanie) porównuje A* z niezależną wyrocznią Dijkstry na tym
 *  samym, zamrożonym snapshocie kosztów.
 *
 *  Uruchomienie konsolowe: EasyRiderTests --audit [filtr]
 *  W GUI testów: kategoria „Audyt mechaniki".
 */
#include <QString>
#include <QStringList>
#include <functional>
#include <vector>
#include "core/SimMonitor.h"
#include "TestFramework.h"

namespace Audit {

struct Result {
    QString id, group, name;
    unsigned seed = 0;
    long    ticks = 0;
    double  simTime = 0;
    int     cars = 0;
    MonitorStats st;
    bool    dynamic = true;   ///< false = test statyczny (trasowanie, bez ruchu)
    bool    pass = false;
    QStringList failures;     ///< przyczyny niepowodzenia (czytelne)
    QStringList notes;        ///< pomiary
    QStringList violations;   ///< pierwsze naruszenia z monitora (tick, auta)
    double  runtimeMs = 0;
};

struct Scenario {
    QString id, group, name;
    std::function<Result()> run;
};

std::vector<Scenario> allScenarios();
/// Konsolowy runner: wypisuje tabelę i szczegóły niepowodzeń. Zwraca 0 gdy PASS.
int runCli(const QString& filter);
/// Adapter do istniejącej infrastruktury (GUI/headless): kategoria testów.
std::vector<TestCase> auditTests();

} // namespace Audit
