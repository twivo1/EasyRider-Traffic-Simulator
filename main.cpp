/**
 * @file main.cpp
 * @brief Punkt wejścia aplikacji EasyRider — tworzy QApplication i pokazuje MainWindow.
 *
 * @mainpage EasyRider — dokumentacja projektu
 *
 * @section opis Opis
 * EasyRider to aplikacja symulująca ruch drogowy na siatce 8 × 8 skrzyżowań
 * (sygnalizacja świetlna i 8 rond). Pozwala wyznaczyć trasę GPS dla
 * wyróżnionego pojazdu (SmartCar) algorytmem A\* z dynamicznymi kosztami
 * zależnymi od natężenia ruchu. Auta przestrzegają zasad pierwszeństwa
 * i nie nakładają się na siebie (gwarancja w
 * @c SimulationEngine::resolveOverlaps).
 *
 * Projekt jest w pełni obiektowy w C++ 17 z GUI w Qt 6 Widgets,
 * a układ okien zaprojektowany jest w Qt Designer (pliki @c .ui).
 *
 * @section architektura Architektura
 * Projekt zbudowany jest z dwóch celów CMake:
 *  - @b EasyRider — aplikacja graficzna (@ref MainWindow),
 *  - @b EasyRiderTests — runner testów (@ref TestWindow, tryb @c --headless N).
 *
 * Warstwy zależne tylko od warstw niżej:
 * @code{.unparsed}
 *   GUI  (src/gui/, ui/*.ui)
 *     ↓
 *   NAV  (src/nav/)  — RouteManager → DynamicPathfinder (A*), TrafficAnalyzer
 *     ↓
 *   CORE (src/core/) — SimulationEngine → SimulationObject ◀ Car (+ RbState)
 * @endcode
 *
 * Diagram dokumentacyjny: @ref architektura_diagram (docs/architecture.dox).
 *
 * @section moduly Główne moduły
 *  - @c src/core — silnik symulacji: @ref Car, @ref SimulationEngine,
 *    geometria dróg (@ref RoadGeometry.h) i rond (@ref Roundabout.h).
 *    Grupa Doxygen: @ref simulation_grp.
 *  - @c src/nav  — nawigacja: @ref DynamicPathfinder (A\*),
 *    @ref TrafficAnalyzer (analiza korków), @ref RouteManager,
 *    sanitizer trasy (@ref RoutePolylineSanitizer).
 *    Grupa Doxygen: @ref routing_grp.
 *  - @c src/gui  — interfejs: @ref MainWindow, @ref SimulationView,
 *    @ref SideMenu, renderery, motywy, ekran startowy.
 *    Grupa Doxygen: @ref gui_grp.
 *  - @c tests    — testy weryfikacyjne (51 testów w 10 kategoriach):
 *    @ref TestWindow, @ref TestFramework.h.
 *    Grupa Doxygen: @ref tests_grp.
 *
 * @section gui_section GUI w Qt Designer
 * Wszystkie statyczne układy okien zdefiniowane są w plikach @c ui/*.ui
 * i wczytywane przez @c ui->setupUi(this). Widżety niestandardowe rysujące
 * (@ref SimulationView, @c TestSimulationCanvas) są promowane w Designerze.
 * Szczegółowa lista — sekcja @ref architektura_gui.
 *
 * @section algorytmy Algorytmy
 *  - <b>A\*</b> (@ref DynamicPathfinder::findRoute) — graf 96 węzłów
 *    (64 skrzyżowania + 32 checkpointy), stan = (węzeł, kierunek wjazdu),
 *    heurystyka euklidesowa / prędkość maks.
 *  - <b>Koszt krawędzi</b> = czas bazowy + opóźnienie z liczby aut na
 *    odcinku + kara za czerwone światło na skrzyżowaniu docelowym.
 *    Patrz @ref DynamicPathfinder::segCost i @ref TrafficAnalyzer.
 *  - <b>AABB</b> — detekcja kolizji oparta o @c QRectF::intersects, z
 *    finalną gwarancją braku nakładania w
 *    @c SimulationEngine::resolveOverlaps.
 *  - <b>Sanitizer trasy</b> (@ref RoutePolylineSanitizer) — usuwa
 *    duplikaty/NaN, dzieli przy segmentach > 600 px (chroni przed
 *    artefaktem „ogona" między starą a nową trasą).
 *
 * @section testy Testy
 * Aplikacja @b EasyRiderTests uruchamia 51 testów w 10 kategoriach
 * (geometria, płynność, samochód, rondo, nawigacja, wizualizacja trasy,
 * ruch drogowy, mapa, spawn, silnik). Każdy test jest @b attempt-based:
 * wymaga zaobserwowania zdefiniowanych „kamieni milowych" w kilku
 * próbach, a nie jedynie upływu czasu. Statusy:
 * @c Zaliczony, @c Niezaliczony, @c Niezweryfikowany (timeout), @c Timeout.
 * Patrz @ref TestFramework.h i grupa @ref tests_grp.
 *
 * @section dokumentacja Generowanie dokumentacji
 * @code
 * sudo apt install doxygen graphviz   # jednorazowo
 * doxygen Doxyfile                    # wynik: docs/html/index.html
 * @endcode
 * Wejście dokumentacji: katalogi @c src, @c tests i plik @c main.cpp.
 * Wykluczone: @c build, @c moc_*, @c ui_*, @c qrc_* (generowane przez Qt).
 *
 * @section budowanie Budowanie i uruchamianie
 * @code
 * cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
 * cmake --build build
 * ./build/EasyRider                      # główna aplikacja
 * ./build/EasyRiderTests                 # runner testów (GUI)
 * ./build/EasyRiderTests --headless 1    # tryb konsolowy (CI)
 * @endcode
 */
#include "gui/MainWindow.h"
#include <QApplication>
#include "gui/UiStyle.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    UiStyle::apply(a);
    MainWindow w;
    w.showFullScreen();   // od razu na pełnym ekranie
    return a.exec();
}
