cmake_minimum_required(VERSION 3.16)
project(EasyRider VERSION 12.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
# Katalog z plikami Qt Designer (.ui) — uic generuje z nich ui_*.h.
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui ${CMAKE_CURRENT_SOURCE_DIR}/ui/widgets)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

qt_add_executable(EasyRider
    assets/controls.qrc
    main.cpp
    # Core simulation
    src/core/RoadGeometry.h
    src/core/PhysicsConstants.h
    src/core/Roundabout.h
    src/core/SimulationObject.h
    src/core/SimulationObject.cpp
    src/core/Car.h
    src/core/Car.cpp
    src/core/SimulationEngine.h
    src/core/SimulationEngine.cpp
    # Navigation system
    src/nav/NavTypes.h
    src/nav/TrafficAnalyzer.h
    src/nav/TrafficAnalyzer.cpp
    src/nav/DynamicPathfinder.h
    src/nav/DynamicPathfinder.cpp
    src/nav/RouteManager.h
    src/nav/RouteManager.cpp
    # Visual route pipeline (SHARED between main app and tests)
    src/nav/VisualRouteTypes.h
    src/nav/VisualRouteBuilder.h
    src/nav/VisualRouteBuilder.cpp
    src/nav/RoutePolylineSanitizer.h
    src/nav/RoutePolylineSanitizer.cpp
    # GUI — Renderers
    src/gui/Theme.h
    src/gui/MapRenderer.h
    src/gui/MapRenderer.cpp
    src/gui/RoutePicker.h src/gui/RoutePicker.cpp
    src/gui/CarRenderer.h
    src/gui/CarRenderer.cpp
    src/gui/OverlayRenderer.h
    src/gui/OverlayRenderer.cpp
    # GUI — Widgets
    src/gui/SimulationView.h
    src/gui/SimulationView.cpp
    src/gui/SideMenu.h
    src/gui/SideMenu.cpp
    src/gui/StartScreen.h
    src/gui/StartScreen.cpp
    src/gui/MainWindow.h
    src/gui/MainWindow.cpp
    # GUI — Qt Designer (.ui)
    ui/main_window.ui
    ui/side_menu.ui
)

target_include_directories(EasyRider PRIVATE src)
target_link_libraries(EasyRider PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets)
set_target_properties(EasyRider PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

# ── Test application ─────────────────────────────────────────────
qt_add_executable(EasyRiderTests
    assets/controls.qrc
    tests/test_main.cpp
    tests/TestWindow.h
    tests/TestWindow.cpp
    tests/TestPreviewView.h
    tests/TestPreviewView.cpp
    src/gui/MapRenderer.cpp
    src/gui/RoutePicker.h src/gui/RoutePicker.cpp
    src/gui/CarRenderer.cpp
    tests/TestFramework.h
    tests/TestSuite.h
    # Wspólne helpery testów (tylko nagłówek, inline)
    tests/utils/TestHelpers.h
    # Konsolowy runner headless (--headless) — współdzieli buildAllCategories()
    tests/HeadlessRunner.h
    # Kategorie testów wydzielone z dawnego monolitu TestSuite.h
    tests/categories/GeometryTests.h
    tests/categories/GeometryTests.cpp
    tests/categories/FlowTests.h
    tests/categories/FlowTests.cpp
    tests/categories/CarTests.h
    tests/categories/CarTests.cpp
    tests/categories/RoundaboutTests.h
    tests/categories/RoundaboutTests.cpp
    tests/categories/NavigationTests.h
    tests/categories/NavigationTests.cpp
    tests/categories/VisualRouteTests.h
    tests/categories/VisualRouteTests.cpp
    tests/categories/TrafficTests.h
    tests/categories/TrafficTests.cpp
    tests/categories/MapTests.h
    tests/categories/MapTests.cpp
    tests/categories/SpawnTests.h
    tests/categories/SpawnTests.cpp
    tests/categories/EngineTests.h
    tests/categories/EngineTests.cpp
    # Audyt mechaniki: deterministyczne scenariusze + niezależny walidator
    tests/audit/MechanicsAudit.h
    tests/audit/MechanicsAudit.cpp
    src/core/SimMonitor.h
    src/core/SimMonitor.cpp
    # Shared source files (core + nav)
    src/core/SimulationObject.cpp
    src/core/SimulationObject.h
    src/core/Car.h
    src/core/Car.cpp
    src/core/SimulationEngine.h
    src/core/SimulationEngine.cpp
    src/core/RoadGeometry.h
    src/core/PhysicsConstants.h
    src/core/Roundabout.h
    src/nav/NavTypes.h
    src/nav/TrafficAnalyzer.h
    src/nav/TrafficAnalyzer.cpp
    src/nav/DynamicPathfinder.h
    src/nav/DynamicPathfinder.cpp
    src/nav/RouteManager.h
    src/nav/RouteManager.cpp
    # Visual route pipeline (SHARED — ten sam kod co w OverlayRenderer.cpp).
    src/nav/VisualRouteTypes.h
    src/nav/VisualRouteBuilder.h
    src/nav/VisualRouteBuilder.cpp
    src/nav/RoutePolylineSanitizer.h
    src/nav/RoutePolylineSanitizer.cpp
    tests/TestCategoryCard.h
    tests/TestCategoryCard.cpp
    tests/TestRow.h
    tests/TestRow.cpp
    # GUI — Qt Designer (.ui)
    ui/test_window.ui
    ui/widgets/test_preview_view.ui
    ui/widgets/test_category_card.ui
    ui/widgets/test_row.ui
)
target_include_directories(EasyRiderTests PRIVATE src tests)
target_link_libraries(EasyRiderTests PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets)

# ── Weryfikacja rzeczywistego GUI (skryptowana sesja na MainWindow) ──
# Opcjonalna: budowana tylko, gdy dostępny jest moduł Qt6::Test.
find_package(Qt6 QUIET COMPONENTS Test)
if(Qt6Test_FOUND)
    qt_add_executable(EasyRiderGuiCheck
        assets/controls.qrc
        tests/gui_check/gui_check.cpp
        src/core/RoadGeometry.h src/core/PhysicsConstants.h src/core/Roundabout.h
        src/core/SimulationObject.h src/core/SimulationObject.cpp
        src/core/Car.h src/core/Car.cpp
        src/core/SimulationEngine.h src/core/SimulationEngine.cpp
        src/core/SimMonitor.h src/core/SimMonitor.cpp
        src/nav/NavTypes.h src/nav/TrafficAnalyzer.h src/nav/TrafficAnalyzer.cpp
        src/nav/DynamicPathfinder.h src/nav/DynamicPathfinder.cpp
        src/nav/RouteManager.h src/nav/RouteManager.cpp
        src/nav/VisualRouteTypes.h src/nav/VisualRouteBuilder.h src/nav/VisualRouteBuilder.cpp
        src/nav/RoutePolylineSanitizer.h src/nav/RoutePolylineSanitizer.cpp
        src/gui/Theme.h src/gui/MapRenderer.h src/gui/MapRenderer.cpp
    src/gui/RoutePicker.h src/gui/RoutePicker.cpp
        src/gui/CarRenderer.h src/gui/CarRenderer.cpp
        src/gui/OverlayRenderer.h src/gui/OverlayRenderer.cpp
        src/gui/SimulationView.h src/gui/SimulationView.cpp
        src/gui/SideMenu.h src/gui/SideMenu.cpp
        src/gui/StartScreen.h src/gui/StartScreen.cpp
        src/gui/MainWindow.h src/gui/MainWindow.cpp
        ui/main_window.ui ui/side_menu.ui
    )
    target_include_directories(EasyRiderGuiCheck PRIVATE src)
    target_link_libraries(EasyRiderGuiCheck PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Test)
endif()

if(WIN32 AND MINGW)
    install(TARGETS EasyRider RUNTIME DESTINATION .)
    configure_file(packaging/windows/EasyRider.iss.in EasyRider.iss @ONLY)
endif()
