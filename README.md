# EasyRider — symulacja ruchu drogowego z nawigacją GPS (A*)

EasyRider symuluje ruch drogowy na siatce **8×8 skrzyżowań** (sygnalizacja
świetlna oraz ronda). Dla wyróżnionego pojazdu (**SmartCar**) wyznacza trasę
GPS algorytmem **A\*** z dynamicznymi kosztami zależnymi od natężenia ruchu.
Auta przestrzegają pierwszeństwa i nie nakładają się na siebie.

Projekt składa się z **dwóch aplikacji** budowanych z jednego drzewa źródeł:

- **`EasyRider`** — główna aplikacja okienkowa (symulacja + interfejs),
- **`EasyRiderTests`** — runner testów weryfikacyjnych (GUI lub tryb konsolowy).

Oba programy współdzielą ten sam kod silnika (`src/core`) i nawigacji
(`src/nav`) — m.in. moduł wizualizacji trasy `VisualRouteBuilder`, dzięki
czemu **testy weryfikują dokładnie to, co rysuje główna aplikacja**.

---

## Wymagania

- **CMake ≥ 3.16** oraz generator **Ninja** (lub Make),
- **Qt 6** (komponenty `Core`, `Gui`, `Widgets`),
- kompilator C++17 (GCC/Clang/MSVC),
- *(opcjonalnie)* **Doxygen** + **Graphviz** do generowania dokumentacji.

Na Ubuntu/Debian:

```bash
sudo apt install cmake ninja-build qt6-base-dev libgl1-mesa-dev
sudo apt install doxygen graphviz          # opcjonalnie (dokumentacja)
```

---

## Budowanie

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Powstaną dwa pliki wykonywalne: `build/EasyRider` oraz `build/EasyRiderTests`.

Aby zbudować tylko jeden cel:

```bash
cmake --build build --target EasyRider        # tylko główna aplikacja
cmake --build build --target EasyRiderTests   # tylko testy
```

---

## Uruchamianie

### Główna aplikacja

```bash
./build/EasyRider
```

Aplikacja startuje w trybie pełnoekranowym. Wybierasz punkty startowy i
docelowy na krawędziach mapy, a SmartCar jedzie wyznaczoną trasą GPS,
aktualizowaną na bieżąco wraz ze zmianami natężenia ruchu.

### Runner testów — tryb graficzny

```bash
./build/EasyRiderTests
```

Otwiera okno z listą **10 kategorii** i **51 testów**. Każdy wiersz jest
klikalny — kliknięcie uruchamia pojedynczy test, animując symulację w prawym
panelu i pozostawiając jej stan końcowy do obejrzenia. Przycisk **„Uruchom
wszystkie testy"** wykonuje cały zestaw po kolei.

### Runner testów — tryb konsolowy (headless)

```bash
./build/EasyRiderTests --headless        # 1 przebieg wszystkich 51 testów
./build/EasyRiderTests --headless 3      # 3 przebiegi z rzędu
./build/EasyRiderTests --run-all 3       # alias --headless
```

Tryb headless uruchamia **te same testy** co GUI i stosuje **identyczną
regułę werdyktu**, ale bez okna, bez ograniczenia do 60 fps i bez pauz między
próbami — pełny przebieg trwa sekundy. Wymusza platformę Qt „offscreen", więc
działa również bez serwera X (CI, zdalne maszyny).

Wypisuje wynik każdego testu (`[ZAL]` / `[NIE]`), podsumowanie każdego
przebiegu (`N / 51`) i kończy proces kodem **0** (wszystko zaliczone) lub
**1** (wystąpiło niepowodzenie) — wygodne do automatyzacji:

```bash
./build/EasyRiderTests --headless && echo "OK" || echo "REGRESJA"
```

> **Determinizm:** generator losowy silnika jest inicjowany stałym ziarnem
> (seed = 42) przy starcie procesu, a krok czasu jest stały (1/60 s).
> Pierwsze przebiegi lokalnej weryfikacji dawały 51/51. Późniejsze przebiegi
> wykazywały niezweryfikowane próby na rondach również w wersji bazowej;
> szczegóły i rzeczywiste wyniki zawiera [raport naprawy wizualizacji](docs/VISUAL_RECOVERY.md).
> Wiele przebiegów w jednym procesie (`--headless 3`) celowo NIE resetuje
> generatora między przebiegami — odwzorowuje to wielokrotne uruchamianie
> testów w GUI, gdzie stan RNG przechodzi dalej.

---

## Organizacja testów

Po refaktorze dawny monolit `tests/TestSuite.h` (~2950 linii) został
rozdzielony programowo na czytelną strukturę:

```
tests/
├── TestFramework.h          # typy: TestCase, AttemptResult, EvidenceReport + helpery (MS, reach, measure, markPass/Fail/Unverified)
├── TestSuite.h              # CIENKI agregator: #include kategorii + buildAllCategories()
├── HeadlessRunner.h         # konsolowy wykonawca testów (--headless)
├── TestWindow.{h,cpp}       # graficzny runner testów
├── TestPreviewView.{h,cpp}  # podgląd symulacji na żywo (nakładki, kamera)
├── utils/
│   └── TestHelpers.h        # wspólne helpery scenowe (countCollisions, routeToPolyline, …) — tylko nagłówkowe
└── categories/              # jedna kategoria = jeden plik .h + .cpp
    ├── GeometryTests.{h,cpp}      (6 testów)
    ├── FlowTests.{h,cpp}          (5)
    ├── CarTests.{h,cpp}           (7)
    ├── RoundaboutTests.{h,cpp}    (5)
    ├── NavigationTests.{h,cpp}    (4)
    ├── VisualRouteTests.{h,cpp}   (8)
    ├── TrafficTests.{h,cpp}       (3)
    ├── MapTests.{h,cpp}           (3)
    ├── SpawnTests.{h,cpp}         (3)
    └── EngineTests.{h,cpp}        (6)
```

Razem **51 testów** w **10 kategoriach**. Plik `TestSuite.h` jest teraz
wyłącznie agregatorem — dołącza nagłówki kategorii i udostępnia
`buildAllCategories()`, które spina je w jeden zestaw zużywany przez oba
runnery (graficzny i headless).

---

## Model „attempt-based" (oparty na próbach)

Testy nie sprawdzają stanu raz — **wielokrotnie obserwują rzeczywiste
zachowanie** silnika. Kluczowa zasada:

> **Czas nie kończy testu.** Test kończy się, gdy zbierze wymaganą liczbę
> udanych prób (`minSuccessfulAttempts`) albo wyczerpie limit prób
> (`maxAttempts`). Limit klatek (`maxFramesPerAttempt`) to jedynie
> bezpiecznik kończący *pojedynczą próbę*, a nie cały test.

Przebieg pojedynczej próby:

1. **`declareMilestones(attemptIdx)`** — deklaruje listę „kamieni milowych"
   (milestone), czyli konkretnych zdarzeń, które trzeba zaobserwować
   (np. *„auto wjechało na rondo"*, *„trasa przeliczona przez GPS"*).
2. **`setupAttempt(engine, attemptIdx)`** — przygotowuje scenariusz danej
   próby (deterministycznie zależny od numeru próby).
3. **`updateAttempt(ctx)`** — wołana co klatkę: odczytuje stan silnika,
   zaznacza osiągnięte milestony (`reach`), zapisuje pomiary (`measure`)
   i aktualizuje podgląd.

Werdykt pojedynczej **próby**:

- **Zaliczona** — osiągnięto wszystkie zadeklarowane milestony (lub test
  jawnie wywołał `markPass`),
- **Niezaliczona** — zaobserwowano twarde naruszenie warunku (`markFail`),
- **Niezweryfikowana** — nie udało się zaobserwować wymaganego zachowania
  zanim minął limit klatek (timeout) — *brak naruszenia, ale i brak dowodu*.

Werdykt całego **testu**: **zaliczony** wtedy i tylko wtedy, gdy liczba
udanych prób ≥ `minSuccessfulAttempts`. Testy inspekcyjne (statyczne, np.
sprawdzenie geometrii siatki) używają jednej próby i „krokują" widocznie
przez wszystkie obiekty.

### „Niezweryfikowany" — co oznacza

W GUI test może otrzymać **bursztynowy** znacznik **„Niezweryfikowany"**
zamiast czerwonego **„Niezaliczony"**. Oznacza to sytuację, w której test
**nie wykrył żadnego błędu** (zero prób niezaliczonych), lecz zabrakło
kompletu *potwierdzonych* prób (część prób skończyła się timeoutem). Innymi
słowy: *nie ma dowodu na poprawność, ale też nie ma dowodu na błąd* — często
skutek zbyt krótkiego limitu klatek lub rzadkiego scenariusza.

> Uwaga: „Niezweryfikowany" jest **etykietą wyłącznie wizualną**. Wewnętrzny
> werdykt pozostaje binarny (zaliczony/niezaliczony), więc karta podsumowania
> liczy taki test jako niezaliczony — kolor jedynie podpowiada *dlaczego*.

### „Raport dowodowy"

Po zakończeniu testu prawy panel pokazuje **Raport dowodowy** — pełną
ewidencję, na której opiera się werdykt:

- liczba prób łącznie oraz rozbicie na udane / nieudane / niezweryfikowane,
- dla każdej próby: ziarno losowe (`seed`), wynik i ewentualny powód,
- lista milestonów z oznaczeniem `[✓]` / `[✗]` (co udało się zaobserwować),
- zmierzone wartości liczbowe (np. zliczone kolizje, długości tras).

Raport ma zapewnić, że „zaliczony" nie jest deklaracją, lecz **wnioskiem z
zaobserwowanych faktów**.

### Przerywanie testów

W trybie graficznym podczas trwania zestawu pojawia się przycisk **„Anuluj
testy"**. Anulowanie jest sprawdzane atomicznie w pętli prób, więc reaguje
niemal natychmiast. Test przerwany w trakcie jest traktowany jako **NIE
uruchomiony** — wraca do stanu *„Oczekuje"*, bez werdyktu i bez raportu, aby
częściowe próby nie zostały błędnie zaliczone jako sukces.

---

## Interfejs graficzny — Qt Widgets i Qt Designer

Interfejs używa płaskiej palety, fontu Segoe UI i wspólnego stylu kontrolek
`src/gui/UiStyle.h`. Mapa jest głównym obszarem okna. Wąska, zwijana kolumna
sterowania jest częścią layoutu, więc nie przykrywa dróg ani checkpointów.

| Plik | Przeznaczenie |
|---|---|
| `ui/main_window.ui` | Mapa, promowany `SideMenu`, kompaktowy pasek statusu |
| `ui/side_menu.ui` | Prędkość, mapa korków, motyw, nawigacja, powrót do menu; panel przewijany |
| `ui/test_window.ui` | Lista testów, podgląd, przewijany raport, uruchamianie i anulowanie |
| `ui/widgets/test_preview_view.ui` | Przewijana diagnostyka i płótno testowe |
| `ui/widgets/test_category_card.ui` | Nagłówek kategorii i kontener wierszy |
| `ui/widgets/test_row.ui` | Klikalny wiersz testu |

`StartScreen` tworzy standardowe kontrolki Qt w kodzie: ekran wejściowy,
wybór palety, podgląd wygenerowanej mapy i konfigurację liczby aut. Podgląd
mapy używa `MapRenderer`, bez dekoracyjnej animacji.

`MapRenderer` i `CarRenderer` są współdzielone przez aplikację i podgląd testów.
Renderery odczytują geometrię oraz stan silnika; pozycje, kierunki, hitboxy,
algorytmy nawigacji i warunki zaliczania testów nie są zmieniane przez wygląd.
Siedem palet zachowuje kolejność i identyfikatory; dawny motyw neonowy ma
teraz płaski wariant „Kontrastowy”.

Pliki `.ui` można otworzyć w Qt Creator / Designer. CMake AUTOUIC generuje
nagłówki `ui_*.h`; nie należy edytować plików wygenerowanych w `build/`.

Szczegóły zmian, wyniki testów, ograniczenia i lokalizacje zrzutów:
[raport naprawy wizualizacji](docs/VISUAL_RECOVERY.md).

### Weryfikacja wizualna poza ekranem

```bash
cmake -S tests/visual -B build/visual-check -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/visual-check
./build/visual-check/EasyRiderVisualCheck build/visual-output
```

Runner tworzy rzeczywiste okna Qt na platformie `offscreen`, wykonuje
scenariusze kontrolek i zapisuje PNG. Nie zastępuje oceny płynności na
fizycznym ekranie ani sprawdzenia skalowania systemowego Windows.

## Dokumentacja Doxygen

Cały kod jest opatrzony komentarzami w stylu **Doxygen po polsku**. Aby
wygenerować dokumentację HTML:

```bash
sudo apt install doxygen graphviz   # graphviz potrzebny do diagramów klas
doxygen Doxyfile
```

Wynik:

```text
docs/html/index.html
```

Wystarczy otworzyć ten plik w przeglądarce. Dokumentacja zawiera:

- **stronę główną** (`@mainpage`) z opisem projektu, modułów, algorytmów
  (A\*, AABB, sanitizer trasy), GUI Qt Designer i sposobu uruchamiania,
- **strony architektury** — `architektura_diagram` (warstwy GUI → NAV →
  CORE, dwa cele kompilacji, pliki `.ui`) i `architektura_gui` (lista
  widżetów promowanych i tego, co pozostaje dynamiczne),
- **grupy tematyczne** (`@defgroup`): `simulation_grp`, `routing_grp`,
  `gui_grp`, `tests_grp` — klasy zorganizowane po roli, nie po
  katalogu,
- **diagramy klas SVG** (dziedziczenie + kolaboracja) generowane przez
  Graphviz dla każdej istotnej klasy,
- **przeglądarkę źródeł** z linkami między deklaracjami a użyciami.

**Co dokumentowane**: główna aplikacja (`MainWindow`, `SimulationView`,
`SideMenu`, `StartScreen`), silnik symulacji (`SimulationEngine`,
`Car`, `SimulationObject`, geometria dróg i rond), nawigacja
(`DynamicPathfinder`, `RouteManager`, `TrafficAnalyzer`,
`VisualRouteBuilder`, `RoutePolylineSanitizer`), aplikacja testowa
(`TestWindow`, `TestFramework`, runner headless, model attempt-based,
kamienie milowe, raporty dowodowe).

**Co wykluczone z dokumentacji**: katalog `build/`, pliki generowane
przez Qt (`moc_*.cpp`, `ui_*.h`, `qrc_*.cpp`), katalog `CMakeFiles/`,
historia git. Konfiguracja w `Doxyfile` (`EXCLUDE_PATTERNS`).

**Diagramy**: `HAVE_DOT = YES`, `CLASS_GRAPH = YES`,
`COLLABORATION_GRAPH = YES`, `UML_LOOK = YES`. Generowane jako SVG —
interaktywne, można klikać klasy. Wyłączone `CALL_GRAPH`/`CALLER_GRAPH`
żeby uniknąć zaszumionych diagramów wywołań przez Qt.

---

## Struktura projektu (skrót)

```
EasyRider/
├── CMakeLists.txt           # dwa cele: EasyRider, EasyRiderTests
├── Doxyfile                 # konfiguracja dokumentacji (polski, HTML)
├── README.md                # ten plik
├── main.cpp                 # punkt wejścia głównej aplikacji
├── ui/                      # pliki Qt Designer (.ui): main_window, side_menu, test_window
│   └── widgets/             # szablony: test_preview_view, test_category_card, test_row
├── src/
│   ├── core/                # silnik: Car, SimulationEngine, geometria, ronda
│   ├── nav/                 # nawigacja: A* (DynamicPathfinder), analiza ruchu, trasa
│   └── gui/                 # interfejs głównej aplikacji (widok, renderery, motyw)
└── tests/                   # runnery i 51 testów (patrz „Organizacja testów")
```
