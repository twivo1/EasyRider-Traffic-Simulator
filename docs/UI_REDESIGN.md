> Raport historyczny pierwszej wersji. Aktualny stan, korekty i ograniczenia opisuje [VISUAL_RECOVERY.md](VISUAL_RECOVERY.md).

# EasyRider — raport modernizacji warstwy wizualnej

## Zakres i kierunek

Mapa pozostaje głównym obszarem aplikacji. Interfejs wykorzystuje tło
`#F2F1ED`, panele `#F8F7F4`, tekst `#242424` / `#666660`, obramowania
`#D0CEC8` i ograniczony akcent `#D96535`. Font: Segoe UI. Bez nowych bibliotek UI.

Audyt objął oba programy, sześć plików Qt Designer, cztery strony kreatora,
renderery mapy, aut i nakładek oraz zależności GUI od silnika. Projekt w tym
katalogu nie zawiera repozytorium Git. Kopia źródeł i plików wykonywalnych sprzed
zmian znajduje się w `build/visual-baseline/`.

## Rzeczywiste zmiany

- **Mapa:** płaskie palety, konsekwentny asfalt i krawędzie, linie rozdzielające
  pasy przerwane na skrzyżowaniach, linie zatrzymania przy pasach wjazdowych.
  Ronda mają jednolitą nawierzchnię, wyspę i cztery strzałki kierunku. Promienie
  używane przez renderer zachowano. Sygnalizator jest pokazany przy każdym
  pasie wjazdowym; usunięto powtórzenia tego samego stanu przy wyjazdach.
- **Auta:** widok z góry, przednia i tylna szyba, reflektory, światła tylne
  oraz istniejące kierunkowskazy. SmartCar wyróżnia kolor i pas na dachu.
  Środek, kąt, długość i szerokość pochodzą z tych samych danych co wcześniej.
- **Trasa i checkpointy:** cienka bursztynowa linia z ciemnym obrysem,
  płaskie markery, mniejszy przesuwany panel nawigacji. Auta są rysowane nad
  linią trasy. Budowanie i przycinanie polilinii oraz hit-testy są niezmienione.
- **Sterowanie:** zwijana kolumna 268 px, w tym uchwyt 24 px, w rzeczywistym
  layoucie zamiast nakładki. Przewijanie przy ograniczonej wysokości.
  Suwak pokazuje aktualny mnożnik; selektor palety odzwierciedla wybór
  dokonany w kreatorze. Pasek statusu pokazuje rzeczywistą liczbę rond i
  skrzyżowań zamiast stałej informacji „8 rond”.
- **Kreator:** standardowe przyciski, wybór palety, podgląd układu i zsynchronizowane
  pole liczby aut z suwakiem. Zachowano generowanie mapy i dotychczasowe etapy.
  Usunięto animowane cząstki, poświaty i pętlę odświeżania dekoracji.
- **Testy GUI:** wspólne renderery mapy i aut, mniejsze podsumowanie, lista
  kategorii, przewijana diagnostyka i raport. Długi wynik nie wymusza powiększenia
  okna ponad wysokość ekranu 1366×768.
- **Spójność:** wspólny `src/gui/UiStyle.h`, płaskie stany normalny, hover,
  pressed, checked, focus i disabled; aktualizacja wszystkich plików `.ui`.
  Siedem wariantów mapy pozostaje dostępnych. Dawny neonowy wariant otrzymał
  płaską paletę „Kontrastowy”, zachowując identyfikator i pozycję na liście.

Nie dodano pauzy, resetu, telemetrii ani wykresów, których nie było w tym projekcie.
Zachowano istniejące skróty Esc, Ctrl+LPM, kółko myszy i F3.

## Weryfikacja

Środowisko: Windows, Qt 6.10.0, MinGW 13.1, CMake/Ninja, Release.

| Sprawdzenie | Wynik |
|---|---|
| Czysty build `EasyRider` i `EasyRiderTests` | Kod 0 |
| Pojedynczy pełny przebieg headless po finalnym buildzie | 51/51, kod 0 |
| Uruchomienie EXE z lokalnymi DLL, bez Qt/MinGW w PATH | 51/51, kod 0 |
| Pierwsze porównanie trzech przebiegów | Przed i po: 51/51, 51/51, 50/51 |
| Późniejsze powtórzenia | Niestabilne późniejsze przebiegi w obu wersjach; szczegóły poniżej |
| Kontrole i renderowanie Qt offscreen | PASS |
| Rozmiary głównego okna | 960×600, 1280×800, 1366×768, 1920×1080 |
| Okno testów z raportem | Mieści się w 1366×768; raport i diagnostyka przewijane |
| Sumy SHA-256 plików `src/core`, `src/nav`, `tests/categories` | Zero zmian |
| Wyszukiwanie gradientów i efektów graficznych w `src/gui` i `ui` | Brak |

`tests/visual/visual_check.cpp` uruchamia rzeczywiste widżety Qt poza ekranem.
Sprawdza kreator i jego przyciski powrotu, generowanie mapy, liczbę aut,
mnożniki prędkości, wszystkie palety, mapę korków, kliknięcia checkpointów
rozpoczynające GPS, zoom, anulowanie przez Esc i przycisk, zwijanie panelu,
powrót do menu, uruchomienie pojedynczego testu i anulowanie całego zestawu.
Sprawdza również brak nakładania panelu na mapę i zapisuje obrazy do kontroli.
Obejrzano wygenerowane obrazy konfiguracji, mapy, nawigacji, powiększenia,
ciemnej palety, małego okna i raportu testów.

### Istniejąca niestabilność testów wielokrotnych

Po czystym buildzie uzyskano 51/51, 51/51, 49/51. Kontrolne ponowienie
niezmienionego bazowego EXE dało 51/51, 50/51, 49/51, a nowego EXE
51/51, 50/51, 50/51. Nie należy więc traktować trzech przebiegów jako
powtarzalnie zielonego zestawu. Problemy dotyczą „Płynne krążenie po rondzie”
i „Brak kolizji ring–ring”: za mało potwierdzonych prób, bez wykrytych
twardych naruszeń w tych niezaliczonych testach (`zle:0`). Nie zmieniano
limitów, kryteriów zaliczenia ani silnika, aby ukryć ten wynik. Przyczyna
niestabilności wymaga osobnej analizy poza zakresem redesignu.

### Koszt rysowania

Osobny tryb `--benchmark` uruchamia nowy proces i tę samą scenę ze 100 autami,
po 180 krokach silnika; następnie rysuje ją 300 razy do obrazu 1280×768.
Pięć par pomiarów: mediana **3,97 ms przed / 3,81 ms po zmianach**.
Nie zaobserwowano regresji tego pomiaru. Jest to pomiar renderowania do obrazu,
nie pomiar FPS całej aplikacji na fizycznym ekranie. Pełne dane:
`build/visual-redesign/benchmark.json`. Pomiar wypisywany przy pełnym smoke-teście
obejmuje inny stan generatora i obciążenie kompilacją; do porównania służy
wyłącznie oddzielny tryb benchmarku.

## Artefakty i odtworzenie

- Aplikacja: [EasyRider.exe](../build/visual-redesign/EasyRider.exe).
- Testy: [EasyRiderTests.exe](../build/visual-redesign/EasyRiderTests.exe).
- [Główne okno po zmianach](../build/visual-redesign/screenshots/05-simulation.png).
- [Nawigacja](../build/visual-redesign/screenshots/08-navigation.png).
- [Okno testów](../build/visual-redesign/screenshots/11-test-result.png).
- Zrzuty przed zmianami: `build/visual-baseline/screenshots/`.
- Logi: `build/visual-redesign/final-build.log`, `single-tests.log`,
  `final-tests.log`, `recheck-tests.log`, `build/visual-check-run.log`.

```text
cmake -S . -B build/visual-redesign -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/visual-redesign
build/visual-redesign/EasyRiderTests.exe --headless

cmake -S tests/visual -B build/visual-check -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/visual-check
build/visual-check/EasyRiderVisualCheck.exe build/visual-redesign/screenshots
build/visual-check/EasyRiderVisualCheck.exe build/visual-redesign/screenshots --benchmark
```

Wymagane są Qt i toolchain dostępne w środowisku CMake. W tej instalacji użyto
`C:/Qt/6.10.0/mingw_64` jako `CMAKE_PREFIX_PATH` i kompilatora z
`C:/Qt/Tools/mingw1310_64/bin`. Przygotowany katalog EXE zawiera lokalne DLL
Qt/MinGW oraz platformy Windows i offscreen.

## Ograniczenia i sprawdzenie ręczne

Computer Use nie udostępnił okna symulatora do inspekcji; jedna próba zakończyła
się limitem czasu autoryzacji. Ocena obrazów i interakcji została wykonana
na platformie Qt offscreen, bez deklarowania testu pulpitu Windows.

Przed prezentacją sprawdzić na docelowym laptopie: płynność podczas ruchu,
zoom i przesuwanie myszą, czytelność świateł z odległości prezentacyjnej,
skalowanie systemowe 125% / 150% oraz obsługę klawiaturą. Nie wykonano
pomiaru FPS na fizycznym monitorze. `windeployqt` zgłosił brak opcjonalnych
`dxcompiler.dll` / `dxil.dll`; aplikacja korzysta z rastrowego QPainter.
