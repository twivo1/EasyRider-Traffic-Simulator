# UI FIX 01 — widok mapy, ograniczenia kamery, wybór START / CEL

Zakres: wyłącznie warstwa GUI (`src/gui`), CMake i narzędzie testowe.
`src/core` i `src/nav` bez zmian (git diff pusty) — mechanika nietknięta.

## A. Widok mapy i kamera

**Przyczyna pustych marginesów.** `WorldView::fit()` liczył skalę jako
`min(sx, sy)` (CONTAIN) z odjętym marginesem 24 px, względem `bounds()` —
kanwy powiększonej o 24 jednostki z każdej strony. Gdy proporcje okna różniły
się od mapy (2380 × 1430 ≈ 1,66), powstawały pasy, a `clampPan()` centrował
mapę w nich; drogi, które logicznie kończą się na krawędzi kanwy, wyglądały na
urwane w środku ciemnego pola. Kamerę dało się też przeciągnąć ~15–20 px poza
mapę. Pomiar przed zmianą (`build/ui-before`): pasy 22–97 px, 43 błędy testów.

**Rozwiązanie (SimulationView + WorldView).**
- Świat logiczny: `WorldView::canvas()` = (0, 0, kCanvasW, kCanvasH) — tam,
  gdzie fizycznie kończą się drogi. Geometria świata nie jest zmieniana ani
  „dorysowywana": drogi brzegowe sięgają krawędzi widoku, bo krawędź kanwy
  pokrywa się z krawędzią viewportu.
- Minimalny zoom (COVER): `minZoom = max(szerokośćWidoku / kCanvasW, wysokośćWidoku / kCanvasH)`,
  gdzie `zoom` = piksele logiczne na jednostkę świata (transformacja
  `scale(zoom) · translate(−pan)`, skala jednakowa w obu osiach).
- Ograniczenie kamery po KAŻDEJ zmianie (przeciąganie, kółko, zmiana rozmiaru,
  panel boczny, reset): `zoom ∈ [minZoom, max(4, minZoom)]`,
  `pan.x ∈ [0, kCanvasW − szer/zoom]`, `pan.y ∈ [0, kCanvasH − wys/zoom]`.
  Przy zoom ≥ minZoom przedziały są niepuste, więc widoczny wycinek zawsze leży
  w kanwie — bez pustych pasów, bez sprężynowania.
- Zoom pod kursorem: punkt świata pod kursorem zostaje pod kursorem, potem
  ograniczenie (przy krawędzi ma pierwszeństwo).
- Zmiana rozmiaru zachowuje środek widoku i stopień przybliżenia względem
  `minZoom`. Panel boczny jest osobną kolumną layoutu — widok dostaje
  rzeczywisty rozmiar po jego odjęciu.
- Kreator mapy (podgląd w `StartScreen`) celowo zostaje przy CONTAIN
  (`WorldView::fitted`) — tam ważniejsze jest pokazanie całej sieci.

## B. Wybór punktów trasy (`RoutePicker`)

- **Etap 1 — START:** koło w kolorze cyjan z białą obwódką i ciemnym grotem
  skierowanym DO mapy; kotwica w końcu pasa wjazdowego na krawędzi mapy.
  Karta: „1 / 2 — WYBIERZ START" / „Kliknij oznaczony punkt wjazdu na mapę."
- **Etap 2 — CEL:** zaokrąglony jasny kwadrat z ciemną obwódką i grotem NA
  ZEWNĄTRZ mapy; kotwica w końcu pasa wyjazdowego. Wybrany START zostaje jako
  koło z etykietą „START". Różnica etapów niesie KSZTAŁT i kierunek grota, nie
  tylko kolor; kolory sygnalizacji nie są używane. Pokazywane są tylko cele
  osiągalne z wybranego startu (zapytanie do istniejącej wyszukiwarki, bez zmian A*).
  Karta: „2 / 2 — WYBIERZ CEL" / „Kliknij oznaczony punkt wyjazdu z mapy."
- **Stały rozmiar ekranowy** (px logiczne) — niezależny od zoomu i DPI.
  Hitbox 40 × 40 px wokół środka znacznika, liczony tą samą funkcją
  `RoutePicker::layout()` co rysunek. Hover: powiększenie, grubsza obwódka,
  kursor wskazujący, podpowiedź („Wybierz punkt startowy/docelowy").
- **Sygnalizatory:** `MapRenderer::signalHeadRects()` — jedno źródło prawdy dla
  rysowania świateł i układu znaczników. Znacznik (i etykieta) wybiera najbliższą
  pozycję, która: mieści się w widoku, nie nachodzi na żaden sygnalizator
  (odstęp 4 px), nie wchodzi na pas ruchu ronda, nie nachodzi na inne hitboxy,
  kartę instrukcji ani zakładki. Gdy znacznik musi odsunąć się od końca pasa,
  cienka linia łączy go z kotwicą. Punkty świata nie są przesuwane.
- **Punkty poza widokiem:** nie są klikalne; przy krawędzi widoku pojawia się
  klikalna zakładka („8 punktów"), która przesuwa kamerę tak, by odsłonić
  najbliższy ukryty punkt (kolejne kliknięcia — kolejne punkty). Mapę można też
  przeciągać w trybie wyboru (poza znacznikami).
- **ESC / zwykły tryb:** znaczniki i karta tylko w trybie wyboru; ESC anuluje
  i czyści stan; ponowne „Wyznacz trasę" zaczyna od etapu START.

## Weryfikacja (rzeczywiste okno Qt, `EasyRiderGuiCheck`, `GUICHECK_UI=1`)

Sterowanie zdarzeniami wejścia (przyciski, kółko, Ctrl+LPM, kliknięcia myszą,
Esc), zrzuty z ekranu. Nie jest to obsługa przez człowieka. Ekran testowy:
1440 × 900 przy DPR 2; rozmiary 1536 × 864 i 1920 × 1080 uzyskane przez
`QT_SCALE_FACTOR` (rzeczywista ścieżka skalowania Qt).

| Konfiguracja | Testy UI |
|---|---|
| DPR 2,0 (natywnie), okno 1280×720 / 1442×864 / 1442×880 + maksymalizacja | 202 / 202 |
| `QT_SCALE_FACTOR=0.75` (DPR 1,5), 1280×720 / 1536×864 / 1920×1080 | 202 / 202 |
| `QT_SCALE_FACTOR=1.25` (DPR 2,5), te same rozmiary | 202 / 202 |

Każdy przebieg: brak pustych pasów (widok domyślny, max oddalenie, 4 kierunki
przeciągania przy różnych zoomach, panel otwarty/zwinięty), jednakowa skala,
zoom pod kursorem; znaczniki nie na sygnalizatorach, hitboxy rozłączne i w
widoku, brak konfliktów rozmieszczenia, każdy z 32 punktów osiągalny przez
zakładki przy kilku zoomach; START i CEL ze wszystkich 4 stron klikane myszą i
zweryfikowane z uruchomioną nawigacją; klik obok znacznika i w sygnalizator nic
nie wybiera; ESC; SmartCar dojeżdża trasą wybraną znacznikami.

Zrzuty: `build/ui-before/` (przed), `build/ui-after*/` (po).

### Regresja (po zmianach)
- Mechanika: `EasyRiderTests --headless 1` — **107 / 107** (jak przed zmianami UI).
- Pełna sesja GUI (`EasyRiderGuiCheck`, kliknięcia w NOWE znaczniki): nawigacja
  10/10 dotarło, pierwszy skręt 10/10; zator 12 aut — trasa go omija, dojazd;
  silnik 0 naruszeń we wszystkich fazach; render ≠ silnik 0, skoki 0, nakładania 0.

### Poprawki towarzyszące (prezentacja / walidator, nie silnik)
- `CarRenderer`: bufor klatek i kolory kluczowane stałym `id` auta zamiast adresu
  (adres zwolnionego auta bywa nadawany nowemu — wtedy interpolowałoby się ono od
  pozy starego, do 40 px).
- `SimMonitor` (instrumentacja testowa, nie linkowana do aplikacji): granica
  przemieszczenia uwzględnia ½·a_max·dt² — przy pominiętym ticku GUI (dt 0,128 s)
  auto może przyspieszyć i zahamować między próbkami.

### Znane ograniczenia
- Testy UI to skryptowana sesja na prawdziwym oknie, nie ocena przez człowieka.
- Rozmiary 1536×864 i 1920×1080 przy natywnej skali przekraczają ekran testowy
  (1440×900) — sprawdzone przez `QT_SCALE_FACTOR=0.75/1.25`.
- W sesji GUI pozostają pojedyncze nieciągłości renderu ≤ 0,23 px logicznego
  (jedno auto, początek fazy nawigacji) — przyczyna nieustalona; oraz zbiorcze
  przy przestojach UI > 100 ms (zrzuty ekranu harnessu), gdzie CarRenderer
  celowo synchronizuje się bez interpolacji.
