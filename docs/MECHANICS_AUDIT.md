# Audyt mechaniki symulacji — EasyRider

Data: 2026-11-20. Zakres: trasowanie A*, model kosztów/korków, ruch pojazdów,
ronda, bezpieczne odstępy, stabilność czasowa. UI nie było zmieniane (poza
interpolacją prezentacji ciągłych łuków — patrz „Zmiany w GUI").

## Jak uruchomić

```
EasyRiderTests --audit            # wszystkie scenariusze A–E + S (~2 min)
EasyRiderTests --audit B          # filtr po grupie / prefiksie ID
EasyRiderTests --headless 1       # 51 testów + kategoria „Audyt mechaniki"
EASYRIDER_CAL=1 EasyRiderTests --audit D9   # surowe próbki kalibracji (CAL/LAG)
```

Walidator `src/core/SimMonitor.*` (niezależny od silnika) po każdym kroku sprawdza:
I1 wartości skończone · I2 auto na legalnym torze (oś pasa / okrąg ronda / tor
manewru) · I3 brak skoków położenia (Δ > v·dt), cofnięć i skoków kursu > 20°/krok ·
I4 nakładanie RZECZYWISTYCH obróconych obrysów (SAT) · I5 min. odstęp i styki
(< 1 px) · stan w połowie kroku (przecięcie trajektorii między próbkami) ·
I7 legalne następstwa odcinków, brak zawracania · zniknięcia poza krawędzią ·
korekty awaryjne silnika. Każde naruszenie: tick, czas, id aut, szczegóły.
I9 (A* ≤ wyrocznia) — grupa A: niezależny Dijkstra na wyeksportowanym grafie.

## Stan wyjściowy (przed zmianami): 10 / 39 PASS

Log: `build/audit-BEFORE.log`. Silnik i nawigacja były identyczne z migawkami
`build/visual-baseline` i `build/recovery-before` — ostatnie zmiany dotyczyły
wyłącznie GUI, więc problemy NIE są regresją tych zmian, tylko istniały wcześniej.

| Problem (potwierdzony pomiarem) | Dowód |
|---|---|
| Skręt = przestawienie auta | skok 17 px (prawo) / 52 px (lewo), kurs 90° w 1 kroku |
| Wjazd/zjazd z ronda = przestawienie | wjazd ~25 px + 69°, zjazd ~8 px |
| Kolizje „naprawiane" cofaniem (`resolveOverlaps`) | do 137 korekt / 120 s; cofnięcia aut |
| Auto na rondzie miało prostokąt w orientacji wjazdu | rzeczywiste obrysy nachodziły do 10,6 px |
| Styki zderzaków | odstęp < 1 px (twarde zerowanie prędkości przy gap < 5) |
| Niewidoczna „sygnalizacja" rond (fazy 6 s) | 225 s·auto czekania przed PUSTYM rondem w 120 s |
| Prędkość na rondzie 110 px/s (dokumentacja: 55) | `drive()` nadpisywał `kRingSpeed` |
| Długość odcinków brzegowych R2L/B2T ~2240 px | trasa 8→0: 80,9 s vs 0→8: 32,6 s |
| Kara za światło wg BIEŻĄCEJ fazy dalekich skrzyżowań | skok kosztu 4 s po 0,1 s |
| 4 auta stojące = 4 auta jadące | identyczny koszt odcinka |
| Histereza przeliczania porównywała koszty z różnych punktów startu | 9 zmian trasy w zwykłym przejeździe |
| Błąd A*: wpis odrzucany, gdy g < f (=g+h) | ukryty — h ≡ 0 dla celów-checkpointów |
| Heurystyka niedopuszczalna (310 px/110 vs koszt 240 px/110) | ukryty j.w. |
| Globalny generator losowy | dwa silniki w jednym procesie dzieliły stan |
| dt × mnożnik tempa bez podkroków | ×4 → krok 0,064 s przeskakiwał okna decyzyjne |

Wykryte w trakcie napraw (nie było w hipotezach):

* `Car::m_decided = true` w konstruktorze — każde nowe auto (także SmartCar)
  pomijało decyzję na PIERWSZYM skrzyżowaniu i jechało prosto.
* SmartCar wstawiany na zajęte miejsce krawędzi → cofnięty za margines mapy →
  usunięty → „Samochód zgubił trasę" zaraz po starcie (2/30 przejazdów).
* Kontrola skrętu przed wjazdem i decyzja w strefie miały ten sam próg — auto
  przechodziło pierwszą, a 0,3 s później oblewało drugą i gubiło skręt.
* Linia ustąpienia zbyt blisko ronda: narożniki prostokąta jadącego po okręgu
  zakreślają promień √((R+W/2)²+(L/2)²) ≈ 63 px → wzajemne czekanie (zakleszczenie).

## Naprawy

**Ruch (Car, SimulationEngine)**
* Manewry po ciągłym torze: łuk kołowy styczny do obu pasów (skręt), krzywe
  Béziera styczne do pasa i okręgu (wjazd/zjazd z ronda). Kurs = styczna toru.
* Obrys auta zawsze obrócony zgodnie z kursem; `boundingRect` obejmuje obrys obrócony.
* Bezpieczna prędkość (Gipps) wobec przeszkody na PRZEWIDYWANYM torze (pas, łuk,
  rondo, wjazd/zjazd — niezależnie od odcinka grafu): v ≤ −bτ + √((bτ)² + v_L²·b/b_L + 2b·s),
  s₀ = 6 px odstępu postoju. Odstęp rośnie z prędkością; brak twardych zer prędkości.
* `resolveOverlaps` sprawdza dokładne obrysy i zostaje wyłącznie siatką bezpieczeństwa
  (licznik korekt; w testach każda korekta = błąd). We wszystkich scenariuszach: 0.
* Stały krok logiki ≤ 1/60 s (podkroki) — wynik niezależny od tempa symulacji.
* Rondo: pierwszeństwo aut na rondzie (wjazd, gdy nadjeżdżające auto nie dogoni nas
  w czasie włączania i za punktem włączenia jest miejsce), bez niewidocznych faz;
  linia ustąpienia poza obszarem zakreślanym przez auta na okręgu; zjazd tylko gdy
  na pasie zjazdu jest miejsce (inaczej kolejne okrążenie); prędkość 55 px/s.
* Skręt: hamowanie przed łukiem do prędkości z limitu przyspieszenia bocznego;
  decyzja w strefie niezależnie od fazy (dawniej skręt na żółtym był pomijany).
* Generator losowy per silnik + `setSeed`; testy używają ziarna próby.

**Trasowanie (DynamicPathfinder, TrafficAnalyzer, RouteManager)**
* A*: wpis kolejki niesie g; heurystyka h = k·|n−cel|, k = min(dolne ograniczenie
  kosztu krawędzi / odległość) → dopuszczalna i spójna. ~45% mniej rozwinięć niż Dijkstra.
* Koszt = przewidywany czas [s]: ruch przez węzeł + odcinek (geometria, kolejka
  aut stojących, auta jadące z wagą zależną od prędkości) + oczekiwanie przy węźle
  (światła: z faz dla najbliższego węzła, wartość oczekiwana dalej; rondo: zajętość).
  Parametry wyznaczone pomiarem (D9: 3007 przejazdów krawędzi; D9n: auto nawigowane).
* Wspólna, poprawna geometria odcinków (`segmentIndex`/`segmentLength`).
* Przeliczanie: start od najbliższego NIEZATWIERDZONEGO punktu decyzyjnego (manewr
  w toku / strefa / ostatnie 90 px → wymuszony wyjazd); porównanie z POZOSTAŁĄ
  częścią bieżącej trasy w tym samym snapshocie; zmiana gdy zysk ≥ max(2,5 s; 10%).

## Wyniki końcowe

`EasyRiderTests --audit`: **42/42 PASS** (identyczna tabela w dwóch przebiegach),
stress `--audit S`: **10/10**, `--headless 1`: **103/103** (51 dotychczasowych + 52 audytu).

| Scenariusz | seed | ticki | aut | kolizje | styki | teleport. | skoki kursu | korekty | wynik |
|---|---|---|---|---|---|---|---|---|---|
| A1–A10 trasowanie (A* = Dijkstra: 992–2604 par/scen.) | 0–303 | – | – | – | – | – | – | – | PASS |
| A7/A8 zator w trakcie jazdy, rerouting | 8 | 4013 | 6 | 0 | 0 | 0 | 0 | 0 | PASS |
| A9 brak oscylacji (40 aut) | 9 | 4155 | 41 | 0 | 0 | 0 | 0 | 0 | PASS |
| B1–B3 rondo: prosto / 1. zjazd / dalszy, 4 wloty | 0 | 5000 | 1 | 0 | 0 | 0 | 0 | 0 | PASS |
| B4 5 aut na rondzie | 11 | 2400 | 5 | 0 | 0 | 0 | 0 | 0 | PASS |
| B5/B6 ustępowanie, jednoczesny wjazd z 4 stron | 6 | 5400 | 16 | 0 | 0 | 0 | 0 | 0 | PASS |
| B7 zablokowany wylot | 3 | 2700 | 5 | 0 | 0 | 0 | 0 | 0 | PASS |
| B10 6 rond, 80 aut, 120 s | 12, 13 | 7200 | 80 | 0 | 0 | 0 | 0 | 0 | PASS |
| C1–C3 wolniejszy lider / hamowanie / nagłe zatrzymanie | 1, 2 | 1500–1800 | 2–4 | 0 | 0 | 0 | 0 | 0 | PASS |
| C4/C5 kolejka na czerwonym i ruszanie | 4 | 2400 | 8 | 0 | 0 | 0 | 0 | 0 | PASS |
| C6 skrzyżowanie 4 wloty, skręty | 14, 15 | 3600 | 16 | 0 | 0 | 0 | 0 | 0 | PASS |
| C7/C8 zablokowane rondo, zjazd do kolejki | 5 | 3000 | 6 | 0 | 0 | 0 | 0 | 0 | PASS |
| C9 100 aut, 60 s | 16–18 | 3600 | 100 | 0 | 0 | 0 | 0 | 0 | PASS |
| D9 predykcja krawędzi (MAE 2,28/2,00 s; pierwotny 3,26/3,02 s) | 19, 20 | 10800 | 60 | 0 | 0 | 0 | 0 | 0 | PASS |
| D5s nawigacja z korkami vs najkrótsza (śr. −2,5 s/przejazd) | 500–511 | 75764 | 61 | 0 | 0 | 0 | 0 | 0 | PASS |
| D9n 30 przejazdów nawigowanych (30/30 dotarło) | 900–929 | 104158 | 61 | 0 | 0 | 0 | 0 | 0 | PASS |
| E1 8 aut × 600 s | 41 | 36000 | 8 | 0 | 0 | 0 | 0 | 0 | PASS |
| E2 30 aut × 300 s | 42, 43 | 18000 | 30 | 0 | 0 | 0 | 0 | 0 | PASS |
| E3 70 aut × 300 s | 44 | 18000 | 70 | 0 | 0 | 0 | 0 | 0 | PASS |
| E4 ×4 (dt 0,064 s), 40 aut × 240 s | 45 | 3750 | 40 | 0 | 0 | 0 | 0 | 0 | PASS |
| S stress 100 aut × 120 s, 6 rond | 61–68 | 7200 | 100 | 0 | 0 | 0 | 0 | 0 | PASS |
| S stress ×4, 100 aut × 120 s | 71, 72 | 1875 | 100 | 0 | 0 | 0 | 0 | 0 | PASS |

Wydajność (silnik, 1 krok 16 ms, Release): 30 aut 0,014 → 0,058 ms; 60 aut
0,052 → 0,225 ms; 100 aut 0,145 → 0,593 ms. Przy ×4 (4 podkroki) ≈ 2,4 ms/klatkę.

## Znane ograniczenia (nie ukrywane)

* Predykcja ze snapshotu nie zna czasu trwania awarii ani tego, że jednorazowy
  pluton rozładuje się przed dojazdem (A3i, A3/D5 — raportowane jako OGRANICZENIE;
  reakcję dynamiczną na zator przed autem sprawdza A7/A8).
* Czas czekania SmartCara na lukę przy skręcie w lewo ma „ciężki ogon" zależny od
  ruchu z naprzeciwka; stała go nie oddaje (bias lewo ≈ −1 s/krawędź).
* Pojedyncze przejazdy nawigowane bywają wolniejsze od trasy najkrótszej (D5s:
  6 szybciej, 4 wolniej, średnio −2,5 s) — zysk jest statystyczny, nie gwarantowany.
* Brak formalnego dowodu bezkolizyjności: testy obejmują ok. 23 mln próbek aut·kroków bez
  nakładania, ale to nie jest dowód dla dowolnych scenariuszy.
* Kontrola stref konfliktu na skrzyżowaniach jest pośrednia (reguły świateł +
  tor przewidywany + obrysy); osobnego rezerwowania stref nie dodano.
