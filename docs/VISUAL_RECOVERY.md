# EasyRider — odzyskanie jakości wizualizacji

Stan weryfikacji: 16 listopada 2026. Ten raport zastępuje ocenę wyglądu w `UI_REDESIGN.md`.
Zmiany dotyczą Qt Widgets/QPainter i prezentacji. Pliki silnika i nawigacji pozostały bez zmian.

## Przyczyny ustalone w kodzie

1. **Niezależne obliczenia kadru.** Kreator, symulacja i podgląd testów wyliczały kadr osobno. Dopasowanie do samej kanwy nie przewidywało marginesu markerów i obrysów. Każdy resize symulacji resetował zoom. Nie stwierdziłem niezależnego skalowania aut względem dróg w zapisanej wersji bazowej.
2. **Za duża narysowana wyspa.** Stały promień wyspy 32 nie uwzględniał narożników samochodu przy przejściu między ruchem po okręgu a prostą. Odtworzono dwa przekroczenia przy zjeździe z ronda `(6,3)`, w krokach 338 i 339 kontrolowanej sekwencji. Pozycje środków: `(2013.41,633)` i `(2015.17,633)`. Samochód miał już kierunek 0°, a jego tylny narożnik wchodził na narysowaną wyspę. To rozbieżność rysunku wyspy z istniejącą trajektorią, nie błędna transformacja auta.
3. **Skokowe zakręty silnika.** `Car::applyTurn()` jednocześnie zmienia pozycję i kierunek. `enterRoundabout()` / `exitRoundabout()` również przełączają reprezentację ruchu. Silnik nie dostarcza pośredniej krzywej zakrętu na zwykłym skrzyżowaniu. Wygładzenie takiego skoku dowolnym łukiem tworzyłoby trajektorię, której silnik nie wykonał.
4. **Auta poza końcem mapy.** `removeOutside()` usuwa pojazdy dopiero z marginesem `kCarL + 15`. W badanej sekwencji było 484 stanów ze środkiem poza kanwą. Są to rzeczywiste stany wjazdu/wyjazdu, zachowane również po zmianach. Renderer nie przesuwa ich na asfalt ani nie ukrywa maską jezdni.
5. **Rytm prezentacji.** Poprzednio rysowanie pokazywało bezpośrednio ostatni stan kroku 16 ms. Nie było próbek prezentacyjnych pomiędzy stanami. Cała mapa była rysowana ponownie. Nowy timer prezentacji początkowo dublował odświeżenia z kroku silnika; pomiar wykrył to i duplikację usunięto.

Zielonej poziomej linii w kreatorze nie odtworzono. W najnowszej wiadomości nie było dostępnego załącznika z tym artefaktem. Nie przypisuję mu niepotwierdzonej przyczyny ani nie deklaruję jego naprawy. Zachowano rzeczywistą mapę korków i diagnostykę F3.

## Zmiany w kodzie

- `WorldView.h`: wspólne granice obejmujące jezdnie, obrysy możliwych rond i margines checkpointów, izotropowe dopasowanie oraz transformacja kamery. Kreator, symulacja i podgląd testów korzystają z tych samych zasad. Hit testing używa odwrotnej transformacji. Resize zachowuje powiększenie względem dopasowania i środek widoku z ograniczeniem pan. Nowe uruchomienie przywraca cały układ.
- `RoadVisuals.h`, `MapRenderer`: spokojne ciemne tło, oddzielone powierzchnie między drogami, asfalt, krawędzie, oznaczenia i ronda. Obwód ronda jest przerwany przy wlotach. Promień wyspy wyliczany jest z istniejących wymiarów auta i offsetów wjazdu/zjazdu; uwzględnia obrys i margines. Promień toru ruchu 50 oraz zewnętrzny asfalt 65 pozostają niezmienione. Żadna z nowych wartości prezentacyjnych nie trafia do silnika.
- `CarRenderer`: profilowane nadwozie 40×20, dwie szyby, światła, delikatny płaski cień kontaktowy, pięć kolorów bez używania RNG symulacji. Zachowano SmartCar i kierunkowskazy. Obrót jest wokół środka logicznego prostokąta. Auta nie mają osobnego mnożnika skali.
- Prezentacja przechowuje dwie próbki pozycji. Na prostej interpoluje pozycję; na tym samym rondzie interpoluje kąt po rzeczywistym okręgu i orientację najkrótszą drogą. Nie prowadzi auta po cięciwie przez wyspę. Zmiana rodzaju ruchu, skok kierunku, duży przeskok lub długa przerwa oznaczają pokazanie aktualnego stanu, bez wymyślania trajektorii. Opóźnienie prezentacji wynosi do jednego kroku, nominalnie 16 ms.
- `SimulationView`: osobny timer prezentacji 8 ms; zatrzymuje się po braku nowych próbek. Reset czyści próbki. Statyczna mapa jest buforowana w rozdzielczości viewportu i DPR; cache jest odtwarzany po zmianie kamery, motywu, rozmiaru albo układu rond. Sygnały, auta, korki i GPS pozostają dynamiczne. GPS nie zasłania sygnalizatorów.
- `MainWindow`: zachowany krok `engine.update(0.016 * m_speedMult)` i kolejność trasowania; publikowanie próbek po kroku. Timer silnika ma precyzyjny tryb Qt. Nie wykonuje dodatkowego przerysowania mapy obok timera prezentacji.
- `SideMenu`, `.ui`, `UiStyle.h`, zasoby kontrolek: kompaktowe grupy, wspólne płaskie stany przycisków, suwaka, radia, comboboxu i spinboxu, poprawne strzałki oraz czytelny uchwyt panelu. Podstawowe sterowanie pojawia się ponownie po uruchomieniu z menu. Font Segoe UI, jasne powierzchnie i ograniczony pomarańczowy akcent.
- `StartScreen`: wspólny kadr mapy, uporządkowany pasek Wróć / Nowa mapa / Akceptuj. Mapa pozostaje widoczna także przy wyborze liczby aut. Domyślna paleta nosi nazwę „Techniczny”; identyfikator i siedem pozycji listy zachowano.
- Aplikacja testowa używa wspólnego renderera i kamery, wraz ze stabilnymi kolorami pojazdów w podglądzie.

Nie dodano nowych bibliotek UI, wykresów, danych pozornych ani gradientów. W tej wersji projektu nie było osobnych kontrolek Pause/Stop/Reset; nie dopisano ich. Zachowano powrót do menu, start, mnożnik tempa, GPS, mapę korków, palety oraz istniejące skróty.

Początek rysowanej trasy GPS i sylwetka samochodu używają tej samej próbki prezentacji. Linia nie wyprzedza interpolowanego auta; obliczenia i dane trasy pozostają bez zmian.

## Wyniki weryfikacji

Windows, Qt 6.10.0, MinGW 13.1, CMake/Ninja, Release. Testy GUI wykonywano na prawdziwych widgetach Qt poprzez platformę `offscreen`, bez automatyzacji fizycznego pulpitu.

| Próba | Wynik i zakres |
|---|---|
| Build aplikacji i testów | Zakończony poprawnie; `build/recovery-build-final.log` |
| Bazowe testy silnika | 51/51 przed zmianami |
| Końcowe testy silnika | 51/51, jeden pełny przebieg; `build/recovery-tests-final.log` |
| Runner wizualny | PASS; kreator, 30 aut, palety, korki, wybór obu checkpointów, GPS, anulowanie, zoom, pan, resize, menu, ponowny start, okno testów i anulowanie jego przebiegu |
| Pusta mapa / kreator | PNG obu etapów z mapą; matematyczne testy pełnych granic dla 640×480, 960×600 i 1920×1080 |
| Resize głównego okna | 960×600, 1366×768, 1920×1080; brak nakładania panelu na mapę, zachowany aspekt |
| Transformacja i orientacja | Odwracalność screen/world, równa skala osi, pan bez zmiany zoomu, zgodność obróconej sylwetki z logicznym prostokątem we wszystkich czterech kierunkach |
| Rondo | 585 rzeczywistych stanów na okręgu: 0 narożników poza zewnętrznym asfaltem; 101 próbek interpolacji zachowuje promień 50; test przejścia kąta 179°/−179° |
| Wjazdy i zjazdy | Po korekcie wyspy 0 przekroczeń na 105 620 narożników wewnątrz kanwy; wcześniej 2. Audyt obejmuje 900 kroków, 30 aut i tempa 0,25× / 1× / 4× |
| Skrzyżowania | Zgodność orientacji z logicznym kierunkiem; skoków `applyTurn()` nie zastąpiono sztucznym łukiem |
| Pauza / wznowienie | Osobnej pauzy nie ma. Sprawdzono powrót do menu, reset prezentacji i ponowny start z domyślnym kadrem i dostępnym panelem |
| Logika przed i po | Identyczny SHA256 obserwowalnego stanu w 900 krokach z renderowaniem po zmianach; 41/41 plików core/nav/kategorii testowych identycznych z bazą |

Ślad obejmuje pozycje, boundingRect, prędkości, kierunki, stan ronda, kierunkowskazy i stany świateł. Hash:

`4881a6978de5c52433528c609e2f36064cd7cecf98d57fc45e227c666aa3ffe1`

To dowód dla konkretnej powtarzalnej sekwencji, nie formalny dowód wszystkich możliwych stanów. Generator układu map w kreatorze nadal jest losowy. Z wcześniejszej weryfikacji znane są niezaliczone próby przy wielokrotnym uruchamianiu kategorii rond w jednym procesie, wynikające z niewystarczającej liczby zweryfikowanych przejazdów; były również w bazie. Nie zmieniano ich kryteriów.

## Wydajność i zapis ruchu

Trzy parowane próby renderowania widgetu 1280×768 ze 100 autami, po rozgrzaniu, po 300 renderów:

| Próba | Przed, ms/klatkę | Po, ms/klatkę |
|---|---:|---:|
| 1 | 1,630 | 1,418 |
| 2 | 1,537 | 1,410 |
| 3 | 1,559 | 1,366 |

Osobny późniejszy pomiar podczas zapisywania porównawczych obrazów dał 2,394 / 2,520 ms. Wyniki zależą od obciążenia i taktowania komputera; nie wyciągam z nich gwarancji poprawy FPS. Pełne, niebuforowane rysowanie wszystkich warstw jest droższe od starego prostszego rysunku. Główny widok korzysta z bufora mapy.

Pomiar pętli zdarzeń bez zapisu PNG: baza 374 kroki silnika i 374 zdarzenia paint w 5994 ms. Po usunięciu podwójnych odświeżeń: 374 kroki i 746 paint w 5993 ms przy 1× oraz 374 / 746 w 5992 ms przy 4×. Krok logiczny nie zwolnił w tej próbie; prezentacja dostaje około dwie klatki na krok. To zdarzenia `offscreen`, **nie FPS fizycznego monitora** ani pomiar zużycia CPU/GPU.

[Zapis ruchu przy rondzie](../build/recovery-motion/motion.gif) pochodzi z faktycznego widgetu i kroków silnika. PNG zapisywane są po zakończeniu przechwytywania. Log czasu i liczby kroków: `build/recovery-record.log`. GIF służy ocenie sylwetek i trajektorii, nie benchmarkowi odświeżania.

Końcowy zapis ruchu: 150 klatek, 375 kroków silnika, 6005 ms; kodowanie PNG po zakończeniu przechwytywania.

## Porównanie obrazów

- Ta sama scena i identyczny stan silnika: [przed](../build/recovery-before/same-scene.png), [po](../build/recovery-after/same-scene.png).
- Całe okno: [przed](../build/recovery-before/screenshots/05-simulation.png), [po](../build/recovery-after/05-simulation.png). Te dwa zrzuty kreatorów mają losowane układy i różne liczby aut; nie służą porównaniu logiki.
- [Podgląd układu](../build/recovery-after/03-map-preview.png), [wybór liczby aut](../build/recovery-after/04-configuration.png), [GPS w powiększeniu](../build/recovery-after/09-detail.png), [okno testów](../build/recovery-after/10-tests.png).

## Pozostałe ograniczenia i ręczna ocena

- **Nie usunięto skokowej kinematyki skrętu na zwykłym skrzyżowaniu ani przełączeń wjazd/wyjazd ronda.** Pełne ciągłe zakręty wymagałyby uzgodnionej reprezentacji trajektorii pośredniej w silniku i osobnych testów kolizji, pierwszeństwa, czasu przejazdu i GPS. Zmiana samego obrazka nie zapewni zgodności z obliczeniami.
- Przy widoku całej siatki samochody mają mały rozmiar wynikający z proporcji 40×20 do świata 2380×1430. Detale są czytelniejsze po zoomie; nie powiększono aut niezależnie od dróg.
- Ruch na monitorze, skalowanie Windows 125–200%, długie obciążenie 100 aut z GPS/korkami oraz wrażenie płynności wymagają ręcznego sprawdzenia. Nie było skutecznego dostępu do natywnego okna przez narzędzie pulpitu.
- Zielona linia zgłoszona na niedostępnym zrzucie pozostaje niepotwierdzona.
- Nie ogłaszam spełnienia wszystkich kryteriów płynności całej symulacji. Naprawy kadru, wyspy, rysowania i zgodności stanów mają opisane wyżej dowody; ograniczenia silnika pozostają jawne.

## Uruchomienie i odtworzenie prób

Skrót `EasyRider.lnk` w katalogu projektu wskazuje na `build/visual-redesign/EasyRider.exe` i korzysta z `assets/easyrider.ico`.

W terminalu z Qt/MinGW w PATH:

```powershell
cmake --build build/visual-redesign
build/visual-redesign/EasyRiderTests.exe --headless
cmake -S tests/visual -B build/visual-check -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/visual-check
build/visual-check/EasyRiderVisualCheck.exe build/recovery-after
build/visual-check/EasyRiderViewProbe.exe --render
build/visual-check/EasyRiderViewProbe.exe --benchmark
build/visual-check/EasyRiderViewProbe.exe --timing --fast
build/visual-check/EasyRiderViewProbe.exe --record --detail build/recovery-motion
```

Baza porównania to lokalna kopia `build/recovery-before/src` i `ui`; projekt nie ma katalogu Git. Parametr CMake `EASYRIDER_SOURCE_DIR` pozwala zbudować `EasyRiderViewProbe` przeciwko tej kopii. Logi i obrazy pozostawiono w `build/recovery-*`.
