# Generowanie dokumentacji EasyRider

Cały kod posiada komentarze w stylu Doxygen (po polsku). Aby wygenerować
dokumentację HTML:

```bash
sudo apt install doxygen graphviz   # jednorazowo
doxygen Doxyfile
```

Dokumentacja pojawi się w `docs/html/index.html`.

## Struktura projektu

- `src/core/`  — silnik symulacji (Car, SimulationEngine, geometria, ronda)
- `src/nav/`   — nawigacja (A*, analiza ruchu, menedżer trasy)
- `src/gui/`   — interfejs (widok, renderery, motywy, menu, ekran startowy)
- `tests/`     — testy jednostkowe (aplikacja okienkowa EasyRiderTests)

## Budowanie

```bash
mkdir build && cd build
cmake ..
cmake --build .
./EasyRider        # główna aplikacja
./EasyRiderTests   # testy jednostkowe
```
