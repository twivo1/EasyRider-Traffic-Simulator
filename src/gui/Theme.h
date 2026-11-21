#pragma once
#include <QColor>
#include <QString>
#include <vector>

/// Flat map palettes; identifiers and selection order remain stable.
struct MapTheme {
    QString id, name, emoji;
    QColor grassTop, grassBottom, road, roadEdge, laneMark, stopLine;
    QColor island, islandEdge, carBody, carAccent, smartCar;
    bool night = false;
};
inline std::vector<MapTheme> allThemes() {
    struct Palette { const char *id, *name, *ground, *road, *island, *car; bool dark; };
    const Palette palettes[] = {
        {"summer", "Techniczny", "#242B2F", "#515C62", "#30393D", "#ADCBD4", false},
        {"spring", "Wiosna", "#E4E6DC", "#555B5D", "#C4CEB6", "#AFCDCF", false},
        {"autumn", "Jesień", "#E4DDD1", "#595957", "#C6B69E", "#B8C9DA", false},
        {"night", "Noc", "#303638", "#484F52", "#667367", "#B5CDDA", true},
        {"neon", "Kontrastowy", "#25292B", "#50575B", "#7B8479", "#E2EBEE", true},
        {"gloom", "Szaruga", "#D5D8D7", "#535B60", "#B0BAB3", "#B8CEDC", false},
        {"winter", "Zima", "#E8ECEB", "#586269", "#CFD8D4", "#B4CFDE", false}
    };
    std::vector<MapTheme> result;
    for (const auto& p : palettes) {
        MapTheme m;
        m.id = p.id; m.name = QString::fromUtf8(p.name);
        m.grassTop = m.grassBottom = QColor(p.ground);
        m.road = QColor(p.road); m.roadEdge = QColor("#819096");
        m.laneMark = QColor("#CBD2D2"); m.stopLine = QColor("#F4F3E9");
        m.island = QColor(p.island); m.islandEdge = QColor("#819096");
        m.carBody = QColor(p.car); m.carAccent = QColor("#33454F");
        m.smartCar = QColor("#F0AE57"); m.night = p.dark;
        result.push_back(m);
    }
    return result;
}
inline MapTheme defaultTheme() { return allThemes().front(); }
