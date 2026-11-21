#pragma once
#include "../core/Roundabout.h"
#include <algorithm>
#include <cmath>

// Presentation footprint derived from the engine's existing path and body size.
// These values never feed back into the simulation or collision geometry.
namespace RoadVisuals {
inline constexpr double outerRadius = kRingR + kCarW/2 + 5;
inline double islandRadius() {
    static const double radius = [] {
        double clearance = kRingR-kCarW/2;
        for (double offset : {Rb::southOff, Rb::northMag}) {
            // At the straight/ring transition the rear (or front) inner corner
            // is closer to the island than a tangential body on the circle.
            const double along = std::sqrt(kRingR*kRingR-offset*offset)-kCarL/2;
            const double across = offset-kCarW/2;
            clearance = std::min(clearance, std::hypot(along,across));
        }
        return clearance-2; // includes a 1-unit rim and antialiasing clearance
    }();
    return radius;
}
}
