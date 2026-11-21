#pragma once
/**
 * @file Obb.h
 * @brief Obrócone prostokąty (obrys auta) — szybki test nakładania (SAT).
 *
 * Wspólne dla silnika (wykrywanie przeszkody na torze, siatka bezpieczeństwa)
 * i walidatora testowego. Auta NIE są punktami ani prostokątami osiowymi:
 * na łuku i na rondzie obrys jest obrócony zgodnie z kursem.
 */
#include <QPointF>
#include <array>
#include <cmath>
#include <algorithm>

namespace Obb {

using Quad = std::array<QPointF, 4>;

inline Quad corners(QPointF c, double h, double L, double W) {
    const double cs = std::cos(h), sn = std::sin(h);
    const QPointF ax(cs * L / 2, sn * L / 2), ay(-sn * W / 2, cs * W / 2);
    return { c + ax + ay, c + ax - ay, c - ax - ay, c - ax + ay };
}

/// Głębokość nakładania wzdłuż osi minimalnej (≤0 = rozłączne).
inline double penetration(const Quad& a, const Quad& b) {
    double best = 1e18;
    const Quad* polys[2] = { &a, &b };
    for (const Quad* p : polys)
        for (int i = 0; i < 2; ++i) {
            const QPointF e = (*p)[(i + 1) % 4] - (*p)[i];
            double nx = -e.y(), ny = e.x();
            const double len = std::hypot(nx, ny);
            if (len < 1e-12) continue;
            nx /= len; ny /= len;
            double amin = 1e18, amax = -1e18, bmin = 1e18, bmax = -1e18;
            for (const auto& q : a) { double t = q.x() * nx + q.y() * ny; amin = std::min(amin, t); amax = std::max(amax, t); }
            for (const auto& q : b) { double t = q.x() * nx + q.y() * ny; bmin = std::min(bmin, t); bmax = std::max(bmax, t); }
            const double ov = std::min(amax, bmax) - std::max(amin, bmin);
            if (ov <= 0) return ov;
            best = std::min(best, ov);
        }
    return best;
}

inline bool overlap(QPointF ca, double ha, double La, double Wa,
                    QPointF cb, double hb, double Lb, double Wb, double eps = 0.0) {
    return penetration(corners(ca, ha, La, Wa), corners(cb, hb, Lb, Wb)) > eps;
}

} // namespace Obb
