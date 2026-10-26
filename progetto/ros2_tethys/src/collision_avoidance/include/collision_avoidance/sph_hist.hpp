#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <vector>

// ── Dimensioni griglia sferica ────────────────────────────────────────────────
static constexpr int    N_AZ       = 36;
static constexpr int    N_EL       = 18;
static constexpr double AZ_RES     = 2.0 * M_PI / N_AZ;
static constexpr double EL_RES     = M_PI / N_EL;
static constexpr double EL_MIN     = -M_PI / 2.0;

// ── Limiti portata sonar ──────────────────────────────────────────────────────
static constexpr double RANGE_MAX  = 35.0;
static constexpr double RANGE_MIN  = 0.08;
static constexpr double SONAR_STALE = 1.5;   // secondi oltre cui il dato è vecchio

// ── Identificatori sensori ────────────────────────────────────────────────────
enum SensorId { FRONT = 0, LEFT, RIGHT, BOTTOM, TOP, N_SENSORS };

// ─────────────────────────────────────────────────────────────────────────────
// SphHist — istogramma sferico azimuth × elevation
//
// Per ogni settore (k, l):
//   h[k][l]     = densità ostacoli (somma pesi 1/r²)
//   min_r[k][l] = distanza minima rilevata in quel settore (m)
// ─────────────────────────────────────────────────────────────────────────────
struct SphHist
{
    std::array<std::array<double, N_EL>, N_AZ> h{};
    std::array<std::array<double, N_EL>, N_AZ> min_r{};
    double last_time{-1.0};
    bool   received{false};
    // Fresh surface returns in the level navigation frame, relative to the
    // current hull origin. Preserve their position for geometric clearance:
    // quantizing azimuth into 10-degree bins can invent or conceal a collision.
    // Forward, lateral and uncertainty from half the lidar angular spacing.
    std::vector<std::array<double, 3>> horizontal_points;
    bool horizontal_points_valid{false};

    void reset()
    {
        for (auto &row : h)     row.fill(0.0);
        for (auto &row : min_r) row.fill(99.0);
        horizontal_points.clear();
        horizontal_points_valid = false;
    }

    // Indice azimuth: az ∈ [-π, π] → k ∈ [0, N_AZ-1]
    // k=0 → az=-180°, k=18 → az=0° (avanti)
    static int az_idx(double az)
    {
        int k = static_cast<int>((az + M_PI) / AZ_RES);
        return std::clamp(k, 0, N_AZ - 1);
    }

    // Indice elevazione: el ∈ [-π/2, π/2] → l ∈ [0, N_EL-1]
    // l=0 → el=-90° (giù), l=9 → el=0° (orizzonte)
    static int el_idx(double el)
    {
        int l = static_cast<int>((el - EL_MIN) / EL_RES);
        return std::clamp(l, 0, N_EL - 1);
    }

    // Angolo al centro del settore k (azimuth) o l (elevazione)
    static double az_center(int k) { return -M_PI + (k + 0.5) * AZ_RES; }
    static double el_center(int l) { return  EL_MIN + (l + 0.5) * EL_RES; }

    // Smussamento separabile 2D: kernel [0.25, 0.50, 0.25].
    // Az: wrap circolare (periodicità). El: clamp ai poli.
    // min_r NON viene diffuso in elevazione per evitare contaminazione
    // dal fondale (l=7) sui settori frontali (l=9).
    static void smooth(const SphHist &raw, SphHist &out)
    {
        static constexpr double K[3] = {0.25, 0.5, 0.25};

        std::array<std::array<double, N_EL>, N_AZ> tmp{};
        for (int k = 0; k < N_AZ; ++k)
            for (int l = 0; l < N_EL; ++l)
                for (int dk = -1; dk <= 1; ++dk)
                    tmp[k][l] += K[dk+1] * raw.h[(k + dk + N_AZ) % N_AZ][l];

        out.reset();
        for (int k = 0; k < N_AZ; ++k) {
            for (int l = 0; l < N_EL; ++l) {
                double s = 0.0, w = 0.0;
                for (int dl = -1; dl <= 1; ++dl) {
                    int l2 = l + dl;
                    if (l2 < 0 || l2 >= N_EL) continue;
                    s += K[dl+1] * tmp[k][l2];
                    w += K[dl+1];
                }
                out.h[k][l]     = (w > 0.0) ? s / w : tmp[k][l];
                out.min_r[k][l] = raw.min_r[k][l];
            }
        }
        out.last_time = raw.last_time;
        out.received  = raw.received;
        out.horizontal_points = raw.horizontal_points;
        out.horizontal_points_valid = raw.horizontal_points_valid;
    }

    // Trasformazione sensor frame → body frame.
    // Le formule tengono conto della rotazione base SDF di -90° del modello tethys.
    //   frontal : pose(-1,0,0, 0,0,π)    → (x,y,z)→( x, y, z)
    //   left    : pose(0,-0.25,0,0,0,-π/2) → (-y, x, z)
    //   right   : pose(0,0.25,0,0,0,+π/2)  → ( y,-x, z)
    //   bottom  : pose(0,0,-0.15,0,π/2,0)  → (-z,-y,-x)
    //   top     : pose(0,0,0.15,0,-π/2,0)  → ( z,-y, x)
    static void to_body(double xs, double ys, double zs, SensorId id,
                        double &xb, double &yb, double &zb)
    {
        switch (id) {
            case FRONT:  xb= xs; yb= ys; zb= zs; break;
            case LEFT:   xb=-ys; yb= xs; zb= zs; break;
            case RIGHT:  xb= ys; yb=-xs; zb= zs; break;
            case BOTTOM: xb=-zs; yb=-ys; zb=-xs; break;
            case TOP:    xb= zs; yb=-ys; zb= xs; break;
            default:     xb= xs; yb= ys; zb= zs; break;
        }
    }
};
