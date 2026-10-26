#include "collision_avoidance/vfh_planner.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

// ─────────────────────────────────────────────────────────────────────────────

bool VFHPlanner::forward_blocked() const
{
    if (!sm_.received) return false;
    int az0 = SphHist::az_idx(0.0);
    int el0 = SphHist::el_idx(0.0);
    for (int dk = -3; dk <= 3; ++dk)
        for (int dl = -FWD_DL; dl <= FWD_DL; ++dl) {
            int l = el0 + dl;
            if (l < 0 || l >= N_EL) continue;
            int k = (az0 + dk + N_AZ) % N_AZ;
            double r   = sm_.min_r[k][l];
            double lat = std::abs(r * std::sin(SphHist::az_center(k)));
            if (lat < COLLISION_WIDTH && sm_.h[k][l] > HIST_THRESH) return true;
            if (lat < COLLISION_WIDTH && r < BLOCKED_DIST)           return true;
            if (lat < BLOCKED_WIDTH   && r < WIDE_APPROACH_DIST)    return true;
        }
    return false;
}

double VFHPlanner::forward_min_range() const
{
    if (!sm_.received) return 99.0;
    int az0 = SphHist::az_idx(0.0);
    int el0 = SphHist::el_idx(0.0);
    double mn = 99.0;
    for (int dk = -3; dk <= 3; ++dk)
        for (int dl = -FWD_DL; dl <= FWD_DL; ++dl) {
            int l = el0 + dl;
            if (l < 0 || l >= N_EL) continue;
            mn = std::min(mn, sm_.min_r[(az0 + dk + N_AZ) % N_AZ][l]);
        }
    return mn;
}

double VFHPlanner::wall_dist_in_dir(double world_yaw) const
{
    // Convenzione world standard:
    // yaw = 0      -> +X
    // yaw = +pi/2  -> +Y
    double wx = std::cos(world_yaw);
    double wy = std::sin(world_yaw);

    double d = 99.0;

    // Parete Nord (+Y)
    if (wy > 0.01)
        d = std::min(d, (wall_wy_ - state_.y) / wy);

    // Parete Sud (-Y)
    if (wy < -0.01)
        d = std::min(d, (-wall_wy_ - state_.y) / wy);

    // Parete Est (+X)
    if (wx > 0.01)
        d = std::min(d, (wall_wx_ - state_.x) / wx);

    // Parete Ovest (-X)
    if (wx < -0.01)
        d = std::min(d, (-wall_wx_ - state_.x) / wx);

    return std::max(d, 0.0);
}

double VFHPlanner::wall_fwd_dist() const
{
    return wall_dist_in_dir(state_.yaw);
}

void VFHPlanner::compute_az_min(std::array<double, N_AZ> &az_min) const
{
    for (int k = 0; k < N_AZ; ++k) {
        az_min[k] = 99.0;
        for (int l = AZ_EL_LO; l < AZ_EL_HI; ++l)
            az_min[k] = std::min(az_min[k], sm_.min_r[k][l]);
    }
}

bool VFHPlanner::find_escape(double &best_az, double preferred_az)
{
    if (!sm_.received) return false;

    std::array<double, N_AZ> az_min;
    compute_az_min(az_min);

    // Settore bloccato se c'è qualcosa entro DECEL_DIST.
    auto sector_has_wall = [&](int k) -> bool {
        return az_min[k] < DECEL_DIST;
    };

    // DEBUG: stampa az_min per settori k=8..28 (zona muro frontale)
    static int fe_call = 0;
    if ((fe_call++ % 5) == 0) {
        fprintf(stderr, "[FE_INFO] pref=%.0f° yaw=%.3f xy=(%.1f,%.1f)\n",
                preferred_az*180.0/M_PI, state_.yaw, state_.x, state_.y);
        for (int k = 8; k <= 28; ++k) {
            if (az_min[k] < 90.0)
                fprintf(stderr, "  k%d(%.0f°)=%.1fm%s\n",
                        k, SphHist::az_center(k)*180.0/M_PI, az_min[k],
                        sector_has_wall(k)?" WALL":"");
        }
    }

    // Settore completamente libero: non bloccato, non near_obs, non verso muro fisico.
    auto is_free = [&](int k) -> bool {
        if (sector_has_wall(k)) return false;
        for (int dk = -2; dk <= 2; ++dk) {
            if (sector_has_wall((k + dk + N_AZ) % N_AZ)) return false;
        }
        return wall_dist_in_dir(normalize(state_.yaw + SphHist::az_center(k))) > BRAKE_DIST + 1.0;
    };

    // Valley selection: scansione da preferred_az verso DESTRA e SINISTRA,
    // prende il primo settore libero su ciascun lato, sceglie il più vicino.
    // Più robusto del best-score globale perché trova sempre il bordo del gap,
    // non l'apertura "dietro" il muro che appare libera per distanza > sonar range.
    int pref_k = SphHist::az_idx(preferred_az);

    for (int pass = 0; pass < 2; ++pass) {
        int right_k = -1, left_k = -1;

        for (int dk = 0; dk < N_AZ; ++dk) {
            if (right_k < 0) {
                int k = (pref_k - dk + N_AZ) % N_AZ;
                double az = SphHist::az_center(k);
                if (pass == 0 && std::abs(normalize(az)) > M_PI / 2.0) {/* skip */}
                else if (is_free(k)) right_k = k;
            }
            if (left_k < 0) {
                int k = (pref_k + dk + N_AZ) % N_AZ;
                double az = SphHist::az_center(k);
                if (pass == 0 && std::abs(normalize(az)) > M_PI / 2.0) {/* skip */}
                else if (is_free(k)) left_k = k;
            }
            if (right_k >= 0 && left_k >= 0) break;
        }

        if (right_k < 0 && left_k < 0) continue;

        int chosen_k;
        if (right_k < 0)     chosen_k = left_k;
        else if (left_k < 0) chosen_k = right_k;
        else {
            double re = std::abs(normalize(SphHist::az_center(right_k) - preferred_az));
            double le = std::abs(normalize(SphHist::az_center(left_k)  - preferred_az));
            // Bias DESTRA di 0.10 rad (≈6°): quando le due opzioni sono quasi equidistanti
            // dal goal, preferisce DESTRA. Evita che piccole variazioni floating-point di
            // preferred_az (es. 0.0008 rad da yaw noise) invertano la scelta.
            chosen_k = (re <= le + 0.10) ? right_k : left_k;
        }

        best_az = SphHist::az_center(chosen_k);
        fprintf(stderr, "[FE] pass=%d rk=%d(%.0f°) lk=%d(%.0f°) → k=%d(%.0f°) pref=%.0f°\n",
                pass,
                right_k, right_k>=0?SphHist::az_center(right_k)*180.0/M_PI:0.0,
                left_k,  left_k>=0?SphHist::az_center(left_k)*180.0/M_PI:0.0,
                chosen_k, best_az*180.0/M_PI, preferred_az*180.0/M_PI);
        return true;
    }
    return false;
}

double VFHPlanner::corridor_clearance(double candidate_az) const
{
    if (!sm_.received) return 99.0;

    if (sm_.horizontal_points_valid) {
        double clearance = wall_dist_in_dir(normalize(state_.yaw + candidate_az));
        const double c = std::cos(candidate_az), s = std::sin(candidate_az);
        for (const auto &point : sm_.horizontal_points) {
            const double fwd = c*point[0] + s*point[1];
            const double lateral = -s*point[0] + c*point[1];
            const double half_width = CORRIDOR_HALF_WIDTH + point[2];
            if (fwd > 0.0 && fwd < LOOKAHEAD_DIST && std::abs(lateral) < half_width)
                clearance = std::min(clearance, fwd);
        }
        return clearance;
    }

    // Conservative fallback for histogram-only callers.
    std::array<double, N_AZ> az_min;
    compute_az_min(az_min);

    double min_fwd = 99.0;
    for (int k = 0; k < N_AZ; ++k) {
        double d = az_min[k];
        if (d >= RANGE_MAX * 0.95) continue;
        double beta  = SphHist::az_center(k);
        // A return may lie anywhere in this 10-degree bin. Using only its
        // centre moves a straight-ahead return sideways by 1.74 m at 20 m.
        double delta = std::max(0.0, std::abs(normalize(beta - candidate_az)) - AZ_RES / 2.0);
        double fwd   = d * std::cos(delta);
        double lat   = std::abs(d * std::sin(delta));
        if (fwd > 0.0 && fwd < LOOKAHEAD_DIST && lat < CORRIDOR_HALF_WIDTH)
            min_fwd = std::min(min_fwd, fwd);
    }
    // Parete del tank nella direzione candidate
    double world_az = normalize(state_.yaw + candidate_az);
    min_fwd = std::min(min_fwd, wall_dist_in_dir(world_az));
    return min_fwd;
}

double VFHPlanner::local_waypoint_length(double bearing, double clearance) const
{
    double length = std::clamp(clearance-2.0, 5.0, 12.0);
    if (forward_min_range() < 10.0 && std::abs(bearing) > 0.35)
        length = std::min(length, 6.0);
    return length;
}

bool VFHPlanner::waypoint_arc_free(double bearing, double length) const
{
    if (!sm_.horizontal_points_valid) return true;
    if (std::abs(bearing) >= M_PI/2.0) return false;
    // A circular entry tangent to the current heading and ending at the
    // waypoint has radius L/(2 sin(bearing)) and sweeps twice the bearing.
    // Runs30-39 at forward speed0.8-1.5 and rudder0.43-0.46 give a median
    // measured radius4.62m (129 samples); use5m as the planning lower bound.
    const double radius = std::abs(bearing) > 1e-5
        ? length/(2.0*std::sin(bearing)) : 1e9;
    if (std::abs(radius) < 5.0) return false;
    const double arc_length = std::abs(bearing) > 1e-5
        ? std::abs(2.0*bearing*radius) : length;
    const int samples = std::max(1, static_cast<int>(std::ceil(arc_length/0.25)));
    const double c = std::cos(state_.yaw), s = std::sin(state_.yaw);
    for (int i=1; i<=samples; ++i) {
        const double fraction = static_cast<double>(i)/samples;
        const double angle = 2.0*bearing*fraction;
        const double x = std::abs(bearing) > 1e-5 ? radius*std::sin(angle) : length*fraction;
        const double y = std::abs(bearing) > 1e-5 ? radius*(1.0-std::cos(angle)) : 0.0;
        const double wx = state_.x+c*x-s*y, wy = state_.y+s*x+c*y;
        if (std::abs(wx) > wall_wx_-CORRIDOR_HALF_WIDTH
            || std::abs(wy) > wall_wy_-CORRIDOR_HALF_WIDTH) return false;
        for (const auto &point : sm_.horizontal_points) {
            const double margin = CORRIDOR_HALF_WIDTH + point[2] + 0.125;
            if (std::hypot(point[0]-x, point[1]-y) < margin) return false;
        }
    }
    return true;
}

bool VFHPlanner::find_lookahead_escape(double &best_az, double preferred_az,
                                       double *path_len_out)
{
    if (!sm_.received) return false;

    // Distanza minima frontale corrente (per bonus laterale quando muro vicino)
    double fwd_now    = forward_min_range();
    bool   wall_close = (fwd_now < DECEL_DIST * 0.55);  // < ~17m

    double best_score   = -1e9;
    double best_min_fwd = 0.0;  // clearance della direzione scelta → distanza waypoint locale
    bool   found        = false;

    // Campiona candidati da -100° a +100° con passo 5°
    for (int deg = -100; deg <= 100; deg += 5) {
        double cand = deg * M_PI / 180.0;

        // Penalizza fortemente il dietrofront (> ±162°)
        bool behind = (std::abs(cand) >= 0.9 * M_PI);

        // Valuta clearance del corridoio fisico in direzione cand (body frame)
        double min_fwd = corridor_clearance(cand);

        if (min_fwd < ESCAPE_CLEAR) continue;  // corridoio non sicuro
        const double waypoint_length = local_waypoint_length(cand, min_fwd);
        if (!waypoint_arc_free(cand, waypoint_length)) continue;

        // Reward clearance only inside the planning horizon. Far tank walls
        // must not pull a clear path away from the goal; obstacles within the
        // horizon still justify an early detour instead of a late brake.
        double score = std::min(min_fwd, LOOKAHEAD_DIST) * 2.0;

        // Forte preferenza verso la direzione goal/preferred: decisiva
        double dist_from_pref = std::abs(normalize(cand - preferred_az));
        // Linear angular error has no dead zone near goal. Previously a clear
        // tank with preferred=10 degrees selected 35 degrees because the far
        // wall distance outweighed the goal term.
        // Prefer a small detour with 25 m clearance over a much wider turn
        // chosen only for extra space beyond it. The latter repeatedly sent
        // the AUV toward a side wall in the denser corridor.
        score -= dist_from_pref * 40.0;

        // Penalità per deviazione dall'heading corrente: preferisce dritto/leggere correzioni
        score -= std::abs(cand) * 2.0;

        // Penalità dietrofront
        if (behind) score -= 40.0;

        if (score > best_score) {
            best_score   = score;
            best_az      = cand;
            best_min_fwd = min_fwd;
            found        = true;
        }
    }

    if (found) {
        if (path_len_out) *path_len_out = best_min_fwd;
        fprintf(stderr, "[LH] pref=%.0f° best=%.0f° score=%.1f clr=%.1f fwd_now=%.1f wall_close=%d\n",
                preferred_az * 180.0 / M_PI, best_az * 180.0 / M_PI,
                best_score, best_min_fwd, fwd_now, wall_close ? 1 : 0);
    } else {
        fprintf(stderr, "[LH] nessuna direzione libera pref=%.0f°\n",
                preferred_az * 180.0 / M_PI);
    }
    return found;
}

bool VFHPlanner::find_vertical_escape(double &new_depth, int *el_span_out,
                                      double range_thresh)
{
    if (!sm_.received) return false;
    const int AZ_FWD = SphHist::az_idx(0.0);

    double fwd_el[N_EL];
    for (int l = 0; l < N_EL; ++l) {
        fwd_el[l] = 99.0;
        for (int dk = -3; dk <= 3; ++dk)
            fwd_el[l] = std::min(fwd_el[l],
                sm_.min_r[(AZ_FWD + dk + N_AZ) % N_AZ][l]);
    }

    int el_span = 0;
    for (int l = AZ_EL_LO; l < N_EL; ++l)
        if (fwd_el[l] < range_thresh) ++el_span;
    if (el_span_out) *el_span_out = el_span;

    int blocked_l = SphHist::el_idx(0.0);
    double min_r_obs = 99.0;
    for (int l = AZ_EL_LO; l < N_EL; ++l)
        if (fwd_el[l] < min_r_obs) { min_r_obs = fwd_el[l]; blocked_l = l; }
    if (min_r_obs > range_thresh) return false;

    // Stima z dell'ostacolo con correzione pitch del drone.
    // Fix 131: clamp pitch a ±0.3 rad — con pitch molto alto (es. 1.17 post-STABILIZE)
    // el_world_obs sarebbe sbagliato e la scelta UP/DOWN risulterebbe invertita.
    double pitch_clamped = std::clamp(state_.pitch, -0.3, 0.3);
    double el_world_obs = SphHist::el_center(blocked_l) + pitch_clamped;
    double obs_z = state_.z + min_r_obs * std::sin(el_world_obs);

    const double VERT_HALF = 18.0;
    double d_above = std::clamp(obs_z + VERT_HALF, 2.0, 30.0);
    double d_below = std::clamp(obs_z - VERT_HALF, 1.5, 30.0);

    bool ok_above = (d_above - state_.z) > 1.0 && d_above < 32.0;
    bool ok_below = (state_.z - d_below) > 1.0 && d_below > 1.5;

    // Vincolo basato sul segno dell'elevazione rilevata.
    // Soglia ±0.35 rad (20°): i muri visti quasi orizzontalmente (|el|<20°) non sopprimono
    // nessuna direzione — la scelta viene fatta dal clamp di d_above/d_below.
    if (el_world_obs < -0.35) ok_below = false;
    if (el_world_obs > +0.35) ok_above = false;

    if (!ok_above && !ok_below) return false;

    if (ok_above && ok_below)
        new_depth = ((d_above - state_.z) >= (state_.z - d_below)) ? d_above : d_below;
    else
        new_depth = ok_above ? d_above : d_below;
    return true;
}
