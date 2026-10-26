#pragma once

#include "sph_hist.hpp"
#include "vehicle_state.hpp"
#include <array>
#include <cmath>

// ── Costanti operative VFH ────────────────────────────────────────────────────
static constexpr double HIST_THRESH        = 2.5;
static constexpr double BRAKE_DIST         = 6.0;   // Fix 82: 5→6 — più margine prima degli ostacoli
static constexpr double BLOCKED_DIST       = 16.0;
static constexpr double CLEARED_DIST       = 17.0;
static constexpr double ESCAPE_CLEAR       = 11.0;  // Fix 82: 9→11 — richiede più spazio libero per via di fuga
static constexpr double CRUISE_THR         = 42.0;   // 35→42: più flusso sulle pinne (portanza ∝ V²) e crociera più spedita
static constexpr double TURN_THR           = 25.0;   // 20→25: a 20N le pinne non avevano autorità sufficiente per virare
static constexpr double TURN_BOOST_THR     = 50.0;  // spinta alta durante svolta profonda: R=V/ω∝1/V → R piccolo
static constexpr double APPROACH_DIST      = 8.0;
static constexpr double WIDE_APPROACH_DIST = 10.0;
static constexpr double DECEL_DIST         = 32.0;   // Fix 111: 28→32 — inizia a decelerare prima dalle pareti
static constexpr double COLLISION_WIDTH    = 2.0;    // Fix 82: 1.55→2.0 — span reale pinne ~1.3m; 0.35m margine/lato
static constexpr double BUOY_COMP_THR     = 30.0;   // Fix 66: 50→30 — permette frenata reale sotto 50N in goal mode
static constexpr double BLOCKED_WIDTH      = 2.5;   // Fix 82: 2.0→2.5 — coerente con COLLISION_WIDTH
static constexpr double LOOKAHEAD_DIST     = 30.0;  // profondità corridoio lookahead (m)
static constexpr double LOOKAHEAD_WIDTH    = 3.5;   // semi-larghezza corridoio lookahead (m)
static constexpr double CORRIDOR_HALF_WIDTH     = 1.5;  // semi-larghezza corridoio fisico drone (m)
static constexpr double SAFE_PASS_DIST          = 25.0;  // distanza min per passaggio sicuro → trigger ESCAPE
static constexpr double EMERGENCY_COLLISION_DIST = 3.0;  // true emergency: fwd_min < 3m → BRAKE in ogni caso

static constexpr int FWD_DL   = 1;   // ±1 settore el per forward check (±10°)
static constexpr int AZ_EL_LO = 8;   // limite inferiore elevazione per az_min (el=−10°)
static constexpr int AZ_EL_HI = 12;  // limite superiore (escluso) az_min (el=+20°)

// Normalizza un angolo nell'intervallo (-π, +π].
inline double normalize(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─────────────────────────────────────────────────────────────────────────────
// VFHPlanner — pianificatore locale VFH* 3D
//
// Riceve per riferimento const l'istogramma sferico smussato (sm) e lo stato
// cinematico del veicolo (state). Non possiede dati propri: è un puro motore
// di pianificazione interrogabile a ogni tick di controllo.
// ─────────────────────────────────────────────────────────────────────────────
class VFHPlanner
{
public:
    // wall_wx/wy: semi-larghezza vasca in X e Y (default 79.75 = vasca 160×160 m).
    VFHPlanner(const SphHist &sm, const VehicleState &state,
               double wall_wx = 79.75, double wall_wy = 79.75)
        : sm_(sm), state_(state), wall_wx_(wall_wx), wall_wy_(wall_wy) {}

    void set_walls(double wx, double wy) { wall_wx_ = wx; wall_wy_ = wy; }
    void set_inner_walls(double wx, double wy) { inner_wx_ = wx; inner_wy_ = wy; }

    // Distanza analitica alla parete del cubo interno nella direzione world_yaw.
    double inner_wall_dist_in_dir(double world_yaw) const {
        double wx = std::cos(world_yaw), wy = std::sin(world_yaw);
        double d = 99.0;
        if (wy >  0.01) d = std::min(d, ( inner_wy_ - state_.y) / wy);
        if (wy < -0.01) d = std::min(d, (-inner_wy_ - state_.y) / wy);
        if (wx >  0.01) d = std::min(d, ( inner_wx_ - state_.x) / wx);
        if (wx < -0.01) d = std::min(d, (-inner_wx_ - state_.x) / wx);
        return std::max(d, 0.0);
    }

    // Cono ±30° az × ±10° el con filtro corridoio fisico.
    // Tre trigger: densità alta nel corridoio stretto, range stretto < 12 m,
    // range largo < 10 m (per anticipare ostacoli off-axis).
    bool forward_blocked() const;

    bool waypoint_arc_free(double bearing, double length) const;
    double local_waypoint_length(double bearing, double clearance) const;

    // Cono ±30° az × ±10° el senza filtro corridoio: distanza minima reale.
    double forward_min_range() const;

    // Distanza analitica alla parete in una direzione world yaw arbitraria.
    // Vasca 70×70 m, pareti a ±34.75 m.
    double wall_dist_in_dir(double world_yaw) const;

    // Wrapper per la direzione di avanzamento corrente.
    double wall_fwd_dist() const;

    // Cerca la migliore direzione libera (solo azimuth).
    // Passata 1: semispazio frontale (|az|<90°).
    // Passata 2 (fallback): cerchio completo.
    // Ritorna true se trovata; best_az è relativo al body frame.
    // preferred_az: direzione preferita in body frame (0 = heading corrente);
    //   lo scoring minimizza |az − preferred_az| anziché |az|.
    bool find_escape(double &best_az, double preferred_az = 0.0);

    // Lookahead planner: campiona direzioni candidate (-100°..+100°, passo 5°),
    // valuta corridoio futuro LOOKAHEAD_DIST × CORRIDOR_HALF_WIDTH per ognuna,
    // sceglie la direzione con punteggio migliore (clearance + allineamento goal).
    // path_len_out (opzionale): clearance del corridoio scelto → usata per waypoint locale.
    // Ritorna true se trovata almeno una direzione sicura.
    bool find_lookahead_escape(double &best_az, double preferred_az = 0.0,
                               double *path_len_out = nullptr);

    // Distanza al primo ostacolo fisicamente nel corridoio del drone nella direzione
    // candidate_az (body frame). Usa CORRIDOR_HALF_WIDTH × LOOKAHEAD_DIST.
    // Ritorna 99.0 se il corridoio è libero, altrimenti la distanza all'ostacolo.
    double corridor_clearance(double candidate_az) const;

    // Cerca una quota di sicurezza sopra/sotto un ostacolo orizzontale.
    // el_span_out: bande di elevazione con ostacolo < range_thresh
    //   (piccolo → trave/sfera, grande → pilastro/parete).
    // range_thresh: BLOCKED_DIST per ESCAPE, DECEL_DIST per PRE-VERT in CRUISE.
    bool find_vertical_escape(double &new_depth, int *el_span_out = nullptr,
                              double range_thresh = BLOCKED_DIST);

private:
    // Collassa min_r sui livelli AZ_EL_LO..AZ_EL_HI per ogni azimuth.
    void compute_az_min(std::array<double, N_AZ> &az_min) const;

    const SphHist      &sm_;
    const VehicleState &state_;
    double wall_wx_{79.75};   // semi-larghezza vasca in X [m]
    double wall_wy_{79.75};   // semi-larghezza vasca in Y [m]
    double inner_wx_{60.0};   // cubo interno semi-larghezza X [m]
    double inner_wy_{60.0};   // cubo interno semi-larghezza Y [m]
};
