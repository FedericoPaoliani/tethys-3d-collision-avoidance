// CollisionAvoidanceNode: direzione preferita, waypoint locali, limite di sterzo.

#include "collision_avoidance/collision_avoidance_node.hpp"

// Direzione preferita per find_escape() in body frame:
// in goal mode = verso il target; in wander = heading di wander.
void CollisionAvoidanceNode::create_local_waypoint(double yaw_world, double clearance)
{
    double wp_len    = std::clamp(clearance - 2.0, 5.0, 12.0);
    // Vicino a un ostacolo con molta rotta da recuperare, un waypoint lungo cade
    // oltre il cilindro e il drone punta la' attraversandolo: prima si chiude la
    // curva, poi si ripunta il target.
    {
        double fwd_now     = planner_.forward_min_range();
        double yaw_err_now = std::abs(normalize(yaw_world - state_.yaw));
        if (mode_ == Mode::ESCAPE && !vertical_escape_
            && fwd_now < 10.0 && yaw_err_now > 0.35) {
            double old_len = wp_len;
            wp_len = planner_.local_waypoint_length(yaw_err_now, clearance);
            RCLCPP_WARN(get_logger(),
                "[WP-SHORT-CURVE] fwd=%.1f yaw_err=%.2f len %.1f→%.1f",
                fwd_now, yaw_err_now, old_len, wp_len);
        }
    }
    local_wp_x_      = state_.x + wp_len * std::cos(yaw_world);
    local_wp_y_      = state_.y + wp_len * std::sin(yaw_world);
    local_wp_active_ = true;
    wp_last_check_t_ = -1.0;
    wp_last_dist_    = 0.0;
    // Anti-chattering: ogni nuovo percorso locale apre una manovra di schivata che
    // deve poter essere eseguita prima che TARGET-REJOIN possa annullarla.
    double t_wp           = now().seconds();
    avoid_entry_t_        = t_wp;
    last_local_path_t_    = t_wp;
    rejoin_allowed_after_ = t_wp + 3.0;
    RCLCPP_INFO(get_logger(),
        "[LOCAL-PATH] yaw=%.0f° len=%.1f wp=(%.1f,%.1f) clr=%.1f",
        yaw_world * 180.0 / M_PI, wp_len, local_wp_x_, local_wp_y_, clearance);
}

// Crea un waypoint locale solo se non ne è stato creato uno da poco: impedisce che
// AVOID-MINIMAL/SIDE-AVOID rigenerino un percorso ad ogni tick (200 ms).
void CollisionAvoidanceNode::create_local_waypoint_throttled(double yaw_world, double clearance, double t)
{
    if ((t - last_local_path_t_) < 1.5 && local_wp_active_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[LOCAL-PATH-SKIP] recente %.1fs fa, mantengo wp attuale",
            t - last_local_path_t_);
        return;
    }
    create_local_waypoint(yaw_world, clearance);
}

// Limite di sterzo dinamico. Misura empirica del rateo d'imbardata in funzione della
// barra: 0.28→0.058 rad/s, 0.42→0.153, 0.44→0.171. L'angolo del giunto non è
// l'incidenza reale — la deriva laterale la sottrae — quindi serve barra ampia per
// virare stretto. Si stringe solo quando l'assetto è davvero compromesso.
double CollisionAvoidanceNode::steer_limit(double fwd_min) const
{
    if (std::abs(state_.pitch) > 0.45 || std::abs(state_.yaw_rate) > 1.5) return 0.30;
    if (fwd_min < BRAKE_DIST + 1.0) return 0.45;
    return 0.42;
}

double CollisionAvoidanceNode::preferred_dir() const
{
    if (goal_active_) {
        double dx = target_x_ - state_.x;
        double dy = target_y_ - state_.y;

        // Vera direzione geometrica verso il target.
        double goal_yaw = std::atan2(dy, dx);

        return normalize(goal_yaw - state_.yaw);
    }

    return normalize(wander_yaw_ - state_.yaw);
}

// Lookahead preferred direction: quando un ostacolo è vicino e blocca il percorso
// diretto al goal, bypassa la direzione goal (che porterebbe a un'altra collisione
// geometrica) e usa una direzione fortemente laterale verso il lato del gap.
//
// Logica: la parete ha un gap a destra o sinistra. Si sceglie il lato in base
// alla posizione X del goal rispetto alla posizione X attuale:
//   - goal_x > state_x + 3m  → gap probabilmente a sinistra (drone va a sinistra)
//   - goal_x < state_x - 3m  → gap probabilmente a destra  (drone va a destra)
//   - altrimenti              → default destra (labirinto alternato, prima parete)
//
// Fornisce preferred_az ≈ ±1.2 rad (body frame) così find_escape() sceglie una
// direzione ~55-65° laterale che supera geometricamente tutti i gap del labirinto.
double CollisionAvoidanceNode::lookahead_preferred_dir(double fwd_min) const
{
    double pref = preferred_dir();

    // Con lateral_gap_bias_=0 (default) non si applica: il bias fisso di 1.2 rad
    // spingeva il drone 69 gradi di lato ad OGNI schivata, facendolo derivare negli
    // angoli della vasca invece di puntare al bersaglio. Serve solo nel labirinto.
    if (!goal_active_ || lateral_gap_bias_ <= 0.0
        || fwd_min >= 25.0 || std::abs(pref) >= 0.6)
        return pref;

    // Scegli il lato laterale verso cui girare in base a dove si trova il goal in X
    double dx_goal = target_x_ - state_.x;
    double lateral;
    if (dx_goal > 3.0)
        lateral = +lateral_gap_bias_;   // goal a destra in X → gap a sinistra
    else
        lateral = -lateral_gap_bias_;   // goal a sinistra o allineato → gap a destra

    fprintf(stderr, "[LOOKAHEAD] fwd=%.1f dx_goal=%.1f lateral_bias=%.1f rad (pref was %.2f)\n",
            fwd_min, dx_goal, lateral, pref);

    return lateral;
}
