// CollisionAvoidanceNode: comandi agli attuatori (quota, repulsione, limiti, filtri).

#include "collision_avoidance/collision_avoidance_node.hpp"

// ── Pubblicazione comandi agli attuatori ──────────────────────────────────
void CollisionAvoidanceNode::stop_actuators()
{
    // Bypass depth control, wall repulsion and smoothing when observations
    // are missing/stale. A request for cmd(0,0,0) is not a neutral command
    // because cmd() normally adds those feedback terms.
    last_thr_cmd_ = last_az_cmd_ = last_el_cmd_ = 0.0;
    std_msgs::msg::Float64 zero;
    zero.data = 0.0;
    thr_pub_->publish(zero);
    vert_pub_->publish(zero);
    horiz_pub_->publish(zero);
}

void CollisionAvoidanceNode::cmd(double thr, double az, double el)
{
    // One depth law for every state. Navigation follows model -X, so
    // positive model pitch climbs in forward motion. A negative physical
    // elevator command raises that pitch (verified with the intended
    // Hydrodynamics configuration). Its initial heave has the opposite
    // sign: controlling that transient directly caused long-term runaway.
    if (pose_received_ && sonar_first_seen_) {
        const double reference = std::clamp(
            vertical_escape_ ? escape_depth_ : (goal_active_ ? target_z_ : wander_z_),
            3.0, depth_limit_high_ - 1.0);
        // Negative thrust first brakes forward motion: only measured
        // reverse flow changes the pitch/actuator conventions.
        const double flow = forward_speed_ < -0.15 ? -1.0 : 1.0;
        const double depth_t = now().seconds();
        const double trim = depth_trim_.update(reference-state_.z, state_.vz,
            state_.pitch, forward_speed_,
            !safety_braking_ && mode_ != Mode::BRAKE && !vertical_escape_,
            last_depth_control_t_ >= 0.0 ? depth_t-last_depth_control_t_ : 0.0);
        last_depth_control_t_ = depth_t;
        const double desired_pitch = std::clamp(
            flow*(0.12*(reference-state_.z) - 0.50*state_.vz
                + (flow > 0.0 && !vertical_escape_ ? trim : 0.0)), -0.30, 0.30);
        el = std::clamp(flow*1.2*(desired_pitch-state_.pitch), -0.25, 0.25);
        // The elevator's immediate heave opposes its steady pitch effect.
        // While stopping/backing slowly, forward flow disappears before
        // that pitch can produce the desired depth change. Repeated brakes
        // therefore pumped the vehicle upward with saturated down-pitch.
        // The corrected model is neutrally buoyant: coast with neutral fins
        // during the brake, then resume depth tracking with forward flow.
        if (safety_braking_ || mode_ == Mode::BRAKE) el = 0.0;
    }
    // ── Repulsione morbida dai confini della vasca ─────────────────────────
    // Si applica in tutti i modi (CRUISE/ESCAPE/BRAKE). Forza proporzionale
    // alla distanza: 0 alla soglia esterna, massima a contatto col bordo.
    // Non interferisce con DEPTH_LIMIT (che scatta prima) ma fa da rete di sicurezza.

    // Internal el>0 commands a climb after the pitch response. Both the
    // pitch needed for a climb and the fin response reverse with the flow.
    constexpr double REPEL_FLOOR = 4.0;
    constexpr double REPEL_CEIL_MARGIN = 4.0;
    // Fix 87: ceil_soft non può scendere sotto target_z+1.5m.
    // Bug: con depth_limit_high=12 e target_z=10 → ceil_soft=8m → il soffitto
    // soft lottava contro il controller di profondità a z=10m (bias el=-0.20 DOWN).
    double ceil_soft = depth_limit_high_ - REPEL_CEIL_MARGIN;
    if (goal_active_)
        ceil_soft = std::max(ceil_soft, target_z_ + 1.5);
    if (state_.z < REPEL_FLOOR) {
        double bias = (REPEL_FLOOR - state_.z) / REPEL_FLOOR * 0.40;
        el = std::max(el, +bias);
    }
    if (state_.z > ceil_soft) {
        double bias = (state_.z - ceil_soft) / REPEL_CEIL_MARGIN * 0.40;
        el = std::min(el, -bias);
    }

    // Muri laterali: az bias proporzionale verso l'interno (entro 5m)
    constexpr double REPEL_WALL = 5.0;
    double wall_left  = planner_.wall_dist_in_dir(state_.yaw + M_PI / 2.0);
    double wall_right = planner_.wall_dist_in_dir(state_.yaw - M_PI / 2.0);
    if (wall_left  < REPEL_WALL) az -= (REPEL_WALL - wall_left)  / REPEL_WALL * 0.15;
    if (wall_right < REPEL_WALL) az += (REPEL_WALL - wall_right) / REPEL_WALL * 0.15;

    // Also cover reverse commands from legacy BRAKE / quota recovery paths.
    bool stopping_reverse = false;
    if (thr < 0.0 && forward_speed_ < 0.1) {
        const double back = std::min(planner_.wall_dist_in_dir(state_.yaw + M_PI),
            obstacle_memory_.clearance(state_.x, state_.y, state_.z,
                                        state_.yaw + M_PI, now().seconds()));
        const double rear_stop = rear_stopping_distance(forward_speed_);
        if (back < rear_stop) {
            stopping_reverse = true;
            thr = forward_speed_ < -0.1 ? 50.0 : 0.0;
            az = 0.0;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[REAR-BRAKE] observed=%.2f stop=%.2f speed=%.2f thr=%.1f",
                back, rear_stop, forward_speed_, thr);
        }
    }

    // With hydrodynamics now active, a fixed TURN_THR is not a speed limit:
    // it kept accelerating through a turn. Limit forward effort using
    // measured speed, while retaining the requested effort below the limit.
    if (goal_active_ && !vertical_escape_ && thr > 0.0
        && !safety_braking_ && !stopping_reverse && forward_speed_ >= 0.0) {
        const double turning = std::min(1.0, std::abs(az)/0.45);
        const double desired_speed = 1.5 - 0.4*turning;
        const double speed_effort = std::clamp(7.0*desired_speed*desired_speed
            + 35.0*(desired_speed-forward_speed_), 0.0, 42.0);
        thr = std::min(thr, speed_effort);
    }

    // Low-pass sui comandi: smorza le correzioni a gradino e rende le manovre fluide.
    // Bypassato nelle emergenze reali (retromarcia, BRAKE, limiti di quota, recupero
    // assetto), dove il ritardo introdotto dal filtro sarebbe pericoloso.
    bool bypass_smooth = stopping_reverse || (mode_ == Mode::BRAKE) || (thr < 0.0) || alt_recovery_active_ ||
                         state_.z < 2.5 || state_.z > depth_limit_high_;
    double a_cmd = bypass_smooth ? 1.00 : 0.35;
    double a_thr = (bypass_smooth || thr == 0.0) ? 1.00 : 0.25;
    az  = last_az_cmd_  + a_cmd * (az  - last_az_cmd_);
    el  = last_el_cmd_  + a_cmd * (el  - last_el_cmd_);
    thr = last_thr_cmd_ + a_thr * (thr - last_thr_cmd_);
    last_az_cmd_  = az;
    last_el_cmd_  = el;
    last_thr_cmd_ = thr;

    std_msgs::msg::Float64 m;
    m.data = std::clamp(az, -0.50, 0.50);
    vert_pub_->publish(m);
    // Joint limits in the model are +/-15 degrees.
    m.data = std::clamp(-el, -0.261799, 0.261799);
    horiz_pub_->publish(m);
    m.data = thr;
    thr_pub_->publish(m);
}
