// CollisionAvoidanceNode: costruttore (parametri, sottoscrizioni, publisher, timer) e ciclo di controllo.

#include "collision_avoidance/collision_avoidance_node.hpp"

using namespace std::chrono_literals;

CollisionAvoidanceNode::CollisionAvoidanceNode()
    : Node("collision_avoidance_vfh"),
      planner_(sm_, state_)        // planner_ tiene riferimenti a sm_ e state_; wall_wx/wy aggiornati sotto
{
    // ── Parametri ─────────────────────────────────────────────────────────
    declare_parameter<std::string>("world_name",  "unknown");
    declare_parameter<int>        ("n_obstacles", 0);
    declare_parameter<std::string>("csv_path",    "");
    declare_parameter<std::string>("drone_name",  "tethys");
    declare_parameter<double>     ("target_x",   std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("target_y",   std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("target_z",   std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("goal_radius",      3.0);
    declare_parameter<double>     ("yaw_bias",         0.0);
    declare_parameter<double>     ("depth_limit_high", 30.0);  // soglia soffitto DEPTH_LIMIT
    declare_parameter<double>     ("spawn_x",  std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("spawn_y",  std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("spawn_z",  std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>     ("wall_wx",  79.75);  // semi-larghezza vasca X [m]
    declare_parameter<double>     ("wall_wy",  79.75);  // semi-larghezza vasca Y [m]
    declare_parameter<double>     ("inner_wx", 60.0);   // cubo interno semi-larghezza X [m]
    declare_parameter<double>     ("inner_wy", 60.0);   // cubo interno semi-larghezza Y [m]
    declare_parameter<bool>       ("allow_vertical_escape", true);  // false=solo fuga orizzontale
    // Bias laterale per attraversare i gap del labirinto: ha senso solo dove i muri
    // attraversano l'intera vasca e il passaggio sta su un lato noto. Con ostacoli
    // isolati resta 0, altrimenti spinge il drone di lato ad ogni schivata.
    declare_parameter<double>     ("lateral_gap_bias", 0.0);

    world_name_        = get_parameter("world_name").as_string();
    n_obstacles_       = get_parameter("n_obstacles").as_int();
    csv_path_          = get_parameter("csv_path").as_string();
    drone_name_        = get_parameter("drone_name").as_string();
    target_x_          = get_parameter("target_x").as_double();
    target_y_          = get_parameter("target_y").as_double();
    target_z_          = get_parameter("target_z").as_double();
    goal_radius_       = get_parameter("goal_radius").as_double();
    yaw_bias_          = get_parameter("yaw_bias").as_double();
    depth_limit_high_  = get_parameter("depth_limit_high").as_double();
    inner_wx_          = get_parameter("inner_wx").as_double();
    inner_wy_          = get_parameter("inner_wy").as_double();
    planner_.set_walls(get_parameter("wall_wx").as_double(),
                       get_parameter("wall_wy").as_double());
    planner_.set_inner_walls(inner_wx_, inner_wy_);
    allow_vert_esc_ = get_parameter("allow_vertical_escape").as_bool();
    lateral_gap_bias_ = get_parameter("lateral_gap_bias").as_double();
    goal_active_ = std::isfinite(target_x_) && std::isfinite(target_y_) && std::isfinite(target_z_);
    // Fix Q rev3: inizializza stato da spawn per filtro phantom preciso dal frame 0
    {
        double sx = get_parameter("spawn_x").as_double();
        double sy = get_parameter("spawn_y").as_double();
        double sz = get_parameter("spawn_z").as_double();
        if (std::isfinite(sx)) { state_.x = sx; prev_x_ = sx; }
        if (std::isfinite(sy)) { state_.y = sy; prev_y_ = sy; }
        if (std::isfinite(sz)) { prev_z_ = sz; }
    }

    // ── Sottoscrizioni sonar (topic auto-namespaced per drone_name) ────────
    auto qos = rclcpp::SensorDataQoS();
    const std::string m = "/model/" + drone_name_;
    for (const char *link : {"base_link", "horizontal_fins", "vertical_fins", "propeller",
                             "buoyancy_engine", "acoustic_transponder"}) {
        contact_subs_.push_back(create_subscription<ros_gz_interfaces::msg::Contacts>(
            m + "/contacts/" + link, 10,
            [this](ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
                contacts_received_ = true;
                if (msg->contacts.empty()) return;
                double t = now().seconds();
                if (t - last_physical_contact_t_ > 1.0) {
                    ++collision_count_;
                    RCLCPP_WARN(get_logger(), "[PHYSICAL-CONTACT #%d] %s / %s",
                        collision_count_, msg->contacts.front().collision1.name.c_str(),
                        msg->contacts.front().collision2.name.c_str());
                }
                last_physical_contact_t_ = t;
            }));
    }
    const std::string sonar_topics[N_SENSORS] = {
        m + "/frontal_sonar/points",
        m + "/left_sonar/points",
        m + "/right_sonar/points",
        m + "/bottom_sonar/points",
        m + "/top_sonar/points",
    };
    for (int i = 0; i < N_SENSORS; ++i) {
        sonar_subs_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
            sonar_topics[i], qos,
            [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                sonar_callback(msg, static_cast<SensorId>(i));
            });
    }

    pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/world/buoyant_tethys/dynamic_pose/info", 10,
        std::bind(&CollisionAvoidanceNode::pose_callback, this, std::placeholders::_1));

    vert_pub_  = create_publisher<std_msgs::msg::Float64>(m + "/vertical_fins/cmd_pos", 10);
    horiz_pub_ = create_publisher<std_msgs::msg::Float64>(m + "/horizontal_fins/cmd_pos", 10);
    thr_pub_   = create_publisher<std_msgs::msg::Float64>(m + "/joint/propeller_joint/cmd_thrust", 10);

    // Timer sul clock del nodo, non wall: con use_sim_time il loop resta a 5 Hz di tempo
    // SIMULATO anche se la simulazione gira più veloce o più lenta del tempo reale.
    // Con create_wall_timer la cadenza dipendeva dal carico della macchina, quindi il
    // numero di cicli di controllo per secondo simulato variava fra un run e l'altro.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration(200ms),
                                  std::bind(&CollisionAvoidanceNode::control_loop, this));

    metrics_pub_   = create_publisher<std_msgs::msg::String>("/ca_metrics", 10);
    metrics_timer_ = create_wall_timer(5000ms,
        std::bind(&CollisionAvoidanceNode::publish_metrics, this));

    if (goal_active_)
        RCLCPP_INFO(get_logger(), "VFH 3D [%s] — goal (%.1f, %.1f, %.1f) r=%.1f",
            drone_name_.c_str(), target_x_, target_y_, target_z_, goal_radius_);
    else
        RCLCPP_INFO(get_logger(), "VFH 3D [%s] — wander mode", drone_name_.c_str());
}

// ── Loop di controllo (200 ms) ────────────────────────────────────────────
void CollisionAvoidanceNode::control_loop()
{
    build_merged();
    // Contact sensors emit events, not an empty heartbeat. A run without
    // impacts can therefore have zero messages: track bridge availability
    // separately from receipt of an impact (all six links must be bridged).
    contacts_available_ = contacts_received_ || std::all_of(
        contact_subs_.begin(), contact_subs_.end(),
        [](const auto &sub) { return sub->get_publisher_count() > 0; });

    const double sensor_now = now().seconds();
    if (!sm_.received || !raw_[FRONT].received ||
        sensor_now - raw_[FRONT].last_time > SONAR_STALE ||
        !pose_received_ || sensor_now - last_pose_t_ > SONAR_STALE) {
        // Conta per wall-clock throttle (clock sim=0 non funziona con RCLCPP_WARN_THROTTLE)
        if (++no_sonar_ticks_ % 50 == 1)
            RCLCPP_WARN(get_logger(), "[%s] nessun sonar (%.0f s attesa)",
                drone_name_.c_str(), no_sonar_ticks_ * 0.2);
        stop_actuators(); return;
    }
    // Fix 133: reset EMA vz/yr al primo sonar — l'EMA si avvelena durante la fase
    // pre-sonar (molti frame TF con z oscillante prima che la fisica si stabilizzi).
    // Con vz negative falsamente → el_cmd diventa positivo (UP) → pitch cresce.
    if (!sonar_first_seen_) {
        sonar_first_seen_ = true;
        state_.vz       = 0.0;
        state_.yaw_rate = 0.0;
        RCLCPP_INFO(get_logger(), "[INIT] Reset vz=0 yr=0 al primo sonar");
    }

    double t = now().seconds();

    // Fix N: warmup 2s dopo il primo sonar (wall clock — sim time può essere 0 se /clock
    // non è ricevuto, il che bloccherebbe la warmup per sempre).
    if (sonar_wall_start_.time_since_epoch().count() == 0)
        sonar_wall_start_ = std::chrono::steady_clock::now();
    double wall_since_sonar = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - sonar_wall_start_).count();
    if (wall_since_sonar < 2.0) { stop_actuators(); return; }

    // Inizializzazione metriche al primo tick con posa disponibile (indipendente da sim time)
    if (!metrics_initialized_ && pose_received_) {
        start_x_ = prev_metric_x_ = state_.x;
        start_y_ = prev_metric_y_ = state_.y;
        start_z_ = prev_metric_z_ = state_.z;
        stuck_pos_x_ = state_.x;
        stuck_pos_y_ = state_.y;
        stuck_pos_t_ = t;
        sim_start_t_ = last_pose_t_;
        real_start_  = std::chrono::steady_clock::now();
        cpu_start_   = std::clock();
        metrics_initialized_ = true;
        initial_goal_dist_ = goal_active_ ? std::hypot(std::hypot(target_x_-state_.x,
            target_y_-state_.y), target_z_-state_.z) : 0.0;
        prev_metric_t_ = t;
    }
    if (metrics_initialized_) {
        double metric_dt = t - prev_metric_t_;
        if (metric_dt > 0.01 && metric_dt < 1.0) {
            double speed = ((state_.x - prev_metric_x_) * std::cos(state_.yaw)
                + (state_.y - prev_metric_y_) * std::sin(state_.yaw)) / metric_dt;
            forward_speed_ = 0.5*forward_speed_ + 0.5*std::clamp(speed, -5.0, 5.0);
        }
        prev_metric_t_ = t;
        double step = std::sqrt(
            std::pow(state_.x - prev_metric_x_, 2) +
            std::pow(state_.y - prev_metric_y_, 2) +
            std::pow(state_.z - prev_metric_z_, 2));
        total_dist_ += step;
        speed_sum_  += step / std::max(metric_dt, 0.01);
        ++speed_n_;
        prev_metric_x_ = state_.x;
        prev_metric_y_ = state_.y;
        prev_metric_z_ = state_.z;
    }

    double fwd_min = planner_.forward_min_range();
    const double sonar_fwd_min = fwd_min;
    bool   blocked = planner_.forward_blocked();

    // Integra distanza analitica dalle pareti
    {
        double wd = planner_.wall_fwd_dist();
        if (wd < fwd_min) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[WALL] parete a %.1f m in direzione avanzamento", wd);
            fwd_min = wd;
        }
        if (wd < BLOCKED_DIST) blocked = true;
    }

    // ── SONAR-BLIND: memoria di pericolo frontale ─────────────────────────
    // A contatto l'ostacolo è più vicino della portata minima del sensore (0.1 m) e
    // non produce ritorni: forward_min_range() restituisce il sentinella 99, che vale
    // "non vedo nulla" e non "non c'è nulla". Trattarlo come corridoio libero portava
    // il drone appoggiato a un cilindro a concludere di avere 99 m davanti e a
    // spingerci contro (osservato: 0.55 → 99.0 → 0.17 m, poi collisione).
    // Se un ostacolo ravvicinato è stato visto di recente, quella memoria prevale.
    if (sm_.received) {
        if (sonar_fwd_min < SONAR_BLIND_DIST) {
            last_close_fwd_t_   = t;
            last_close_fwd_min_ = sonar_fwd_min;
        } else if (sonar_fwd_min >= 99.0 && (t - last_close_fwd_t_) < SONAR_BLIND_MEM) {
            fwd_min = std::min(fwd_min, last_close_fwd_min_);
            blocked = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[SONAR-BLIND] nessun eco ma ostacolo a %.2f m %.1fs fa → fwd=%.2f (non libero)",
                last_close_fwd_min_, t - last_close_fwd_t_, fwd_min);
        }
    }

    // ── Goal check anticipato ─────────────────────────────────────────────
    // Eseguito prima di BRAKE/DEPTH_LIMIT: wall_imminent scatta a wall_fwd<8m che coincide
    // con la posizione del target (spawn y=-18, target y=+18, muro a y≈+26 → margine 8m).
    // Se il check fosse solo nel blocco CRUISE il BRAKE lo sopprimerebbe.
    if (goal_active_ && last_pose_t_ > 0.0) {
        double dx3 = target_x_ - state_.x;
        double dy3 = target_y_ - state_.y;
        double dz3 = target_z_ - state_.z;
        double dist3 = std::hypot(std::hypot(dx3, dy3), dz3);
        min_goal_dist_ = std::min(min_goal_dist_, dist3);
        if (dist3 <= goal_radius_) {
            goal_reached_ = true;
            if (final_approach_active_)
                RCLCPP_INFO(get_logger(), "[FINAL-GOAL] dist=%.2f", dist3);
            RCLCPP_INFO(get_logger(), "[GOAL] %s raggiunto! dist=%.2f m",
                drone_name_.c_str(), dist3);
            stop_actuators();
            {
                std::string rm = "ign service -s /world/buoyant_tethys/remove"
                    " --reqtype ignition.msgs.Entity"
                    " --reptype ignition.msgs.Boolean"
                    " --timeout 2000"
                    " --req 'name: \"" + drone_name_ + "\" type: 2' &";
                std::system(rm.c_str());
            }
            write_csv();
            rclcpp::shutdown();
            return;
        }
    }

    // A short, fixed-duration brake cannot stop a moving AUV. Keep braking
    // until measured motion is arrested and recover clearance before turning.
    const double speed_fwd = std::max(0.0, forward_speed_);
    // Exact clearance is measured from the hull origin, one metre behind
    // the front sonar. Preserve the previous physical stopping margin.
    const double front_sensor_offset = sm_.horizontal_points_valid ? 1.0 : 0.0;
    const double stop_distance = 2.0 + front_sensor_offset
        + 0.4*speed_fwd + speed_fwd*speed_fwd / (2.0*0.30);
    double brake_clearance = planner_.corridor_clearance(0.0);
    if (sonar_fwd_min >= 99.0 && t - last_close_fwd_t_ < SONAR_BLIND_MEM)
        brake_clearance = std::min(brake_clearance, last_close_fwd_min_);
    // At low speed a fixed six-metre trigger repeatedly stopped the flow
    // needed to start a turn. The measured stopping envelope remains the
    // hard constraint; the close-obstacle override below still applies.
    bool collision_course = brake_clearance < stop_distance;
    if (!safety_braking_ && forward_speed_ > 0.4)
        obstacle_memory_.end_recovery();
    const double emergency_clearance = EMERGENCY_COLLISION_DIST + front_sensor_offset;
    // Depth recovery still translates the hull forwards. It must never
    // disable the stopping envelope (physical contacts in runs 34 and 37).
    if (!safety_braking_ && ((collision_course && forward_speed_ > 0.20)
        || brake_clearance < emergency_clearance)) {
        safety_braking_ = true;
        brake_stop_reverse_ = false;
        vertical_escape_ = false;
    }
    if (safety_braking_) {
        obstacle_memory_.begin_recovery(t);
        const double back = std::min(planner_.wall_dist_in_dir(state_.yaw + M_PI),
            obstacle_memory_.clearance(state_.x, state_.y, state_.z, state_.yaw + M_PI, t));
        // Six metres stopped the vehicle but repeatedly left too little
        // space to accelerate and close the escape turn (runs 14-15).
        // Recover turning room before releasing the brake, subject to the
        // same observed rear-clearance guard.
        constexpr double TURN_RECOVERY_CLEARANCE = 9.0;
        if ((forward_speed_ < 0.15 && brake_clearance > TURN_RECOVERY_CLEARANCE)
            || back < rear_stopping_distance(forward_speed_) + 0.5)
            brake_stop_reverse_ = true;
        if (brake_stop_reverse_ && recovery_reverse_can_resume(
                forward_speed_, brake_clearance, back, emergency_clearance)) {
            brake_stop_reverse_ = false;
            RCLCPP_INFO(get_logger(), "[CURVE-BRAKE-RESUME-REVERSE] front=%.2f rear=%.2f",
                brake_clearance, back);
        }
        if (brake_stop_reverse_ && forward_speed_ > -0.15 && forward_speed_ < 0.25
            && brake_clearance >= emergency_clearance) {
            safety_braking_ = false;
            brake_stop_reverse_ = false;
            double baz = 0.0, clearance = 0.0;
            if (planner_.find_lookahead_escape(baz, preferred_dir(), &clearance)) {
                escape_yaw_ = normalize(state_.yaw + baz);
                escape_depth_ = goal_active_ ? target_z_ : wander_z_;
                mode_ = Mode::ESCAPE;
                escape_start_t_ = plan_t_ = t;
                mode_until_ = t + 20.0;
                create_local_waypoint(escape_yaw_, clearance);
            }
            RCLCPP_INFO(get_logger(), "[CURVE-BRAKE-RELEASE] fwd=%.2f speed=%.2f", fwd_min, forward_speed_);
        } else {
            mode_ = Mode::BRAKE;
            local_wp_active_ = false;
            // First arrest reverse motion; applying forward-turn steering
            // while still backing at ~1 m/s turns away from the chosen gap.
            double thrust = recovery_thrust(forward_speed_, back, brake_stop_reverse_);
            // With reversed flow, keep fins neutral while backing away.
            double steer = forward_speed_ > 0.25
                ? std::clamp(-0.8*state_.yaw_rate, -0.42, 0.42) : 0.0;
            cmd(thrust, steer, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[CURVE-BRAKE] fwd=%.2f rear=%.2f stop=%.2f speed=%.2f thr=%.1f",
                brake_clearance, back, stop_distance, forward_speed_, thrust);
            return;
        }
    }

    // ── ALT-RECOVERY: recupero quota/assetto in goal mode + !allow_vert_esc_ ──
    // Priorità su DEPTH_LIMIT: cattura instabilità prima che diventi irrecuperabile.
    // Trigger: z fuori target±4/2m, |pitch|>0.65, yr da solo solo se >2.5 o con z/pitch off.
    // Sospende tutto: committed_wp, GOAL-REPLAN, LOCAL-PATH, BRAKE, retromarcia, DL-RETRO.
    // Eccezioni: cede a BRAKE se parete imminente (fwd<8m), cede a DEPTH_LIMIT se z<2m.
    // Uscita: z±1.2 + |pitch|<0.25 + |vz|<0.25 + |yr|<0.35 per almeno 3s (timeout 15s).
    // Cooldown dopo uscita: 8s (timeout) o 3s (done) per evitare retrigger immediato.
    if (goal_active_ && !allow_vert_esc_) {
        // yr da sola non scatta ALT-RECOVERY se z e pitch sono ok.
        // Richiede yr>2.5 standalone, oppure yr>1.8 con z off>1.5m o |pitch|>0.30.
        bool yr_bad = std::abs(state_.yaw_rate) > 2.5 ||
                      (std::abs(state_.yaw_rate) > 1.8 &&
                       (std::abs(state_.z - target_z_) > 1.5 ||
                        std::abs(state_.pitch)         > 0.30));
        bool alt_rec_trig = (state_.z > target_z_ + 4.0 ||
                             state_.z < target_z_ - 2.0 ||
                             std::abs(state_.pitch) > 0.65 ||
                             yr_bad);
        if (!alt_recovery_active_ && alt_rec_trig && t > alt_recovery_until_) {
            alt_recovery_active_  = true;
            alt_recovery_entry_t_ = t;
            alt_recovery_until_   = t + 15.0;
            local_wp_active_      = false;
            RCLCPP_WARN(get_logger(),
                "[ALT-RECOVERY-START] z=%.1f target=%.1f pitch=%.2f vz=%.2f yr=%.2f",
                state_.z, target_z_, state_.pitch, state_.vz, state_.yaw_rate);
        }
    }
    if (alt_recovery_active_) {
        double rec_dur   = t - alt_recovery_entry_t_;
        bool alt_done    = rec_dur >= 3.0 &&
                           std::abs(state_.z     - target_z_) < 1.2 &&
                           std::abs(state_.pitch)             < 0.25 &&
                           std::abs(state_.vz)                < 0.25 &&
                           std::abs(state_.yaw_rate)          < 0.35;
        bool alt_timeout = (t > alt_recovery_until_);
        if (alt_done || alt_timeout) {
            alt_recovery_active_        = false;
            local_wp_active_            = false;
            vertical_escape_            = false;
            escape_lateral_             = false;
            goal_clear_since_           = -1.0;
            plan_t_                     = t;
            stabilize_reentry_until_    = t + 3.0;
            sonar_brake_until_          = std::max(sonar_brake_until_, t + 1.0);
            // Cooldown anti-retrigger: alt_recovery_until_ usato come "non prima di"
            // quando !alt_recovery_active_. 8s dopo timeout (drone instabile), 3s dopo done.
            alt_recovery_until_         = t + (alt_timeout ? 8.0 : 3.0);
            // Se corridoio goal bloccato → ESCAPE + circum invece di CRUISE
            double g_rel_ar = preferred_dir();
            double g_clr_ar = (goal_active_ && sm_.received)
                ? planner_.corridor_clearance(g_rel_ar) : 99.0;
            if (goal_active_ && sm_.received && g_clr_ar < SAFE_PASS_DIST) {
                mode_            = Mode::ESCAPE;
                escape_yaw_      = normalize(state_.yaw + g_rel_ar);
                escape_depth_    = std::clamp(target_z_, 4.0, 15.0);
                mode_until_      = t + 25.0;
                escape_start_t_  = t;
                escape_lateral_  = true;
                circum_active_   = true;
                if (circum_side_ == 0) circum_side_ = +1;  // default se non ancora impostato
                circum_until_    = t + 12.0;
                RCLCPP_WARN(get_logger(),
                    "[ALT-RECOVERY-%s→CIRCUM] z=%.1f pitch=%.2f clr=%.1f → ESCAPE+circum side=%+d",
                    alt_timeout ? "TIMEOUT" : "DONE",
                    state_.z, state_.pitch, g_clr_ar, circum_side_);
            } else {
                mode_ = Mode::CRUISE;
                if (alt_timeout) {
                    RCLCPP_WARN(get_logger(),
                        "[ALT-RECOVERY-TIMEOUT] z=%.1f pitch=%.2f → CRUISE (non recuperato)",
                        state_.z, state_.pitch);
                } else {
                    RCLCPP_INFO(get_logger(),
                        "[ALT-RECOVERY-DONE] z=%.1f pitch=%.2f → CRUISE (stable %.1fs)",
                        state_.z, state_.pitch, rec_dur);
                }
            }
            // non fa return: riprende dal tick corrente
        } else {
            local_wp_active_     = false;
            double z_err_ar      = target_z_ - state_.z;
            double el_ar = std::clamp(
               -0.25 * z_err_ar + 1.4 * state_.vz - 1.2 * state_.pitch,
                -0.35, 0.35);
            // Saturazione elevator: quota o pitch estremo hanno priorità assoluta
            if (state_.z > target_z_ + 1.5) el_ar = std::max(el_ar, +0.30);
            if (state_.z < target_z_ - 1.5) el_ar = std::min(el_ar, -0.30);
            if (state_.pitch >  0.45)        el_ar = std::min(el_ar, -0.30);
            if (state_.pitch < -0.45)        el_ar = std::max(el_ar, +0.30);
            // Yaw: smorzamento yr + pull lento verso goal
            double goal_yaw_ar = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
            double yaw_err_ar  = normalize(goal_yaw_ar - state_.yaw);
            double az_ar = std::clamp(0.25 * yaw_err_ar - 0.8 * state_.yaw_rate, -0.25, 0.25);
            // Thrust: TURN_THR (20N) garantisce flusso minimo sulle pinne per autorità elevator.
            // Quando z è molto sotto target, aumenta a 25N per forza di risalita maggiore.
            double thr_ar = (state_.z < target_z_ - 3.0) ? 25.0 : static_cast<double>(TURN_THR);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[ALT-RECOVERY] z=%.1f target=%.1f pitch=%.2f vz=%.2f yr=%.2f el=%.2f thr=%.0f",
                state_.z, target_z_, state_.pitch, state_.vz, state_.yaw_rate, el_ar, thr_ar);
            // Cede a DEPTH_LIMIT se sul fondo (z<2): retro UP è più efficace di elevator stall
            if (state_.z < 2.0) {
                cmd(thr_ar, az_ar, el_ar);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[ALT-RECOVERY-FONDO] z=%.1f < 2.0 → cedo a DEPTH_LIMIT", state_.z);
                // non fa return: DEPTH_LIMIT interviene
            // Cede a BRAKE/SIDE-AVOID se parete imminente
            } else if (fwd_min < BRAKE_DIST + 2.0) {
                cmd(thr_ar, az_ar, el_ar);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                    "[ALT-RECOVERY-PARETE] fwd=%.1f < %.0f → cedo a BRAKE",
                    fwd_min, BRAKE_DIST + 2.0);
                // non fa return: BRAKE/SIDE-AVOID interviene
            } else {
                cmd(thr_ar, az_ar, el_ar);
                return;
            }
        }
    }

    // ── Guardia profondità assoluta ───────────────────────────────────────
    // Fix 45: soglia fondale alzata 3.5→5.5m.
    // Fix 67: 5.5→7.5m — con VERT ESC e spin il drone può perdere 2m in 3 secondi
    // (z=9.56→7.55→0.09) senza che il handler intervenga. Con 7.5m la caduta
    // viene catturata prima che diventi irrecuperabile.
    // Fix 130: in ESCAPE verticale verso quota alta (es. sopra muro 25m), il controller
    // ESCAPE gestisce già z → DL-CEIL non deve sovrascriverlo.
    bool dl_skip_ceil = (mode_ == Mode::ESCAPE && vertical_escape_ && escape_depth_ > 20.0);
    // Fix ALT: in goal mode + !allow_vert_esc_, DL è ultima spiaggia dopo ALT-RECOVERY.
    // Soglie allargate: floor 7.5→5.0m, ceil depth_limit_high_→22.0m.
    double dl_floor_thr = (goal_active_ && !allow_vert_esc_) ? 5.0  : 7.5;
    double dl_ceil_thr  = depth_limit_high_;
    if ((state_.z > dl_ceil_thr && !dl_skip_ceil) || state_.z < dl_floor_thr) {
        double safe_z = std::clamp(wander_z_, 2.0, 16.0);
        double z_err  = safe_z - state_.z;
        // Convezione: el>0=UP, el<0=DOWN (CLAUDE.md corretto).
        // el_lim: per fondale (z_err>0 → sale): el_lim>0 → UP ✓
        double el_lim = std::clamp(-0.35*z_err + 0.70*state_.vz - 0.15*state_.pitch,
                                   -0.40, 0.40);
        double depth_wall_d = planner_.wall_fwd_dist();
        if (state_.z > depth_limit_high_) {
            // Fix 48: forward + el<0=DOWN.
            // Fix 42 usava retromarcia + el=+0.45, ma se il drone è incollato al tetto
            // fisico della vasca (es. z=34m con tetto a 35m), la retromarcia non genera
            // flusso → LiftDrag≈0 → galleggiabilità (+9N) mantiene il drone al tetto.
            // Forward + el=-0.45 genera flusso avanti + forza verso il basso ✓.
            // Fix 61: in goal mode punta verso il goal, non verso il centro.
            // Il vecchio centro pull causava spin crescente quando il drone era
            // lontano dal centro (es. y=40→68) e si scontrava con la direzione goal.
            // Fix 71B: rimosso steer da az_ceil (causava yawrate → accoppiamento Fossen
            // → pitch UP → retro debole). Solo smorzatura pura per mantenere drone dritto.
            // Fix 86: steer verso il goal durante DL-CEIL invece di volare dritto.
            // Bug: az_ceil=0 con thr=43N → drone vola 65m in direzione sbagliata in 60s.
            // Ora: PD yaw verso goal (0.5 * errore - 2.0 * yawrate).
            double goal_yaw_dl = goal_active_
                ? std::atan2(target_y_ - state_.y, target_x_ - state_.x)
                : state_.yaw;
            double az_err_dl   = normalize(goal_yaw_dl - state_.yaw);
            double az_ceil     = std::clamp(0.5 * az_err_dl - 2.0 * state_.yaw_rate,
                                            -0.45, 0.45);
            // Fix 71B: strategia unificata per abbassare z al soffitto.
            //
            // Il problema precedente (Fix 70): a pitch≈0, forward+el=-0.45 NON genera
            // forza Z diretta (asse giunto verticale→LiftDrag orizzontale). Drone saliva
            // a velocità terminale buoyancy (0.09 m/s) per 60s con el=-0.45 inefficace.
            //
            // Soluzione: retro -50N + el=+0.40 (retro+el>0=DOWN sempre, empiricamente
            // ~12N a v≥0.3m/s — confermato in BRAKE Fix 55). Batte buoyancy +3N → z scende.
            // Eccezione: pitch < -0.10 (muso giù) → retro darebbe thr_z UP → usa forward.
            // Fix 76: rimosso il path RETRO in DL-CEIL.
            // Bug osservato: RETRO+el=+0.40 era pensato per BRAKE (drone fermo), ma
            // durante DL-CEIL il drone ha velocità forward positiva → LiftDrag in regime
            // forward → forward+el=+0.40=UP (non DOWN!). Effetto: JPC spinge pitch→+0.98,
            // vz sale a +1.42 m/s, z a 18.67m, poi instabilità totale.
            // Fix: forward+el=-0.45 è DOWN affidabile a qualsiasi velocità forward.
            // Fix 77: quando pitch>0.10 (muso su) o vz>1.0 (troppo veloce verso su),
            // la spinta forward amplifica thr_z=thr×sin(pitch+)=UP → peggiora la salita.
            // Caso osservato: pitch=+0.62 → thr_z=45×sin(0.62)=+26N UP → z a 19.6m.
            // Soluzione: thr=0 elimina thr_z; el=-0.45 guida pitch verso 0 (DOWN).
            // Fix 81: DL-CEIL riscritta per eliminare oscillazione z≈12m e blocco z=16m.
            //
            // Problema osservato: vz=+0.2-0.3 stabile per 50s a z=12m → poi spike a +1 → z=16m.
            // Root cause 1: branch FWD0 (el=0 a pitch<-0.20) eliminava la forza DOWN → loop.
            // Root cause 2: thr=0 a pitch+/vz+ lascia buoyancy vincere (no flusso → no LiftDrag).
            //
            // Soluzione: retro quando pitch>0.10 e drone ancora ascende (vz>0).
            // Retro con pitch+ → thr_z = retro*sin(pitch) = DOWN direttamente.
            // Retro + el=+0.45 → flusso retro → retro+el>0=DOWN da LiftDrag.
            // Entrambe le forze sono DOWN: molto più efficace di thr=0.
            //
            // Per la discesa (vz≤0) o pitch non eccessivo: forward+el=-0.45 (sempre, no FWD0).
            // Fix 85: rimosso DL-CEIL-STOP (thr=0).
            // Bug: thr=0 svuotava la velocità forward → F_LiftDrag→0 → 9N buoyancy vinceva
            // → vz saltava da 0.29 a 0.84 m/s. Con thr=CRUISE_THR la velocità si mantiene
            // → LiftDrag DOWN sempre attivo → z scende in pochi secondi.
            // Fix 85b: pitch guard |pitch|>0.40 → el=0 (impedisce runaway pitch a -1.06 rad).
            // La buoyancy reale è +9N (neutral_volume=0.004m³): serve el+thr_z combinati.
            // Fix 112A: se c'è un ostacolo ravvicinato davanti, non spingere forward.
            // Con gli ostacoli path nel corridoio il drone può rimanere incastrato e
            // DL-CEIL-FWD lo spingerebbe ancora di più nell'ostacolo.
            double dl_fwd_min = planner_.forward_min_range();
            bool dl_fwd_blocked = (dl_fwd_min < APPROACH_DIST * 1.5);  // < 12m
            double ceil_thr, el_ceil;
            if (state_.vz < -0.5) {
                // Fix 102A: già in caduta veloce — non aggiungere forza DOWN.
                ceil_thr = 0.0;
                el_ceil  = 0.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                    "[DL-CEIL-COAST] già in caduta vz=%.2f pitch=%.2f → coast",
                    state_.vz, state_.pitch);
            } else if (state_.pitch > 0.10 && state_.vz > 0.0 && (!goal_active_ || allow_vert_esc_)) {
                // Muso su + drone ascende: retro → thr_z=retro*sin(pitch+)=DOWN, el=+0.45 retro→DOWN.
                // Fix 113B: dl_fwd_blocked rimosso. Retro con pitch<0 → thr_z=retro*sin(-)=UP: sale!
                // Bug osservato: pitch=-0.19 → thr_z=-30*sin(-0.19)=+5.76N UP + 9N buoy = drone z=29m.
                // Fix ALT: soppresso in goal mode + allow_vert_esc_=false — in quel caso
                // DL-CEIL-RETRO non è affidabile e si usa il path FWD+el<0=DOWN più sotto.
                ceil_thr = -30.0;
                el_ceil  = +0.45;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                    "[DL-CEIL-RETRO] pitch=%.2f vz=%.2f → retro DOWN",
                    state_.pitch, state_.vz);
            } else if (dl_fwd_blocked) {
                // Ostacolo/muro a <12m davanti: retromarcia per allontanarsi + el=+0.45 (retro→DOWN).
                // Fix 130: forward nell'ostacolo era assorbito dal muro → nessun flusso → no LiftDrag →
                // il drone rimaneva incastrato al soffitto. Retro si allontana dal muro e scende.
                ceil_thr = -static_cast<double>(TURN_THR);
                el_ceil  = +0.45;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[DL-CEIL-RETRO-FWD] fwd=%.1fm → retro+el DOWN (allontana da muro)",
                    dl_fwd_min);
            } else {
                // Muso neutro/giù, spazio libero davanti: forward + el=-0.45 DOWN.
                double safe_thr = std::max(CRUISE_THR * std::max(std::cos(state_.pitch), 0.0),
                                           static_cast<double>(TURN_THR));
                ceil_thr = safe_thr;
                el_ceil  = (std::abs(state_.pitch) < 0.40) ? +0.45 : 0.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[DL-CEIL-FWD] pitch=%.2f vz=%.2f el=%.2f → fwd+el DOWN",
                    state_.pitch, state_.vz, el_ceil);
            }
            cmd(ceil_thr, az_ceil, el_ceil);
            // Fix 86B: aggiorna escape_yaw_ ogni tick di DL-CEIL (non solo al primo ingresso).
            // Bug: escape_yaw_ era fissato quando il drone era a y=-43m e non veniva mai
            // aggiornato. Dopo 60s di DL-CEIL il drone si trovava a (-29,+12) ma
            // escape_yaw_ puntava ancora verso y=-43m → direzione di fuga sbagliata.
            // Fix: aggiorna escape_yaw_ in continuazione; mode_until_ sempre rinnovato
            // per evitare timeout immediato all'uscita da DL-CEIL.
            escape_yaw_ = goal_active_
                ? std::atan2(target_y_ - state_.y, target_x_ - state_.x)
                : state_.yaw;
            mode_until_ = t + 10.0;  // keep alive finché DL-CEIL è attivo
            if (mode_ != Mode::ESCAPE || !vertical_escape_) {
                // DL-CEIL: scendi a quota sicura. Con allow_vert_esc_=false usa target_z_
                // per non impostare una quota arbitraria alta; vertical_escape_ resta false
                // così lo stato ESCAPE rimane orizzontale e il drone continua a girare.
                escape_depth_ = allow_vert_esc_
                    ? std::clamp(wander_z_, 3.0, 12.0)
                    : (goal_active_ ? std::clamp(target_z_, 4.0, 15.0)
                                   : std::clamp(wander_z_, 3.0, 12.0));
                vertical_escape_ = allow_vert_esc_;
                mode_            = Mode::ESCAPE;
                plan_t_          = t;
            }
            // Fix O: reset brake_entry_t_ per stuck check corretto
            if ((t - brake_entry_t_) > 20.0) {
                brake_entry_t_ = t;
                stuck_count_   = 0;
            }
            // Soppressione BRAKE mentre scende (speculare a Fix 26 fondale)
            stuck_escape_until_   = std::max(stuck_escape_until_,   t + 5.0);
            sonar_brake_until_    = std::max(sonar_brake_until_,     t + 5.0);
            wall_fwd_brake_until_ = std::max(wall_fwd_brake_until_,  t + 5.0);
            wall_stuck_until_     = std::max(wall_stuck_until_,      t + 3.0);
        } else {
            // Fix 26: fondale — NON impostare BRAKE.
            // Fix 32: muro davanti → TURN_THR (bloccato dal muro) + el_lim.
            // Fix 36: se il sonar vede un ostacolo < APPROACH_DIST davanti, sterzare
            // verso una direzione libera (find_escape) invece di spingere nell'ostacolo.
            // Con V=0 e ostacolo in avanti, CRUISE_THR non genera LiftDrag → drone
            // incollato al fondale. Con la direzione libera il drone può accelerare.
            // Fix 102C: in goal mode punta al target, non al centro.
            double esc_yaw_now = goal_active_
                ? std::atan2(target_y_ - state_.y, target_x_ - state_.x)
                : std::atan2(-state_.y, -state_.x);
            double az_depth;
            {
                double fwd_sonar = sm_.received ? planner_.forward_min_range() : 99.0;
                double base_az;
                if (fwd_sonar < APPROACH_DIST && sm_.received) {
                    double baz_dl = 0.0;
                    base_az = planner_.find_escape(baz_dl, preferred_dir())
                              ? 1.2 * normalize(baz_dl) : 0.40;
                } else {
                    base_az = 1.2 * normalize(esc_yaw_now - state_.yaw);
                }
                az_depth = std::clamp(base_az - 2.0 * state_.yaw_rate, -0.45, 0.45);
            }
            if (depth_wall_d < BRAKE_DIST) {
                // Fix 96: retro+el>0=DOWN (CLAUDE.md), non UP come credeva Fix 50.
                // Fix 50 invertì il segno con la logica errata "el>0=UP sempre".
                // retro+el<0=UP → usiamo -0.30.
                double back_dl = planner_.wall_dist_in_dir(state_.yaw + M_PI);
                if (back_dl > BRAKE_DIST) {
                    cmd(-50.0, az_depth, -0.30);
                } else {
                    // Incastrato tra due pareti: coast + elevator
                    cmd(TURN_THR, az_depth, el_lim);
                }
            } else {
                // Fix 74B: floor-stuck rescue — esteso a tutto il range DL-FLOOR (z<7.4).
                // Bug Fix 74: threshold z<3.0 ignorava l'equilibrio a z≈6.6m.
                // Bug reset: "z>5.0" era sempre vero a z=6.6 → timer si resettava ogni ciclo.
                // Fix: accumula timer a qualsiasi z<7.4; resetta solo quando z≥7.4.
                // Rescue: thr=0 (elimina thr_z DOWN da accoppiamento pitch) + el=+0.45 UP.
                bool floor_stuck_rescue = false;
                if (state_.z < 7.4) {
                    if (floor_stuck_t_ < 0.0) floor_stuck_t_ = t;
                    if (t - floor_stuck_t_ > 20.0) {
                        floor_stuck_rescue = true;
                        // Fix 104B: retro+el<0=UP (per CLAUDE.md). Genera thr_z=UP quando
                        // pitch<0 (tipico in fondale). Più efficace di thr=0+el UP che
                        // richiedeva flusso avanzamento per generare LiftDrag.
                        double az_fs = std::clamp(-2.0 * state_.yaw_rate, -0.40, 0.40);
                        cmd(-30.0, az_fs, -0.30);
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[DL-FLOOR-STUCK] z=%.2f stuck %.0fs → retro+el<0 UP rescue",
                            state_.z, t - floor_stuck_t_);
                    }
                } else {
                    floor_stuck_t_ = -1.0;
                }
                if (!floor_stuck_rescue) {
                // Fix 68C/104C: zona di pericolo forward-thrust quando pitch<0.
                // CRUISE_THR*cos(p)*sin(p) > buoyancy(9N) per |pitch| > asin(9/45)≈0.20 rad.
                // Abbassare soglia retro -0.35 → -0.20: qualsiasi pitch<-0.20 usa retro UP
                // invece del forward che genererebbe thr_z DOWN > buoyancy → netto DOWN.
                // Retro: thr_z=-50×sin(pitch<0)=+UP → sicuro a qualsiasi pitch<0.
                // Fix 72B: soglia retro abbassata -0.8 → -0.35 (ora → -0.20).
                if (state_.pitch < -0.20) {
                    // Fix 80: yawrate estremo (>3 rad/s) rende la retromarcia pericolosa:
                    // l'heading cambia di 120°+ in 0.5s → retro spinge contro pareti random.
                    // In quel caso: thr=0 + buoyancy + elevator UP è più sicuro.
                    double az_spdamp = std::clamp(-2.0 * state_.yaw_rate, -0.40, 0.40);
                    if (std::abs(state_.yaw_rate) > 3.0) {
                        cmd(0.0, az_spdamp, 0.45);
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                            "[DL-FLOOR-RETRO-SPIN] pitch=%.2f yr=%.2f → thr=0 el UP (spin)",
                            state_.pitch, state_.yaw_rate);
                    } else {
                    cmd(-50.0, az_spdamp, -0.30);
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                        "[DL-FLOOR-RETRO] pitch=%.2f yr=%.2f → retro UP",
                        state_.pitch, state_.yaw_rate);
                    }
                } else {
                // Fix 52: a z<2m CRUISE_THR genera pitch-thrust coupling netto DOWN
                // (bilancia buoyancy → drone resta incollato al fondale).
                // Con thr=0: solo buoyancy(+9N) → salita lenta ma garantita.
                // Fix 63C: come Fix 60 per soffitto — nose-down estremo → thr piena
                // spinge giù (sin(-0.95)=−0.81 → 48N DOWN a 60N), peggiorando il crash
                // al fondale. pitch<-0.5 (30°+): thr=0, buoyancy +9N garantita verso su.
                double floor_thr;
                if (state_.z < 2.0 || state_.pitch < -0.40) {
                    // Fix 94B: soglia pitch -0.5→-0.40. A pitch=-0.5 con CRUISE_THR=45N:
                    // thr_z=45*sin(-0.5)=-21.6N DOWN > buoyancy+9N → netto DOWN → ripiomba.
                    floor_thr = 0.0;  // buoyancy garantita UP; thr forward peggiora
                } else {
                    double cos_pf = std::max(std::cos(state_.pitch), 0.0);
                    floor_thr = std::max(CRUISE_THR * cos_pf, static_cast<double>(TURN_THR));
                }
                cmd(floor_thr, az_depth, el_lim);
                }
                } // end !floor_stuck_rescue
            }
            if (mode_ != Mode::ESCAPE || !vertical_escape_) {
                escape_yaw_         = esc_yaw_now;
                escape_depth_       = std::clamp(safe_z, 3.0, 15.0);
                vertical_escape_    = true;
                mode_               = Mode::ESCAPE;
                mode_until_         = t + 10.0;
                plan_t_             = t;
            }
            // Fix 39: refresh timer soppressione BRAKE ad ogni tick mentre z < 1.2m.
            // Senza questo, dopo il primo ingresso i timer scadono in 5 sim-sec;
            // quando z supera 1.2m, BRAKE (wall_fwd_near / wall_imminent) scatta
            // immediatamente e riporta il drone al fondale con retromarcia.
            // Refreshando ogni tick, BRAKE resta soppresso per 5 sim-sec DOPO
            // l'ultima volta in cui z era < 1.2m, dando tempo di risalire.
            stuck_escape_until_   = std::max(stuck_escape_until_, t + 5.0);
            sonar_brake_until_    = std::max(sonar_brake_until_, t + 5.0);
            wall_fwd_brake_until_ = std::max(wall_fwd_brake_until_, t + 5.0);
            wall_stuck_until_     = std::max(wall_stuck_until_, t + 3.0);
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "[DEPTH LIMIT] z=%.2f → target=%.1f", state_.z, safe_z);
        return;
    }

    // ── FINAL APPROACH: atterraggio sul bersaglio ─────────────────────────
    // Sotto FINAL_APPROACH_DIST il drone deve smettere di ragionare da "aggira" e
    // puntare il target: restando in logica ESCAPE arrivava vicino e poi lo superava
    // (osservato: passaggio a 3.6 m, poi overshoot fino a y=58 con target a y=55).
    // Check the finite path to the target. An obstacle beyond the target
    // must not prevent arrival, but a clear current heading alone does not
    // make the different heading toward the goal safe.
    if (goal_active_ && sm_.received && !alt_recovery_active_) {
        double fa_dx   = target_x_ - state_.x;
        double fa_dy   = target_y_ - state_.y;
        double fa_dz   = target_z_ - state_.z;
        double fa_dist = std::hypot(std::hypot(fa_dx, fa_dy), fa_dz);

        if (fa_dist < FINAL_APPROACH_DIST) {
            double fa_clr = planner_.corridor_clearance(0.0);
            const double fa_goal_rel = normalize(std::atan2(fa_dy, fa_dx) - state_.yaw);
            const double fa_goal_clr = planner_.corridor_clearance(fa_goal_rel);
            const double fa_path_length = std::hypot(fa_dx, fa_dy) + 1.5;
            if (fwd_min < FINAL_ABORT_DIST || fa_clr < FINAL_ABORT_DIST + front_sensor_offset
                || fa_goal_clr < fa_path_length) {
                if (final_approach_active_) {
                    final_approach_active_ = false;
                    RCLCPP_WARN(get_logger(),
                        "[FINAL-ABORT] motivo=ostacolo fwd=%.1f clr=%.1f goal_clr=%.1f path=%.1f dist=%.1f",
                        fwd_min, fa_clr, fa_goal_clr, fa_path_length, fa_dist);
                }
                // non fa return: BRAKE/ESCAPE gestiscono l'ostacolo immediato
            } else {
                if (!final_approach_active_) {
                    final_approach_active_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[FINAL-APPROACH-START] dist=%.1f clr=%.1f", fa_dist, fa_clr);
                }
                circum_active_    = false;
                escape_lateral_   = false;
                local_wp_active_  = false;
                vertical_escape_  = false;
                mode_             = Mode::CRUISE;
                goal_clear_since_ = -1.0;

                // Velocità di avvicinamento: >0 si avvicina, <0 sta superando il bersaglio.
                double fa_closing = (prev_goal_dist_ > 0.0) ? (prev_goal_dist_ - fa_dist) : 0.0;
                prev_goal_dist_ = fa_dist;

                // La resistenza idrodinamica assiale è bassa (xUabsU=-6.23): a 1.5 m/s
                // il solo coast impiegherebbe una decina di metri per fermarsi, molto più
                // dei 3 m di raggio. Sotto i 3.5 m, o se il bersaglio sta scappando
                // all'indietro, serve retromarcia vera per smaltire l'inerzia.
                double fa_thr = (fa_dist > 12.0) ? 25.0
                              : (fa_dist >  8.0) ? 18.0
                              : (fa_dist >  5.0) ? 12.0
                              : (fa_dist >  3.5) ?  6.0
                                                 : -10.0;
                bool fa_aligned = std::abs(normalize(std::atan2(fa_dy, fa_dx) - state_.yaw)) < 0.5;
                if (fa_dist < 8.0 && fa_closing < -0.01 && fa_aligned) fa_thr = -12.0;
                // Disallineato e vicino: tenere spinta minima per conservare autorita'
                // di sterzo, altrimenti a velocita' nulla le pinne non governano piu'.
                else if (fa_dist < 5.0 && !fa_aligned) fa_thr = std::max(fa_thr, 12.0);

                double fa_el   = std::clamp(-0.18 * fa_dz + 0.8 * state_.vz, -0.25, 0.25);
                // In retromarcia il flusso sulle pinne si inverte: per ottenere lo stesso
                // effetto verticale il comando va specchiato.
                if (fa_thr < 0.0) fa_el = -fa_el;
                double fa_yerr = normalize(std::atan2(fa_dy, fa_dx) - state_.yaw);
                double fa_az   = std::clamp(1.0 * fa_yerr - 0.7 * state_.yaw_rate,
                                            -0.40, 0.40);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "[FINAL-APPROACH] dist=%.1f closing=%+.2f zerr=%+.2f thr=%.0f yaw_err=%+.2f fwd=%.1f",
                    fa_dist, fa_closing, fa_dz, fa_thr, fa_yerr, fwd_min);
                cmd(fa_thr, fa_az, fa_el);
                return;
            }
        } else {
            final_approach_active_ = false;
            prev_goal_dist_        = -1.0;
        }
    }

    // Fix 142/B: committed_wp — goal mode con waypoint locale attivo.
    // ESCAPE + CRUISE: in CRUISE il drone perde protezione se rientra subito dopo BRAKE;
    // estendendo a CRUISE, committed_wp_skip sopprime BRAKE/SIDE-AVOID anche lì.
    bool committed_wp =
        goal_active_ &&
        (mode_ == Mode::ESCAPE || mode_ == Mode::CRUISE) &&
        !vertical_escape_ &&
        local_wp_active_;

    // ── Fix 71A/82: STABILIZE — recupero instabilità + contatti pinne ────────
    // Fix 113A: soglie più alte + guard ESCAPE + cooldown richiesta.
    //   - Pitch threshold: 0.50→0.80 (pitch=0.5 è normale in volo forward/ESCAPE)
    //   - In ESCAPE: yr threshold 1.0→1.8 (ESCAPE gira con steer=±0.4 → yr~1.0 atteso)
    //   - Cooldown di 5s dopo ogni exit per spezzare il loop STABILIZE→ESCAPE→STABILIZE
    // Fix 113D: STABILIZE non scatta in BRAKE.
    //   In BRAKE, yr alto è un impulso di contatto normale; BRAKE gestisce già l'emergenza
    //   con retro -50N. Se STABILIZE prende il controllo: thr=0 (vz>0.30) → galleggiabilità
    //   vince indisturbata → drone sale a 0.15 m/s per 40s → z=14→20m → DL-CEIL incontrollabile.
    // Fix 131: threshold CRUISE alzato 1.0→1.5→4.0. In CRUISE lo yr sale a 1.8 per TF glitch
    // o Munk instability senza che il drone stia realmente girando su sé stesso.
    // STABILIZE in CRUISE è controproducente: el comandi durante STABILIZE peggiorano il pitch
    // (Munk moment supera l'autorità delle pinne a thr=20N). Fix B (pitch guard in CRUISE)
    // gestisce il pitch prima che STABILIZE sia necessario.
    // In ESCAPE yr>1.8 è più credibile (steer±0.4 genera yr~1.0 atteso → soglia 1.8).
    double stab_yr_thr    = (mode_ == Mode::ESCAPE) ? 1.8 : 4.0;
    double stab_pitch_thr = 0.45;
    if (committed_wp) { stab_yr_thr = 2.5; stab_pitch_thr = 0.75; }
    // Log quando STABILIZE verrebbe scattato ma è soppresso da committed_wp
    if (committed_wp && !stabilize_active_
        && (std::abs(state_.yaw_rate) > 1.8 || std::abs(state_.pitch) > 0.45)) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[STABILIZE-SKIP-WP] pitch=%.2f yr=%.2f committed path", state_.pitch, state_.yaw_rate);
    }
    if (!stabilize_active_
        && t > stabilize_reentry_until_
        && mode_ != Mode::BRAKE
        && (std::abs(state_.yaw_rate) > stab_yr_thr || std::abs(state_.pitch) > stab_pitch_thr)) {
        stabilize_active_     = true;
        stabilize_coast_until_ = t + 1.5;   // prima: coast puro (nessun movimento)
        stabilize_until_      = t + 3.0;
        stabilize_max_until_  = t + 12.0;
        RCLCPP_WARN(get_logger(),
            "[STABILIZE] yr=%.2f pitch=%.2f pos=(%.1f,%.1f) → coast 1.5s poi damp",
            state_.yaw_rate, state_.pitch, state_.x, state_.y);
    }
    if (stabilize_active_) {
        if (t < stabilize_coast_until_) {
            double z_ref_sc = goal_active_ ? target_z_ : wander_z_;
            double z_err_sc = z_ref_sc - state_.z;
            double des_p_sc = std::clamp(-0.18 * z_err_sc, -0.20, 0.20);
            double el_sc    = std::clamp(1.0 * (des_p_sc - state_.pitch) + 0.8 * state_.vz, -0.35, 0.35);
            // Saturazione forte se molto fuori target o vz elevato: evita derive lente
            // verso DL-FLOOR/DL-CEIL mentre STABILIZE tiene il controllo.
            if (state_.z > z_ref_sc + 2.0 || state_.vz > 0.5)
                el_sc = +0.35;
            else if (state_.z < z_ref_sc - 2.0 || state_.vz < -0.5)
                el_sc = -0.35;
            double az_sc = std::clamp(-0.4 * state_.yaw_rate, -0.25, 0.25);
            cmd(TURN_THR, az_sc, el_sc);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[STABILIZE-HOLD] thr=TURN_THR az=%.2f el=%.2f z=%.1f target=%.1f vz=%.2f",
                az_sc, el_sc, state_.z, z_ref_sc, state_.vz);
            return;
        }
        // Fix 92: settled su yawrate + vz, non pitch (pitch si stabilizza da solo).
        bool settled = std::abs(state_.yaw_rate) < 0.20
                    && std::abs(state_.vz)       < 0.15
                    && t > stabilize_until_;
        bool timeout = (t > stabilize_max_until_);
        if (settled || timeout) {
            stabilize_active_           = false;
            stabilize_reentry_until_    = t + 5.0;  // Fix 113A: cooldown anti-loop
            // Fix 104A: se c'è ancora un ostacolo frontale vicino, vai in BRAKE
            // invece di ESCAPE. Dopo STABILIZE i timer sonar/stuck sono azzerati (t+3s),
            // ma BRAKE_DIST=6m → fwd_min<6 col reset soppresso → nessun freno scatta.
            if (fwd_min < BRAKE_DIST) {
                mode_                = Mode::BRAKE;
                brake_entry_t_       = t;
                brake_begin_t_       = t;
                sonar_brake_until_   = t;  // permetti BRAKE immediato
                stuck_escape_until_  = t;
                wall_fwd_brake_until_ = t;
                RCLCPP_WARN(get_logger(),
                    "[STABILIZE→BRAKE] fwd_min=%.2f < %.1f → BRAKE (yr=%.2f pitch=%.2f, %s)",
                    fwd_min, BRAKE_DIST, state_.yaw_rate, state_.pitch,
                    settled ? "settled" : "timeout");
                // non fa return: la logica BRAKE parte nel tick corrente
            } else {
            escape_yaw_ = goal_active_
                ? std::atan2(target_y_ - state_.y, target_x_ - state_.x)
                : std::atan2(-state_.y, -state_.x);
            escape_depth_    = goal_active_
                ? std::clamp(target_z_, 4.0, 15.0)
                : std::clamp(wander_z_, 4.0, 15.0);
            vertical_escape_ = false;
            mode_            = Mode::ESCAPE;
            escape_start_t_  = t;
            mode_until_      = t + 15.0;
            plan_t_          = t;
            goal_replan_t_   = -1.0;
            sonar_brake_until_  = std::max(sonar_brake_until_,  t + 3.0);
            stuck_escape_until_ = std::max(stuck_escape_until_, t + 3.0);
            goal_clear_since_ = -1.0;
            if (goal_active_)
                create_local_waypoint(escape_yaw_, ESCAPE_CLEAR + 3.0);
            else
                local_wp_active_ = false;
            RCLCPP_INFO(get_logger(),
                "[STABILIZE→ESCAPE] yr=%.2f pitch=%.2f → goal yaw=%.2f (%s)",
                state_.yaw_rate, state_.pitch, escape_yaw_,
                settled ? "settled" : "timeout");
            // non fa return: la logica ESCAPE parte nel tick corrente
            }
        } else {
            // Fix 92: STABILIZE unificato — controllo diretto Z + yawrate.
            // NON controlla più il pitch direttamente: troppo accoppiato con Fossen.
            // Strategia: mantieni TURN_THR forward (efficacia pinne garantita),
            // el = PD su errore quota (el>0=UP, el<0=DOWN in forward flow),
            // az = smorzamento yawrate leggero (gain 0.3, no oscillazioni).
            // Il pitch si stabilizza da solo quando Z e yaw sono controllati.
            //
            // Bug precedenti:
            //   - coast thr=0 → Munk moment peggiora pitch -0.51→-1.18
            //   - damp retro+el+0.45 → in flusso retro spinge coda↓ → naso↑ → pitch runaway
            //   - damp TURN_THR+pitch+0.16 → thr_z+3.8N+buoy+9N=+12.8N UP → z 11.8→14m
            double z_err_s  = (goal_active_ ? target_z_ : wander_z_) - state_.z;
            // Fix 95: STABILIZE adattivo — due regimi basati su vz.
            //
            // REGIME RISALITA RAPIDA (vz > 0.30):
            //   Bug Fix 94: con vz=+1.23 pitch=-0.58, el=-0.35 DOWN crea momento nose-UP.
            //   Pitch va da -0.58 → +1.10. thr_z = TURN_THR*sin(+1.10) = +17.6N UP → rocket a z=21m.
            //   Fix: controllare il pitch verso -0.15 (leggermente nose-down) invece di z.
            //   Con pitch ≈ -0.15: thr_z = 20*sin(-0.15) = -3N DOWN. Fossen+buoyancy gestiscono quota.
            //   Gain 0.8: pitch=-0.58 → el=-0.344 DOWN (nose-UP, lento); pitch=+0.45 → el=+0.35 UP (nose-DOWN).
            //
            // REGIME NORMALE (vz ≤ 0.30):
            //   Fix 94: el = 0.15*z_err - 1.20*vz. Damping vz domina su errore quota.
            //   Con vz=-0.60 z=11.56: el=+0.35 UP (frena caduta). ✓
            double el_s;
            double thr_s = TURN_THR;
            if (state_.vz > 0.30 && z_err_s < 1.0) {
                // Fix 133B: TURN_THR invece di 0 — mantiene flusso sulle pinne per autorità.
                // Con thr=0 il momento di Munk peggiora il pitch (pitch +0.87→+1.27 osservato).
                // Fix 140: condizione z_err_s<1.0 — non frenare se ancora >1m sotto target.
                thr_s = TURN_THR;
                el_s  = +0.35;
            } else if (state_.vz < -0.5) {
                // Fix 104D: caduta rapida (vz<-0.5) → frena subito indipendentemente dal pitch.
                // Fix 102B coprì solo pitch<-0.35: a pitch=-0.07 con vz=-1.0, forward=20N
                // dà thr_z=-1.4N DOWN; il freno è troppo debole (netto UP solo +7.6N).
                // Con pitch<0: retro thr_z=+UP immediato, poi DL-FLOOR gestisce il resto.
                // Con pitch≥0: forward+el=UP: thr_z=+UP + buoyancy.
                if (state_.pitch < 0.0) {
                    thr_s = -50.0;
                    el_s  = -0.30;   // retro+el<0=UP
                } else {
                    thr_s = TURN_THR;
                    el_s  = -0.35;   // con -el: el<0 = muso giu' = salita
                }
            } else if (state_.pitch < -0.35 && state_.vz < -0.30) {
                // Fix 102B: muso giù + caduta → retro (thr_z UP con pitch<0) + el<0 (retro+el<0=UP).
                thr_s = -50.0;
                el_s  = -0.30;
            } else {
                el_s = std::clamp(-0.15 * z_err_s + 1.20 * state_.vz, -0.35, 0.35);
            }
            // Pitch guard STABILIZE: quando pitch è grande, ignora vz e correggi il pitch.
            // Senza questo, vz<-0.5 + pitch>0 comanda el=+0.35 (muso SU) peggiorando pitch.
            if (state_.pitch > 0.35) {
                el_s = std::clamp(-2.0 * state_.pitch, -0.35, 0.0);
            } else if (state_.pitch > 0.20 && el_s > 0.0) {
                el_s = 0.0;
            }
            // Fix 140: pitch guard negativo bypassa quota emergenza (z >> target con vz>0).
            // Senza il bypass: pitch=-0.25 + z=20 + vz=0.3 → el azzerato → drone non scende.
            bool z_above_urgent = (z_err_s < -2.0) && (state_.vz > 0.0);
            if (!z_above_urgent) {
                if (state_.pitch < -0.35) {
                    el_s = std::clamp(-2.0 * state_.pitch, 0.0, 0.35);
                } else if (state_.pitch < -0.20 && el_s < 0.0) {
                    el_s = 0.0;
                }
            }
            // Fix ALT: se vz>0.5 e z al/sopra target, forza DOWN indipendente dal pitch guard.
            // In STABILIZE il pitch guard negativo (pitch<-0.35) può trasformare el in positivo
            // anche quando il drone sta salendo verso quota troppo alta. Questo lo previene.
            if (goal_active_ && state_.vz > 0.5 && state_.z >= target_z_ - 0.5)
                el_s = +0.35;
            // Fix 98B: aggiungi leggero pull verso goal durante STABILIZE
            // (gain 0.06 per non interferire col damping yawrate).
            double goal_pull = goal_active_ ? 0.06 * preferred_dir() : 0.0;
            double az_s = std::clamp(-0.40 * state_.yaw_rate + goal_pull, -0.25, 0.25);
            cmd(thr_s, az_s, el_s);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[STABILIZE] yr=%.2f pitch=%.2f z=%.2f vz=%.2f az=%.2f el=%.2f",
                state_.yaw_rate, state_.pitch, state_.z, state_.vz, az_s, el_s);
            return;
        }
    }

    // ── Fix 75: pitch-dive guard ──────────────────────────────────────────
    // Dopo uscita DL-CEIL, pitch può essere -0.20→-0.45 per accumulo el<0.
    // Con forward 45N e pitch=-0.25: thr_z=45×sin(-0.25)=-11N DOWN → caduta rapida.
    // Ridurre a TURN_THR (20N) + el=+0.40 limita thr_z a -4.9N mentre el recupera pitch.
    if (state_.pitch < -0.25 && state_.vz < -0.20 && state_.z > 7.5) {
        cmd(TURN_THR, 0.0, +0.40);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[PITCH-DIVE] pitch=%.2f vz=%.2f z=%.2f → min thr+el UP",
            state_.pitch, state_.vz, state_.z);
        return;
    }

    // ── Fix 33: Hard-stuck detection ─────────────────────────────────────
    // Se la posizione non cambia di > 0.5 m in 25 sim-sec (60s in CRUISE):
    // il drone è incastrato. Anche CRUISE viene monitorato: può restare
    // incollato a una parete piatta (sonar=99, fwd_min alto) senza mai
    // passare a BRAKE/ESCAPE perché i trigger standard vengono soppressi.
    if (last_pose_t_ > 0.0) {
        double stuck_timeout_hs = (mode_ == Mode::CRUISE) ? 20.0 : 15.0;
        double dm = std::hypot(state_.x - stuck_pos_x_, state_.y - stuck_pos_y_);
        if (dm > 0.5) {
            stuck_pos_x_ = state_.x;
            stuck_pos_y_ = state_.y;
            stuck_pos_t_ = t;
        } else if ((t - stuck_pos_t_) > stuck_timeout_hs) {
            // HARD STUCK: scegli la direzione orizzontale migliore
            double baz_hs = 0.0;
            double best_hs;
            if (planner_.find_escape(baz_hs, preferred_dir())) {
                best_hs = normalize(state_.yaw + baz_hs);
            } else {
                double lat_r = planner_.wall_dist_in_dir(normalize(state_.yaw - M_PI / 2));
                double lat_l = planner_.wall_dist_in_dir(normalize(state_.yaw + M_PI / 2));
                double center_yw = std::atan2(-state_.y, -state_.x);
                double center_d  = planner_.wall_dist_in_dir(center_yw);
                if (center_d > 15.0)
                    best_hs = center_yw;
                else
                    best_hs = (lat_r > lat_l)
                        ? normalize(state_.yaw - M_PI / 2)
                        : normalize(state_.yaw + M_PI / 2);
            }

            // Fuga verticale solo se abilitata
            double rescue_depth = goal_active_
                ? std::clamp(target_z_, 4.0, 15.0)
                : std::clamp(wander_z_, 4.0, 15.0);
            bool vert_hs = false;
            if (allow_vert_esc_) {
                bool ok_v = planner_.find_vertical_escape(rescue_depth, nullptr, APPROACH_DIST);
                if (!ok_v) rescue_depth = (state_.z > 9.0) ? 4.0 : 16.0;
                vert_hs = true;
            }

            RCLCPP_WARN(get_logger(),
                "[HARD STUCK] pos=(%.1f,%.1f) immobile da %.0f s — %s→z=%.1f hz→%.2f rad",
                state_.x, state_.y, t - stuck_pos_t_,
                vert_hs ? "VERT" : "HORIZ", rescue_depth, best_hs);
            escape_yaw_         = best_hs;
            escape_depth_       = rescue_depth;
            vertical_escape_    = vert_hs;
            stuck_escape_until_ = t + 20.0;   // 20s di ESCAPE garantita
            wall_stuck_until_   = 0.0;
            sonar_brake_until_  = t + 5.0;    // 5s prima che sonar_brake ri-triggeri
            wall_fwd_brake_until_ = 0.0;
            post_brake_thr_until_ = 0.0;
            brake_hold_until_   = 0.0;
            mode_               = Mode::ESCAPE;
            mode_until_         = t + 20.0;
            plan_t_             = t;
            stuck_count_        = 0;
            stuck_pos_t_        = t;           // evita re-trigger immediato
        }
    }

    // ── Gestione BRAKE ────────────────────────────────────────────────────
    double raw_wall_fwd = planner_.wall_fwd_dist();

    // Fix 31: [BRAKE wall] — se c'è spazio libero dietro (back > 2×BRAKE_DIST=10m),
    // mantieni la retromarcia invece di sopprimere BRAKE per 15s.
    // La soppressione 15s serviva per dare tempo di girare, ma con V=0 (pinna senza
    // autorità) il drone rimane immobile per 15 sim-secondi sprecati. Se il retro è
    // libero, bastano 8s di retromarcia per costruire V_back e poi poter sterzare.
    // Fix 37: soglia ridotta da BLOCKED_DIST a 8m — Fix31a estendeva retromarcia fino a
    // che wall_fwd raggiungeva 16m (30+ sim-sec a 0.5m/s) causando sink di 3-6m.
    // Limitare a 8m riduce la durata massima a ~14s dal punto di ingresso a 1m.
    if (mode_ == Mode::BRAKE && raw_wall_fwd < 8.0
        && (t - brake_entry_t_) > 2.0 && t >= stuck_escape_until_) {
        double back_wall_bw = planner_.wall_dist_in_dir(state_.yaw + M_PI);
        if (back_wall_bw > BRAKE_DIST * 2.0) {
            // Retro libero: prolunga retromarcia (ridotto da 8s a 3s per limitare sink)
            brake_hold_until_ = std::max(brake_hold_until_, t + 3.0);
            RCLCPP_WARN(get_logger(),
                "[BRAKE wall back-ok] fwd=%.1f back=%.1f m, retromarcia estesa 3s",
                raw_wall_fwd, back_wall_bw);
        } else {
            // Incastrato (muro anche dietro): uscita rapida verso arena
            double wall_window = (vertical_escape_ && raw_wall_fwd >= BRAKE_DIST + 3.0)
                                 ? 3.0 : 15.0;
            stuck_escape_until_   = t + 15.0;
            wall_stuck_until_     = t + wall_window;
            sonar_brake_until_    = std::max(sonar_brake_until_, t + 15.0);
            wall_fwd_brake_until_ = t + 3.0;
            RCLCPP_WARN(get_logger(),
                "[BRAKE wall] parete a %.1f m, uscita rapida verso arena", raw_wall_fwd);
        }
    }

    double eff_brake_dist = BRAKE_DIST;
    if (!vertical_escape_ && raw_wall_fwd < BLOCKED_DIST && eff_brake_dist < APPROACH_DIST)
        eff_brake_dist = APPROACH_DIST;

    // Fix 105A: parete INTERNA (cubo ±inner_wx/wy) trattata come parete fisica.
    double inner_fwd = planner_.inner_wall_dist_in_dir(state_.yaw);
    // Fix 109A: inner wall check MAI soppresso da stuck_escape_until_ né wall_fwd_brake_until_.
    // La parete interna è un vincolo duro: il drone non deve mai uscire dal cubo ±60m.
    // Il cubo interno NON è un muro duro quando esiste un goal.
    // In goal mode il Tethys deve poter raggiungere anche target
    // esterni al cubo interno (es. target_y=+70 con inner_y=+60).
    bool inner_wall_brake =
        !goal_active_ &&
        (inner_fwd < BRAKE_DIST &&
        inner_fwd < raw_wall_fwd);
    bool wall_imminent  = ((raw_wall_fwd < BRAKE_DIST + 3.0) && (t >= stuck_escape_until_))
        || inner_wall_brake;
    bool emergency      = (fwd_min < BRAKE_DIST * 0.5) && (t >= wall_stuck_until_);
    // Fix E: parete < 5m in direzione corrente → BRAKE con cooldown 3s indipendente dai 15s
    // In CRUISE il timer di soppressione NON si applica: il drone può essere incollato
    // a una parete piatta (sonar=99) e il BRAKE deve scattare sempre.
    bool wall_fwd_near  = ((raw_wall_fwd < BRAKE_DIST) && (mode_ == Mode::CRUISE || t >= wall_fwd_brake_until_))
        || inner_wall_brake;  // Fix 109A: inner wall mai soppresso da wall_fwd_brake_until_
    // Fix 28: durante ESCAPE ALIGN non interrompere la svolta con BRAKE (sonar o main).
    // wall_imminent rimane attivo: sicurezza assoluta se il heading corrente entra in parete.
    // Durante escape laterale (escape_lateral_), abbassa la soglia a 3m: con lookahead
    // e TURN_BOOST_THR il drone deve completare l'arco senza che BRAKE interrompa a 6m.
    double in_align_brake_floor = (escape_lateral_ && !vertical_escape_) ? 3.0 : BRAKE_DIST;
    bool in_align_turn = (mode_ == Mode::ESCAPE) && !vertical_escape_
        && std::abs(normalize(escape_yaw_ - state_.yaw)) > 0.30
        && fwd_min >= in_align_brake_floor  // 3m in lateral escape, BRAKE_DIST altrove
        && (escape_lateral_ || fwd_min < APPROACH_DIST + 2.0)
        && std::abs(state_.yaw_rate) > 0.01;  // Fix A: consente BRAKE se rotazione bloccata

    // ── TURN FEASIBILITY: la curva sta chiudendo davvero? ─────────────────
    // Osservato prima di una collisione: fwd 11.1→7.6→4.1→0.25 mentre yaw_err
    // restava a 0.30-0.44 con barra gia' satura. Il drone vedeva l'ostacolo e
    // sterzava, ma il raggio di virata era troppo ampio per lo spazio rimasto.
    double turn_yaw_err   = 0.0;
    bool   turn_not_closing = false;
    bool   turn_yaw_improving = false;
    if (mode_ == Mode::ESCAPE && !vertical_escape_) {
        turn_yaw_err = std::abs(normalize(escape_yaw_ - state_.yaw));
        double turn_dt = t - prev_turn_t_;
        bool have_prev = prev_turn_t_ > 0.0 && turn_dt > 0.01 && turn_dt < 1.0
            && std::abs(normalize(escape_yaw_ - prev_turn_target_)) < 0.10;
        bool obstacle_getting_closer = have_prev && fwd_min < prev_turn_fwd_min_ - 0.15;
        turn_yaw_improving = have_prev &&
            (prev_turn_yaw_err_ - turn_yaw_err) / turn_dt > 0.03;
        bool yaw_not_improving = !turn_yaw_improving;
        bool dangerous_turn_zone     = fwd_min < 10.0 && turn_yaw_err > 0.35;
        turn_not_closing = dangerous_turn_zone && obstacle_getting_closer && yaw_not_improving;
        if (turn_not_closing) {
            ++turn_not_closing_count_;
            turn_brake_until_ = t + 1.5;
            RCLCPP_WARN(get_logger(),
                "[TURN-NOT-CLOSING] fwd %.2f→%.2f yaw_err %.2f→%.2f yr=%.2f count=%d",
                prev_turn_fwd_min_, fwd_min, prev_turn_yaw_err_, turn_yaw_err,
                state_.yaw_rate, turn_not_closing_count_);
        }
        prev_turn_fwd_min_ = fwd_min;
        prev_turn_yaw_err_ = turn_yaw_err;
        prev_turn_t_       = t;
        prev_turn_target_  = escape_yaw_;
    } else {
        prev_turn_fwd_min_      = 99.0;
        prev_turn_yaw_err_      = 0.0;
        prev_turn_t_            = -1.0;
        turn_not_closing_count_ = 0;
    }

    // ── 3) curva che non chiude: BRAKE non va piu' soppresso ──────────────
    if (turn_not_closing || t < turn_brake_until_) {
        in_align_turn = false;
        sonar_brake_until_ = std::min(sonar_brake_until_, t);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "[TURN-BRAKE-ALLOW] curva non chiude: permetto BRAKE fwd=%.1f yaw_err=%.2f",
            fwd_min, turn_yaw_err);
    }
    // In goal mode: sonar_brake usa corridor_clearance(0.0), non forward_min_range.
    // fwd_min < EMERGENCY_COLLISION_DIST (3m) è override assoluto vero pericolo fisico.
    double sonar_corr  = sm_.received ? planner_.corridor_clearance(0.0) : 99.0;
    // Fix 17c+21: sonar_brake bypassa stuck_escape_until_ ma è soppresso da sonar_brake_until_
    bool sonar_brake   = !vertical_escape_
        && !in_align_turn
        && (sonar_corr < eff_brake_dist || fwd_min < EMERGENCY_COLLISION_DIST)
        && (t >= sonar_brake_until_);

    // Fix 142: in committed_wp, sopprime BRAKE/SIDE-AVOID se fwd>1.4 e nessuna parete fisica.
    // Soglia 1.4m (era 0.6): con circumnav, un drone che segue un wp laterale non deve
    // frenare fino a 0.6m dal cilindro — a 1.4m è già troppo vicino, meglio creare nuovo wp.
    // Soglia alzata da 1.4 a WP_BRAKE_FLOOR: a 1.4 m un AUV che ha bisogno di metri
    // per fermarsi non ha più spazio, e il contatto diventa geometricamente inevitabile
    // (osservato: "no brake" a 1.6 m, frenata solo a 0.55 m, collisione).
    const bool curve_brake_required = !vertical_escape_ &&
        (sonar_corr < EMERGENCY_COLLISION_DIST + front_sensor_offset ||
         (t < turn_brake_until_ && sonar_corr < BRAKE_DIST));
    bool committed_wp_skip = committed_wp && fwd_min > WP_BRAKE_FLOOR
        && raw_wall_fwd > 8.0 && !curve_brake_required;
    if (committed_wp_skip) {
        double wp_dist_cw = std::hypot(local_wp_x_ - state_.x, local_wp_y_ - state_.y);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[COMMITTED-WP] fwd=%.1f wp_dist=%.1f no brake", fwd_min, wp_dist_cw);
    } else if (committed_wp && fwd_min <= WP_BRAKE_FLOOR) {
        // Scala di sicurezza: sopra WP_BRAKE_FLOOR si segue il waypoint senza frenare;
        // fra WP_DANGER_FLOOR e WP_BRAKE_FLOOR si tenta ancora un aggiramento laterale;
        // sotto WP_DANGER_FLOOR si cede a BRAKE, perché non c'è più spazio per manovrare.
        if (circum_active_ && goal_active_ && sm_.received && fwd_min > WP_DANGER_FLOOR
            && !curve_brake_required) {
            // Keep the spatial waypoint while it is safe. Recreating it
            // at heading+60 degrees each tick moved the target with the
            // vehicle and made the turn error stay at 60 degrees forever.
            double current_wp_yaw = std::atan2(local_wp_y_-state_.y, local_wp_x_-state_.x);
            if (planner_.corridor_clearance(normalize(current_wp_yaw-state_.yaw)) < EMERGENCY_COLLISION_DIST) {
                double baz = 0.0, clr = 0.0;
                if (planner_.find_lookahead_escape(baz, preferred_dir(), &clr)) {
                    escape_yaw_ = normalize(state_.yaw+baz);
                    create_local_waypoint(escape_yaw_, clr);
                }
            }
            // Comanda subito verso il nuovo wp: evitiamo che il tick cada nel BRAKE sotto.
            double wp_yaw_cw = std::atan2(local_wp_y_ - state_.y, local_wp_x_ - state_.x);
            double az_cw = std::clamp(normalize(wp_yaw_cw - state_.yaw), -0.40, 0.40);
            double z_err_cw = target_z_ - state_.z;
            double el_cw = std::clamp(0.18 * z_err_cw - 0.45 * state_.vz, -0.35, 0.35);
            cmd(TURN_THR, az_cw, el_cw);
            return;
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[WP-CLOSE-RETURN] cono=%.1f corridoio=%.1f: verifico condizioni di frenata",
                fwd_min, sonar_corr);
        }
    }

    if (!committed_wp_skip
        && ((!in_align_turn && ((sonar_corr < eff_brake_dist || fwd_min < EMERGENCY_COLLISION_DIST)
            && (t >= stuck_escape_until_ || emergency)))
        || wall_imminent || sonar_brake || wall_fwd_near || curve_brake_required))
    {
        if (mode_ != Mode::BRAKE) {
            // [BRAKE-REASON]: distingui causa per diagnostica
            const char* brk_reason =
                (wall_imminent || wall_fwd_near) ? "WALL" :
                (emergency || fwd_min < EMERGENCY_COLLISION_DIST) ? "EMERGENCY" : "CORRIDOR";
            RCLCPP_WARN(get_logger(),
                "[BRAKE-REASON] reason=%s corridor_clr=%.1f fwd=%.1f wall=%.1f",
                brk_reason, sonar_corr, fwd_min, raw_wall_fwd);
            brake_begin_t_ = t;  // Fix G2: reset ad ogni ingresso in BRAKE
            if (mode_ == Mode::CRUISE || (t - brake_entry_t_) > 20.0) {
                brake_entry_t_ = t;
                stuck_count_   = 0;
            }
        }
        // stuck_in_block: se in BRAKE da >5s prova a forzare ESCAPE al tick successivo.
        // Fix 53: ridotto 12→5s; rimossa condizione fwd_min>0.2 (bloccava il trigger in
        // contatto fisico fwd_min=0.10m); aggiunto fwd_min<0.5 come trigger esplicito per
        // contatto fisico. Il meccanismo setta sonar_brake_until_=t+4s che sopprime
        // brake_now da sonar per 4s, permettendo la transizione BRAKE→ESCAPE.
        if (mode_ == Mode::BRAKE && !curve_brake_required) {
            bool stuck_in_block = (t - brake_entry_t_) > 5.0
                && (fwd_min < 0.5                          // contatto fisico prolungato
                    || fwd_min >= BRAKE_DIST
                    || raw_wall_fwd < BRAKE_DIST + 3.0
                    || (t - brake_entry_t_) > 15.0);
            if (stuck_in_block) {
                // Fix 33b: vicino alle pareti nessun backoff esponenziale (le finestre 32s
                // con V=0 e pinne senza autorità sprecano tempo → Hard Stuck interviene prima).
                double stuck_dur = (raw_wall_fwd < BRAKE_DIST + 3.0)
                    ? 15.0
                    : std::max(4.0, 4.0 * (1 << std::min(stuck_count_, 3)));
                stuck_escape_until_   = t + stuck_dur;
                sonar_brake_until_    = std::max(sonar_brake_until_, t + stuck_dur);  // Fix 21
                wall_stuck_until_     = t + (raw_wall_fwd < BLOCKED_DIST
                    ? stuck_dur : std::min(stuck_dur, 3.0));
                wall_fwd_brake_until_ = std::max(wall_fwd_brake_until_, t + stuck_dur);
                RCLCPP_WARN(get_logger(),
                    "[BRAKE stuck-wall] %.0f s (wall=%.1fm fwd=%.2f) uscita forzata finestra=%.0fs #%d",
                    t - brake_entry_t_, raw_wall_fwd, fwd_min, stuck_dur, stuck_count_ + 1);
                ++stuck_count_;
                return;
            }
        }
        if (curve_brake_required) {
            // This branch must precede SIDE-AVOID: its positive thrust used
            // to undo an emergency brake even at 1-3 metres from an obstacle.
            mode_ = Mode::BRAKE;
            local_wp_active_ = false;
            double az_brake = std::clamp(1.2 * normalize(escape_yaw_ - state_.yaw)
                - 0.8 * state_.yaw_rate, -0.42, 0.42);
            double back_clear = std::min(planner_.wall_dist_in_dir(state_.yaw + M_PI),
                obstacle_memory_.clearance(state_.x, state_.y, state_.z, state_.yaw + M_PI, t));
            safety_braking_ = true;
            brake_stop_reverse_ = false;
            double brake_thrust = recovery_thrust(forward_speed_, back_clear, false);
            cmd(brake_thrust, az_brake, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[CURVE-BRAKE] fwd=%.2f yaw_err=%.2f thr=%.1f", fwd_min,
                normalize(escape_yaw_ - state_.yaw), brake_thrust);
            return;
        }
        if (mode_ == Mode::CRUISE) {
            brake_entry_t_ = t;
            brake_begin_t_ = t;
            // In goal mode con fwd_min >= 1.0m: SIDE-AVOID diretto, niente retromarcia.
            // La retromarcia può portare a collisioni posteriori e perde l'autorità delle pinne.
            if (goal_active_ && fwd_min >= 1.0 && sm_.received) {
                double baz_sa = 0.0, len_sa = 0.0;
                bool found_sa = planner_.find_lookahead_escape(baz_sa, preferred_dir(), &len_sa);
                if (!found_sa) found_sa = planner_.find_escape(baz_sa, preferred_dir());
                if (found_sa) {
                    escape_yaw_       = normalize(state_.yaw + baz_sa);
                    escape_depth_     = std::clamp(target_z_, 4.0, 15.0);
                    vertical_escape_  = false;
                    escape_lateral_   = (std::abs(normalize(baz_sa)) > M_PI / 4.0);
                    mode_             = Mode::ESCAPE;
                    escape_start_t_   = t;
                    mode_until_       = t + (escape_lateral_ ? 25.0 : 12.0);
                    plan_t_           = t;
                    sonar_brake_until_   = std::max(sonar_brake_until_,   t + 2.0);
                    stuck_escape_until_  = std::max(stuck_escape_until_,  t + 2.0);
                    goal_clear_since_ = -1.0;
                    create_local_waypoint_throttled(escape_yaw_, len_sa, t);
                    circum_active_ = true;
                    circum_side_   = (baz_sa >= 0) ? +1 : -1;
                    circum_until_  = t + 12.0;
                    RCLCPP_WARN(get_logger(),
                        "[SIDE-AVOID] az=%.0f° len=%.1f thr=TURN_THR fwd=%.1f → ESCAPE+circum side=%+d",
                        baz_sa * 180.0 / M_PI, len_sa, fwd_min, circum_side_);
                    // Invia comando ESCAPE-like per questo tick
                    double az_lim_sa = steer_limit(fwd_min);
                    double az_sa = local_wp_active_
                        ? std::clamp(normalize(escape_yaw_ - state_.yaw), -az_lim_sa, az_lim_sa)
                        : std::clamp(1.2 * normalize(escape_yaw_ - state_.yaw), -az_lim_sa, az_lim_sa);
                    double z_err_sa  = escape_depth_ - state_.z;
                    double des_p_sa  = std::clamp(+0.18 * z_err_sa, -0.20, 0.20);
                    double el_sa     = std::clamp(1.4*(des_p_sa - state_.pitch) - 0.45*state_.vz, -0.35, 0.35);
                    cmd(TURN_THR, az_sa, el_sa);
                    return;
                }
                RCLCPP_WARN(get_logger(),
                    "[NO-REVERSE] nessuna dir libera goal mode, emergenza fwd=%.1f", fwd_min);
            }
            // Non-goal o vera emergenza (fwd_min < 1.0m): coast standard
            mode_ = Mode::BRAKE;
            cmd(0.0, 0.0, 0.0);
            RCLCPP_WARN(get_logger(),
                "[CRUISE→BRAKE] ostacolo imminente fwd=%.2f → coast", fwd_min);
            return;
        }
        // In goal mode con fwd_min >= 1.0m: SIDE-AVOID — thr positivo, sterzo laterale, no retro.
        // La retromarcia causa collisioni posteriori e perde autorità delle pinne (V<0).
        if (goal_active_ && fwd_min >= 1.0) {
            // Tenta transizione a ESCAPE laterale se sonar disponibile
            if (sm_.received) {
                double baz_sa = 0.0, len_sa = 0.0;
                bool found_sa = planner_.find_lookahead_escape(baz_sa, preferred_dir(), &len_sa);
                if (!found_sa) found_sa = planner_.find_escape(baz_sa, preferred_dir());
                if (found_sa) {
                    escape_yaw_       = normalize(state_.yaw + baz_sa);
                    escape_depth_     = std::clamp(target_z_, 4.0, 15.0);
                    vertical_escape_  = false;
                    escape_lateral_   = (std::abs(normalize(baz_sa)) > M_PI / 4.0);
                    mode_             = Mode::ESCAPE;
                    escape_start_t_   = t;
                    mode_until_       = t + (escape_lateral_ ? 25.0 : 12.0);
                    plan_t_           = t;
                    sonar_brake_until_  = std::max(sonar_brake_until_,  t + 2.0);
                    stuck_escape_until_ = std::max(stuck_escape_until_, t + 2.0);
                    goal_clear_since_ = -1.0;
                    create_local_waypoint_throttled(escape_yaw_, len_sa, t);
                    circum_active_ = true;
                    circum_side_   = (baz_sa >= 0) ? +1 : -1;
                    circum_until_  = t + 12.0;
                    RCLCPP_WARN(get_logger(),
                        "[SIDE-AVOID] az=%.0f° len=%.1f thr=TURN_THR fwd=%.1f → ESCAPE+circum side=%+d",
                        baz_sa * 180.0 / M_PI, len_sa, fwd_min, circum_side_);
                }
            }
            // Comanda thr positivo verso waypoint + depth hold (sia in BRAKE che in ESCAPE)
            double az_lim_sa2 = steer_limit(fwd_min);
            double az_sa = 0.0;
            if (local_wp_active_) {
                double wp_yaw = std::atan2(local_wp_y_ - state_.y, local_wp_x_ - state_.x);
                az_sa = std::clamp(normalize(wp_yaw - state_.yaw), -az_lim_sa2, az_lim_sa2);
            } else if (mode_ == Mode::ESCAPE) {
                az_sa = std::clamp(1.2 * normalize(escape_yaw_ - state_.yaw)
                                   - 0.8 * state_.yaw_rate, -az_lim_sa2, az_lim_sa2);
            }
            double z_err_sa = (target_z_ - state_.z);
            double des_p_sa = std::clamp(+0.18 * z_err_sa, -0.20, 0.20);
            double el_sa    = std::clamp(1.4*(des_p_sa - state_.pitch) - 0.45*state_.vz, -0.35, 0.35);
            cmd(TURN_THR, az_sa, el_sa);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[NO-REVERSE] goal SIDE-AVOID thr=%.0f az=%.0f° z=%.1f→%.1f fwd=%.1f",
                TURN_THR, az_sa * 180.0 / M_PI, state_.z, target_z_, fwd_min);
            if (fwd_min < 0.25 && (t - last_collision_t_) > 3.0) {
                ++near_contact_count_; last_collision_t_ = t;
                RCLCPP_WARN(get_logger(), "[NEAR-CONTACT #%d] fwd_min=%.2f m pos=(%.1f,%.1f) z=%.2f",
                    near_contact_count_, fwd_min, state_.x, state_.y, state_.z);
            }
            return;
        }
        // Fix 22a: vicino al fondale usa sempre el=+0.45 (thr=0, elevator non invertito)
        bool floor_near_prelim = (state_.z < 2.0);
        double el_brake = floor_near_prelim ? +0.45
                        : (vertical_escape_ && state_.vz > 0.05) ? +0.45 : 0.0;
        {
            double back_wall = planner_.wall_dist_in_dir(state_.yaw + M_PI);
            bool floor_near  = (state_.z < 2.0);
            if (back_wall < BRAKE_DIST || floor_near) {
                bool cornered = (back_wall < BRAKE_DIST && fwd_min < APPROACH_DIST);
                if (cornered)
                    el_brake = (state_.z > wander_z_ + 1.0) ? -0.45 : +0.45;
                double safe_thr = (cornered || floor_near) ? TURN_THR : TURN_THR;
                cmd(safe_thr, 0.0, el_brake);
                if (back_wall < BRAKE_DIST)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[BRAKE] muro dietro a %.1f m, no retromarcia", back_wall);
                if (floor_near)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[BRAKE] vicino al fondale z=%.2f, no retromarcia", state_.z);
            } else {
                // In retromarcia: el>0=DOWN, el<0=UP (CLAUDE.md).
                // Fix 55: quando z ≥ target applica el=+0.45 (DOWN max in retro) per
                // contrastare la galleggiabilità (+9N). Empirico: el=+0.10 era troppo
                // debole (2.8N < 9N buoy) → z saliva 3.54m in 67s di BRAKE.
                // Con el=+0.45 e v≥0.3m/s: LiftDrag_down=12.4N > 9N → z stabile.
                if (state_.z >= wander_z_) {
                    el_brake = +0.45;
                } else if (!floor_near_prelim && !(vertical_escape_ && state_.vz > 0.05)) {
                    el_brake = 0.0;  // z sotto target: galleggiabilità risolleva naturalmente
                }
                // Fix 52: az=0 in retromarcia — confermato empiricamente che az=±0.40
                // producono la stessa rotazione CCW (le pinne non hanno autorità a V<0).
                // Fix 68B: con pitch>0.25 (muso su), retromarcia a -50N eccita oscillazione
                // Fossen: il flusso invertito amplifica il pitch fino a -1.246 rad (70°).
                // A -20N il rallentamento è sufficiente e i dinamismi restano stabili.
                double brake_thr_raw = (state_.pitch > 0.25) ? -20.0 : -50.0;
                cmd(brake_thr_raw, 0.0, el_brake);
                // In goal mode: non estendere brake_hold_until_ — ESCAPE deve poter intervenire
                // subito. La retromarcia lunga in goal mode è la causa delle collisioni posteriori.
                if (!goal_active_ && (fwd_min < BRAKE_DIST || raw_wall_fwd < BLOCKED_DIST))
                    brake_hold_until_ = std::max(brake_hold_until_, t + 3.0);
            }
        }
        mode_ = Mode::BRAKE;
        if (wall_fwd_near) {
            wall_fwd_brake_until_ = t + 3.0;  // Fix E: cooldown dopo trigger
            // Fix 30b: in VERT ESCAPE con angolo (yaw_err > 90°), la retromarcia one-tick
            // non basta a liberarsi dal muro. Teniamo BRAKE per 5s sim così il drone accumula
            // V_backward sufficiente (~13 m di distanza) e poi in ESCAPE le pinne hanno autorità.
            if (vertical_escape_
                && std::abs(normalize(escape_yaw_ - state_.yaw)) > M_PI / 2.0) {
                brake_hold_until_ = std::max(brake_hold_until_, t + 5.0);
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[BRAKE wall-fwd-near] parete a %.1f m in heading corrente", raw_wall_fwd);
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "[BRAKE] fwd_min=%.2f z=%.2f", fwd_min, state_.z);
        if (fwd_min < 0.25 && (t - last_collision_t_) > 3.0) {
            ++near_contact_count_;
            last_collision_t_ = t;
            RCLCPP_WARN(get_logger(),
                "[NEAR-CONTACT #%d] fwd_min=%.2f m pos=(%.1f,%.1f) z=%.2f",
                near_contact_count_, fwd_min, state_.x, state_.y, state_.z);
        }
        return;
    }

    // ── Transizione BRAKE → ESCAPE ────────────────────────────────────────
    if (mode_ == Mode::BRAKE) {
        // Fix 30b: mantieni retromarcia piena per brake_hold_until_ (impostato da wall_fwd_near
        // in VERT ESCAPE con angolo di svolta > 90°). Senza questo, il ciclo BRAKE one-tick →
        // ESCAPE 40N contro muro non lascia mai spazio sufficiente per girare.
        if (t < brake_hold_until_) {
            double back_wall_h = planner_.wall_dist_in_dir(state_.yaw + M_PI);
            bool retromarcia_h = !(back_wall_h < BRAKE_DIST || state_.z < 2.0);
            // Fix 37/40: stesso trattamento anti-sink della sezione BRAKE principale
            // Fix 51: -0.15=DOWN affondava il drone; +0.10=UP lieve risalita
            double el_h = (state_.z < 2.0) ? +0.45
                        : (retromarcia_h && state_.z < wander_z_) ? +0.10
                        : 0.0;
            if (retromarcia_h)
                cmd(-50.0, 0.0, el_h);
            else
                cmd(TURN_THR, 0.0, el_h);
            return;
        }
        bool stuck = (t - brake_entry_t_) > 5.0
            && (fwd_min >= BRAKE_DIST || wall_imminent || (t - brake_entry_t_) > 30.0);
        if (stuck) {
            // Fix 33b: vicino alle pareti nessun backoff (finestre costanti 15s)
            double stuck_dur = (raw_wall_fwd < BRAKE_DIST + 3.0)
                ? 15.0
                : std::max(4.0, 4.0 * (1 << std::min(stuck_count_, 3)));
            stuck_escape_until_ = t + stuck_dur;
            wall_stuck_until_   = t + (raw_wall_fwd < BLOCKED_DIST
                ? stuck_dur : std::min(stuck_dur, 3.0));
            RCLCPP_WARN(get_logger(),
                "[BRAKE stuck] %.0f s bloccato, uscita forzata fwd_min=%.2f (finestra=%.0fs #%d)",
                t - brake_entry_t_, fwd_min, stuck_dur, stuck_count_ + 1);
            ++stuck_count_;
        }
        if (!sm_.received && !stuck) {
            cmd(0.0, 0.0, 0.0);
            return;
        }
        // Fix G3: se muro ancora vicino nel heading corrente e non stuck → retromarcia piena
        // finché wall_fwd > BRAKE_DIST+3. Evita ESCAPE ALIGN con muro a <8m e thr=28.
        // -50 (non -12): il drone arriva da ESCAPE a thr=28 con V≈1 m/s; -12 non frena abbastanza.
        if (!vertical_escape_ && raw_wall_fwd < BRAKE_DIST + 3.0 && !stuck) {
            double back_wall = planner_.wall_dist_in_dir(state_.yaw + M_PI);
            if (back_wall > BRAKE_DIST) {
                if (goal_active_) {
                    // In goal mode: SIDE-AVOID — la retromarcia può causare collisioni posteriori.
                    // Questo ramo non dovrebbe essere raggiunto in goal mode (CRUISE→BRAKE già converte).
                    // Se siamo qui: thr positivo + depth hold come sicurezza.
                    double z_err_hold = target_z_ - state_.z;
                    double des_p_hold = std::clamp(+0.18 * z_err_hold, -0.20, 0.20);
                    double el_hold    = std::clamp(1.4*(des_p_hold - state_.pitch) - 0.45*state_.vz, -0.35, 0.35);
                    double az_hold    = local_wp_active_
                        ? std::clamp(normalize(std::atan2(local_wp_y_ - state_.y, local_wp_x_ - state_.x) - state_.yaw), -0.40, 0.40)
                        : 0.0;
                    cmd(TURN_THR, az_hold, el_hold);
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[BACKOFF-DISABLED] goal mode: creep+ muro fwd=%.1f z=%.1f→%.1f",
                        raw_wall_fwd, state_.z, target_z_);
                } else {
                    // Non-goal: piccola retro con z hold (el in retromarcia: el>0=DOWN)
                    double hold_z  = wander_z_;
                    double el_hold = (state_.z >= hold_z + 0.3) ? +0.30
                                   : (state_.z >= hold_z)        ? +0.10
                                   : 0.0;
                    cmd(-TURN_THR, 0.0, el_hold);
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[BRAKE-CREEP] muro fwd=%.1f m, retro z=%.1f→%.1f (back=%.1f)",
                        raw_wall_fwd, state_.z, hold_z, back_wall);
                }
                return;
            }
        }
        // Se libero post-BRAKE: salta ESCAPE e torna in CRUISE (o ESCAPE-ALIGN).
        // In goal mode: esci solo se il corridoio verso il target è libero (goal-first).
        // In non-goal mode: comportamento precedente (fwd_min/blocked based).
        double brake_g_rel = goal_active_ ? preferred_dir() : 0.0;
        double brake_g_clr = (goal_active_ && sm_.received)
            ? planner_.corridor_clearance(brake_g_rel) : 99.0;
        bool brake_clear = goal_active_
            ? (brake_g_clr >= SAFE_PASS_DIST && raw_wall_fwd > BRAKE_DIST + 2.0)
            : (!blocked && fwd_min > CLEARED_DIST && raw_wall_fwd > CLEARED_DIST);
        if (brake_clear) {
            // Fix 52: se goal attivo e grande errore di heading, entra in ESCAPE-ALIGN
            // per girare a TURN_THR invece di accelerare a CRUISE_THR verso la direzione sbagliata
            // (drone 1 faceva un'escursione di 80m a est prima di correggere).
            if (goal_active_) {
                double dx_g  = target_x_ - state_.x;
                double dy_g  = target_y_ - state_.y;
                double goal_dir = std::atan2(dy_g, dx_g);
                double yaw_err_to_goal = std::abs(normalize(goal_dir - state_.yaw));
                if (yaw_err_to_goal > 1.5) {
                    escape_yaw_           = goal_dir;
                    escape_depth_         = std::clamp(target_z_, 4.0, 15.0);
                    vertical_escape_      = false;
                    mode_                 = Mode::ESCAPE;
                    mode_until_           = t + 20.0;
                    plan_t_               = t;
                    post_brake_thr_until_ = t + 5.0;
                    post_brake_long_      = (t - brake_begin_t_) > 3.0;
                    stuck_count_          = 0;
                    RCLCPP_INFO(get_logger(),
                        "[BRAKE→ESCAPE-ALIGN] yaw_err=%.2f rad → goal=%.2f",
                        yaw_err_to_goal, goal_dir);
                    return;
                }
            }
            mode_ = Mode::CRUISE;
            stuck_count_ = 0;
            vertical_escape_ = false;
            if (goal_active_)
                RCLCPP_INFO(get_logger(),
                    "[BRAKE→GOAL] goal_clr=%.1f fwd=%.2f, riprendo navigazione target",
                    brake_g_clr, fwd_min);
            else
                RCLCPP_INFO(get_logger(), "[BRAKE→CRUISE] libero fwd=%.2f wall=%.2f",
                            fwd_min, raw_wall_fwd);
            return;
        }

        bool prev_vert       = vertical_escape_;
        double prev_esc_depth = escape_depth_;

        double baz = 0.0, new_depth;
        int    el_span = 0;
        double path_len_brake = 0.0;
        bool horiz_lh = planner_.find_lookahead_escape(baz, lookahead_preferred_dir(fwd_min), &path_len_brake);
        bool horiz    = horiz_lh;
        if (!horiz) horiz = planner_.find_escape(baz, lookahead_preferred_dir(fwd_min));
        bool vert    = allow_vert_esc_ && planner_.find_vertical_escape(new_depth, &el_span);
        // Fix I: preferisce sempre la fuga orizzontale quando disponibile (|baz|<π/2).
        // L'el_span<=3 (trave/sfera) non forza più il verticale se esiste un'apertura
        // orizzontale: girare è più veloce che cambiare quota (idrodinamica heave lenta).
        // Il verticale rimane fallback quando horiz=false o porta all'indietro.
        // Fix J: in goal mode mai fuga verticale — target ha quota fissa.
        // Fix 73: eccezione Fix J — permetti fuga verticale in goal mode quando la
        // direzione orizzontale è >120° dalla direzione goal (andare indietro di 156°
        // è peggio che cambiare quota di qualche metro per scavalcare un ostacolo).
        double goal_dir_bk = goal_active_
            ? normalize(std::atan2(target_y_ - state_.y, target_x_ - state_.x) - state_.yaw)
            : 0.0;
        bool horiz_very_backward = horiz
            && std::abs(normalize(baz)) > M_PI / 2.0
            && goal_active_
            && std::abs(normalize(baz - goal_dir_bk)) > 2.0 * M_PI / 3.0;
        // Fix 130: permetti fuga verticale solo se lookahead non ha trovato nulla e
        // find_escape (fallback) punta >45° lontano dal goal. Quando lookahead ha
        // verificato un corridoio libero, ci si fida di lui indipendentemente dall'angolo.
        bool horiz_off_goal = !horiz_lh && horiz && goal_active_
            && std::abs(normalize(baz - goal_dir_bk)) > M_PI / 4.0;
        bool use_vert = vert && (!horiz || std::abs(normalize(baz)) > M_PI / 2.0 || horiz_off_goal)
                        && (!goal_active_ || horiz_very_backward || horiz_off_goal);

        escape_yaw_ = use_vert ? state_.yaw
                    : (horiz ? normalize(state_.yaw + baz) : state_.yaw);
        // Fix 57: in goal mode, se la direzione di fuga punta opposta al goal
        // (errore > 2 rad) e il drone è bloccato da molti stucks, usa il goal
        // direttamente come direzione di fuga per evitare di allontanarsi.
        if (goal_active_ && !use_vert && stuck_count_ >= 2) {
            double goal_dir = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
            double esc_err  = std::abs(normalize(escape_yaw_ - goal_dir));
            if (esc_err > 2.0) {
                escape_yaw_ = goal_dir;
                RCLCPP_INFO(get_logger(),
                    "[BRAKE-STUCK-GOAL] esc invertita (err=%.1f rad) → goal=%.2f",
                    esc_err, escape_yaw_);
            }
        }

        bool same_dir = use_vert && prev_vert
            && ((prev_esc_depth - state_.z) * (new_depth - state_.z) > 0.0);

        if (use_vert) {
            escape_depth_ = same_dir ? prev_esc_depth : std::clamp(new_depth, 4.0, 30.0);
        } else {
            // In goal mode: mantieni la quota del target, non quella di wander
            // (PRE-VERT potrebbe aver corrotto wander_z_ prima che questa linea venisse raggiunta)
            escape_depth_ = goal_active_
                ? std::clamp(target_z_, 4.0, 15.0)
                : std::clamp(wander_z_, 4.0, 15.0);
        }
        // Fix C: redirect al centro se heading punta verso muro vicino
        if (use_vert) {
            double ew = planner_.wall_dist_in_dir(escape_yaw_);
            if (ew < ESCAPE_CLEAR + 5.0) {
                escape_yaw_ = std::atan2(-state_.y, -state_.x);
                RCLCPP_WARN(get_logger(),
                    "[BRAKE→ESCAPE-VERT redirect] muro heading=%.1f m, centro=%.2f",
                    ew, escape_yaw_);
            }
        }
        vertical_escape_ = use_vert;
        mode_             = Mode::ESCAPE;
        escape_start_t_   = t;
        escape_lateral_   = !use_vert && goal_active_ && std::abs(normalize(baz)) > M_PI/4.0;
        mode_until_       = t + (escape_lateral_ ? 25.0 : 10.0);
        plan_t_           = t;
        if (!use_vert || !same_dir)
            post_brake_thr_until_ = t + (use_vert ? 3.0 : 5.0);   // Fix B: 5s per fuga orizzontale (era 10s)
        post_brake_long_ = !use_vert && ((t - brake_begin_t_) > 3.0);  // Fix G2
        stuck_escape_until_   = std::max(stuck_escape_until_, t + 2.0);
        sonar_brake_until_    = std::max(sonar_brake_until_, t + 2.0);  // Fix 21
        wall_fwd_brake_until_ = std::max(wall_fwd_brake_until_, t + 3.0);  // Fix E
        goal_clear_since_     = -1.0;
        // Genera waypoint locale in escape orizzontale goal mode
        if (!use_vert && goal_active_)
            create_local_waypoint(escape_yaw_, horiz_lh ? path_len_brake : ESCAPE_CLEAR + 3.0);
        else
            local_wp_active_ = false;
        RCLCPP_INFO(get_logger(),
            "[BRAKE→ESCAPE] fwd_min=%.2f yaw=%.3f esc=%.2f vert=%s el_span=%d",
            fwd_min, state_.yaw, escape_yaw_, vertical_escape_ ? "SI" : "NO", el_span);
    }

    // ── Vincoli di profondità ─────────────────────────────────────────────
    bool too_low  = state_.z < 1.4;
    bool too_high = state_.z > depth_limit_high_;

    // ── Transizioni CRUISE → ESCAPE ───────────────────────────────────────
    // Fix 46 rimosso: find_escape() ora usa densità istogramma per rilevare muri
    // a qualsiasi distanza (non solo < ESCAPE_CLEAR). Il bug originale (spirale 80s
    // direzione sbagliata) era causato dal fatto che find_escape() trovava la direzione
    // SBAGLIATA (sinistra quando il gap era a destra) perché i settori muro non erano
    // filtrati. Con il fix a find_escape() + buffer ±2, la direzione è ora corretta.
    double straight_clr = sm_.received ? planner_.corridor_clearance(0.0) : 99.0;
    const double route_clearance = goal_active_
        ? std::min(straight_clr, planner_.corridor_clearance(preferred_dir())) : straight_clr;
    bool obstacle_ahead_early = sm_.received && route_clearance < SAFE_PASS_DIST;
    // Log goal-oriented: mostra distanza, direzione goal e clearance corridoio target
    if (goal_active_ && sm_.received) {
        double g_rel = preferred_dir();
        double g_clr = planner_.corridor_clearance(g_rel);
        double g_dist = std::hypot(target_x_ - state_.x, target_y_ - state_.y);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[GOAL] dist=%.1f goal_rel=%.0f° goal_clr=%.1f straight_clr=%.1f",
            g_dist, g_rel * 180.0 / M_PI, g_clr, straight_clr);
    }
    if (mode_ == Mode::CRUISE && sm_.received && !obstacle_ahead_early && !too_low && !too_high) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[PASS] corridoio libero clr=%.1f m, CRUISE verso target", straight_clr);
    }
    if (mode_ == Mode::CRUISE && (too_low || too_high || obstacle_ahead_early)) {
        double baz = 0.0, new_depth;
        int    el_span = 0;
        double path_len_lh2 = 0.0;
        bool horiz_lh2 = planner_.find_lookahead_escape(baz, lookahead_preferred_dir(fwd_min), &path_len_lh2);
        bool horiz     = horiz_lh2;
        if (!horiz) horiz = planner_.find_escape(baz, lookahead_preferred_dir(fwd_min));
        bool vert    = allow_vert_esc_ && planner_.find_vertical_escape(new_depth, &el_span);
        // Fix I: preferisce sempre la fuga orizzontale quando disponibile (|baz|<π/2).
        // Fix J: in goal mode mai fuga verticale.
        // Fix 73 (identico a BRAKE→ESCAPE): permetti verticale in goal mode se l'orizzontale è >120° dal goal.
        double goal_dir_bk2 = goal_active_
            ? normalize(std::atan2(target_y_ - state_.y, target_x_ - state_.x) - state_.yaw)
            : 0.0;
        bool horiz_very_backward2 = horiz && std::abs(normalize(baz)) > M_PI / 2.0
            && goal_active_ && std::abs(normalize(baz - goal_dir_bk2)) > 2.0 * M_PI / 3.0;
        // Fix 130: verticale permesso solo se lookahead non ha trovato nulla e
        // find_escape (fallback) punta >45° lontano dal goal.
        bool horiz_off_goal2 = !horiz_lh2 && horiz && goal_active_
            && std::abs(normalize(baz - goal_dir_bk2)) > M_PI / 4.0;
        bool use_vert = vert && (!horiz || std::abs(normalize(baz)) > M_PI / 2.0 || horiz_off_goal2)
                        && (!goal_active_ || horiz_very_backward2 || horiz_off_goal2);

        if (horiz || vert) {
            escape_yaw_   = use_vert ? state_.yaw
                          : (horiz ? normalize(state_.yaw + baz) : state_.yaw);
            escape_depth_ = use_vert ? std::clamp(new_depth, 4.0, 30.0)
                         : goal_active_ ? std::clamp(target_z_, 4.0, 15.0)
                                       : std::clamp(wander_z_, 4.0, 15.0);
            if (use_vert) {
                double ew = planner_.wall_dist_in_dir(escape_yaw_);
                if (ew < ESCAPE_CLEAR + 5.0) {
                    escape_yaw_ = std::atan2(-state_.y, -state_.x);
                    RCLCPP_WARN(get_logger(),
                        "[CRUISE→ESCAPE-VERT redirect] muro heading=%.1f m, centro=%.2f",
                        ew, escape_yaw_);
                }
            }
            vertical_escape_   = use_vert;
            escape_lateral_    = !use_vert && goal_active_ && std::abs(normalize(baz)) > M_PI/4.0;
            mode_              = Mode::ESCAPE;
            escape_start_t_    = t;
            mode_until_        = t + (escape_lateral_ ? 25.0 : 12.0);
            plan_t_            = t;
            sonar_brake_until_ = t + 2.0;
            goal_clear_since_  = -1.0;
            // Genera waypoint locale in escape orizzontale goal mode
            if (!use_vert && goal_active_)
                create_local_waypoint_throttled(
                    escape_yaw_, horiz_lh2 ? path_len_lh2 : ESCAPE_CLEAR + 3.0, t);
            else
                local_wp_active_ = false;
            RCLCPP_WARN(get_logger(),
                "[AVOID-MINIMAL] corridoio bloccato clr=%.1f m, az=%.0f° → yaw=%.2f vert=%s",
                straight_clr, baz * 180.0 / M_PI, escape_yaw_,
                vertical_escape_ ? "SI" : "NO");
        } else {
            cmd(-15.0, 0.0, 0.0);
            return;
        }
    }

    // ── Logica ESCAPE: uscita, timeout, replan ────────────────────────────

    // TARGET-FIRST: controlla ad ogni tick se possiamo rientrare verso il goal.
    // Si attiva in ESCAPE con circum_active_ e anche senza, se il goal è libero.
    if (mode_ == Mode::ESCAPE && !vertical_escape_ && goal_active_ && sm_.received) {
        double g_rel_tf  = preferred_dir();
        double g_clr_tf  = planner_.corridor_clearance(g_rel_tf);
        double g_yaw_tf  = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
        double wall_tf   = planner_.wall_dist_in_dir(g_yaw_tf);
        double straight_clr_tf = planner_.corridor_clearance(0.0);

        // Il rientro va concesso solo a manovra conclusa: senza questi guard il drone
        // alterna schivata e rientro ad ogni tick (oscillazione osservata a 5 Hz).
        bool rejoin_time_ok     = t > rejoin_allowed_after_;
        bool rejoin_not_chatter = (t - last_local_path_t_) > 2.0;
        bool stable_attitude    = std::abs(state_.pitch)    < 0.35 &&
                                  std::abs(state_.yaw_rate) < 0.8;
        // straight_clr: il corridoio goal può risultare libero mentre l'ostacolo è
        // appena fuori asse; richiedere anche l'avanti sgombro evita il falso rientro.
        bool clear_enough       = g_clr_tf        >= SAFE_PASS_DIST &&
                                  straight_clr_tf >= SAFE_PASS_DIST &&
                                  fwd_min         >  BRAKE_DIST + 3.0 &&
                                  wall_tf         >  BRAKE_DIST + 4.0;

        bool can_rejoin = !local_wp_active_ && rejoin_time_ok && rejoin_not_chatter &&
                          stable_attitude && clear_enough;

        if (can_rejoin) {
            last_target_rejoin_t_ = t;
            circum_active_   = false;
            local_wp_active_ = false;
            escape_lateral_  = false;
            vertical_escape_ = false;
            mode_            = Mode::CRUISE;
            goal_clear_since_ = -1.0;
            RCLCPP_INFO(get_logger(),
                "[TARGET-REJOIN] goal libero clr=%.1f straight=%.1f wall=%.1f rel=%.0f° esc=%.1fs → CRUISE verso target",
                g_clr_tf, straight_clr_tf, wall_tf, g_rel_tf * 180.0 / M_PI,
                t - escape_start_t_);
            // non fa return: riparte da CRUISE in questo tick
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500,
                "[TARGET-REJOIN-SKIP] wait=%.1fs gclr=%.1f straight=%.1f fwd=%.1f pitch=%.2f yr=%.2f",
                std::max(0.0, rejoin_allowed_after_ - t),
                g_clr_tf, straight_clr_tf, fwd_min, state_.pitch, state_.yaw_rate);
        }
    }

    // CIRCUMNAV TIMEOUT: se circum attivo da troppo tempo, prova rientro o target-biased wp.
    if (circum_active_ && mode_ == Mode::ESCAPE && t > circum_until_) {
        double g_rel_ct = preferred_dir();
        double g_clr_ct = planner_.corridor_clearance(g_rel_ct);
        if (g_clr_ct >= BRAKE_DIST + 2.0) {
            circum_active_   = false;
            local_wp_active_ = false;
            mode_            = Mode::CRUISE;
            RCLCPP_WARN(get_logger(),
                "[CIRCUM-TIMEOUT-REJOIN] provo rientro target clr=%.1f", g_clr_ct);
            // non fa return: riparte da CRUISE
        } else {
            double side_b_ct  = (circum_side_ >= 0) ? +0.45 : -0.45;
            double turn_ct = 0.0, clearance_ct = 0.0;
            if (!planner_.find_lookahead_escape(turn_ct,
                    normalize(preferred_dir() + side_b_ct), &clearance_ct)) {
                safety_braking_ = true;
                brake_stop_reverse_ = false;
                mode_ = Mode::BRAKE;
                local_wp_active_ = false;
                cmd(forward_speed_ > 0.2 ? -50.0 : 0.0, 0.0, 0.0);
                return;
            }
            escape_yaw_       = normalize(state_.yaw + turn_ct);
            local_wp_active_  = false;
            create_local_waypoint(escape_yaw_, clearance_ct);
            circum_until_     = t + 8.0;
            RCLCPP_WARN(get_logger(),
                "[CIRCUM-TIMEOUT-REPLAN] goal bloccato clr=%.1f → wp verificato yaw=%.2f",
                g_clr_ct, escape_yaw_);
            // non fa return: prosegue con il nuovo wp
        }
    }

    // Tracking waypoint locale: aggiorna escape_yaw_ dinamicamente verso il wp.
    // Quando il wp è raggiunto (<2.5m), rivaluta goal corridor.
    if (mode_ == Mode::ESCAPE && local_wp_active_ && !vertical_escape_) {
        double wp_dx   = local_wp_x_ - state_.x;
        double wp_dy   = local_wp_y_ - state_.y;
        double wp_dist = std::hypot(wp_dx, wp_dy);
        if (wp_dist < 2.5) {
            // CIRCUMNAV: se circum attivo e corridoio goal ancora bloccato → continua aggiramento
            if (circum_active_ && goal_active_ && sm_.received) {
                double g_rel_wp = preferred_dir();
                double g_clr_wp = planner_.corridor_clearance(g_rel_wp);
                if (g_clr_wp < SAFE_PASS_DIST) {
                    // TARGET-FIRST: nuovo wp orientato verso goal+bias laterale, non heading+offset.
                    // Bias ridotto a 0.35 rad: resta molto più attaccato alla rotta target;
                    // sale a 0.55 solo con ostacolo ravvicinato, dove serve scartare di più.
                    double goal_yaw_wr  = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
                    double bias_mag_wr  = (fwd_min < BRAKE_DIST + 2.0) ? 0.55 : 0.35;
                    double side_bias_wr = (circum_side_ >= 0) ? bias_mag_wr : -bias_mag_wr;
                    double new_yaw_wr   = normalize(goal_yaw_wr + side_bias_wr);
                    double rel_new_wr   = normalize(new_yaw_wr - state_.yaw);
                    double clr_new_wr   = planner_.corridor_clearance(rel_new_wr);
                    double wall_new_wr  = planner_.wall_dist_in_dir(new_yaw_wr);
                    if (clr_new_wr < ESCAPE_CLEAR || wall_new_wr < BRAKE_DIST + 4.0
                        || !planner_.waypoint_arc_free(rel_new_wr,
                            planner_.local_waypoint_length(rel_new_wr, clr_new_wr))) {
                        // fallback: lookahead con preferred biasato verso target+lato
                        double baz_wr = 0.0, len_wr = 0.0;
                        double pref_wr = normalize(preferred_dir() + side_bias_wr);
                        bool ok_wr = planner_.find_lookahead_escape(baz_wr, pref_wr, &len_wr);
                        if (ok_wr) {
                            new_yaw_wr = normalize(state_.yaw + baz_wr);
                            clr_new_wr = len_wr;
                            wall_new_wr = planner_.wall_dist_in_dir(new_yaw_wr);
                        } else {
                            safety_braking_ = true;
                            brake_stop_reverse_ = false;
                            mode_ = Mode::BRAKE;
                            local_wp_active_ = false;
                            cmd(forward_speed_ > 0.2 ? -50.0 : 0.0, 0.0, 0.0);
                            return;
                        }
                    }
                    escape_yaw_      = new_yaw_wr;
                    local_wp_active_ = false;
                    create_local_waypoint(new_yaw_wr, clr_new_wr);
                    RCLCPP_WARN(get_logger(),
                        "[CIRCUM-CONTINUE-TARGET] clr=%.1f<%.0f side=%+d goal_yaw=%.0f° bias=%.2f → wp yaw=%.0f° (clr=%.1f wall=%.1f)",
                        g_clr_wp, SAFE_PASS_DIST, circum_side_,
                        goal_yaw_wr * 180.0 / M_PI, side_bias_wr,
                        new_yaw_wr * 180.0 / M_PI, clr_new_wr, wall_new_wr);
                    return;
                } else {
                    circum_active_ = false;
                    RCLCPP_WARN(get_logger(),
                        "[CIRCUM-DONE] corridoio libero clr=%.1f wp_dist=%.1f → exit circum",
                        g_clr_wp, wp_dist);
                }
            }
            local_wp_active_ = false;
            goal_clear_since_ = -1.0;
            RCLCPP_INFO(get_logger(),
                "[WP-REACHED] waypoint raggiunto dist=%.1f wp=(%.1f,%.1f), valuto goal",
                wp_dist, local_wp_x_, local_wp_y_);
            if (goal_active_) {
                // Do not keep the completed leg's heading until another
                // obstacle is within 11 m. Plan the next observed segment
                // now, while there is still room to enter its turn.
                double next_az = 0.0, next_clearance = 0.0;
                if (!planner_.find_lookahead_escape(next_az, preferred_dir(), &next_clearance)) {
                    safety_braking_ = true;
                    brake_stop_reverse_ = false;
                    mode_ = Mode::BRAKE;
                    cmd(forward_speed_ > 0.2 ? -50.0 : 0.0, 0.0, 0.0);
                    return;
                }
                escape_yaw_ = normalize(state_.yaw + next_az);
                create_local_waypoint(escape_yaw_, next_clearance);
                plan_t_ = escape_start_t_ = t;
                mode_until_ = t + 20.0;
                RCLCPP_INFO(get_logger(), "[WP-NEXT] turn=%.0f clr=%.1f",
                    next_az*180.0/M_PI, next_clearance);
            }
        } else {
            double wp_yaw_new = std::atan2(wp_dy, wp_dx);
            double rel_wp     = normalize(wp_yaw_new - state_.yaw);
            if (std::abs(rel_wp) > 2.0) {
                // Fix A: waypoint stale (>115° rispetto heading).
                // escape_yaw_ clippato a ±90° dall'heading: garantisce waypoint davanti,
                // avvicinandosi al goal il più possibile senza puntare mai verso una parete.
                double turn_s = 0.0, clearance_s = 0.0;
                if (!planner_.find_lookahead_escape(turn_s, preferred_dir(), &clearance_s)) {
                    safety_braking_ = true;
                    brake_stop_reverse_ = false;
                    mode_ = Mode::BRAKE;
                    local_wp_active_ = false;
                    cmd(forward_speed_ > 0.2 ? -50.0 : 0.0, 0.0, 0.0);
                    return;
                }
                escape_yaw_ = normalize(state_.yaw + turn_s);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[WP-STALE-ANGLE] rel=%.0f° dist=%.1f → replan→yaw=%.2f (turn=%.0f°)",
                    rel_wp * 180.0 / M_PI, wp_dist, escape_yaw_, turn_s * 180.0 / M_PI);
                create_local_waypoint(escape_yaw_, clearance_s);
            } else {
                // Hold the leg bearing beside a close obstacle. Updating
                // it unconditionally passed a geometric tracking test but
                // caused repeated recovery manoeuvres in corridor run 27.
                if (wp_dist > 1.0 && fwd_min > BRAKE_DIST + 1.0)
                    escape_yaw_ = wp_yaw_new;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[TRACK-WP] dist=%.1f yaw_to_wp=%.0f° z=%.1f→%.1f fwd=%.1f",
                    wp_dist, wp_yaw_new * 180.0 / M_PI,
                    state_.z, goal_active_ ? target_z_ : wander_z_, fwd_min);
            }
        }
    }

    // Fix 142: WP progress tracking — deattiva wp se nessun avanzamento in 4s.
    // Evita che il drone si blocchi su un wp irraggiungibile senza mai fare replan.
    if (committed_wp && local_wp_active_) {
        double wp_dx_p = local_wp_x_ - state_.x;
        double wp_dy_p = local_wp_y_ - state_.y;
        double wp_dist_p = std::hypot(wp_dx_p, wp_dy_p);
        if (wp_last_check_t_ < 0.0) {
            wp_last_check_t_ = t;
            wp_last_dist_    = wp_dist_p;
        } else if ((t - wp_last_check_t_) >= 4.0) {
            double progress = wp_last_dist_ - wp_dist_p;
            RCLCPP_INFO(get_logger(),
                "[WP-PROGRESS] dist_prev=%.1f dist_now=%.1f progress=%.1f",
                wp_last_dist_, wp_dist_p, progress);
            double pending_turn = normalize(escape_yaw_ - state_.yaw);
            const bool closing_turn = std::abs(pending_turn) > 0.35
                && pending_turn * state_.yaw_rate > 0.0
                && std::abs(state_.yaw_rate) > 0.02;
            if (progress < 0.5 && !closing_turn) {
                // A turn can make angular progress before translating.
                // Never replace a safe detour by an unchecked ray to goal.
                double turn_p = 0.0, clearance_p = 0.0;
                if (planner_.find_lookahead_escape(turn_p, preferred_dir(), &clearance_p)) {
                    escape_yaw_ = normalize(state_.yaw + turn_p);
                    RCLCPP_WARN(get_logger(),
                        "[WP-REPLAN-BLOCKED] progresso %.1f m → corridoio verificato yaw=%.2f clr=%.1f",
                        progress, escape_yaw_, clearance_p);
                    create_local_waypoint(escape_yaw_, clearance_p);
                } else {
                    wp_last_dist_ = wp_dist_p;
                    wp_last_check_t_ = t;
                }
            } else {
                wp_last_dist_    = wp_dist_p;
                wp_last_check_t_ = t;
            }
        }
    }

    // Stabilità goal-corridor: esci da ESCAPE solo se goal corridor libero per 1.5s
    // E waypoint locale completato. Evita uscite premature da singolo tick positivo.
    if (mode_ == Mode::ESCAPE && goal_active_ && !vertical_escape_
        && fwd_min > BRAKE_DIST + 2.0) {
        double goal_rel = preferred_dir();
        double goal_clr = planner_.corridor_clearance(goal_rel);
        if (goal_clr >= SAFE_PASS_DIST && !local_wp_active_) {
            if (goal_clear_since_ < 0) goal_clear_since_ = t;
            if ((t - goal_clear_since_) >= 1.5) {
                mode_            = Mode::CRUISE;
                vertical_escape_ = false;
                escape_lateral_  = false;
                stuck_count_     = 0;
                goal_clear_since_ = -1.0;
                RCLCPP_INFO(get_logger(),
                    "[GOAL-CLEAR-STABLE] libero da %.1fs clr=%.1f → CRUISE",
                    1.5, goal_clr);
            }
        } else if (goal_clr < SAFE_PASS_DIST) {
            goal_clear_since_ = -1.0;  // reset se corridoio torna bloccato
        }
    }

    if (mode_ == Mode::ESCAPE) {
        // Fix 69: se escape_yaw_ è già allineata col goal (entro 0.5 rad), min_esc_t
        // ridotto a 1.5s — il drone ha già la direzione giusta, non serve aspettare 3-5s.
        double goal_w_clr = goal_active_
            ? normalize(std::atan2(target_y_ - state_.y, target_x_ - state_.x))
            : escape_yaw_;
        double esc_goal_err_clr = std::abs(normalize(escape_yaw_ - goal_w_clr));
        double min_esc_t = (post_brake_thr_until_ > plan_t_) ? 5.0 : 3.0;
        if (goal_active_ && esc_goal_err_clr < 0.5 && fwd_min > APPROACH_DIST)
            min_esc_t = std::min(min_esc_t, 1.5);
        // In goal mode: fuga laterale deve durare almeno 5s (era 20s) perché
        // l'uscita anticipata goal-corridor (sopra) gestisce il ritorno al target.
        if (escape_lateral_ && goal_active_)
            min_esc_t = std::max(min_esc_t, 5.0);
        // Fix 127: usa escape_start_t_ (fisso all'ingresso in ESCAPE) invece di plan_t_
        // (resettato ad ogni REPLAN WALL). Con plan_t_, ogni replan ogni 1.5s impedisce
        // l'uscita che richiede t-plan_t_>20s: il drone restava in ESCAPE per decine di m.
        // In goal mode: esci quando il corridoio verso il target è libero (goal-first).
        // In non-goal mode: comportamento precedente (fwd_min/blocked).
        double esc_g_clr = 0.0;
        bool cleared;
        if (goal_active_ && !vertical_escape_ && sm_.received) {
            double esc_g_rel = preferred_dir();
            esc_g_clr = planner_.corridor_clearance(esc_g_rel);
            // !local_wp_active_: non uscire prima che il waypoint locale sia raggiunto
            cleared = (esc_g_clr >= SAFE_PASS_DIST) && !local_wp_active_
                   && (t - escape_start_t_) > min_esc_t;
        } else {
            cleared = !blocked && fwd_min > CLEARED_DIST && (t - escape_start_t_) > min_esc_t;
        }
        // Fix 56: non uscire da VERT ESC finché z non è a meno di 1.5m dal target.
        // Senza questo, il drone usciva a z=14.85 con alta vz_down → overshoot al fondale.
        if (vertical_escape_ && std::abs(state_.z - escape_depth_) > 1.5)
            cleared = false;

        if (cleared) {
            mode_ = Mode::CRUISE;
            stuck_count_ = 0;
            if (vertical_escape_) { wander_z_ = escape_depth_; wander_z_t_ = t; }
            vertical_escape_ = false;
            if (circum_active_) {
                circum_active_ = false;
                RCLCPP_INFO(get_logger(),
                    "[CIRCUM-DONE-ESCAPE] corridoio libero clr=%.1f → exit circum+CRUISE",
                    esc_g_clr);
            }
            if (goal_active_)
                RCLCPP_INFO(get_logger(),
                    "[ESCAPE→CRUISE-GOAL] corridoio target libero clr=%.1f z=%.2f",
                    esc_g_clr, state_.z);
            else
                RCLCPP_INFO(get_logger(), "[CRUISE] ostacolo superato z=%.2f", state_.z);
        } else if (t > mode_until_) {
            mode_until_ = t + 10.0;
            plan_t_     = t - 2.1;
            RCLCPP_WARN(get_logger(), "[ESCAPE] timeout, replan forzato yaw=%.2f", state_.yaw);
        } else if (vertical_escape_) {
            // Fix I: VERT→HORIZ — se la discesa non converge abbastanza (siamo ancora
            // lontani dalla quota target mentre l'ostacolo si avvicina), prova a girare.
            if (fwd_min < APPROACH_DIST
                && std::abs(escape_depth_ - state_.z) > 2.0
                && escape_depth_ < 20.0    // Fix 130: non switchare durante salita sopra muro alto
                && (t - plan_t_) > 2.0
                && state_.z > 4.0) {  // vicino al fondale: mai switchare orizzontale
                double baz_new = 0.0;
                bool horiz_now = planner_.find_escape(baz_new, preferred_dir());
                if (horiz_now && std::abs(normalize(baz_new)) < M_PI / 2.0) {
                    vertical_escape_ = false;
                    escape_yaw_      = normalize(state_.yaw + baz_new);
                    plan_t_          = t;
                    RCLCPP_WARN(get_logger(),
                        "[VERT→HORIZ] discesa troppo lenta (Δz=%.1f fwd=%.1f), giro esc=%.2f",
                        std::abs(escape_depth_ - state_.z), fwd_min, escape_yaw_);
                }
            }
            if (vertical_escape_) {
                // Replan analitico pareti in fuga verticale
                double esc_wall_d = planner_.wall_dist_in_dir(escape_yaw_);
                double fwd_wall_d = planner_.wall_fwd_dist();
                // Fix 27: in vert escape usa soglia piu' conservativa per fwd_wall
                // (BLOCKED_DIST=12m invece di DECEL_DIST=20m): la direzione "avanti"
                // non e' quella di fuga, quindi non serve reagire cosi' presto.
                if ((esc_wall_d < DECEL_DIST || fwd_wall_d < BLOCKED_DIST) && (t - plan_t_) > 1.5) {
                    // Fix 61: vicino al goal usa direzione goal, non centro.
                    // Il centro pull a y=68 (goal y=70) faceva mancare il traguardo per 1.6m.
                    double dx_g = target_x_ - state_.x;
                    double dy_g = target_y_ - state_.y;
                    double goal_d2d = std::hypot(dx_g, dy_g);
                    if (goal_active_ && goal_d2d < 35.0) {
                        escape_yaw_ = std::atan2(dy_g, dx_g);
                    } else {
                        escape_yaw_ = std::atan2(-state_.y, -state_.x);
                    }
                    plan_t_ = t;
                    RCLCPP_WARN(get_logger(),
                        "[REPLAN WALL-VERT] muro esc=%.1f fwd=%.1f m, redirect yaw=%.2f",
                        esc_wall_d, fwd_wall_d, escape_yaw_);
                }
            }
        } else {
            // Replan azimutale in fuga orizzontale
            double esc_wall_d    = planner_.wall_dist_in_dir(escape_yaw_);
            double fwd_wall_d    = planner_.wall_fwd_dist();
            double yaw_err_to_esc = std::abs(normalize(escape_yaw_ - state_.yaw));
            // A short waypoint ends before the distant wall. Testing an
            // infinite escape ray against 32 m cancelled valid 6-12 m
            // detours throughout the 30 m wide corridor, often reversing
            // the turn every 1.5 s. Check the actual remaining path; the
            // stopping envelope above still protects the current heading.
            const double wp_remaining = local_wp_active_
                ? std::hypot(local_wp_x_-state_.x, local_wp_y_-state_.y) : DECEL_DIST;
            const double esc_wall_limit = local_wp_active_
                ? std::min(DECEL_DIST, wp_remaining+2.0) : DECEL_DIST;
            const double fwd_wall_limit = local_wp_active_ ? stop_distance+2.0 : DECEL_DIST;
            if ((esc_wall_d < esc_wall_limit || fwd_wall_d < fwd_wall_limit) && (t - plan_t_) > 1.5) {
                bool wall_urgent = fwd_wall_d < stop_distance+2.0 || esc_wall_d < 6.0;
                if (yaw_err_to_esc < 0.30 || wall_urgent) {
                    double baz, path_clearance = 0.0;
                    bool ok_lh = planner_.find_lookahead_escape(baz,
                        lookahead_preferred_dir(fwd_min), &path_clearance);
                    if (!ok_lh && !goal_active_)
                        ok_lh = planner_.find_escape(baz, lookahead_preferred_dir(fwd_min));
                    if (ok_lh) {
                        escape_yaw_ = normalize(state_.yaw + baz);
                        if (goal_active_)
                            create_local_waypoint(escape_yaw_, path_clearance);
                        else
                            local_wp_active_ = false;
                        RCLCPP_WARN(get_logger(),
                            "[REPLAN WALL] muro %.1f/%.1f m, nuovo target=%.2f",
                            esc_wall_d, fwd_wall_d, escape_yaw_);
                    } else if (wall_urgent && goal_active_) {
                        safety_braking_ = true;
                        brake_stop_reverse_ = false;
                        mode_ = Mode::BRAKE;
                        local_wp_active_ = false;
                        cmd(forward_speed_ > 0.2 ? -50.0 : 0.0, 0.0, 0.0);
                        return;
                    } else if (wall_urgent) {
                        escape_yaw_ = std::atan2(-state_.y, -state_.x);
                        RCLCPP_WARN(get_logger(),
                            "[REPLAN WALL centro] nessuna via libera, → centro yaw=%.2f",
                            escape_yaw_);
                    }
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[REPLAN WALL skip] yaw_err=%.2f>0.30, in svolta verso %.2f",
                        yaw_err_to_esc, escape_yaw_);
                }
                plan_t_ = t;
            } else if ((t - plan_t_) > 4.0) {
                double rel_esc = normalize(escape_yaw_ - state_.yaw);
                const double required_clearance = local_wp_active_
                    ? std::min(ESCAPE_CLEAR, wp_remaining+1.5) : ESCAPE_CLEAR;
                bool esc_blocked = planner_.corridor_clearance(rel_esc) < required_clearance;
                if (esc_blocked) {
                    double yaw_err_to_tgt = std::abs(normalize(escape_yaw_ - state_.yaw));
                    if (yaw_err_to_tgt < 0.30) {
                        double baz, replacement_clearance = 0.0;
                        bool ok_lh2 = planner_.find_lookahead_escape(baz, lookahead_preferred_dir(fwd_min),
                                                                    &replacement_clearance);
                        if (!ok_lh2 && !goal_active_)
                            ok_lh2 = planner_.find_escape(baz, lookahead_preferred_dir(fwd_min));
                        if (ok_lh2) {
                            double new_esc  = normalize(state_.yaw + baz);
                            double esc_jump = std::abs(normalize(new_esc - escape_yaw_));
                            // Guard: con ostacolo a <10m non saltare >0.50 rad (impossibile eseguire)
                            if (fwd_min > 10.0 || esc_jump < 0.50) {
                                escape_yaw_ = new_esc;
                                // The old waypoint has just failed its
                                // corridor check. Replace it too: changing
                                // only heading was undone by tracking on
                                // the next tick, restoring the blocked leg.
                                if (goal_active_)
                                    create_local_waypoint(escape_yaw_, replacement_clearance);
                                else
                                    local_wp_active_ = false;
                                RCLCPP_WARN(get_logger(),
                                    "[REPLAN] dir bloccata, nuovo target_yaw=%.2f (jump=%.2f)",
                                    escape_yaw_, esc_jump);
                            } else {
                                RCLCPP_WARN(get_logger(),
                                    "[REPLAN skip] jump=%.2f>0.50 con fwd=%.1f, mantengo %.2f",
                                    esc_jump, fwd_min, escape_yaw_);
                            }
                        }
                    } else {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 4000,
                            "[REPLAN wait] yaw_err=%.2f, ancora in svolta", yaw_err_to_tgt);
                    }
                    plan_t_ = t;
                }
            }

            // Fix 43 / Fix 69: goal-directed replan ogni 0.5s (era 1s).
            // Criteri allargati per riallinearsi al goal più rapidamente:
            // (a) goal direction aperta (<0.6 rad, era 0.5) → punta diretto al goal;
            // (b) miglioramento >0.15 rad (era 0.20) E risultato entro 90° dal goal.
            // Fix 125: soppresso per i primi 20s di ESCAPE laterale: il drone deve
            // attraversare il gap prima di riorientarsi verso il target.
            bool goal_replan_suppress = local_wp_active_
                || (escape_lateral_ && (t - escape_start_t_) < 5.0);
            // Complete the committed detour before considering a rejoin.
            // A single clear goal ray can be a gap between sonar beams;
            // cancelling here repeatedly undid the turn begun above.
            if (circum_active_ && local_wp_active_) {
                goal_replan_suppress = true;
            }
            if (goal_active_ && (t - goal_replan_t_) > 0.5 && !goal_replan_suppress) {
                double pdir = preferred_dir();
                double baz;
                if (planner_.find_escape(baz, pdir)) {
                    double candidate = normalize(state_.yaw + baz);
                    double goal_w    = normalize(state_.yaw + pdir);
                    double curr_err  = std::abs(normalize(escape_yaw_ - goal_w));
                    double new_err   = std::abs(normalize(candidate - goal_w));
                    double old_esc   = escape_yaw_;
                    // Rate limit: un replan non può ruotare l'heading di fuga più di
                    // ~14° per volta. I salti da 0.5 a 2.5 rad producevano correzioni
                    // che il veicolo non riesce a eseguire e che degenerano in pitch.
                    constexpr double MAX_YAW_STEP = 0.25;
                    if (new_err < 0.6 && fwd_min > DECEL_DIST) {
                        // Dir goal libera E nessun ostacolo davanti: punta al goal.
                        // Guard fwd_min>DECEL_DIST: evita di puntare al goal quando
                        // un muro è ancora rilevato in avanti (baz≈0 ma muro a 16m).
                        double d_goal = std::clamp(normalize(goal_w - escape_yaw_),
                                                   -MAX_YAW_STEP, MAX_YAW_STEP);
                        escape_yaw_ = normalize(escape_yaw_ + d_goal);
                        if (!committed_wp)
                            create_local_waypoint_throttled(escape_yaw_, ESCAPE_CLEAR + 3.0, t);
                        RCLCPP_INFO(get_logger(),
                            "[GOAL-REPLAN-SMOOTH→GOAL] old=%.2f target=%.2f new=%.2f step=%.2f (Δ=%.2f)",
                            old_esc, goal_w, escape_yaw_, d_goal, new_err);
                    } else if (new_err < (curr_err - 0.15) && new_err < M_PI / 2.0) {
                        // Miglioramento significativo E entro 90° dal goal
                        double d_cand = std::clamp(normalize(candidate - escape_yaw_),
                                                   -MAX_YAW_STEP, MAX_YAW_STEP);
                        escape_yaw_ = normalize(escape_yaw_ + d_cand);
                        if (!committed_wp)
                            create_local_waypoint_throttled(escape_yaw_, ESCAPE_CLEAR + 3.0, t);
                        RCLCPP_INFO(get_logger(),
                            "[GOAL-REPLAN-SMOOTH] old=%.2f target=%.2f new=%.2f step=%.2f (Δgoal %.2f→%.2f)",
                            old_esc, candidate, escape_yaw_, d_cand, curr_err, new_err);
                    }
                }
                goal_replan_t_ = t;
            }

            // Fix 101C: replan se la direzione di fuga punta verso la parete interna.
            // Usa la distanza lookahead nella direzione escape_yaw_ per intercettare
            // il problema PRIMA che il drone esca dal cubo interno.
            // Il cubo interno viene usato solo in wander mode.
            // In goal mode il target può trovarsi oltre ±inner_wx/inner_wy
            // (es. target_y=+70 con inner_y=+60), quindi non deve provocare replan.
            if (!goal_active_) {
                double inner_esc_d = planner_.inner_wall_dist_in_dir(escape_yaw_);
                double inner_fwd_d = planner_.inner_wall_dist_in_dir(state_.yaw);

            if ((inner_esc_d < DECEL_DIST || inner_fwd_d < DECEL_DIST) &&
                (t - plan_t_) > 1.5)
            {
                double baz;

                if (planner_.find_escape(baz, preferred_dir())) {
                    escape_yaw_ = normalize(state_.yaw + baz);

                    RCLCPP_WARN(get_logger(),
                    "[REPLAN INNER] cubo interno esc=%.1fm fwd=%.1fm → esc=%.2f",
                    inner_esc_d, inner_fwd_d, escape_yaw_);
            } else {
                escape_yaw_ = std::atan2(-state_.y, -state_.x);

                RCLCPP_WARN(get_logger(),
                "[REPLAN INNER centro] nessuna via, → centro esc=%.2f",
                escape_yaw_);
                }

            plan_t_ = t;
            }
}

            // Fix 101D: se il drone è in ESCAPE da >20s e viaggia nella direzione
            // opposta al goal in y, forza replan verso goal.
            if (goal_active_ && (t - escape_start_t_) > 20.0) {
                double esc_y  = std::sin(escape_yaw_);
                double need_y = target_y_ - state_.y;
                if ((need_y > 5.0 && esc_y < -0.2) || (need_y < -5.0 && esc_y > 0.2)) {
                    double elapsed = t - escape_start_t_;
                    double baz;
                    if (planner_.find_escape(baz, preferred_dir())) {
                        escape_yaw_ = normalize(state_.yaw + baz);
                    } else {
                        escape_yaw_ = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
                    }
                    create_local_waypoint(escape_yaw_, ESCAPE_CLEAR + 3.0);
                    escape_start_t_ = t;
                    plan_t_ = t;
                    RCLCPP_WARN(get_logger(),
                        "[Y-REPLAN] %.0fs contro-goal (esc_y=%.2f need_y=%.0f), esc=%.2f",
                        elapsed, esc_y, need_y, escape_yaw_);
                }
            }
        }
    }

    // ── Calcolo comandi attuatori (PD controller) ─────────────────────────
    double thr, az_cmd, el_cmd;
    bool stop_turn_active = false;
    double turn_thrust_cap = std::numeric_limits<double>::infinity();

    if (mode_ == Mode::CRUISE) {
        if (goal_active_) {
            // Navigazione verso il target: ricalcola heading ogni tick
            // Formula: body+X = world+Y a yaw=0 → goal_yaw = atan2(x-tx, ty-y)
            double dx = target_x_ - state_.x;
            double dy = target_y_ - state_.y;
            double dist_goal = std::hypot(dx, dy);
            double goal_yaw  = std::atan2(dy, dx);  // Fix 49: atan2(dy,dx) = heading corretto
            wander_yaw_  = goal_yaw;
            wander_z_    = std::clamp(target_z_, 4.0, 15.0);
            wander_wp_t_ = t;

// VFH reattivo in goal mode.
// Se il corridoio verso il goal è libero → heading diretto al goal (nessuna deviazione).
// Se il corridoio è bloccato → devia il minimo indispensabile.
{
    double goal_clr_vfh = sm_.received ? planner_.corridor_clearance(preferred_dir()) : 99.0;
    if (goal_clr_vfh < SAFE_PASS_DIST && sm_.received) {
        double baz = 0.0;
        if (planner_.find_escape(baz, preferred_dir())) {
            double yaw_raw = normalize(state_.yaw + baz);
            wander_yaw_ = normalize(wander_yaw_ + 0.35 * normalize(yaw_raw - wander_yaw_));
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[VFH-GOAL] corridoio goal bloccato clr=%.1f m, az=%.0f° heading=%.2f",
                goal_clr_vfh, baz * 180.0 / M_PI, wander_yaw_);
        }
    } else {
        // Corridoio libero: mantieni esattamente il goal come heading
        wander_yaw_ = goal_yaw;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "[CRUISE-GOAL] corridoio libero clr=%.1f m heading=%.2f dist=%.1f",
            goal_clr_vfh, goal_yaw, dist_goal);
    }
}

            // Controlla se il goal è raggiunto
            double dz = target_z_ - state_.z;
            if (std::hypot(dist_goal, dz) <= goal_radius_) {
                goal_reached_ = true;
                RCLCPP_INFO(get_logger(), "[GOAL] %s raggiunto! dist=%.1f m", drone_name_.c_str(), std::hypot(dist_goal, dz));
                stop_actuators();
                // Rimuove il modello da Gazebo (type:2 = MODEL)
                {
                    std::string rm = "ign service -s /world/buoyant_tethys/remove"
                        " --reqtype ignition.msgs.Entity"
                        " --reptype ignition.msgs.Boolean"
                        " --timeout 2000"
                        " --req 'name: \"" + drone_name_ + "\" type: 2' &";
                    std::system(rm.c_str());
                }
                write_csv();
                rclcpp::shutdown();
                return;
            }
        } else {
            // Navigazione casuale (wander)
            if (t - wander_wp_t_ > 7.0) {
                if (wander_wp_t_ < 0.0) {
                    std::uniform_real_distribution<double> full_yaw(-M_PI, M_PI);
                    wander_yaw_ = full_yaw(rng_);
                } else {
                    std::uniform_real_distribution<double> yaw_d(-1.2, 1.2);
                    wander_yaw_ = normalize(wander_yaw_ + yaw_d(rng_));
                }
                if (planner_.wall_dist_in_dir(wander_yaw_) < 18.0) {
                    wander_yaw_ = std::atan2(-state_.y, -state_.x);
                    RCLCPP_INFO(get_logger(), "[WANDER] parete vicina → centro yaw=%.2f", wander_yaw_);
                } else {
                    RCLCPP_INFO(get_logger(), "[WANDER] nuovo heading yaw=%.2f", wander_yaw_);
                }
                wander_wp_t_ = t;
            }
            if (t - wander_z_t_ > 25.0) {
                std::uniform_real_distribution<double> z_d(4.0, 15.0);
                wander_z_   = z_d(rng_);
                wander_z_t_ = t;
                RCLCPP_INFO(get_logger(), "[WANDER] nuova profondità z=%.1f", wander_z_);
            }
        }

        // Adeguamento preventivo quota (PRE-VERT)
        // Fix I: non cambia quota se c'è già una via orizzontale libera (girerà invece di scendere).
        // In goal mode la quota obiettivo è già target_z_ — PRE-VERT non deve sovrascrivere wander_z_.
        if (!goal_active_ && fwd_min < DECEL_DIST) {
            double pre_depth = state_.z; int pre_span = 0;
            bool pre_vert = planner_.find_vertical_escape(pre_depth, &pre_span, DECEL_DIST);
            double pre_depth_safe = std::clamp(pre_depth, 4.0, 15.0);
            double pre_baz = 0.0;
            bool pre_horiz = planner_.find_escape(pre_baz, preferred_dir());
            bool pre_use_vert = pre_vert && pre_span <= 3
                && (!pre_horiz || std::abs(normalize(pre_baz)) > M_PI / 2.0);
            if (pre_use_vert
                && std::abs(pre_depth_safe - state_.z)    > 1.5
                && std::abs(pre_depth_safe - wander_z_)   > 2.0
                && (t - pre_vert_t_) > 5.0) {
                wander_z_   = pre_depth_safe;
                wander_z_t_ = t;
                pre_vert_t_ = t;
                RCLCPP_INFO(get_logger(),
                    "[PRE-VERT] fwd_min=%.1f el_span=%d → wander_z=%.2f",
                    fwd_min, pre_span, pre_depth_safe);
            }
        }

        // Fix 35: se wander_yaw_ punta verso una parete entro BLOCKED_DIST (ogni tick,
        // non solo ogni 7s), reindirizza verso il centro per non infilarsi negli angoli.
        if (!goal_active_ && planner_.wall_dist_in_dir(wander_yaw_) < BLOCKED_DIST) {
            wander_yaw_ = std::atan2(-state_.y, -state_.x);
            wander_wp_t_ = t;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                "[CRUISE] wander→parete<%.0fm, redirect centro yaw=%.2f",
                BLOCKED_DIST, wander_yaw_);
        }

        // PD yaw — Kp=0.7, Kd=1.5.
        // Fix 47: Kp 1.5→0.7, Kd 1.0→2.5 → eliminava oscillazioni a ±1.2 rad/s.
        // Fix 52: Kd 2.5→1.5 — con Kd=2.5 e yawrate≥0.5 rad/s il termine derivativo
        // dominava e causava oscillazione crescente ±3 rad/s in drone 1 dopo BRAKE.
        // Kp 0.7→1.1 e Kd 1.5→0.9: il termine derivativo cancellava gran parte della
        // barra non appena il drone iniziava a ruotare, bloccando il rateo d'imbardata
        // intorno a 0.05 rad/s con errori di rotta di 0.9 rad.
        double yaw_err_w = normalize(wander_yaw_ - state_.yaw);
        az_cmd = std::clamp(1.1*yaw_err_w - 0.9*state_.yaw_rate, -0.45, 0.45);

        // Fix 83: controllo a cascata profondità→pitch per volo livellato.
        // Problema precedente: PD diretto z_err→el saturava a el=-0.45 per errori di
        // 1m → drone volava perennemente a -25°/-30° anche a quota corretta.
        // Fisicamente sbagliato: un AUV reale usa cascata
        //   loop esterno (lento):  z_err → pitch_desiderato (max ±12°)
        //   loop interno (veloce): pitch_err → elevator
        // A quota target: pitch_desiderato=0 → el→0 → drone livellato.
        double z_err = wander_z_ - state_.z;
        // Loop esterno: z_err → pitch desiderato.
        // Fix 91: clip aumentato da ±0.20 a ±0.30 rad.
        // Bug: con buoyancy+9N e CRUISE_THR=45N, equilibrio richiede pitch=-0.201rad.
        // Il clip ±0.20 era esattamente al limite → drone saliva lentamente (+1.7m in 60s).
        // Con clip ±0.30: a z=target+0.7m → des_pitch=-0.21 → thr_z=-9.5N > buoyancy → z scende.
        double des_pitch = std::clamp(-0.22 * z_err, -0.30, 0.30);
        // Fix 99: gain pitch 1.4→0.7, damping vz 0.35→0.70.
        // Bug: gain 1.4 con decelerazione sonar (thr 45→30N) produce oscillazione el
        // 0.05→0.31 ("trembling" vicino ostacoli). Gain 0.7 dimezza l'ampiezza.
        double el_raw = 0.7 * (des_pitch - state_.pitch) + 0.70 * state_.vz;
        if (state_.z > 27.0) el_raw = std::min(el_raw, -0.15);  // forza DOWN vicino tetto
        if (state_.z <  2.0) el_raw = std::max(el_raw, +0.15);  // forza UP vicino fondale
        // Cap ±0.35 (era ±0.45): limita il momento pitch-giù e il thr_z coupling
        el_cmd = std::clamp(el_raw, -0.35, 0.35);
        // Pitch guard CRUISE: intervento precoce per battere il Munk moment.
        // Soglia abbassata 0.40→0.20: a pitch=0.13 il Munk già amplifica; aspettare 0.40
        // è troppo tardi (pitch cresce a 0.26 in un solo tick da 0.13).
        // Senza Fix134: el<0 = pubblicato negativo = muso GIÙ ✓, el>0 = muso SU ✓.
        // Segni invertiti rispetto alla versione originale: con l'uscita elevatore non
        // più negata, el>0 produce muso giù. Le guardie devono quindi comandare el>0
        // per correggere un muso su, altrimenti amplificano l'escursione invece di
        // smorzarla (misurato: in CRUISE z medio 12.84 contro 10.11 in ESCAPE, che
        // queste guardie non le ha).
        if (state_.pitch > 0.20) {
            el_cmd = std::clamp(-2.0 * state_.pitch, -0.35, 0.0);
            sm_el_ = el_cmd;  // bypass EMA: risposta immediata
        } else if (state_.pitch > 0.10 && el_cmd > 0.0) {
            el_cmd = 0.0;  // non accentuare il muso su quando è già positivo
        }
        if (state_.pitch < -0.20) {
            el_cmd = std::clamp(-2.0 * state_.pitch, 0.0, 0.35);
            sm_el_ = el_cmd;
        } else if (state_.pitch < -0.10 && el_cmd < 0.0) {
            el_cmd = 0.0;
        }
        thr    = CRUISE_THR;

        // Decelerazione preventiva sonar (DECEL_DIST → BRAKE_DIST)
        // Usa clearance del corridoio fisico (CORRIDOR_HALF_WIDTH) nella direzione seguita.
        // fwd_min interviene solo come emergenza ravvicinata (< BLOCKED_DIST=16m).
        bool pre_vert_active = std::abs(wander_z_ - state_.z) > 1.5;
        bool safety_decel    = (fwd_min < EMERGENCY_COLLISION_DIST);  // solo vera emergenza (3m)
        {
            double path_dir_d = normalize(wander_yaw_ - state_.yaw);
            double path_clr_d = sm_.received ? planner_.corridor_clearance(path_dir_d) : 99.0;
            // Emergenza: se fwd_min molto basso e < path_clr, usalo come limite
            double decel_d = path_clr_d;
            if (safety_decel && fwd_min < decel_d) decel_d = fwd_min;
            if ((!pre_vert_active || safety_decel) && decel_d < DECEL_DIST) {
                double factor = std::clamp(
                    (decel_d - BRAKE_DIST) / (DECEL_DIST - BRAKE_DIST), 0.0, 1.0);
                thr = std::max(CRUISE_THR * factor, TURN_THR);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[PATH] path_clr=%.1f fwd=%.1f decel_d=%.1f → thr=%.1f",
                    path_clr_d, fwd_min, decel_d, thr);
            }
        }

        // Decelerazione analitica da parete (il sonar non rileva le pareti piatte)
        {
            double wall_d = planner_.wall_fwd_dist();
            if (wall_d < DECEL_DIST) {
                double factor = std::clamp(
                    (wall_d - BRAKE_DIST) / (DECEL_DIST - BRAKE_DIST), 0.0, 1.0);
                thr = std::min(thr, std::max(CRUISE_THR * factor, TURN_THR));
            }
        }

        // Decelerazione verso il goal (evita overshooting a CRUISE_THR elevato)
        if (goal_active_) {
            double dx_g = target_x_ - state_.x;
            double dy_g = target_y_ - state_.y;
            double dz_g = target_z_ - state_.z;
            double dist_g = std::hypot(std::hypot(dx_g, dy_g), dz_g);
            if (dist_g < DECEL_DIST) {
                double factor = std::clamp(
                    (dist_g - goal_radius_) / (DECEL_DIST - goal_radius_), 0.0, 1.0);
                thr = std::min(thr, std::max(CRUISE_THR * factor, TURN_THR));
            }
        }

        // Fix 93: cap thrust durante svolta stretta per ridurre accoppiamento Fossen.
        // Bug: steer=-0.45 prolungato → pitch=-0.40 → thr_z=-18N → z crolla da 9.8 a 6.3m.
        // Cap a BUOY_COMP_THR (30N) per preservare efficacia buoyancy compensation.
        // Il floor buoy_comp sotto si comporta simmetricamente → thr=30N fisso in svolta.
        if (std::abs(az_cmd) > 0.30)
            thr = std::min(thr, static_cast<double>(BUOY_COMP_THR));

        // Floor thr per contrastare la galleggiabilità positiva (+9N).
        // Applicato DOPO tutte le decelerazioni per non essere sovrascritto.
        // Compensazione buoyancy solo durante navigazione libera.
// Vicino a ostacoli e durante svolte deve essere possibile rallentare.
if (goal_active_
&& fwd_min >= DECEL_DIST
&& std::abs(az_cmd) < 0.25)
{
thr = std::max(thr, BUOY_COMP_THR);
}


    } else {
        // ESCAPE: PD più aggressivi per la manovra di fuga
        double yaw_err = normalize(escape_yaw_ - state_.yaw);
        double z_err   = escape_depth_ - state_.z;

        // Kp ridotto 2.5→1.2, Kd aumentato 0.5→2.5.
        // Fix 44: spin brake — quando |yawrate|>3 il termine proporzionale 1.2*yaw_err
        // cambia segno ad ogni mezzo giro amplificando la rotazione (risonanza).
        // Sotto spin: pura smorzatura; thr ridotto a TURN_THR dopo init (vedi sotto).
        bool spin_brake_ = std::abs(state_.yaw_rate) > 3.0;
        if (spin_brake_) {
            // Smorzamento anti-spin: mantiene autorità piena, non va limitato.
            az_cmd = std::clamp(-2.5 * state_.yaw_rate, -0.40, 0.40);
        } else {
            double az_lim_esc = steer_limit(fwd_min);
            az_cmd = std::clamp(1.2*yaw_err - 0.8*state_.yaw_rate, -az_lim_esc, az_lim_esc);
        }
        // Fix 59B: in fuga verticale usa smorzamento attivo invece di az=0.
        // az=0 lascia il drone girare libero a CRUISE_THR → spin amplificato dal flusso.
        if (vertical_escape_ && std::abs(state_.yaw_rate) > 1.5)
            az_cmd = std::clamp(-2.5 * state_.yaw_rate, -0.40, 0.40);
        // PD z ESCAPE
        if (!vertical_escape_ && goal_active_) {
            // ESCAPE orizzontale goal mode: damping vz forte per evitare oscillazioni z.
            // Il coefficiente 1.4 su vz smorza rapidamente escursioni ±1 m/s senza
            // aspettare che il pitch si aggiusti da solo (come faceva la cascata originale).
            double z_ref_e = target_z_;
            double z_err_e = z_ref_e - state_.z;
            // Fix 141: cap -0.30 (era -0.18). Con buoyancy=+9N e thrust=35N
            // serve pitch < -arcsin(9/35)=-0.26 per scendere. Cap=0.18 era fisicamente
            // impossibile vincere la galleggiabilità.
            double des_p_e = std::clamp(-0.12 * z_err_e + 0.35 * state_.vz, -0.30, 0.30);
            el_cmd = std::clamp(0.9 * (des_p_e - state_.pitch) + 1.4 * state_.vz,
                                -0.35, 0.35);
        } else {
            // VERT ESC e wander: controller originale
            double des_p_esc = vertical_escape_
                ? std::clamp(+0.30 * z_err, -0.35, 0.35)
                : std::clamp(+0.18 * z_err, -0.20, 0.20);
            double el_esc_raw = 1.4 * (des_p_esc - state_.pitch) - 0.45 * state_.vz;
            double el_cap = vertical_escape_ ? 0.45 : 0.35;
            el_cmd = std::clamp(el_esc_raw, -el_cap, el_cap);
            if (state_.pitch < -1.0) el_cmd = 0.0;
            // Fix 2 e 63A solo per non-goal (nel nuovo controller goal il termine -1.4*vz
            // gestisce già questi casi senza annullare il comando).
            if (!vertical_escape_ && fwd_min < APPROACH_DIST && state_.vz > 0.05)
                el_cmd = std::min(el_cmd, 0.0);
            if (!vertical_escape_ && state_.vz < -0.5 && el_cmd < 0.0)
                el_cmd = 0.0;
        }

        // Pre-calcola yaw_err per decidere floor nella rampa (e riuso sotto)
        double yaw_err_abs = std::abs(normalize(escape_yaw_ - state_.yaw));

        // Rampa decelerazione: floor elevato in svolta profonda (R=V/ω ∝ 1/V: più veloce → raggio minore)
        thr = CRUISE_THR;
        if (fwd_min < DECEL_DIST) {
            double factor = std::clamp(
                (fwd_min - BRAKE_DIST) / (DECEL_DIST - BRAKE_DIST), 0.0, 1.0);
            double thr_floor = (!vertical_escape_ && yaw_err_abs > 0.4) ? TURN_BOOST_THR : TURN_THR;
            thr = std::max(CRUISE_THR * factor, thr_floor);
        }

        // Fix 44: sotto spin (|yawrate|>3) limita thrust a TURN_THR — la velocità
        // alta alimenta le forze idrodinamiche che amplificano la rotazione.
        if (spin_brake_)
            thr = std::min(thr, TURN_THR);

        // Fix L: in fuga orizzontale goal mode, mantieni thr ≥ BUOY_COMP_THR.
        // Non applicare se in spin brake (spin ha priorità).
        // Fix 103B-pre: non applicare nella zona di decelerazione (fwd_min < DECEL_DIST)
        // per non annullare la rampa che rallenta il drone prima degli ostacoli.
        if (!vertical_escape_ && goal_active_ && !spin_brake_ && fwd_min > DECEL_DIST)
            thr = std::max(thr, BUOY_COMP_THR);

        // Fix 52 ALIGN-GOAL: grande yaw_err con percorso libero → TURN_THR per girare
        // sul posto invece di volare lontano in direzione sbagliata (come drone 1 che
        // faceva 80m a est a CRUISE_THR=60N prima di correggere l'heading verso nord).
        // Non interferire quando decel ramp è attiva (fwd_min < DECEL_DIST): in quel
        // caso il floor TURN_BOOST_THR è necessario per ridurre il raggio di curva.
        if (!vertical_escape_ && goal_active_ && !spin_brake_
            && yaw_err_abs > 1.0 && fwd_min > APPROACH_DIST
            && fwd_min >= DECEL_DIST) {
            // Fix 58: usa BUOY_COMP_THR (50N) anziché TURN_THR (20N) quando non c'è
            // un ostacolo vicino. A 20N la velocità è troppo bassa → LiftDrag < galleggiabilità
            // → z sale incontrollato durante la rotazione (da z=8.5 a z=22 in 30s).
            // A 50N la velocità è sufficiente per le pinne a controbilanciare i +9N.
            thr = goal_active_ ? BUOY_COMP_THR : TURN_THR;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[ALIGN-GOAL] yaw_err=%.2f rad, thr=%.1f (z=%.2f tgt=%.2f)",
                yaw_err_abs, thr, state_.z, escape_depth_);
        }
        // Fix 113C: durante U-turn (yaw_err > 2.0 rad), drone si muove quasi all'opposto del goal.
        // BUOY_COMP_THR=30N → ~78m di deriva in direzione sbagliata; TURN_THR=20N → ~52m.
        // Non interferire con decel ramp (stessa condizione di ALIGN-GOAL sopra).
        if (!vertical_escape_ && goal_active_ && !spin_brake_ && yaw_err_abs > 2.0
            && fwd_min >= DECEL_DIST)
            thr = std::min(thr, static_cast<double>(TURN_THR));

        if (!vertical_escape_ && yaw_err_abs > 0.20 && fwd_min < APPROACH_DIST) {
            // Fix H: sempre TURN_THR in ALIGN. Il wall-boost accelerava il drone verso la
            // parete durante la svolta → V≈1.4 m/s → BRAKE insufficiente a fermarsi in 4m.
            thr = (state_.z < 4.0) ? CRUISE_THR : TURN_THR;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[ALIGN] yaw_err=%.2f rad fwd_min=%.2f thr=%.1f z=%.2f",
                yaw_err_abs, fwd_min, thr, state_.z);
        }

        // Finestra post-BRAKE: impone thr≥TURN_THR, neutralizza steer se yaw_rate invertito
        if (t < post_brake_thr_until_) {
            thr = std::max(thr, TURN_THR);
            // Fix G/G2: in ALIGN, sopprime steer solo se il BRAKE era lungo (V<0 significativa).
            // BRAKE breve (<3s, es. wall_fwd_near a 1 tick) → V≈0 → steer funziona → bypass.
            // BRAKE lungo (≥3s, stuck con thr=-50) → V<0 → LiftDrag invertita → mantieni soppressione.
            bool in_align_steer = !vertical_escape_ && yaw_err_abs > 0.20 && fwd_min < APPROACH_DIST;
            bool bypass_suppress = in_align_steer && !post_brake_long_;
            if (!vertical_escape_
                && !bypass_suppress
                && yaw_err * state_.yaw_rate < 0.0
                && std::abs(state_.yaw_rate) > 0.01)  // Fix C: soglia abbassata da 0.05 per catturare spin nascente
                az_cmd = 0.0;
        }

        // Fix 120/126 → Fix 135: fwd_min < 14m E yaw_err > 0.4 rad → coast (thr=0)
        // per decelerare mentre le pinne girano.
        // La retromarcia (-15N) invertiva il flusso sulle pinne: az<0 (EST in forward)
        // diventava az<0 (OVEST in retro) → il drone non girava. Con thr=0 il flusso
        // residuo della velocità acquisita mantiene le pinne efficaci nella direzione corretta.
        if (!vertical_escape_ && fwd_min < 14.0 && yaw_err_abs > 0.4 && state_.z > 3.5) {
            // In fuga orizzontale non azzerare mai thr: le pinne hanno bisogno di flusso
            // (LiftDrag ∝ V²). Tenere sempre almeno TURN_THR per autorità di sterzo.
            // La portanza delle pinne va con V²: più ampia è la correzione di rotta, più
            // spinta serve per generarla. Ridurla in virata allarga il raggio invece di
            // stringerlo — misurato: a 20N il rateo d'imbardata resta sotto 0.06 rad/s.
            // La spinta alta stringe la curva solo se la curva sta chiudendo. Se
            // l'ostacolo si avvicina e yaw_err non scende, spingere porta dentro
            // l'ostacolo: in quel caso si rallenta e si lascia lavorare BRAKE.
            // Segno verificato sul log: yaw_err e yaw_rate concordi = sta convergendo.
            bool yawrate_helping = (yaw_err * state_.yaw_rate) > 0.0;
            bool can_turn_boost  = !turn_not_closing
                                && fwd_min > BRAKE_DIST + 3.0
                                && yawrate_helping && turn_yaw_improving
                                && t >= turn_brake_until_;
            if (can_turn_boost) {
                double thr_turn = (yaw_err_abs > 0.9) ? static_cast<double>(TURN_BOOST_THR)
                                : (yaw_err_abs > 0.5) ? static_cast<double>(CRUISE_THR)
                                                      : static_cast<double>(BUOY_COMP_THR);
                thr = std::max(thr, thr_turn);
                stop_turn_active = true;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[TURN-BOOST] yaw_err=%.2f yr=%.2f thr=%.1f az=%.2f fwd=%.1f",
                    yaw_err_abs, state_.yaw_rate, thr, az_cmd, fwd_min);
            } else {
                double thr_safe = (sonar_corr < stop_distance || turn_not_closing)
                                ? 0.0 : static_cast<double>(TURN_THR);
                turn_thrust_cap = thr_safe;
                thr = std::min(thr, thr_safe);
                stop_turn_active = true;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                    "[TURN-SLOWDOWN] curva non fattibile fwd=%.1f yaw_err=%.2f yr=%.2f thr→%.1f",
                    fwd_min, yaw_err_abs, state_.yaw_rate, thr);
            }
        }

        // Quota-band ESCAPE orizzontale goal mode: forza el e thr se z fuori banda ±2m.
        // Applicato solo con thrust positivo (il controller quota non funziona in retro).
        // Previene derive lente verso DL-FLOOR/DL-CEIL durante manovre prolungate.
        if (!vertical_escape_ && goal_active_ && thr > 0.0) {
            if (state_.z > target_z_ + 2.0) {
                el_cmd = std::max(el_cmd, +0.25);
                thr    = std::max(thr, static_cast<double>(BUOY_COMP_THR));
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[ESC-Z-HOLD] z=%.1f target=%.1f vz=%.2f pitch=%.2f el=%.2f thr=%.0f (DOWN)",
                    state_.z, target_z_, state_.vz, state_.pitch, el_cmd, thr);
            } else if (state_.z < target_z_ - 2.0) {
                el_cmd = std::min(el_cmd, -0.25);
                thr    = std::max(thr, static_cast<double>(BUOY_COMP_THR));
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[ESC-Z-HOLD] z=%.1f target=%.1f vz=%.2f pitch=%.2f el=%.2f thr=%.0f (UP)",
                    state_.z, target_z_, state_.vz, state_.pitch, el_cmd, thr);
            }
        }

        // Fuga verticale: spinta quasi piena, rampa con floor=20
        if (vertical_escape_) {
            thr = CRUISE_THR;
            if (fwd_min < DECEL_DIST) {
                double factor = std::clamp(
                    (fwd_min - BRAKE_DIST) / (DECEL_DIST - BRAKE_DIST), 0.0, 1.0);
                // Fix 85C: quando si deve scendere (escape_depth < z) usa BUOY_COMP_THR come
                // floor invece di TURN_THR. Con 9N buoyancy, thr=20N×sin(pitch<0.47)=<9N→sale.
                // Con thr=30N: sin(pitch)>0.30 → thr_z=9N → equilibrio. Raggiungibile in 3-5s.
                double going_down = (escape_depth_ < state_.z - 0.5);
                double min_thr_vert = going_down ? BUOY_COMP_THR : TURN_THR;
                thr = std::max(CRUISE_THR * factor, min_thr_vert);
            }
            // Fix F: rampa analitica proporzionale se heading punta verso muro vicino.
            // Floor a TURN_THR=12 per mantenere autorita' pinne durante la svolta.
            // (Fix 24 rimosso: thr=0 azzerava autorita' pinne, drone non girava)
            {
                double wall_fwd = planner_.wall_fwd_dist();
                if (wall_fwd < BLOCKED_DIST) {
                    double wf = std::clamp(
                        (wall_fwd - BRAKE_DIST) / (BLOCKED_DIST - BRAKE_DIST), 0.0, 1.0);
                    thr = std::min(thr, std::max(CRUISE_THR * wf, TURN_THR));
                }
            }
            // Fix 22b: sopprimere el solo se si deve salire ma la velocità post-BRAKE
            // è ancora verso il basso (retromarcia residua). Se il drone deve scendere,
            // el resta attivo: è l'unico modo per abbassarsi dalla quota eccessiva.
            if (t < post_brake_thr_until_) {
                bool need_up = (escape_depth_ > state_.z + 0.5);
                if (need_up && state_.vz < -0.1)
                    el_cmd = 0.0;
            }
            // Fix 67C: pitch runaway in VERT ESC — el comanda un momento che il modello
            // Fossen amplifica in risonanza quando |pitch| è elevato.
            // pitch=1.116 rad (64°) con el=-0.45 DOWN porta z da 10m a 0m in 9s.
            // Con el=0: il modello fisico ammortizza naturalmente il pitch senza eccitarlo.
            if (std::abs(state_.pitch) > 0.5) el_cmd = 0.0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[VERT ESC] quota %.2f→%.2f thr=%.1f fwd_min=%.2f",
                state_.z, escape_depth_, thr, fwd_min);
        }
    }

    // Cap BUOY_COMP_THR in ESCAPE orizzontale per evitare z-rise a velocità alta.
    // Bypassa il cap durante svolta profonda (yaw_err>0.4): serve TURN_BOOST_THR per
    // ridurre il raggio di curva (R∝1/V) e attraversare il gap laterale nel labirinto.
    if (mode_ == Mode::ESCAPE && !vertical_escape_) {
        double yaw_err_esc = std::abs(normalize(escape_yaw_ - state_.yaw));
        if (yaw_err_esc <= 0.4)
            thr = std::min(thr, BUOY_COMP_THR);
    }

    // Fix 103B: in ESCAPE goal mode, scala spinta per allineamento con il goal.
    // Se escape_yaw_ punta lontano dal goal, il drone rallenta invece di sparare in
    // direzione sbagliata. La rampa di decelerazione (fwd_min<DECEL_DIST) riduce già
    // la spinta; qui si aggiunge uno scaling azimutale. floor = TURN_THR.
    // Skip quando in svolta profonda (yaw_err>0.4): non ridurre TURN_BOOST_THR durante arco.
    if (mode_ == Mode::ESCAPE && !vertical_escape_ && goal_active_ && !stop_turn_active
        && std::abs(normalize(escape_yaw_ - state_.yaw)) < 0.4) {
        double goal_dir   = std::atan2(target_y_ - state_.y, target_x_ - state_.x);
        double align      = std::cos(normalize(escape_yaw_ - goal_dir)); // 1=allineato, -1=opposto
        // factor 0.5..1.0: 90° → 0.5, 0° → 1.0, 180° → 0.5
        double thr_factor = 0.5 + 0.5 * std::max(0.0, align);
        double thr_scaled = std::max(static_cast<double>(TURN_THR), thr * thr_factor);
        if (thr_scaled < thr - 0.5) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[THR-ALIGN] esc=%.2f goal=%.2f align=%.2f factor=%.2f thr %.1f→%.1f",
                escape_yaw_, goal_dir, align, thr_factor, thr, thr_scaled);
        }
        thr = thr_scaled;
    }

    // Fix 63B: compensazione pitch per thr in CRUISE e ESCAPE.
    // Quando |pitch|>0.3 rad (17°), il thruster genera componente verticale sbagliata:
    // nose-down → thr spinge giù (es. pitch=-0.85: sin=0.75 → 45N DOWN a thr=60N).
    // Scala thr×cos(pitch) per mantenere solo la componente orizzontale. Floor a TURN_THR.
    // Fix 132: soglia alzata 0.3→0.5 rad. Con 0.3 (17°) scattava troppo presto:
    // riduceva la spinta → meno flusso → meno autorità pinne → pitch non scendeva.
    if (std::abs(state_.pitch) > 0.5) {
        double cos_p = std::max(std::cos(std::abs(state_.pitch)), 0.15);
        double thr_pitch_cap = std::max(CRUISE_THR * cos_p, static_cast<double>(TURN_THR));
        if (thr > thr_pitch_cap) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[PITCH-THR-CAP] pitch=%.3f → thr %.1f→%.1f",
                state_.pitch, thr, thr_pitch_cap);
            thr = thr_pitch_cap;
        }
    }

    // Fix 59A: spin brake in CRUISE — quando yawrate cresce (ESCAPE lascia residuo spin),
    // il PD normale satura le pinne ma l'EMA ritarda il damping → risonanza crescente.
    // Pura smorzatura con az immediato (bypass EMA) e cap thr per ridurre energia rotazionale.
    if (mode_ == Mode::CRUISE && std::abs(state_.yaw_rate) > 1.5) {
        az_cmd = std::clamp(-2.5 * state_.yaw_rate, -0.45, 0.45);
        thr    = std::min(thr, static_cast<double>(TURN_THR));  // 20N: togli energia allo spin
        sm_az_ = az_cmd;                         // bypass EMA: risposta immediata
        double el_alpha = (std::abs(wander_z_ - state_.z) > 1.5 || std::abs(state_.pitch) > 0.25) ? 0.50 : 0.35;
        sm_el_ = (1.0 - el_alpha)*sm_el_ + el_alpha*el_cmd;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[SPIN-BRAKE-CRUISE] yawrate=%.2f → thr=%.1f az=%.3f",
            state_.yaw_rate, thr, az_cmd);
        cmd(thr, sm_az_, sm_el_);
        return;
    }

    // Filtraggio EMA dei comandi: α=0.25 in CRUISE, α=1.0 (nessuno) in ESCAPE
    thr = std::min(thr, turn_thrust_cap);
    if (mode_ == Mode::CRUISE) {
        sm_az_ = 0.75*sm_az_ + 0.25*az_cmd;
        double el_alpha = (std::abs(wander_z_ - state_.z) > 1.5 || std::abs(state_.pitch) > 0.25) ? 0.50 : 0.35;
        sm_el_ = (1.0 - el_alpha)*sm_el_ + el_alpha*el_cmd;
    } else {
        sm_az_ = az_cmd;
        sm_el_ = el_cmd;
    }

    cmd(thr, sm_az_, sm_el_);

    double tgt_log = (mode_ == Mode::CRUISE) ? wander_yaw_ : escape_yaw_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "mode=%s xy=(%.1f,%.1f) z=%.2f yaw=%.3f pitch=%.3f yawrate=%.4f fwd_min=%.2f "
        "tgt_yaw=%.2f steer=%.3f elev=%.3f thr=%.1f",
        (mode_==Mode::CRUISE) ? "CRUISE" : (mode_==Mode::ESCAPE) ? "ESCAPE" : "BRAKE",
        state_.x, state_.y, state_.z, state_.yaw, state_.pitch, state_.yaw_rate,
        fwd_min, tgt_log, std::clamp(last_az_cmd_, -0.50, 0.50),
        std::clamp(-last_el_cmd_, -0.261799, 0.261799), last_thr_cmd_);
}
