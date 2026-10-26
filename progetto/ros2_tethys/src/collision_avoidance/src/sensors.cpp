// CollisionAvoidanceNode: acquisizione sonar e posa, istogramma unificato.

#include "collision_avoidance/collision_avoidance_node.hpp"

// ── Callback sonar ────────────────────────────────────────────────────────
void CollisionAvoidanceNode::sonar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, SensorId id)
{
    SphHist &h = raw_[id];
    h.reset();
    h.received  = true;
    h.last_time = now().seconds();
    observed_points_[id].clear();

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    double angular_step = std::numeric_limits<double>::infinity();
    double previous_bearing = 0.0;
    int previous_column = -1;
    size_t index = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++index) {
        const int column = index % std::max(1u, msg->width);
        if (column == 0) previous_column = -1;
        double xs = *it_x, ys = *it_y, zs = *it_z;
        if (!std::isfinite(xs) || !std::isfinite(ys) || !std::isfinite(zs)) continue;
        // Organized lidar rows retain column positions even across misses.
        // Infer spacing from actual ray bearings, avoiding world-specific
        // assumptions about sensor FOV or number of samples.
        const double bearing = std::atan2(ys, xs);
        if (previous_column >= 0) {
            const double step = std::abs(normalize(bearing-previous_bearing))
                / (column-previous_column);
            if (step > 1e-6) angular_step = std::min(angular_step, step);
        }
        previous_column = column;
        previous_bearing = bearing;

        double xb, yb, zb;
        SphHist::to_body(xs, ys, zs, id, xb, yb, zb);

        double r = std::sqrt(xb*xb + yb*yb + zb*zb);
        if (r < RANGE_MIN || r > RANGE_MAX) continue;

        double az = std::atan2(yb, xb);
        double el = std::atan2(zb, std::sqrt(xb*xb + yb*yb));
        double w  = 1.0 / std::max(r * r, 0.01);

        int k = SphHist::az_idx(az), l = SphHist::el_idx(el);
        h.h[k][l]     += w;
        h.min_r[k][l]  = std::min(h.min_r[k][l], r);

        // Preserve unquantized returns in world coordinates. Reproject
        // them at control time so vehicle rotation between sonar frames
        // cannot leave an old observation at the wrong body-frame angle.
        if (pose_received_) {
            if (id == FRONT) xb += 1.0;
            if (id == LEFT) yb += 0.25;
            if (id == RIGHT) yb -= 0.25;
            if (id == BOTTOM) zb -= 0.15;
            if (id == TOP) zb += 0.15;
            // Navigation frame is model rotated by pi around Z: its roll
            // and pitch have opposite signs to the model Euler angles.
            const double cr = std::cos(state_.roll), sr = std::sin(state_.roll);
            const double cp = std::cos(state_.pitch), sp = std::sin(state_.pitch);
            const double yr = cr*yb + sr*zb, zr = -sr*yb + cr*zb;
            const double xp = cp*xb - sp*zr, zp = sp*xb + cp*zr;
            const double wx = state_.x + std::cos(state_.yaw)*xp - std::sin(state_.yaw)*yr;
            const double wy = state_.y + std::sin(state_.yaw)*xp + std::cos(state_.yaw)*yr;
            const double wz = state_.z + zp;
            observed_points_[id].push_back({wx, wy, wz, r});
            // Retain nearby observations separately after they leave FOV,
            // for the rear guard. Neither data set uses world geometry.
            if (id <= RIGHT && r > 2.0 && r < 12.0 && std::abs(el) < 0.20)
                obstacle_memory_.observe(wx, wy, wz, h.last_time);
        }
    }
    // A discrete ray is not a complete obstacle outline. Retain its
    // half-sample lateral uncertainty in metres at observation time.
    const double uncertainty_scale = std::isfinite(angular_step)
        ? std::sin(0.5*angular_step) : 0.0;
    for (auto &point : observed_points_[id]) point[3] *= uncertainty_scale;
}

// ── Costruzione istogramma unificato ──────────────────────────────────────
void CollisionAvoidanceNode::build_merged()
{
    double t = now().seconds();
    merged_.reset();
    merged_.received = false;
    merged_.horizontal_points_valid = pose_received_ && raw_[FRONT].received;
    const double c = std::cos(state_.yaw), s = std::sin(state_.yaw);

    for (int i = 0; i < N_SENSORS; ++i) {
        if (!raw_[i].received || (t - raw_[i].last_time) > SONAR_STALE) continue;
        merged_.received = true;
        for (const auto &point : observed_points_[i]) {
            // Exclude floor/ceiling returns by actual relative height,
            // rather than their elevation while the vehicle was pitched.
            if (std::abs(point[2] - state_.z) > 1.5) continue;
            const double dx = point[0] - state_.x, dy = point[1] - state_.y;
            merged_.horizontal_points.push_back({c*dx+s*dy, -s*dx+c*dy, point[3]});
        }
        for (int k = 0; k < N_AZ; ++k)
            for (int l = 0; l < N_EL; ++l) {
                merged_.h[k][l]     += raw_[i].h[k][l];
                merged_.min_r[k][l]  = std::min(merged_.min_r[k][l], raw_[i].min_r[k][l]);
            }
    }
    merged_.last_time = t;
    SphHist::smooth(merged_, sm_);
}

// ── Callback posa ─────────────────────────────────────────────────────────
void CollisionAvoidanceNode::pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for (const auto &tf : msg->transforms) {
        if (tf.child_frame_id != drone_name_) continue;

        double t     = now().seconds();
        double new_x = tf.transform.translation.x;
        double new_y = tf.transform.translation.y;
        double new_z = tf.transform.translation.z;

        double qx = tf.transform.rotation.x, qy = tf.transform.rotation.y;
        double qz = tf.transform.rotation.z, qw = tf.transform.rotation.w;

        double sinp = 2.0 * (qw * qy - qz * qx);
        double new_pitch = (std::abs(sinp) >= 1.0)
                           ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
        double new_yaw = std::atan2(2.0*(qw*qz + qx*qy),
                                    1.0 - 2.0*(qy*qy + qz*qz));

        // Fix Q rev3: phantom detection 3D — scarta intera posa se salta impossibile.
        // prev_x_, prev_y_ inizializzati dallo spawn (parametro) → funziona dal frame 0.
        // Soglie: xy 3m (max ~2 m/s × 0.2s), z 10m (impossibile fisicamente).
        {
            double dz = std::abs(new_z - prev_z_);
            double dx = std::abs(new_x - prev_x_);
            double dy = std::abs(new_y - prev_y_);
            if (dz > 10.0 || dx > 3.0 || dy > 3.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[TF phantom] x=%.1f y=%.1f z=%.2f (Δx=%.1f Δy=%.1f Δz=%.1f) — scartata",
                    new_x, new_y, new_z, dx, dy, dz);
                // Fix 108A: aggiorna prev_ al valore scartato così il frame successivo
                // viene confrontato con la posizione nuova (non con quella pre-salto).
                // Senza questo fix, il drone si "perde" dopo un contatto fisico: tutte le
                // frame successive vengono scartate perché Delta è ancora > 3m.
                prev_x_ = new_x; prev_y_ = new_y; prev_z_ = new_z;
                return;
            }
        }
        if (last_pose_t_ > 0.0) {
            double dt = t - last_pose_t_;
            if (dt > 0.001 && dt < 1.0) {
                // Fix 130: clamp derivate finite prima dell'EMA per evitare valori folli
                // (es. 1467 m/s vz, 1421 rad/s yr) da frame post-collisione in Gazebo.
                double raw_vz = std::clamp((new_z - prev_z_) / dt, -5.0, 5.0);
                // Clamp a ±5.0 rad/s: filtra spike TF isolati (glitch Gazebo post-frame)
                // senza mascherare spin reali sostenuti (che attraversano l'EMA in 2+ frame).
                double raw_yr = std::clamp(normalize(new_yaw - prev_yaw_) / dt, -5.0, 5.0);
                state_.vz       = 0.7*state_.vz       + 0.3*raw_vz;
                state_.yaw_rate = 0.7*state_.yaw_rate + 0.3*raw_yr;
            }
        }
        state_.x = new_x; state_.y = new_y; state_.z = new_z;
        state_.pitch = new_pitch; state_.yaw = new_yaw + yaw_bias_;
        state_.roll = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
        prev_x_ = new_x; prev_y_ = new_y; prev_z_ = new_z;
        prev_yaw_ = new_yaw; last_pose_t_ = t;
        pose_received_ = true;
    }
}
