// CollisionAvoidanceNode: metriche sperimentali (/ca_metrics) e CSV dei risultati.

#include "collision_avoidance/collision_avoidance_node.hpp"

// ── Metriche sperimentali ─────────────────────────────────────────────────
void CollisionAvoidanceNode::publish_metrics()
{
    if (!metrics_initialized_) return;
    auto now_r     = std::chrono::steady_clock::now();
    double real_t  = std::chrono::duration<double>(now_r - real_start_).count();
    double sim_t   = last_pose_t_ - sim_start_t_;
    double dx = state_.x - start_x_, dy = state_.y - start_y_, dz = state_.z - start_z_;
    double eucl = std::sqrt(dx*dx + dy*dy + dz*dz);
    double eff  = total_dist_ > 0.01 ? (goal_active_ ?
        (goal_reached_ ? std::min(1.0, initial_goal_dist_/total_dist_) : 0.0) : eucl/total_dist_) : 0.0;
    double vavg = sim_t > 0.01 ? total_dist_/sim_t : 0.0;
    double rt   = (real_t > 0.1) ? sim_t / real_t : 0.0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "{\"world\":\"" << world_name_ << "\","
        << "\"n_obs\":" << n_obstacles_ << ","
        << "\"collisions\":" << (contacts_available_ ? collision_count_ : -1) << ","
        << "\"goal_reached\":" << (goal_reached_ ? "true" : "false") << ","
        << "\"dist\":" << total_dist_ << ","
        << "\"eucl\":" << eucl << ","
        << "\"eff\":" << eff << ","
        << "\"vavg\":" << vavg << ","
        << "\"sim_t\":" << sim_t << ","
        << "\"real_t\":" << real_t << ","
        << "\"rt_factor\":" << rt << "}";
    std_msgs::msg::String m;
    m.data = oss.str();
    metrics_pub_->publish(m);
}

void CollisionAvoidanceNode::write_csv()
{
    if (!metrics_initialized_) return;
    // Al raggiungimento del goal write_csv() viene invocata esplicitamente e poi di
    // nuovo dal distruttore: senza questa guardia ogni missione completata lasciava
    // due righe quasi identiche nel CSV, falsando ogni statistica calcolata su di esso.
    if (csv_written_) return;
    csv_written_ = true;
    double cpu_t  = (double)(std::clock() - cpu_start_) / CLOCKS_PER_SEC;
    double sim_t  = last_pose_t_ - sim_start_t_;
    double dx = state_.x - start_x_, dy = state_.y - start_y_, dz = state_.z - start_z_;
    double eucl = std::sqrt(dx*dx + dy*dy + dz*dz);
    double eff  = total_dist_ > 0.01 ? (goal_active_ ?
        (goal_reached_ ? std::min(1.0, initial_goal_dist_/total_dist_) : 0.0) : eucl/total_dist_) : 0.0;
    double vavg = sim_t > 0.01 ? total_dist_/sim_t : 0.0;
    double wall_t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - real_start_).count();
    double rt   = (wall_t > 0.01) ? sim_t / wall_t : 0.0;
    auto sys_now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(sys_now);
    char ts[32]; std::strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", std::localtime(&tt));
    std::string path = csv_path_.empty()
        ? std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/ros2_tethys/risultati.csv"
        : csv_path_;
    const std::string header = "timestamp,mondo,n_ostacoli,collisioni,dist_m,eucl_m,efficienza,v_media_ms,t_sim_s,t_cpu_s,rt_factor,goal_raggiunto,dist_min_target_m,dist_finale_target_m,contatti_disponibili,near_contact";
    // Preserve historical files: their collision and efficiency definitions differ.
    std::ifstream existing(path);
    std::string old_header;
    if (std::getline(existing, old_header) && old_header != header) {
        auto dot = path.rfind('.');
        path.insert(dot == std::string::npos ? path.size() : dot, "_v2");
    }
    bool need_header = !std::ifstream(path).good();
    std::ofstream f(path, std::ios::app);
    if (!f) { RCLCPP_ERROR(get_logger(), "CSV non apribile: %s", path.c_str()); return; }
    if (need_header)
        f << header << "\n";
    f << std::fixed << std::setprecision(3)
      << ts << "," << world_name_ << "," << n_obstacles_ << "," << (contacts_available_ ? collision_count_ : -1) << ","
      << total_dist_ << "," << eucl << "," << eff << "," << vavg << ","
      << sim_t << "," << cpu_t << "," << rt << "," << goal_reached_ << ","
      << (goal_active_ ? min_goal_dist_ : -1.0) << ","
      << (goal_active_ ? std::hypot(std::hypot(target_x_-state_.x, target_y_-state_.y), target_z_-state_.z) : -1.0)
      << "," << contacts_available_ << "," << near_contact_count_ << "\n";
    RCLCPP_INFO(get_logger(), "[CSV] Salvato: %s (collisioni=%d dist=%.1fm eff=%.3f)",
        path.c_str(), contacts_available_ ? collision_count_ : -1, total_dist_, eff);
}
