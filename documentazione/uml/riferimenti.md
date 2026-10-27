# Riferimenti e dichiarazioni del controller

I diagrammi sono una vista selettiva. Questa appendice riporta tutte le righe dichiarative non vuote dell’header del nodo (commenti rimossi, visibilità conservata), per consultare anche timer, contatori e parametri omessi nel diagramma. Non è un nuovo sorgente da compilare.

- [collision_avoidance_node.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/collision_avoidance_node.hpp)
- [depth_controller.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/depth_controller.hpp)
- [motion_safety.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/motion_safety.hpp)
- [obstacle_memory.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/obstacle_memory.hpp)
- [sph_hist.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/sph_hist.hpp)
- [vehicle_state.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/vehicle_state.hpp)
- [vfh_planner.hpp](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/vfh_planner.hpp)

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "collision_avoidance/sph_hist.hpp"
#include "collision_avoidance/vehicle_state.hpp"
#include "collision_avoidance/vfh_planner.hpp"
#include "collision_avoidance/obstacle_memory.hpp"
#include "collision_avoidance/motion_safety.hpp"
#include "collision_avoidance/depth_controller.hpp"
class CollisionAvoidanceNode : public rclcpp::Node
{
public:
    CollisionAvoidanceNode();
    ~CollisionAvoidanceNode() { write_csv(); }
private:
    void sonar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, SensorId id);
    void build_merged();
    void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void control_loop();
    void stop_actuators();
    void cmd(double thr, double az, double el);
    void create_local_waypoint(double yaw_world, double clearance);
    void create_local_waypoint_throttled(double yaw_world, double clearance, double t);
    double steer_limit(double fwd_min) const;
    double preferred_dir() const;
    double lookahead_preferred_dir(double fwd_min) const;
    void publish_metrics();
    void write_csv();
    std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, N_SENSORS> sonar_subs_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
    std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subs_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vert_pub_, horiz_pub_, thr_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<SphHist, N_SENSORS> raw_;
    std::array<std::vector<std::array<double, 4>>, N_SENSORS> observed_points_;
    SphHist merged_, sm_;
    VehicleState state_;
    ObstacleMemory obstacle_memory_;
    double prev_x_{0.0}, prev_y_{0.0}, prev_z_{2.5}, prev_yaw_{0.0}, last_pose_t_{0.0};
    VFHPlanner planner_;
    enum class Mode { CRUISE, ESCAPE, BRAKE };
    Mode   mode_{Mode::CRUISE};
    double mode_until_{0.0}, plan_t_{0.0}, goal_replan_t_{-100.0};
    double escape_yaw_{0.0}, escape_depth_{2.6};
    bool   vertical_escape_{false};
    double escape_start_t_{0.0};
    bool   escape_lateral_{false};
    bool   local_wp_active_{false};
    double local_wp_x_{0.0}, local_wp_y_{0.0};
    double wp_last_dist_{0.0};
    double wp_last_check_t_{-1.0};
    double goal_clear_since_{-1.0};
    double sm_az_{0.0}, sm_el_{0.0};
    double wander_yaw_{0.0}, wander_z_{10.0};
    double wander_wp_t_{-30.0}, wander_z_t_{-25.0};
    std::mt19937 rng_{std::random_device{}()};
    double brake_entry_t_{0.0};
    double stuck_escape_until_{0.0};
    double wall_stuck_until_{0.0};
    double sonar_brake_until_{0.0};
    double post_brake_thr_until_{0.0};
    double wall_fwd_brake_until_{0.0};
    std::chrono::steady_clock::time_point sonar_wall_start_{};
    double brake_begin_t_{-1000.0};
    double brake_hold_until_{0.0};
    bool   post_brake_long_{false};
    int    stuck_count_{0};
    double pre_vert_t_{-99.0};
    bool   stabilize_active_{false};
    double stabilize_coast_until_{0.0};
    double stabilize_until_{0.0};
    double stabilize_max_until_{0.0};
    double stabilize_reentry_until_{0.0};
    bool   alt_recovery_active_{false};
    double alt_recovery_entry_t_{0.0};
    double alt_recovery_until_{0.0};
    bool   circum_active_{false};
    int    circum_side_{0};
    double circum_until_{0.0};
    static constexpr double FINAL_APPROACH_DIST = 18.0;
    static constexpr double FINAL_ABORT_DIST    = 3.0;
    bool   final_approach_active_{false};
    double prev_turn_fwd_min_{99.0};
    double prev_turn_yaw_err_{0.0};
    double prev_turn_t_{-1.0};
    double prev_turn_target_{0.0};
    double turn_brake_until_{0.0};
    int    turn_not_closing_count_{0};
    bool   csv_written_{false};
    double prev_goal_dist_{-1.0};
    static constexpr double WP_BRAKE_FLOOR   = BRAKE_DIST;
    static constexpr double WP_DANGER_FLOOR  = 2.0;
    static constexpr double SONAR_BLIND_DIST = 3.0;
    static constexpr double SONAR_BLIND_MEM  = 2.0;
    double last_close_fwd_t_{-100.0};
    double last_close_fwd_min_{99.0};
    double avoid_entry_t_{-100.0};
    double rejoin_allowed_after_{-100.0};
    double last_local_path_t_{-100.0};
    double last_target_rejoin_t_{-100.0};
    double last_az_cmd_{0.0};
    double last_el_cmd_{0.0};
    double last_thr_cmd_{0.0};
    double floor_stuck_t_{-1.0};
    double stuck_pos_x_{0.0}, stuck_pos_y_{0.0}, stuck_pos_t_{-999.0};
    int    collision_count_{0};
    int near_contact_count_{0};
    bool contacts_received_{false}, goal_reached_{false};
    bool contacts_available_{false};
    double last_physical_contact_t_{-100.0};
    double initial_goal_dist_{0.0}, min_goal_dist_{std::numeric_limits<double>::infinity()};
    double last_collision_t_{-99.0};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
    std::string world_name_{"unknown"};
    int         n_obstacles_{0};
    std::string csv_path_{""};
    std::string drone_name_{"tethys"};
    double      target_x_{std::numeric_limits<double>::quiet_NaN()};
    double      target_y_{std::numeric_limits<double>::quiet_NaN()};
    double      target_z_{std::numeric_limits<double>::quiet_NaN()};
    double      goal_radius_{3.0};
    double      yaw_bias_{0.0};
    double      depth_limit_high_{30.0};
    double      inner_wx_{60.0};
    double      inner_wy_{60.0};
    bool        goal_active_{false};
    std::chrono::steady_clock::time_point real_start_;
    std::clock_t cpu_start_{0};
    double sim_start_t_{0.0};
    double start_x_{0.0}, start_y_{0.0}, start_z_{0.0};
    double prev_metric_x_{0.0}, prev_metric_y_{0.0}, prev_metric_z_{0.0};
    double prev_metric_t_{0.0}, forward_speed_{0.0};
    bool safety_braking_{false};
    DepthTrim depth_trim_;
    double last_depth_control_t_{-1.0};
    bool brake_stop_reverse_{false};
    double total_dist_{0.0}, speed_sum_{0.0};
    int    speed_n_{0};
    bool   metrics_initialized_{false};
    bool   pose_received_{false};
    int    no_sonar_ticks_{0};
    bool   sonar_first_seen_{false};
    bool   allow_vert_esc_{true};
    double lateral_gap_bias_{0.0};
};
```
