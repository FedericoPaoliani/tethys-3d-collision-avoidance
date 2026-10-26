#include "collision_avoidance/vfh_planner.hpp"
#include "collision_avoidance/obstacle_memory.hpp"
#include "collision_avoidance/motion_safety.hpp"
#include "collision_avoidance/depth_controller.hpp"
#include "initial_front_samples.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

int main()
{
    DepthTrim trim;
    double bias = 0.0;
    for (int i=0; i<50; ++i)
        bias = trim.update(1.6, 0.0, 0.09, 0.9, true, 0.2);
    assert(bias > 0.05 && bias <= 0.15);
    assert(trim.update(1.6, 0.0, 0.09, 0.9, false, 0.2) == bias);
    assert(trim.update(1.6, 0.0, 0.09, -0.3, true, 0.2) == bias);
    assert(trim.update(1.6, 0.0, 0.09, 0.9, true, 10.0) == bias);
    assert(trim.update(10.0, 0.0, 0.09, 0.9, true, 0.2) == bias);
    assert(trim.update(-0.2, 0.0, 0.0, 0.9, true, 0.2) < bias);
    // Run 19: a remembered obstacle behind the moving drone disabled the
    // front brake (zero thrust at +0.85 m/s), leading to a physical collision.
    assert(recovery_thrust(0.85, 2.0, true) == -50.0);
    assert(recovery_thrust(0.85, 20.0, false) == -50.0);
    assert(recovery_thrust(-0.3, 2.0, false) == 50.0);
    assert(recovery_thrust(-0.3, 20.0, true) == 50.0);
    assert(recovery_thrust(0.0, 2.0, false) == 0.0);
    assert(recovery_thrust(0.0, 20.0, false) < 0.0);
    // Slow reverse uses the same measured stopping envelope as the final
    // rear guard, with 0.5 m reserve, instead of an unrelated 6 m cutoff.
    assert(recovery_thrust(0.0, 4.8, false) < 0.0);
    assert(recovery_thrust(-0.3, rear_stopping_distance(-0.3)+0.4, false) > 0.0);
    assert(recovery_thrust(-0.3, 4.8, false) <= 0.0);
    assert(rear_stopping_distance(-1.0) > rear_stopping_distance(-0.3));
    // Run40: a stale stop-reverse latch held zero thrust at front3/rear9 m.
    assert(recovery_reverse_can_resume(0.04, 3.0, 9.5, 4.0));
    assert(!recovery_reverse_can_resume(-0.3, 3.0, 9.5, 4.0));
    assert(!recovery_reverse_can_resume(0.04, 3.0, 3.8, 4.0));
    assert(!recovery_reverse_can_resume(0.04, 5.0, 9.5, 4.0));
    SphHist histogram;
    histogram.reset();
    histogram.received = true;
    VehicleState state;
    VFHPlanner planner(histogram, state);
    assert(planner.corridor_clearance(0.0) > 70.0);
    double goal_heading = 0.0;
    assert(planner.find_lookahead_escape(goal_heading, 10.0*M_PI/180.0));
    assert(std::abs(goal_heading-10.0*M_PI/180.0) < 1e-8);

    // A return on the heading lies on a bin edge, not at the bin centre.
    // At 20-25 m, centre-only geometry used to classify it outside the hull corridor.
    for (double range : {20.0, 25.0}) {
        histogram.reset();
        histogram.min_r[SphHist::az_idx(0.0)][SphHist::el_idx(0.0)] = range;
        assert(planner.corridor_clearance(0.0) <= range + 1e-8);
        double heading = 0.0;
        assert(planner.find_lookahead_escape(heading, 0.0));
        assert(std::abs(heading) > 0.01);
    }

    // An obstacle clearly to the side must not obstruct the forward corridor.
    histogram.reset();
    histogram.min_r[SphHist::az_idx(M_PI/3)][SphHist::el_idx(0.0)] = 10.0;
    assert(planner.corridor_clearance(0.0) > 70.0);

    // A real return at (6,2) clears the 1.5 m half-width. Widening its
    // entire angular bin used to trigger a false forward emergency brake.
    histogram.reset();
    histogram.min_r[SphHist::az_idx(std::atan2(2.0, 6.0))][SphHist::el_idx(0.0)] = std::hypot(6.0, 2.0);
    histogram.horizontal_points_valid = true;
    histogram.horizontal_points = {{6.0, 2.0}};
    assert(planner.corridor_clearance(0.0) > 70.0);
    assert(planner.corridor_clearance(M_PI/6) < 7.0);
    assert(planner.find_lookahead_escape(goal_heading, 0.0));
    assert(std::abs(goal_heading) < 1e-8);

    // Exact points must still detect an on-axis obstacle and choose a detour.
    histogram.horizontal_points = {{20.0, 0.0}, {25.0, 0.0}};
    assert(planner.corridor_clearance(0.0) == 20.0);
    assert(planner.find_lookahead_escape(goal_heading, 0.0));
    assert(std::abs(goal_heading) > 0.01);
    SphHist smoothed;
    SphHist::smooth(histogram, smoothed);
    assert(smoothed.horizontal_points_valid);
    assert(smoothed.horizontal_points == histogram.horizontal_points);
    smoothed.reset();
    assert(!smoothed.horizontal_points_valid && smoothed.horizontal_points.empty());

    // Two safe alternatives: +5 degrees / 25 m and -20 degrees / 44 m.
    // Extra distant clearance must not win over the modest goal detour.
    histogram.reset();
    histogram.horizontal_points_valid = true;
    histogram.horizontal_points = {{11.9,-10.9}, {12.5,-10.3}, {25.6,-7.5},
        {14.7,-3.4}, {14.0,-2.5}, {14.0,-1.8}, {14.4,-1.1},
        {25.4,2.0}, {25.5,3.3}, {11.3,4.4}, {10.8,4.9}, {10.8,5.5}};
    planner.set_walls(15.0, 60.0);
    state.x = 0.0; state.y = -50.0; state.yaw = M_PI/2;
    assert(planner.find_lookahead_escape(goal_heading, 0.0));
    assert(goal_heading > 0.0 && goal_heading <= 10.0*M_PI/180.0);
    assert(planner.corridor_clearance(goal_heading) > 20.0);

    // Measured clouds: a 5-degree initial yaw change must not open a phantom
    // 86 m corridor between discrete rays. Both observations select world 95°.
    histogram.horizontal_points = nominal_front_samples;
    state.yaw = M_PI/2;
    assert(planner.find_lookahead_escape(goal_heading, 0.0));
    assert(std::abs(goal_heading - 5.0*M_PI/180.0) < 1e-8);
    histogram.horizontal_points = rotated_front_samples;
    state.yaw = M_PI/2 + 5.0*M_PI/180.0;
    assert(planner.corridor_clearance(5.0*M_PI/180.0) < 26.0);
    assert(planner.find_lookahead_escape(goal_heading, -5.0*M_PI/180.0));
    assert(std::abs(goal_heading) < 1e-8);
    // Reproduce the old false-clear result on exactly the same measurements.
    for (auto &point : histogram.horizontal_points) point[2] = 0.0;
    assert(planner.corridor_clearance(5.0*M_PI/180.0) > 80.0);
    assert(planner.find_lookahead_escape(goal_heading, -5.0*M_PI/180.0));
    assert(std::abs(goal_heading-5.0*M_PI/180.0) < 1e-8);

    // Nearby off-axis returns remain passable with the measured uncertainty.
    histogram.horizontal_points = {{6.0, 2.0, 0.16}};
    assert(planner.corridor_clearance(0.0) > 30.0);

    state.x=0.0; state.y=0.0; state.yaw=0.0;
    histogram.horizontal_points = {{6.0, 1.5, 0.0}};
    // The straight diagonal clears this return, but its entry arc hits it.
    assert(planner.corridor_clearance(M_PI/4) > 12.0);
    assert(!planner.waypoint_arc_free(M_PI/4, 12.0));
    histogram.horizontal_points = {{6.0, -1.5, 0.0}};
    assert(!planner.waypoint_arc_free(-M_PI/4, 12.0));
    assert(planner.waypoint_arc_free(M_PI/4, 12.0));
    histogram.horizontal_points.clear();
    assert(planner.waypoint_arc_free(M_PI/4, 12.0));
    assert(planner.waypoint_arc_free(-M_PI/4, 12.0));
    assert(!planner.waypoint_arc_free(M_PI/4, 6.0));
    assert(planner.waypoint_arc_free(0.0, 12.0));
    // World-frame wall checks must rotate the path with the vehicle.
    planner.set_walls(15.0, 60.0);
    state.x = 10.0;
    assert(!planner.waypoint_arc_free(0.0, 12.0));
    state.yaw = M_PI/2;
    assert(planner.waypoint_arc_free(0.0, 12.0));
    assert(!planner.waypoint_arc_free(-M_PI/4, 12.0));

    // Tank geometry still bounds a direction with no sonar returns.
    histogram.reset();
    planner.set_walls(14.75, 59.75);
    state.y = 0.0; state.yaw = 0.0;
    state.x = 10.0;
    assert(std::abs(planner.corridor_clearance(0.0) - 4.75) < 1e-8);

    // An observed obstacle remains behind the vehicle after it passes. Keeping
    // the old body-frame range would incorrectly leave it in front instead.
    ObstacleMemory memory;
    memory.observe(0.0, 5.0, 8.0, 1.0);
    assert(std::abs(memory.clearance(0, 0, 8, M_PI/2, 2) - 5) < 1e-8);
    assert(std::abs(memory.clearance(0, 8, 8, -M_PI/2, 3) - 3) < 1e-8);
    assert(memory.clearance(0, 8, 8, M_PI/2, 3) == 99.0);
    assert(memory.clearance(0, 8, 12, -M_PI/2, 3) == 99.0);
    assert(memory.clearance(0, 8, 8, -M_PI/2, 32) == 99.0);

    // An ongoing reverse manoeuvre must retain the observation that made
    // its rear corridor unsafe, even when newer sonar frames prune history.
    memory.begin_recovery(25.0);
    memory.observe(20, 20, 8, 40.0);
    assert(memory.clearance(0, 8, 8, -M_PI/2, 40) == 3.0);
    memory.begin_recovery(41.0);  // Repeated brake/replan ticks cannot discard it.
    assert(memory.clearance(0, 8, 8, -M_PI/2, 41) == 3.0);
    memory.end_recovery();
    assert(memory.clearance(0, 8, 8, -M_PI/2, 41) == 99.0);

}
