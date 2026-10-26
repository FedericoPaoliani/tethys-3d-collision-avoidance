#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

// Short-lived observations in world coordinates. In particular, an obstacle
// just passed must not disappear from the reverse corridor when it leaves FOV.
class ObstacleMemory
{
public:
    void observe(double x, double y, double z, double t)
    {
        points_.push_back({x, y, z, t});
        while (!points_.empty() && (t - points_.front().t > 30.0 || points_.size() > 30000))
            points_.pop_front();
    }

    // A long brake/reverse must not forget the obstacle that limited its
    // retreat just because its sonar observation becomes 30 seconds old.
    // Keep this bounded snapshot until forward travel has actually resumed.
    void begin_recovery(double t)
    {
        if (recovery_active_) return;
        recovery_active_ = true;
        recovery_points_.clear();
        for (const auto &p : points_)
            if (t >= p.t && t-p.t <= 30.0) recovery_points_.push_back(p);
    }

    void end_recovery()
    {
        recovery_active_ = false;
        recovery_points_.clear();
    }

    double clearance(double x, double y, double z, double yaw, double t) const
    {
        double result = 99.0;
        const double c = std::cos(yaw), s = std::sin(yaw);
        auto include = [&](const auto &p) {
            if (std::abs(p.z-z) > 1.5) return;
            const double dx = p.x-x, dy = p.y-y;
            const double fwd = c*dx+s*dy, lateral = -s*dx+c*dy;
            if (fwd > 0.0 && std::abs(lateral) < 1.5)
                result = std::min(result, fwd);
        };
        for (const auto &p : points_) {
            if (t - p.t > 30.0 || t < p.t) continue;
            include(p);
        }
        for (const auto &p : recovery_points_) include(p);
        return result;
    }

private:
    struct Point { double x, y, z, t; };
    std::deque<Point> points_;
    std::deque<Point> recovery_points_;
    bool recovery_active_{false};
};
