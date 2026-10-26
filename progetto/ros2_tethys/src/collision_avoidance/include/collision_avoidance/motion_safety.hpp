#pragma once

#include <algorithm>
#include <cmath>

inline double rear_stopping_distance(double forward_speed)
{
    const double speed = std::max(0.0, -forward_speed);
    return 3.0 + 0.4*speed + speed*speed/0.6;
}

inline bool recovery_reverse_can_resume(double speed, double front, double rear,
                                        double emergency_clearance)
{
    // A previous rear-stop request is not permanent. Require a completed stop
    // and extra clearance before backing again; avoid toggling at one threshold.
    return std::abs(speed) < 0.15 && front < emergency_clearance
        && rear > rear_stopping_distance(speed) + 1.0;
}

// Negative thrust while moving forward is a brake, not a reverse manoeuvre.
// A rear obstacle must inhibit backing up without disabling that brake.
inline double recovery_thrust(double forward_speed, double rear_clearance,
                              bool stop_reverse)
{
    if (forward_speed > 0.15) return -50.0;
    if (stop_reverse || rear_clearance <= rear_stopping_distance(forward_speed) + 0.5)
        return forward_speed < -0.15 ? 50.0 : 0.0;
    return std::clamp(-40.0 * (forward_speed + 0.30), -15.0, 15.0);
}
