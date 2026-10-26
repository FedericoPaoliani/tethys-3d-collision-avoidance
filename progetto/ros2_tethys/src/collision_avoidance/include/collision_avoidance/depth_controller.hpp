#pragma once

#include <algorithm>
#include <cmath>

// Slow trim correction for persistent depth error under forward flow. Keep
// the existing pitch feedback; do not integrate during a stop or a reversal.
class DepthTrim
{
public:
    double update(double error, double vz, double pitch, double speed,
                  bool enabled, double dt)
    {
        const double proportional = 0.12*error - 0.50*vz;
        if (enabled && speed > 0.4 && dt > 0.0 && dt <= 0.5) {
            const double desired = proportional + integral_;
            const double effort = 1.2*(std::clamp(desired, -0.30, 0.30)-pitch);
            // Anti-windup: freeze only the direction that worsens saturation.
            const bool saturating = (error > 0.0 && (desired >= 0.30 || effort >= 0.25))
                || (error < 0.0 && (desired <= -0.30 || effort <= -0.25));
            if (!saturating)
                integral_ = std::clamp(integral_ + 0.006*error*dt, -0.15, 0.15);
        }
        return integral_;
    }

private:
    double integral_{0.0};
};
