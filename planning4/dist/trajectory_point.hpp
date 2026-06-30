#pragma once

namespace planning::trajectory {

struct TrajectoryPoint8 {
    float x, y, yaw, h;
    float dx, dy, dyaw, dh;
};

} // namespace planning::trajectory
