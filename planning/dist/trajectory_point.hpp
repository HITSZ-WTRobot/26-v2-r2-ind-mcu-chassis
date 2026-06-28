#pragma once
#include <array>
#include <cstdint>

struct TrajectoryPoint8 {
    float x, y, yaw, h;
    float dx, dy, dyaw, dh;
};
