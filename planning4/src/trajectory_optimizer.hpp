#pragma once
#include <vector>
#include <array>
#include <string>

namespace planning {

struct TrajectoryResult {
    std::vector<double> t;
    std::vector<std::array<double, 8>> trajectory; // x,y,yaw,h,dx,dy,dyaw,dh
    std::string branch;
    std::vector<std::string> branch_reports;
    double total_time = 0;
    size_t n_points = 0;
    bool success = false;
};

TrajectoryResult optimize_trajectory(int trajectory_idx, double sample_dt = 0.002);

} // namespace planning
