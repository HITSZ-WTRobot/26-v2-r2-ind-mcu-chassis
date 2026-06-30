#pragma once
#include <cstddef>
#include <string>
#include <vector>
#include "trajectory_optimizer.hpp"

namespace planning {

void export_trajectory_point_hpp(const std::string& output_dir);
std::size_t export_trajectory_hpp(const std::string& name,
                                  const TrajectoryResult& result,
                                  const std::string& output_dir);
void export_trajectory_all_hpp(const std::vector<std::string>& names,
                               const std::string& output_dir);

void export_trajectory_csv(const std::string& name,
                           const TrajectoryResult& result,
                           const std::string& output_dir);

} // namespace planning
