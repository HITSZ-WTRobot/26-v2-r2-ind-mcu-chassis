#include "config.hpp"
#include "exporter.hpp"
#include "trajectory_optimizer.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace planning;

int main()
{
    std::cout << "=== Trajectory Generator (C++ CasADi OCP) ===\n\n";

    std::map<std::string, TrajectoryResult> results;
    std::vector<std::string> names;
    const int trajectory_count = static_cast<int>(ENTRY_POINTS.size());
    for (int s = 0; s < trajectory_count; ++s)
        names.push_back(trajectory_name(s));

    int done = 0;
    for (int s = 0; s < trajectory_count; ++s)
    {
        const auto name = trajectory_name(s);
        auto t_start = std::chrono::steady_clock::now();

        std::cout << ">>> Solving " << name << "\n";
        auto trajectory = optimize_trajectory(s, SAMPLE_DT);

        auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start)
                               .count();

        ++done;
        if (trajectory.success)
        {
            results[name] = trajectory;
            std::cout << "  [" << done << "/" << trajectory_count << "] " << name << " OK, "
                      << trajectory.branch << ", " << trajectory.n_points << " pts, "
                      << trajectory.total_time << "s, wall " << elapsed << "s\n";
        }
        else
        {
            std::cout << "  [" << done << "/" << trajectory_count << "] " << name
                      << " FAILED, wall "
                      << elapsed << "s\n";
        }

        for (const auto& report : trajectory.branch_reports)
            std::cout << "      " << report << "\n";
        std::cout << "\n";
    }

    std::cout << ">>> Exporting HPP/CSV files...\n";
    const std::string dist_dir = "dist";
    export_trajectory_point_hpp(dist_dir);
    std::size_t total_pts = 0;
    for (const auto& name : names)
    {
        auto it = results.find(name);
        if (it != results.end() && it->second.success)
        {
            total_pts += export_trajectory_hpp(name, it->second, dist_dir);
            export_trajectory_csv(name, it->second, dist_dir);
        }
    }
    export_trajectory_all_hpp(names, dist_dir);

    std::cout << "\n=== Summary ===\n";
    int succeeded = 0;
    for (const auto& name : names)
    {
        auto it = results.find(name);
        if (it != results.end() && it->second.success)
        {
            ++succeeded;
            std::cout << "  " << name << ": " << it->second.total_time << "s, "
                      << it->second.n_points << " pts, " << it->second.branch << "\n";
        }
        else
        {
            std::cout << "  " << name << ": FAILED\n";
        }
    }
    std::cout << "\n  Success: " << succeeded << "/" << trajectory_count
              << ", Total points: " << total_pts << ", Flash: " << (total_pts * 32 / 1024)
              << " KB\n";

    return succeeded == trajectory_count ? 0 : 1;
}
