#include "exporter.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>

namespace planning {

static void ensure_dir(const std::string& dir) {
    mkdir(dir.c_str(), 0755);
}

void export_trajectory_point_hpp(const std::string& output_dir) {
    ensure_dir(output_dir);
    std::string path = output_dir + "/trajectory_point.hpp";
    std::ofstream f(path);
    f << "#pragma once\n"
      << "#include <array>\n"
      << "#include <cstdint>\n\n"
      << "struct TrajectoryPoint8 {\n"
      << "    float x, y, yaw, h;\n"
      << "    float dx, dy, dyaw, dh;\n"
      << "};\n";
    f.close();
    std::cout << "  Wrote " << path << "\n";
}

void export_trajectory_hpp(const std::string& name,
                           const TrajectoryResult& result,
                           const std::string& output_dir) {
    ensure_dir(output_dir);
    std::string path = output_dir + "/trajectory_" + name + ".hpp";
    std::ofstream f(path);
    f << "#pragma once\n"
      << "#include \"trajectory_point.hpp\"\n\n"
      << "inline constexpr std::array<TrajectoryPoint8, "
      << result.n_points << "> kTrajectory_" << name << "{{{\n";

    f << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < result.n_points; ++i) {
        const auto& p = result.trajectory[i];
        // Clamp to 6 decimal places for readability
        f << "    {" << p[0] << "f, " << p[1] << "f, "
          << p[2] << "f, " << p[3] << "f, "
          << p[4] << "f, " << p[5] << "f, "
          << p[6] << "f, " << p[7] << "f}";
        if (i < result.n_points - 1) f << ",\n";
        else f << "\n";
    }
    f << "}}};\n";
    f.close();
    std::cout << "  Wrote " << path << " (" << result.n_points << " points)\n";
}

void export_trajectory_all_hpp(const std::vector<std::string>& names,
                               const std::string& output_dir) {
    ensure_dir(output_dir);
    std::string path = output_dir + "/trajectory_all.hpp";
    std::ofstream f(path);
    f << "#pragma once\n\n";
    for (const auto& name : names) {
        f << "#include \"trajectory_" << name << ".hpp\"\n";
    }
    f.close();
    std::cout << "  Wrote " << path << "\n";
}

void export_trajectory_csv(const std::string& name,
                           const TrajectoryResult& result,
                           const std::string& output_dir) {
    ensure_dir(output_dir);
    std::string path = output_dir + "/" + name + ".csv";
    std::ofstream f(path);
    f << std::fixed << std::setprecision(6);
    f << "t,x,y,yaw,h,dx,dy,dyaw,dh\n";
    for (size_t i = 0; i < result.n_points; ++i) {
        const auto& p = result.trajectory[i];
        f << result.t[i] << ","
          << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << ","
          << p[4] << "," << p[5] << "," << p[6] << "," << p[7] << "\n";
    }
    f.close();
    std::cout << "  Wrote " << path << " (" << result.n_points << " points)\n";
}

} // namespace planning
