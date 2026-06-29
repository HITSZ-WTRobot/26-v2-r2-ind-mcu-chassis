#include "exporter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <vector>

namespace planning
{

namespace
{

constexpr double kExportSampleHz = 500.0;
constexpr double kExportDt       = 1.0 / kExportSampleHz;
constexpr double kTimeEps        = 1e-9;

struct ExportTrajectory
{
    std::vector<double>                t;
    std::vector<std::array<double, 8>> trajectory;
    double                             duration = 0.0;
};

static void ensure_dir(const std::string& dir)
{
    mkdir(dir.c_str(), 0755);
}

static double clean_value(double value)
{
    return std::abs(value) < 1e-7 ? 0.0 : value;
}

static std::array<double, 8> interpolate_sample(const TrajectoryResult& result, double t)
{
    if (result.t.empty() || result.trajectory.empty())
        return {};

    if (t <= result.t.front())
        return result.trajectory.front();
    if (t >= result.t.back())
        return result.trajectory.back();

    const auto   upper = std::upper_bound(result.t.begin(), result.t.end(), t);
    const size_t idx1  = static_cast<size_t>(upper - result.t.begin());
    const size_t idx0  = idx1 - 1;
    const double t0    = result.t[idx0];
    const double t1    = result.t[idx1];
    const double alpha = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0;

    std::array<double, 8> point{};
    for (size_t i = 0; i < point.size(); ++i)
    {
        double a = result.trajectory[idx0][i];
        double b = result.trajectory[idx1][i];
        point[i] = clean_value((1.0 - alpha) * a + alpha * b);
    }
    return point;
}

static bool is_export_grid(const TrajectoryResult& result)
{
    if (!result.success || result.t.empty() || result.trajectory.empty())
        return false;

    const int   full_steps     = static_cast<int>(std::floor(result.total_time / kExportDt + 1e-9));
    std::size_t expected_count = static_cast<std::size_t>(full_steps) + 1;
    if (result.total_time - full_steps * kExportDt > kTimeEps)
        ++expected_count;
    if (result.t.size() != expected_count || result.trajectory.size() != expected_count)
        return false;

    for (int i = 0; i <= full_steps; ++i)
    {
        const double expected_t = std::min(i * kExportDt, result.total_time);
        if (std::abs(result.t[static_cast<std::size_t>(i)] - expected_t) > kTimeEps)
            return false;
    }

    if (result.total_time - full_steps * kExportDt > kTimeEps &&
        std::abs(result.t.back() - result.total_time) > kTimeEps)
    {
        return false;
    }

    return true;
}

static ExportTrajectory resample_for_header(const TrajectoryResult& result)
{
    ExportTrajectory export_result;
    export_result.duration = result.total_time;

    if (!result.success || result.t.empty() || result.trajectory.empty())
        return export_result;

    if (is_export_grid(result))
    {
        export_result.t          = result.t;
        export_result.trajectory = result.trajectory;
        return export_result;
    }

    const int full_steps = static_cast<int>(std::floor(result.total_time / kExportDt + 1e-9));
    export_result.t.reserve(static_cast<size_t>(full_steps) + 2);
    export_result.trajectory.reserve(static_cast<size_t>(full_steps) + 2);

    for (int i = 0; i <= full_steps; ++i)
    {
        double t = std::min(i * kExportDt, result.total_time);
        export_result.t.push_back(t);
        export_result.trajectory.push_back(interpolate_sample(result, t));
    }

    if (export_result.t.empty() || result.total_time - export_result.t.back() > kTimeEps)
    {
        export_result.t.push_back(result.total_time);
        export_result.trajectory.push_back(interpolate_sample(result, result.total_time));
    }

    return export_result;
}

} // namespace

void export_trajectory_point_hpp(const std::string& output_dir)
{
    ensure_dir(output_dir);
    std::string   path = output_dir + "/trajectory_point.hpp";
    std::ofstream f(path);
    f << "#pragma once\n"
      << "\n"
      << "namespace planning::trajectory {\n\n"
      << "struct TrajectoryPoint8 {\n"
      << "    float x, y, yaw, h;\n"
      << "    float dx, dy, dyaw, dh;\n"
      << "};\n\n"
      << "} // namespace planning::trajectory\n";
    f.close();
    std::cout << "  Wrote " << path << "\n";
}

std::size_t export_trajectory_hpp(const std::string&      name,
                                  const TrajectoryResult& result,
                                  const std::string&      output_dir)
{
    ensure_dir(output_dir);
    std::string            path          = output_dir + "/trajectory_" + name + ".hpp";
    const ExportTrajectory export_result = resample_for_header(result);
    std::ofstream          f(path);
    const std::size_t      point_count = export_result.trajectory.size();
    f << "#pragma once\n"
      << "#include <array>\n"
      << "#include <cstddef>\n"
      << "#include \"trajectory_point.hpp\"\n\n"
      << "namespace planning::trajectory::" << name << " {\n\n"
      << "inline constexpr double kSampleHz = 500.000000;\n"
      << "inline constexpr double kDt = 0.002000;\n"
      << "inline constexpr std::size_t kPointCount = " << point_count << ";\n"
      << "inline constexpr double kDuration = " << std::fixed << std::setprecision(6)
      << export_result.duration << ";\n\n"
      << "inline constexpr std::array<TrajectoryPoint8, kPointCount> kPoints{{\n";

    f << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto& p = export_result.trajectory[i];
        // Clamp to 6 decimal places for readability
        f << "    {" << p[0] << "f, " << p[1] << "f, " << p[2] << "f, " << p[3] << "f, " << p[4]
          << "f, " << p[5] << "f, " << p[6] << "f, " << p[7] << "f}";
        if (i < point_count - 1)
            f << ",\n";
        else
            f << "\n";
    }
    f << "}};\n\n"
      << "} // namespace planning::trajectory\n";
    f.close();
    std::cout << "  Wrote " << path << " (" << point_count << " points @ 500Hz)\n";
    return point_count;
}

void export_trajectory_all_hpp(const std::vector<std::string>& names, const std::string& output_dir)
{
    ensure_dir(output_dir);
    std::string   path = output_dir + "/trajectory_all.hpp";
    std::ofstream f(path);
    f << "#pragma once\n\n"
      << "#include \"trajectory_point.hpp\"\n";
    for (const auto& name : names)
    {
        f << "#include \"trajectory_" << name << ".hpp\"\n";
    }
    f.close();
    std::cout << "  Wrote " << path << "\n";
}

void export_trajectory_csv(const std::string&      name,
                           const TrajectoryResult& result,
                           const std::string&      output_dir)
{
    ensure_dir(output_dir);
    std::string            path          = output_dir + "/" + name + ".csv";
    const ExportTrajectory export_result = resample_for_header(result);
    std::ofstream          f(path);
    f << std::fixed << std::setprecision(6);
    f << "t,x,y,yaw,h,dx,dy,dyaw,dh\n";
    for (size_t i = 0; i < export_result.trajectory.size(); ++i)
    {
        const auto& p = export_result.trajectory[i];
        f << export_result.t[i] << "," << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << ","
          << p[4] << "," << p[5] << "," << p[6] << "," << p[7] << "\n";
    }
    f.close();
    std::cout << "  Wrote " << path << " (" << export_result.trajectory.size()
              << " points @ 500Hz)\n";
}

} // namespace planning
