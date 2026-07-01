#include "trajectory_optimizer.hpp"
#include "config.hpp"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace casadi;

namespace planning
{
namespace
{

constexpr double kDegToRad                = M_PI / 180.0;
constexpr double kRadToDeg                = 180.0 / M_PI;
constexpr int    kWheelConstraintSubsteps = 8;
constexpr double kWheelAccelConstraintLimit = MAX_WHEEL_ACCEL - 0.5;
constexpr double kLateYawPenaltyWeight = 2e-4;
constexpr double kLateYawPenaltySigmaY = 0.75;

struct Rect
{
    const char* name;
    double      x_min;
    double      x_max;
    double      y_min;
    double      y_max;
};

struct Corridor
{
    const char* name;
    double      x_min;
    double      x_max;
    double      y_min;
    double      y_max;
    double      yaw_min_deg;
    double      yaw_max_deg;
    Rect        certificate;
    bool        requires_h_03;
};

struct PhaseSpec
{
    const char* corridor_name;
    int         nodes;
};

struct Anchor
{
    const char* name;
    double      x;
    double      y;
    double      yaw_deg;
    double      h;
};

struct BranchSpec
{
    const char*            suffix;
    double                 zone2_yaw_deg;
    std::vector<PhaseSpec> phases;
    std::vector<Anchor>    anchors;
};

constexpr Rect kFreeCertLeftTop{ "left_top_clear", 5.60, 9.30, 4.63, 6.00 };
constexpr Rect kFreeCertZone2Top{ "zone2_top_clear", 8.00, 12.00, 4.63, 6.00 };
constexpr Rect kFreeCertRightUpper{ "right_upper_clear", 10.80, 12.00, 3.40, 6.00 };
constexpr Rect kFreeCertRightNoObs2{ "right_no_obstacle2_clear", 10.80, 12.00, 2.70, 6.00 };
constexpr Rect kFreeCertUpperMid{ "zone3_upper_mid_clear", 9.50, 12.00, 3.40, 4.47 };
constexpr Rect kFreeCertMidNoObs2{ "zone3_no_obstacle2_mid_clear", 9.50, 12.00, 2.70, 4.47 };
constexpr Rect kFreeCertMidClear{ "zone3_mid_clear", 9.50, 11.65, 2.70, 4.47 };
constexpr Rect kFreeCertLowLeft{ "zone3_low_left_clear", 9.50, 11.65, 0.20, 4.47 };

const std::vector<Corridor> kBaseCorridors{
    { "zone4_approach", 6.40, 8.50, 5.35, 5.45, 0.0, 0.0, kFreeCertLeftTop, false },
    { "zone2_top0", 8.50, 11.43, 5.35, 5.45, 0.0, 0.0, kFreeCertZone2Top, true },
    { "right_descend", 11.24, 11.55, 3.90, 5.45, 0.0, 0.0, kFreeCertRightUpper, true },
    { "mid_left", 10.95, 11.30, 3.90, 3.95, 0.0, 0.0, kFreeCertUpperMid, false },
    { "finish", 10.95, 11.20, 3.227, 3.95, 0.0, 0.0, kFreeCertMidClear, false },
    { "right_down0", 11.34, 11.43, 3.235, 5.45, -90.0, 0.0, kFreeCertRightNoObs2, false },
    { "mid_no_obs2", 10.75, 11.43, 3.235, 3.90, -90.0, 0.0, kFreeCertMidNoObs2, false },
    { "lower_left_down", 10.55, 11.10, 2.36, 3.90, -90.0, 0.0, kFreeCertLowLeft, false },
    { "lower_finish", 10.45, 11.10, 2.00, 2.70, -90.0, 0.0, kFreeCertLowLeft, false },
};

const std::vector<BranchSpec> kBranches{
    { "z2yaw0_finish",
      0.0,
      { { "zone4_approach", 30 },
        { "zone2_top0", 42 },
        { "right_descend", 32 },
        { "mid_left", 4 },
        { "finish", 32 } },
      {
              { "zone2_entry", 8.50, 5.40, 0.0, 0.3 },
              { "right_clear", 11.30, 5.40, 0.0, 0.3 },
      } },
    { "z2yaw0_lower_finish",
      0.0,
      { { "zone4_approach", 30 },
        { "zone2_top0", 42 },
        { "right_down0", 22 },
        { "mid_no_obs2", 18 },
        { "lower_left_down", 38 },
        { "lower_finish", 28 } },
      {
              { "zone2_entry", 8.50, 5.40, 0.0, 0.3 },
              { "right_clear", 11.36, 5.40, 0.0, 0.3 },
      } },
};

struct NodeGuess
{
    double x;
    double y;
    double yaw_deg;
    double h;
};

struct BranchSolveResult
{
    TrajectoryResult trajectory;
    std::string      branch;
    std::string      message;
};

const Corridor& base_corridor(const std::string& name)
{
    auto it = std::find_if(kBaseCorridors.begin(),
                           kBaseCorridors.end(),
                           [&](const Corridor& c) { return name == c.name; });
    if (it == kBaseCorridors.end())
        throw std::runtime_error("unknown corridor: " + name);
    return *it;
}

std::vector<Corridor> corridors_for_start(const Waypoint& start)
{
    std::vector<Corridor> corridors = kBaseCorridors;
    for (auto& corridor : corridors)
    {
        if (std::string(corridor.name) == "zone4_approach")
        {
            corridor.x_min = start.x;
            corridor.y_min = std::min(corridor.y_min, start.y);
            corridor.y_max = std::max(corridor.y_max, start.y);
        }
    }
    return corridors;
}

const Corridor& corridor_by_name(const std::vector<Corridor>& corridors, const std::string& name)
{
    auto it = std::find_if(corridors.begin(),
                           corridors.end(),
                           [&](const Corridor& c) { return name == c.name; });
    if (it == corridors.end())
        throw std::runtime_error("unknown start corridor: " + name);
    return *it;
}

std::array<double, 4> trig_linear_range(double a, double b, double lo, double hi)
{
    std::vector<double> candidates{ lo, hi };
    double              phase = std::atan2(b, a);
    for (double base : { phase, phase + M_PI })
    {
        int k_min = static_cast<int>(std::floor((lo - base) / (2.0 * M_PI))) - 1;
        int k_max = static_cast<int>(std::ceil((hi - base) / (2.0 * M_PI))) + 1;
        for (int k = k_min; k <= k_max; ++k)
        {
            double theta = base + 2.0 * M_PI * k;
            if (lo <= theta && theta <= hi)
                candidates.push_back(theta);
        }
    }

    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (double theta : candidates)
    {
        double value = a * std::cos(theta) + b * std::sin(theta);
        min_v        = std::min(min_v, value);
        max_v        = std::max(max_v, value);
    }
    return { min_v, max_v, 0.0, 0.0 };
}

std::array<double, 4> footprint_aabb_over_corridor(const Corridor& corridor)
{
    double yaw_min = corridor.yaw_min_deg * kDegToRad;
    double yaw_max = corridor.yaw_max_deg * kDegToRad;
    double x_low   = std::numeric_limits<double>::infinity();
    double x_high  = -std::numeric_limits<double>::infinity();
    double y_low   = std::numeric_limits<double>::infinity();
    double y_high  = -std::numeric_limits<double>::infinity();

    for (double dx : { -HALF_L_EXPANDED, HALF_L_EXPANDED })
    {
        for (double dy : { -HALF_W_EXPANDED, HALF_W_EXPANDED })
        {
            auto fx = trig_linear_range(dx, -dy, yaw_min, yaw_max);
            auto fy = trig_linear_range(dy, dx, yaw_min, yaw_max);
            x_low   = std::min(x_low, corridor.x_min + fx[0]);
            x_high  = std::max(x_high, corridor.x_max + fx[1]);
            y_low   = std::min(y_low, corridor.y_min + fy[0]);
            y_high  = std::max(y_high, corridor.y_max + fy[1]);
        }
    }
    return { x_low, x_high, y_low, y_high };
}

void certify_corridors_for_start(int trajectory_idx, const BranchSpec& branch)
{
    const auto start     = start_point_for_trajectory(trajectory_idx);
    auto       corridors = corridors_for_start(start);
    for (const auto& phase : branch.phases)
    {
        const auto& corridor   = corridor_by_name(corridors, phase.corridor_name);
        auto        aabb       = footprint_aabb_over_corridor(corridor);
        const auto& cert       = corridor.certificate;
        double      min_margin = std::min({
                aabb[0] - cert.x_min,
                cert.x_max - aabb[1],
                aabb[2] - cert.y_min,
                cert.y_max - aabb[3],
        });
        if (min_margin <= 1e-4)
        {
            std::ostringstream oss;
            oss << trajectory_name(trajectory_idx) << " " << branch.suffix << " corridor "
                << corridor.name << " certificate failed, min clearance " << min_margin;
            throw std::runtime_error(oss.str());
        }
    }
}

int n_intervals(const BranchSpec& branch)
{
    int total = 0;
    for (const auto& phase : branch.phases)
        total += phase.nodes;
    return total;
}

std::vector<std::string> node_phase_names(const BranchSpec& branch)
{
    std::vector<std::string> names;
    for (const auto& phase : branch.phases)
        for (int i = 0; i < phase.nodes; ++i)
            names.emplace_back(phase.corridor_name);
    names.emplace_back(branch.phases.back().corridor_name);
    return names;
}

std::vector<int> interval_phase_indices(const BranchSpec& branch)
{
    std::vector<int> indices;
    for (int i = 0; i < static_cast<int>(branch.phases.size()); ++i)
        for (int j = 0; j < branch.phases[i].nodes; ++j)
            indices.push_back(i);
    return indices;
}

std::array<double, 2> corridor_h_range(const Corridor& corridor)
{
    if (corridor.requires_h_03)
        return { ZONE2_REQUIRED_H, ZONE2_REQUIRED_H };
    return { H_MIN, H_MAX };
}

struct CorridorIntersection
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double yaw_min_deg;
    double yaw_max_deg;
    double h_min;
    double h_max;
};

CorridorIntersection corridor_intersection(const Corridor& a, const Corridor& b)
{
    const auto           a_h = corridor_h_range(a);
    const auto           b_h = corridor_h_range(b);
    CorridorIntersection intersection{
        std::max(a.x_min, b.x_min),
        std::min(a.x_max, b.x_max),
        std::max(a.y_min, b.y_min),
        std::min(a.y_max, b.y_max),
        std::max(a.yaw_min_deg, b.yaw_min_deg),
        std::min(a.yaw_max_deg, b.yaw_max_deg),
        std::max(a_h[0], b_h[0]),
        std::min(a_h[1], b_h[1]),
    };
    if (intersection.x_min > intersection.x_max || intersection.y_min > intersection.y_max ||
        intersection.yaw_min_deg > intersection.yaw_max_deg ||
        intersection.h_min > intersection.h_max)
    {
        std::ostringstream oss;
        oss << "corridor transition has no overlap: " << a.name << " -> " << b.name;
        throw std::runtime_error(oss.str());
    }
    return intersection;
}

NodeGuess clamp_to_intersection(const NodeGuess&            reference,
                                const CorridorIntersection& intersection)
{
    return {
        std::clamp(reference.x, intersection.x_min, intersection.x_max),
        std::clamp(reference.y, intersection.y_min, intersection.y_max),
        std::clamp(reference.yaw_deg, intersection.yaw_min_deg, intersection.yaw_max_deg),
        std::clamp(reference.h, intersection.h_min, intersection.h_max),
    };
}

NodeGuess clamp_to_corridor(const NodeGuess& reference, const Corridor& corridor)
{
    auto h_range = corridor_h_range(corridor);
    return {
        std::clamp(reference.x, corridor.x_min, corridor.x_max),
        std::clamp(reference.y, corridor.y_min, corridor.y_max),
        std::clamp(reference.yaw_deg, corridor.yaw_min_deg, corridor.yaw_max_deg),
        std::clamp(reference.h, h_range[0], h_range[1]),
    };
}

NodeGuess boundary_guess_between(const std::vector<Corridor>& corridors,
                                 const BranchSpec&            branch,
                                 int                          phase_i,
                                 const NodeGuess&             current)
{
    const auto& from = corridor_by_name(corridors, branch.phases[phase_i].corridor_name);
    const auto& to   = corridor_by_name(corridors, branch.phases[phase_i + 1].corridor_name);

    const double x_min   = std::max(from.x_min, to.x_min);
    const double x_max   = std::min(from.x_max, to.x_max);
    const double y_min   = std::max(from.y_min, to.y_min);
    const double y_max   = std::min(from.y_max, to.y_max);
    const double yaw_min = std::max(from.yaw_min_deg, to.yaw_min_deg);
    const double yaw_max = std::min(from.yaw_max_deg, to.yaw_max_deg);
    const auto   from_h  = corridor_h_range(from);
    const auto   to_h    = corridor_h_range(to);
    const double h_min   = std::max(from_h[0], to_h[0]);
    const double h_max   = std::min(from_h[1], to_h[1]);

    if (x_min <= x_max && y_min <= y_max && yaw_min <= yaw_max && h_min <= h_max)
    {
        return {
            std::clamp(current.x, x_min, x_max),
            std::clamp(current.y, y_min, y_max),
            std::clamp(current.yaw_deg, yaw_min, yaw_max),
            std::clamp(current.h, h_min, h_max),
        };
    }
    return clamp_to_corridor(current, to);
}

std::vector<NodeGuess> phase_targets_for_branch(const BranchSpec&            branch,
                                                const std::vector<Corridor>& corridors,
                                                const Waypoint&              end)
{
    if (branch.anchors.size() + 1 > branch.phases.size())
    {
        std::ostringstream oss;
        oss << "branch " << branch.suffix << " has too many initial guess anchors";
        throw std::runtime_error(oss.str());
    }

    std::vector<NodeGuess> targets;
    for (const auto& anchor : branch.anchors)
        targets.push_back({ anchor.x, anchor.y, anchor.yaw_deg, anchor.h });

    while (targets.size() + 1 < branch.phases.size())
    {
        int phase_i = static_cast<int>(targets.size());
        targets.push_back(boundary_guess_between(corridors, branch, phase_i, targets.back()));
    }
    const NodeGuess final{ end.x, end.y, end.yaw_deg, end.h };
    targets.push_back(final);
    return targets;
}

std::vector<NodeGuess> initial_state_nodes(int trajectory_idx, const BranchSpec& branch)
{
    const auto             start     = start_point_for_trajectory(trajectory_idx);
    const auto             end       = end_point_for_trajectory(trajectory_idx);
    const auto             corridors = corridors_for_start(start);
    std::vector<NodeGuess> targets   = phase_targets_for_branch(branch, corridors, end);

    if (targets.size() != branch.phases.size())
    {
        std::ostringstream oss;
        oss << "branch " << branch.suffix << " has " << branch.phases.size() << " phases but "
            << targets.size() << " phase targets";
        throw std::runtime_error(oss.str());
    }

    std::vector<NodeGuess> nodes;
    NodeGuess              current{ start.x, start.y, start.yaw_deg, start.h };
    for (int phase_i = 0; phase_i < static_cast<int>(branch.phases.size()); ++phase_i)
    {
        const auto  target = targets[phase_i];
        const auto& phase  = branch.phases[phase_i];
        for (int local = 0; local < phase.nodes; ++local)
        {
            double a = static_cast<double>(local) / phase.nodes;
            nodes.push_back({ (1.0 - a) * current.x + a * target.x,
                              (1.0 - a) * current.y + a * target.y,
                              (1.0 - a) * current.yaw_deg + a * target.yaw_deg,
                              (1.0 - a) * current.h + a * target.h });
        }
        current = target;
    }
    nodes.push_back(targets.back());

    auto phase_names = node_phase_names(branch);
    for (int k = 0; k < static_cast<int>(nodes.size()); ++k)
    {
        const auto& corridor = base_corridor(phase_names[k]);
        if (corridor.requires_h_03)
            nodes[k].h = ZONE2_REQUIRED_H;
    }

    int boundary_node = 0;
    for (int phase_i = 0; phase_i + 1 < static_cast<int>(branch.phases.size()); ++phase_i)
    {
        boundary_node += branch.phases[phase_i].nodes;
        const auto& from = corridor_by_name(corridors, branch.phases[phase_i].corridor_name);
        const auto& to   = corridor_by_name(corridors, branch.phases[phase_i + 1].corridor_name);
        const auto  intersection = corridor_intersection(from, to);
        if ((from.requires_h_03 || to.requires_h_03) && 0 <= boundary_node &&
            boundary_node < static_cast<int>(nodes.size()))
        {
            nodes[boundary_node] = clamp_to_intersection(nodes[boundary_node], intersection);
        }
    }
    return nodes;
}

std::vector<double> estimate_phase_durations(const std::vector<NodeGuess>& nodes,
                                             const BranchSpec&             branch)
{
    std::vector<double> durations;
    int                 start = 0;
    for (const auto& phase : branch.phases)
    {
        int    end      = start + phase.nodes;
        double max_time = 0.4;
        for (int col = 0; col < 4; ++col)
        {
            double span = 0.0;
            for (int i = start; i < end; ++i)
            {
                const auto& a  = nodes[i];
                const auto& b  = nodes[i + 1];
                double      da = 0.0;
                if (col == 0)
                    da = b.x - a.x;
                else if (col == 1)
                    da = b.y - a.y;
                else if (col == 2)
                    da = b.yaw_deg - a.yaw_deg;
                else
                    da = b.h - a.h;
                span += std::abs(da);
            }
            const double a_max = (col == 2) ? A_MAX_YAW : ((col == 3) ? A_MAX_H : 3.0);
            if (span > 0.0)
                max_time = std::max(max_time, 2.0 * std::sqrt(span / a_max) * 2.2);
        }
        durations.push_back(std::min(max_time, 18.0));
        start = end;
    }
    return durations;
}

std::vector<double> node_times(const std::vector<double>& phase_durations, const BranchSpec& branch)
{
    std::vector<double> times{ 0.0 };
    double              elapsed = 0.0;
    for (int phase_i = 0; phase_i < static_cast<int>(branch.phases.size()); ++phase_i)
    {
        double dt = phase_durations[phase_i] / branch.phases[phase_i].nodes;
        for (int i = 0; i < branch.phases[phase_i].nodes; ++i)
        {
            elapsed += dt;
            times.push_back(elapsed);
        }
    }
    return times;
}

std::array<double, 4> interpolate_node_velocity(const std::vector<NodeGuess>& nodes,
                                                const std::vector<double>&    times,
                                                int                           k)
{
    const int n = static_cast<int>(nodes.size());
    if (k == 0 || k == n - 1)
        return { 0.0, 0.0, 0.0, 0.0 };

    double dt = times[k + 1] - times[k - 1];
    return { (nodes[k + 1].x - nodes[k - 1].x) / dt,
             (nodes[k + 1].y - nodes[k - 1].y) / dt,
             (nodes[k + 1].yaw_deg - nodes[k - 1].yaw_deg) / dt,
             (nodes[k + 1].h - nodes[k - 1].h) / dt };
}

MX wheel_speeds_ca(MX yaw_deg, MX vx, MX vy, MX wz_deg)
{
    MX theta = yaw_deg * kDegToRad;
    MX wz    = wz_deg * kDegToRad;
    MX phi   = theta + M_PI / 4.0;
    MX vw1   = std::sqrt(2.0) * cos(phi) * vx + std::sqrt(2.0) * sin(phi) * vy;
    MX vw2   = std::sqrt(2.0) * sin(phi) * vx - std::sqrt(2.0) * cos(phi) * vy;
    MX w1    = (vw1 + LXY_M * wz) / WHEEL_RADIUS_M;
    MX w2    = (vw2 - LXY_M * wz) / WHEEL_RADIUS_M;
    MX w3    = (vw1 - LXY_M * wz) / WHEEL_RADIUS_M;
    MX w4    = (vw2 + LXY_M * wz) / WHEEL_RADIUS_M;
    return MX::vertcat({ w1, w2, w3, w4 });
}

double clean(double value)
{
    return std::abs(value) < 1e-7 ? 0.0 : value;
}

TrajectoryResult sample_solution(const std::string&         branch_suffix,
                                 const std::vector<double>& t_nodes,
                                 const DM&                  sol_x,
                                 const DM&                  sol_v,
                                 const DM&                  sol_a,
                                 double                     sample_dt)
{
    TrajectoryResult result;
    result.branch     = branch_suffix;
    result.total_time = t_nodes.back();

    const int full_steps = static_cast<int>(std::floor(result.total_time / sample_dt + 1e-9));
    result.t.reserve(static_cast<size_t>(full_steps) + 2);
    result.trajectory.reserve(static_cast<size_t>(full_steps) + 2);

    auto append_sample = [&](double t)
    {
        int idx    = static_cast<int>(std::upper_bound(t_nodes.begin(), t_nodes.end(), t) -
                                      t_nodes.begin()) -
                     1;
        idx        = std::clamp(idx, 0, static_cast<int>(t_nodes.size()) - 2);
        double dt  = t_nodes[idx + 1] - t_nodes[idx];
        double tau = std::clamp(t - t_nodes[idx], 0.0, dt);

        std::array<double, 8> point{};
        for (int row = 0; row < 4; ++row)
        {
            double x0  = static_cast<double>(sol_x(row, idx));
            double v0  = static_cast<double>(sol_v(row, idx));
            double acc = static_cast<double>(sol_a(row, idx));
            point[row] = clean(x0 + v0 * tau + 0.5 * acc * tau * tau);
        }
        for (int row = 0; row < 4; ++row)
        {
            double v0      = static_cast<double>(sol_v(row, idx));
            double acc     = static_cast<double>(sol_a(row, idx));
            point[4 + row] = clean(v0 + acc * tau);
        }
        result.t.push_back(t);
        result.trajectory.push_back(point);
    };

    for (int i = 0; i <= full_steps; ++i)
    {
        double t = std::min(i * sample_dt, result.total_time);
        append_sample(t);
    }

    if (result.t.empty() || result.total_time - result.t.back() > 1e-9)
        append_sample(result.total_time);

    for (int row = 4; row < 8; ++row)
    {
        result.trajectory.front()[row] = 0.0;
        result.trajectory.back()[row]  = 0.0;
    }
    result.n_points = result.trajectory.size();
    result.success  = true;
    return result;
}

bool branch_supports_trajectory(const BranchSpec& branch, int trajectory_idx)
{
    if (std::string(branch.suffix) == "z2yaw0_finish")
        return trajectory_idx == 0;
    if (std::string(branch.suffix) == "z2yaw0_lower_finish")
        return trajectory_idx == 1;
    return false;
}

BranchSolveResult solve_branch(int trajectory_idx, const BranchSpec& branch, double sample_dt)
{
    certify_corridors_for_start(trajectory_idx, branch);

    const auto start         = start_point_for_trajectory(trajectory_idx);
    const auto end           = end_point_for_trajectory(trajectory_idx);
    const int  intervals     = n_intervals(branch);
    const int  nodes         = intervals + 1;
    const auto phase_names   = node_phase_names(branch);
    const auto phase_indices = interval_phase_indices(branch);
    const auto corridors     = corridors_for_start(start);

    Opti opti;
    MX   X                 = opti.variable(4, nodes);     // x, y, yaw_deg, h
    MX   V                 = opti.variable(4, nodes);     // dx, dy, dyaw_deg, dh
    MX   A                 = opti.variable(4, intervals); // acceleration per interval
    MX   DT_PHASE          = opti.variable(branch.phases.size());
    MX   h_down_accel_peak = opti.variable();
    MX   T                 = sum1(DT_PHASE);

    for (int phase_i = 0; phase_i < static_cast<int>(branch.phases.size()); ++phase_i)
    {
        opti.subject_to(opti.bounded(0.2, DT_PHASE(phase_i), 20.0));
    }
    opti.subject_to(opti.bounded(0.0, h_down_accel_peak, A_MAX_H));

    opti.subject_to(X(Slice(), 0) == DM({ start.x, start.y, start.yaw_deg, start.h }));
    opti.subject_to(V(Slice(), 0) == DM({ 0.0, 0.0, 0.0, 0.0 }));
    opti.subject_to(X(Slice(), nodes - 1) == DM({ end.x, end.y, end.yaw_deg, end.h }));
    opti.subject_to(V(Slice(), nodes - 1) == DM({ 0.0, 0.0, 0.0, 0.0 }));

    for (int k = 0; k < nodes; ++k)
    {
        const auto& corridor = corridor_by_name(corridors, phase_names[k]);
        opti.subject_to(opti.bounded(corridor.x_min, X(0, k), corridor.x_max));
        opti.subject_to(opti.bounded(corridor.y_min, X(1, k), corridor.y_max));
        opti.subject_to(opti.bounded(corridor.yaw_min_deg, X(2, k), corridor.yaw_max_deg));
        if (corridor.requires_h_03)
        {
            opti.subject_to(X(3, k) == ZONE2_REQUIRED_H);
            opti.subject_to(V(3, k) == 0.0);
        }
        else
            opti.subject_to(opti.bounded(H_MIN, X(3, k), H_MAX));
    }

    int boundary_node = 0;
    for (int phase_i = 0; phase_i + 1 < static_cast<int>(branch.phases.size()); ++phase_i)
    {
        boundary_node += branch.phases[phase_i].nodes;
        const auto& from = corridor_by_name(corridors, branch.phases[phase_i].corridor_name);
        const auto& to   = corridor_by_name(corridors, branch.phases[phase_i + 1].corridor_name);
        if (!from.requires_h_03 && !to.requires_h_03)
            continue;
        const auto  intersection = corridor_intersection(from, to);
        for (int node : { boundary_node })
        {
            opti.subject_to(opti.bounded(intersection.x_min, X(0, node), intersection.x_max));
            opti.subject_to(opti.bounded(intersection.y_min, X(1, node), intersection.y_max));
            opti.subject_to(
                    opti.bounded(intersection.yaw_min_deg, X(2, node), intersection.yaw_max_deg));
            if (std::abs(intersection.h_max - intersection.h_min) <= 1e-9)
            {
                opti.subject_to(X(3, node) == intersection.h_min);
                opti.subject_to(V(3, node) == 0.0);
            }
            else
                opti.subject_to(opti.bounded(intersection.h_min, X(3, node), intersection.h_max));
        }
    }

    std::vector<MX> wheels;
    wheels.reserve(nodes);
    for (int k = 0; k < nodes; ++k)
    {
        opti.subject_to(opti.bounded(-V_MAX_H, V(3, k), V_MAX_H));

        MX wheel = wheel_speeds_ca(X(2, k), V(0, k), V(1, k), V(2, k));
        wheels.push_back(wheel);
        opti.subject_to(opti.bounded(-MAX_WHEEL_SPEED, wheel, MAX_WHEEL_SPEED));
    }

    for (int k = 0; k < intervals; ++k)
    {
        int phase_idx = phase_indices[k];
        MX  dt        = DT_PHASE(phase_idx) / branch.phases[phase_idx].nodes;
        opti.subject_to(X(Slice(), k + 1) ==
                        X(Slice(), k) + V(Slice(), k) * dt + 0.5 * A(Slice(), k) * dt * dt);
        opti.subject_to(V(Slice(), k + 1) == V(Slice(), k) + A(Slice(), k) * dt);
        opti.subject_to(opti.bounded(-A_MAX_H, A(3, k), A_MAX_H));
        opti.subject_to(h_down_accel_peak >= -A(3, k));

        MX previous_wheel = wheels[k];
        for (int sub = 1; sub <= kWheelConstraintSubsteps; ++sub)
        {
            const double alpha  = static_cast<double>(sub) / kWheelConstraintSubsteps;
            MX           sub_dt = dt / kWheelConstraintSubsteps;
            MX           tau    = dt * alpha;
            MX           xs = X(Slice(), k) + V(Slice(), k) * tau + 0.5 * A(Slice(), k) * tau * tau;
            MX           vs = V(Slice(), k) + A(Slice(), k) * tau;
            MX           wheel = wheel_speeds_ca(xs(2), vs(0), vs(1), vs(2));
            opti.subject_to(opti.bounded(-MAX_WHEEL_SPEED, wheel, MAX_WHEEL_SPEED));
            opti.subject_to(opti.bounded(-kWheelAccelConstraintLimit * sub_dt,
                                         wheel - previous_wheel,
                                         kWheelAccelConstraintLimit * sub_dt));
            previous_wheel = wheel;
        }
    }

    MX late_yaw_cost = 0.0;
    if (trajectory_idx == 1)
    {
        for (int k = 0; k < intervals; ++k)
        {
            int phase_idx = phase_indices[k];
            MX  dt        = DT_PHASE(phase_idx) / branch.phases[phase_idx].nodes;
            MX  dy        = (X(1, k) - end.y) / kLateYawPenaltySigmaY;
            MX  near_end  = exp(-(dy * dy));
            late_yaw_cost += dt * near_end * V(2, k) * V(2, k);
        }
    }

    MX xyz_smoothness = sumsqr(A(Slice(0, 3), Slice()));
    MX cost           = T + H_DOWN_ACCEL_PEAK_WEIGHT * h_down_accel_peak +
              kLateYawPenaltyWeight * late_yaw_cost + 1e-9 * xyz_smoothness;
    opti.minimize(cost);

    const auto guess_nodes     = initial_state_nodes(trajectory_idx, branch);
    const auto phase_durations = estimate_phase_durations(guess_nodes, branch);
    const auto t_guess         = node_times(phase_durations, branch);

    for (int i = 0; i < static_cast<int>(phase_durations.size()); ++i)
        opti.set_initial(DT_PHASE(i), phase_durations[i]);

    for (int k = 0; k < nodes; ++k)
    {
        opti.set_initial(X(0, k), guess_nodes[k].x);
        opti.set_initial(X(1, k), guess_nodes[k].y);
        opti.set_initial(X(2, k), guess_nodes[k].yaw_deg);
        opti.set_initial(X(3, k), guess_nodes[k].h);

        auto vg = interpolate_node_velocity(guess_nodes, t_guess, k);
        for (int row = 0; row < 4; ++row)
            opti.set_initial(V(row, k), vg[row]);
    }

    for (int k = 0; k < intervals; ++k)
    {
        int    phase_idx = phase_indices[k];
        double dt        = phase_durations[phase_idx] / branch.phases[phase_idx].nodes;
        auto   v0        = interpolate_node_velocity(guess_nodes, t_guess, k);
        auto   v1        = interpolate_node_velocity(guess_nodes, t_guess, k + 1);
        for (int row = 0; row < 4; ++row)
            opti.set_initial(A(row, k), (v1[row] - v0[row]) / dt);
    }

    Dict opts;
    opts["expand"]     = true;
    opts["print_time"] = false;
    Dict ipopt;
    ipopt["print_level"]    = 0;
    ipopt["sb"]             = "yes";
    ipopt["max_iter"]       = 4000;
    ipopt["tol"]            = 1e-7;
    ipopt["acceptable_tol"] = 1e-6;
    opts["ipopt"]           = ipopt;
    opti.solver("ipopt", opts);

    try
    {
        OptiSol             sol = opti.solve();
        std::vector<double> durations;
        durations.reserve(branch.phases.size());
        for (int i = 0; i < static_cast<int>(branch.phases.size()); ++i)
            durations.push_back(static_cast<double>(sol.value(DT_PHASE(i))));
        auto t_nodes = node_times(durations, branch);
        auto result  = sample_solution(
                branch.suffix, t_nodes, sol.value(X), sol.value(V), sol.value(A), sample_dt);
        return { result, branch.suffix, "" };
    }
    catch (const std::exception& exc)
    {
        BranchSolveResult failed;
        failed.branch  = branch.suffix;
        failed.message = exc.what();
        return failed;
    }
}

} // namespace

TrajectoryResult optimize_trajectory(int trajectory_idx, double sample_dt)
{
    TrajectoryResult         best;
    std::vector<std::string> reports;

    for (const auto& branch : kBranches)
    {
        if (!branch_supports_trajectory(branch, trajectory_idx))
            continue;

        auto candidate = solve_branch(trajectory_idx, branch, sample_dt);
        if (candidate.trajectory.success)
        {
            std::ostringstream oss;
            oss << candidate.branch << ": OK T=" << std::fixed << std::setprecision(3)
                << candidate.trajectory.total_time << "s";
            reports.push_back(oss.str());
            if (!best.success || candidate.trajectory.total_time < best.total_time)
                best = std::move(candidate.trajectory);
        }
        else
        {
            std::string first_line = candidate.message;
            auto        newline    = first_line.find('\n');
            if (newline != std::string::npos)
                first_line = first_line.substr(0, newline);
            reports.push_back(candidate.branch + ": FAIL " + first_line);
        }
    }

    best.branch_reports = std::move(reports);
    return best;
}

} // namespace planning
