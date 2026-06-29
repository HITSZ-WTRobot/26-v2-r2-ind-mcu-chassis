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

constexpr Rect kFreeCertLeft{ "left_zone1_clear", 8.00, 9.30, 1.23, 6.00 };
constexpr Rect kFreeCertTop{ "top_above_strip_clear", 8.00, 12.00, 4.65, 6.00 };
constexpr Rect kFreeCertRight{ "right_zone3_clear", 10.80, 12.00, 0.20, 6.00 };
constexpr Rect kFreeCertLowRight{ "low_zone3_clear", 9.50, 12.00, 0.20, 4.47 };

const std::vector<Corridor> kBaseCorridors{
    { "approach", 8.43, 8.58, 1.80, 5.28, 0.0, 0.0, kFreeCertLeft, false },
    { "top0", 8.53, 11.24, 4.98, 4.99, 0.0, 0.0, kFreeCertTop, true },
    { "right_down0", 11.22, 11.24, 3.94, 4.99, 0.0, 0.0, kFreeCertRight, false },
    { "lower_down", 10.05, 11.35, 2.78, 3.94, -90.0, 0.0, kFreeCertLowRight, false },
    { "lower_finish", 10.05, 11.35, 2.45, 2.80, -90.0, 0.0, kFreeCertLowRight, false },
};

const std::vector<BranchSpec> kBranches{
    { "z2yaw0",
      0.0,
      { { "approach", 28 },
        { "top0", 36 },
        { "right_down0", 16 },
        { "lower_down", 32 },
        { "lower_finish", 32 } },
      {
              { "top_start", 8.55, 4.985, 0.0, 0.3 },
              { "top_end", 11.23, 4.985, 0.0, 0.3 },
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

std::vector<Corridor> corridors_for_start(double start_y)
{
    std::vector<Corridor> corridors = kBaseCorridors;
    for (auto& corridor : corridors)
    {
        if (std::string(corridor.name) == "approach")
            corridor.y_min = start_y;
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

void certify_corridors_for_start(int start_idx, const BranchSpec& branch)
{
    auto corridors = corridors_for_start(ENTRY_POINTS[start_idx].y);
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
            oss << trajectory_name(start_idx) << " " << branch.suffix << " corridor "
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
        return { H_MIN, H_MIN };
    return { H_MIN, H_MAX };
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
                                                const std::vector<Corridor>& corridors)
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
    const NodeGuess final{ EXIT_POINT.x, EXIT_POINT.y, EXIT_POINT.yaw_deg, EXIT_POINT.h };
    targets.push_back(final);
    return targets;
}

std::vector<NodeGuess> initial_state_nodes(int start_idx, const BranchSpec& branch)
{
    const auto             corridors = corridors_for_start(ENTRY_POINTS[start_idx].y);
    std::vector<NodeGuess> targets   = phase_targets_for_branch(branch, corridors);

    if (targets.size() != branch.phases.size())
    {
        std::ostringstream oss;
        oss << "branch " << branch.suffix << " has " << branch.phases.size() << " phases but "
            << targets.size() << " phase targets";
        throw std::runtime_error(oss.str());
    }

    std::vector<NodeGuess> nodes;
    NodeGuess              current{ ENTRY_POINTS[start_idx].x,
                       ENTRY_POINTS[start_idx].y,
                       ENTRY_POINTS[start_idx].yaw_deg,
                       ENTRY_POINTS[start_idx].h };
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
            nodes[k].h = H_MIN;
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
                                 double                     sample_dt)
{
    TrajectoryResult result;
    result.branch     = branch_suffix;
    result.total_time = t_nodes.back();
    int n_samples     = static_cast<int>(std::ceil(result.total_time / sample_dt)) + 1;
    result.t.resize(n_samples);
    result.trajectory.resize(n_samples);

    for (int i = 0; i < n_samples; ++i)
    {
        double t   = (n_samples == 1) ? 0.0 : result.total_time * i / (n_samples - 1);
        int    idx = static_cast<int>(std::upper_bound(t_nodes.begin(), t_nodes.end(), t) -
                                   t_nodes.begin()) -
                  1;
        idx          = std::clamp(idx, 0, static_cast<int>(t_nodes.size()) - 2);
        double dt    = t_nodes[idx + 1] - t_nodes[idx];
        double alpha = dt > 0.0 ? (t - t_nodes[idx]) / dt : 0.0;

        std::array<double, 8> point{};
        for (int row = 0; row < 4; ++row)
        {
            double a   = static_cast<double>(sol_x(row, idx));
            double b   = static_cast<double>(sol_x(row, idx + 1));
            point[row] = clean((1.0 - alpha) * a + alpha * b);
        }
        for (int row = 0; row < 4; ++row)
        {
            double a       = static_cast<double>(sol_v(row, idx));
            double b       = static_cast<double>(sol_v(row, idx + 1));
            point[4 + row] = clean((1.0 - alpha) * a + alpha * b);
        }
        result.t[i]          = t;
        result.trajectory[i] = point;
    }

    for (int row = 4; row < 8; ++row)
    {
        result.trajectory.front()[row] = 0.0;
        result.trajectory.back()[row]  = 0.0;
    }
    result.n_points = result.trajectory.size();
    result.success  = true;
    return result;
}

BranchSolveResult solve_branch(int start_idx, const BranchSpec& branch, double sample_dt)
{
    certify_corridors_for_start(start_idx, branch);

    const auto start         = ENTRY_POINTS[start_idx];
    const auto end           = EXIT_POINT;
    const int  intervals     = n_intervals(branch);
    const int  nodes         = intervals + 1;
    const auto phase_names   = node_phase_names(branch);
    const auto phase_indices = interval_phase_indices(branch);
    const auto corridors     = corridors_for_start(start.y);

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
            opti.subject_to(X(3, k) == H_MIN);
            opti.subject_to(V(3, k) == 0.0);
        }
        else
            opti.subject_to(opti.bounded(H_MIN, X(3, k), H_MAX));
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
        opti.subject_to(X(Slice(0, 3), k + 1) == X(Slice(0, 3), k) +
                                                        V(Slice(0, 3), k) * dt);
        opti.subject_to(X(3, k + 1) == X(3, k) + V(3, k) * dt +
                                                  0.5 * A(3, k) * dt * dt);
        opti.subject_to(V(Slice(), k + 1) == V(Slice(), k) + A(Slice(), k) * dt);
        opti.subject_to(opti.bounded(-A_MAX_H, A(3, k), A_MAX_H));
        opti.subject_to(h_down_accel_peak >= -A(3, k));

        MX previous_wheel = wheels[k];
        for (int sub = 1; sub <= kWheelConstraintSubsteps; ++sub)
        {
            const double alpha  = static_cast<double>(sub) / kWheelConstraintSubsteps;
            MX           sub_dt = dt / kWheelConstraintSubsteps;
            MX           xs     = X(Slice(), k) + V(Slice(), k) * (dt * alpha);
            MX           vs     = V(Slice(), k) + A(Slice(), k) * (dt * alpha);
            MX           wheel  = wheel_speeds_ca(xs(2), vs(0), vs(1), vs(2));
            opti.subject_to(opti.bounded(-MAX_WHEEL_SPEED, wheel, MAX_WHEEL_SPEED));
            opti.subject_to(opti.bounded(-MAX_WHEEL_ACCEL * sub_dt,
                                         wheel - previous_wheel,
                                         MAX_WHEEL_ACCEL * sub_dt));
            previous_wheel = wheel;
        }
    }

    MX xyz_smoothness = sumsqr(A(Slice(0, 3), Slice()));
    MX cost = T + H_DOWN_ACCEL_PEAK_WEIGHT * h_down_accel_peak + 1e-9 * xyz_smoothness;
    opti.minimize(cost);

    const auto guess_nodes     = initial_state_nodes(start_idx, branch);
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
        auto result =
                sample_solution(branch.suffix, t_nodes, sol.value(X), sol.value(V), sample_dt);
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

TrajectoryResult optimize_trajectory(int start_idx, double sample_dt)
{
    TrajectoryResult         best;
    std::vector<std::string> reports;

    for (const auto& branch : kBranches)
    {
        auto candidate = solve_branch(start_idx, branch, sample_dt);
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
