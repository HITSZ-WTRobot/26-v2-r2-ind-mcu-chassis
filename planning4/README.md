## Curve 4 Offline Trajectory Generation

This directory is a standalone copy of the offline planner used by
`../planning`, adjusted for the curve 4 target in `CLAUDE.md`.

Run the full generation and verification pipeline with:

```bash
uv run python main.py
```

The command uses the normal uv global cache. It configures and builds the C++
CasADi optimizer, exports the 500 Hz trajectory, verifies the exported samples,
and generates plots/GIF diagnostics.

The generated firmware artifacts are:

- `dist/trajectory_point.hpp`
- `dist/trajectory_traj_4.hpp`
- `dist/trajectory_all.hpp`

CSV inspection output is written as `dist/traj_4.csv`. Diagnostic images are:

- `costmap_debug.png`
- `trajectory_plot_v3.png`
- `figures/trajectory_traj_4.gif`

## Curve 4 Target

Boundary poses use `[x, y, yaw_deg, h]`.

- Start: `[6.40, 5.40, 0deg, 0.215]`
- End: `[11.05, 3.227, 0deg, 0.215]`

The planner models the four legal zones from `CLAUDE.md`:

- Zone1: `x=[8.00, 9.44], y=[1.23, 6.00]` except Zone2 overlap.
- Zone2: `x=[9.30, 10.80], y=[4.50, 6.00]`.
- Zone3: `x=[9.50, 12.00], y=[0.20, 6.00]` except Zone2 overlap.
- Zone4: `x=[5.60, 8.00], y=[4.80, 6.00]`.

Obstacles are all space outside those zones plus the four explicit rectangular
obstacles:

- `x=[9.30, 10.80], y=[4.47, 4.63]`
- `x=[11.65, 12.00], y=[3.05, 3.40]`
- `x=[11.65, 12.00], y=[2.35, 2.70]`
- `x=[11.65, 12.00], y=[1.65, 2.00]`

The inflated footprint uses the 0.760 m by 0.520 m chassis rectangle plus the
5 cm collision expansion. If the inflated footprint intersects Zone2, the
verifier requires `yaw=0deg` and `h=0.3`.

## Pipeline

The normal workflow is:

```bash
uv run python main.py
```

Manual C++ generation without Python verification is:

```bash
cmake -S . -B build
cmake --build build --target planning_cli
./build/planning_cli
```

Use the full Python pipeline before consuming the generated firmware headers.
The verifier checks:

- Start/end pose and zero start/end velocities.
- 500 Hz position/velocity integration consistency.
- h-axis bounds and velocity/acceleration limits.
- The exact `master/path_mecanum4` wheel-speed transform.
- Wheel speed limit `160 rad/s` and wheel acceleration limit `60 rad/s^2`.
- Zone2 `yaw=0deg` and `h=0.3`.
- Swept inflated-footprint collision samples between exported trajectory points.
- At least 75% of trajectory time near wheel speed or wheel acceleration limits.

## Editing

Keep `config.py` and `config.hpp` synchronized. Python verification and plotting
read `config.py`; the C++ optimizer reads `config.hpp`.

If route shape changes, update the matching branch/corridor definitions in:

- `branch_specs.py`
- `geometry.py`
- `src/trajectory_optimizer.cpp`

Do not hand-edit files under `dist/`; regenerate them from the planner.
