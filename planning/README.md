## Offline Trajectory Generation

Use the C++/CasADi optimizer through the Python verification pipeline:

```bash
uv run python main.py
```

`main.py` builds and runs `build/planning_cli`, which performs trajectory
optimization and exports headers/CSVs. Python is then used only for CSV loading,
verification, costmap diagnostics, and visualization. The project is pinned to
Python 3.12 in `.python-version`. Use the normal uv global cache; do not override
`UV_CACHE_DIR`.

The generated firmware artifacts are:

- `dist/trajectory_point.hpp`
- `dist/trajectory_traj_1.hpp`
- `dist/trajectory_traj_2.hpp`
- `dist/trajectory_traj_3.hpp`
- `dist/trajectory_all.hpp`

CSV copies are written beside the headers for inspection and plotting.  The
validation stage in `main.py` is mandatory: it checks boundary states, velocity
continuity, h-axis limits, the exact `master/path_mecanum4` wheel-speed transform
and wheel speed/acceleration limits, Zone2 yaw/h requirements, analytic
safe-corridor certificates, swept inflated-footprint collision samples between
exported 100 Hz points, and the proportion of trajectory time spent near wheel
speed/acceleration limits.  Limit-saturation is implemented only in the verifier
as an evaluation gate; it is not added to the optimizer objective.

The generator solves Zone2 yaw branches explicitly (`0deg` and `-90deg`) and
exports the shortest feasible branch for each start.

Current `path_mecanum4` wheel limits used by both the C++ optimizer and Python
verifier are `MAX_WHEEL_SPEED = 160 rad/s` and
`MAX_WHEEL_ACCEL = 60 rad/s^2`.

## Changing Start / End Poses

Boundary poses use the format `[x, y, yaw_deg, h]`:

- `x`, `y`: world-frame position in meters.
- `yaw_deg`: chassis yaw in degrees.
- `h`: lift height in meters.

To change the three start poses, edit `ENTRY_POINTS` in both files:

- `config.py`
- `config.hpp`

To change the common terminal pose, edit `EXIT_POINT` in both files. Keep the
numeric values synchronized exactly: the C++ optimizer reads `config.hpp`, while
Python verification and plotting read `config.py`.

If the new start or end pose changes the intended route shape, also update the
branch definitions in both files:

- `branch_specs.py`
- `src/trajectory_optimizer.cpp`

Those branch definitions describe the corridor sequence, node counts, and branch
anchors used by the two Zone2 yaw candidates. The current `z2yaw-90` anchors are
soft initial-guess points, so the optimizer may move away from them. The current
`z2yaw0` anchors are hard constraints, so changing them changes required
intermediate states.

After editing boundary poses or branch anchors, regenerate and verify:

```bash
uv run python main.py
```

Use the generated headers only after the verifier reports `0 failures`.

## Build / Generate Commands

The normal one-command workflow is:

```bash
uv run python main.py
```

This command performs the full offline pipeline:

1. Generate `costmap_debug.png`.
2. Configure and build `build/planning_cli` with CMake.
3. Run the C++ CasADi optimizer.
4. Export firmware headers and CSV inspection files under `dist/`.
5. Verify boundary states, corridors, collision checks, height limits, wheel
   speed/acceleration limits, and limit-saturation time.
6. Generate `trajectory_plot_v3.png` and per-trajectory GIFs under `figures/`.

The equivalent manual C++ build and generation flow is:

```bash
uv run cmake -S . -B build
uv run cmake --build build --target planning_cli
./build/planning_cli
```

Running `build/planning_cli` only performs optimization and export. It does not
run the Python verifier or regenerate plots/GIFs, so use the full
`uv run python main.py` pipeline before consuming the generated trajectory code.

## Generated Code Layout

The C++ exporter writes these firmware-facing headers:

- `dist/trajectory_point.hpp`: defines `TrajectoryPoint8`.
- `dist/trajectory_traj_1.hpp`: defines `kTrajectory_traj_1`.
- `dist/trajectory_traj_2.hpp`: defines `kTrajectory_traj_2`.
- `dist/trajectory_traj_3.hpp`: defines `kTrajectory_traj_3`.
- `dist/trajectory_all.hpp`: includes all generated trajectory headers.

Each `TrajectoryPoint8` contains one sampled trajectory state:

- `x`, `y`, `yaw`, `h`
- `dx`, `dy`, `dyaw`, `dh`

The default export is sampled at about 100 Hz (`sample_dt = 0.01`). CSV files
with the same trajectory data are written as `dist/traj_1.csv`,
`dist/traj_2.csv`, and `dist/traj_3.csv` for inspection, verification, and
plotting. Do not hand-edit files under `dist/`; change the config or optimizer
inputs and regenerate them.

Generated diagnostic images are:

- `costmap_debug.png`
- `trajectory_plot_v3.png`
- `figures/trajectory_traj_1.gif`
- `figures/trajectory_traj_2.gif`
- `figures/trajectory_traj_3.gif`

Each GIF shows the moving chassis rectangle, lift-height curve, body-axis
velocity/acceleration, and four-wheel velocity/acceleration over time.
