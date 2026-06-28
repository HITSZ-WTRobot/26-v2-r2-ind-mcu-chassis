# Planning Subproject Guidelines

## Authoritative Requirements

`CLAUDE.md` is the requirements source for this offline trajectory planner. Do not
change requirements while implementing or tuning the planner.

## Environment

Use `uv` with the normal global uv cache. Do not override `UV_CACHE_DIR`.
The project is pinned to Python 3.12 through `.python-version` because CasADi is
validated there for this workspace.

## Generator Workflow

Run the authoritative pipeline from this directory:

```bash
uv run python main.py
```

The Python/CasADi implementation is the current generator:

- `src/trajectory_optimizer.cpp` owns the C++ CasADi OCP for x/y/yaw/h,
  velocity, acceleration, branch timing, and export selection.  Its chassis
  constraints must match
  `/home/syhanjin/workspace/robocon2026/master/path_mecanum4`: world-frame
  state integration plus the same wheel-speed transform, wheel speed limit, and
  successive wheel-speed acceleration limit. Current planner constants are
  `MAX_WHEEL_SPEED = 160 rad/s` and `MAX_WHEEL_ACCEL = 60 rad/s^2`; keep
  `config.hpp` and `config.py` synchronized when these change.
- Python must not be used for trajectory optimization. `main.py` builds and runs
  `build/planning_cli`, then Python loads the exported CSVs for validation and
  plotting only. Historical Python optimizer entry points have been removed; keep
  Python as inspection and plotting support only.
- `geometry.py` owns the inflated-footprint geometry, certified safe corridors,
  and strict collision predicates.
- `verify.py` is mandatory; it checks boundary states, velocity continuity,
  h-axis limits, path_mecanum4 wheel speed/acceleration limits, Zone2 yaw/h
  rules, safe-corridor certificates, limit-saturation duration metrics, and
  swept inflated-footprint collision samples between exported trajectory points.
- `src/exporter.cpp` writes both firmware headers and CSV inspection copies under
  `dist/`; Python should only inspect those exported artifacts.

Zone2 yaw is solved as explicit `0deg` and `-90deg` branches.  Export the shortest
feasible branch per start.  Do not add an explicit “hug the limit” reward to the
optimizer; keep that as a verifier/evaluation gate based on sustained time near
wheel speed/acceleration limits.

## Output Artifacts

The current firmware artifacts are:

- `dist/trajectory_point.hpp`
- `dist/trajectory_traj_1.hpp`
- `dist/trajectory_traj_2.hpp`
- `dist/trajectory_traj_3.hpp`
- `dist/trajectory_all.hpp`

The generated trajectory HPPs are resampled at 500 Hz and exported in per-
trajectory namespaces with `kSampleHz`, `kDt`, `kPointCount`, `kDuration`, and
`kPoints`.

Keep CSVs in `dist/` synchronized with the headers when regenerating trajectories.
