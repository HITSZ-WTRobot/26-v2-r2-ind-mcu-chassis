# Repository Guidelines

## Project Structure & Module Organization
Firmware integration lives at the repository root. `UserCode/` is the only project-specific application layer; the current project is split into:
- `UserCode/app.cpp` — RTOS init flow and timer-driven update scheduling.
- `UserCode/device.*` — physical device creation and bus registration.
- `UserCode/connection.*` — unified connection bitmap refresh and startup connection wait logic.
- `UserCode/project_parts.hpp` — the single source of truth for compile-time feature toggles and derived capabilities.
- `UserCode/chassis/` — the combined wheel-chassis + lift motion object, localization/controller setup, and step action state machine under `UserCode/chassis/actions/`.
- `UserCode/grip/` — the grip mechanism entry point, trajectory control, and calibration config.
- `UserCode/protocol/` — upper-host UART frame parsing and command dispatch.
- `UserCode/system.hpp` and `UserCode/sync/` — initialization handoff and clock alignment helpers.
- `UserCode/arena.cpp` — the global static-allocation arena backing `new` / `delete`.

`UserCode/grip.hpp` and `UserCode/protocol.hpp` are only umbrella includes; real implementation lives in their subdirectories. `Modules/` holds reusable libraries such as `BasicComponents`, `ChassisController`, `MotorDrivers`, `Sensors`, `TrajectoryControl`, and `VelocityProfile`; each module owns its own `CMakeLists.txt`, and module-local `AGENTS.md` files override this guide when present. `Core/`, `Drivers/`, and `Middlewares/` are STM32CubeMX-generated support code. Generated package indexes live in `index.md` and `cpkg_index.json`.

## STM32 + CubeMX Project Conventions
All STM32 + CubeMX projects in this workspace follow these conventions. Application code lives only under `UserCode/`. Referenced driver libraries live under `Modules/*`; each `Modules/${Name}` directory corresponds to the upstream GitHub repository name pulled in as a submodule. Large driver repositories may contain smaller drivers, identified by the presence of a `cpkg.toml` file in that subdirectory. Main projects consume driver libraries by first adding the repository with `add_subdirectory`, which makes all drivers in that repository available, then linking only the required driver targets by driver name; driver names match the names declared in each `cpkg.toml`. The generated driver index is `/home/syhanjin/workspace/robocon2026/references/Packages/cpkg_index.json`, produced from the available `cpkg.toml` files. Do not inspect `Core/`, `Drivers/`, or `Middlewares/` unless explicitly requested; they are unmodified STM32CubeMX-generated default code.

This repository also has several project-specific conventions derived from `UserCode/`:
- Treat `UserCode/project_parts.hpp` as the only place to enable or disable major subsystems. Prefer using derived `ProjectParts::EnableXxx` / `NeedXxx` constants in code; do not re-combine the raw `PROJECT_PART_ENABLE_*` macros elsewhere.
- `Chassis::motion` is intentionally a single `IndLiftMecanum4` object that owns both the mecanum wheelset and the dual lift sides. As long as either wheel chassis or lift is enabled, keep that unified motion-object assumption intact.
- `Chassis::loc` and `Chassis::ctrl` exist only when wheel chassis support is enabled. Localization mode is selected at compile time: no gyro uses `JustEncoder`, gyro uses `LocEKF`, and upper-host localization delays EKF creation until the first posture packet arrives.
- `Grip::grip`, `Protocol::pc_rx`, `Chassis::motion`, `Chassis::loc`, and `Chassis::ctrl` are namespace-level singleton-style pointers. Follow the existing ownership model instead of introducing additional dynamic-lifetime managers.
- `UserCode/arena.cpp` overrides global `new` / `delete` with a one-way static arena. Avoid designs that assume `delete` frees memory or that repeatedly allocate and release heap objects at runtime.

## Runtime Flow
The real-time update flow is timer-driven and split by frequency:
- `TIM_Callback_1kHz_1()` in `UserCode/app.cpp` updates connection status, chassis control, device bus output, optional grip control, and feeds the watchdog.
- Grip uses a local 500 Hz prescaler inside the 1 kHz callback for error updates.
- `TIM_Callback_100Hz()` runs the slower profile updates for chassis and grip.

The startup sequence in `Init()` is also part of the project contract:
- Initialize in order: `Device::init()`, `Chassis::init()`, `Protocol::init()`, then optional `Grip::init()`.
- Wait for all enabled devices/protocol links to report connected before proceeding.
- Enable and calibrate motion and grip separately, then wait until all enabled subsystems are ready.
- Call `Chassis::initStandaloneLocCtrl()` only for local-initialization modes; when upper-host localization is enabled, the first posture frame triggers `System::Init::initPostureReceive()` instead.
- Only enable the chassis controller after initialization is complete, and only enable grip after it is calibrated.

When adding a new subsystem, update its init, periodic update hooks, readiness gate, and enable/calibration path together so the startup contract remains coherent.

## Device & Protocol Mapping
The current hardware/software mapping in `UserCode/` is:
- `UART2` (`huart2`) is the yaw gyro (`HWT101CT`); `UART3` (`huart3`) is the upper-host link.
- Wheel motors are DJI motors on CAN IDs 1–4, split across `hcan1` and `hcan2`.
- Lift motors are DM J4310 motors: front on `hcan1` with `id0 = 0xB`, rear on `hcan2` with `id0 = 0xA`.
- Grip uses two DJI motors on `hcan2`: arm `id1 = 1`, turn `id1 = 2`.

`Connection::table` and `Connection::Bit` in `UserCode/connection.hpp` define the canonical connection bitmap. The upper-host link uses bit15 directly. If you add, remove, or repurpose a device/protocol link, update the enum, required mask, table refresh logic, and connection wait path together.

Upper-host protocol behavior is likewise feature-gated:
- Create `Protocol::PCProtocol` only when `ProjectParts::EnableUpperHostProtocol` is true.
- Treat the first `LidarPosture` frame as the delayed initialization trigger when upper-host localization is enabled.
- Keep step-action commands gated by `ProjectParts::EnableStepAction`, which currently means PC control + wheel chassis + lift all enabled.

## Build, Test, and Development Commands
- `cmake --preset Debug` — configure a Ninja debug build with the `arm-none-eabi` toolchain.
- `cmake --build --preset Debug` — build the STM32F407 firmware ELF.
- `cmake --preset Release && cmake --build --preset Release` — build the optimized release variant.
- `stm32tool generate` — regenerate STM32CubeMX-generated code from the project `.ioc` configuration.
- `./update_modules.sh` — switch each module repo to `main` and pull updates; use only when intentionally syncing vendored modules.

## Coding Style & Naming Conventions
Follow the root `.clang-format`: 4-space indentation, Allman braces, no tabs, and a 100-column limit. Keep C at C11 and C++ at C++17. Match local naming patterns instead of forcing one style everywhere: types use `PascalCase`, functions and variables use `snake_case`, low-level module files are usually lowercase (`motor_vel_controller.cpp`), and higher-level chassis files in `UserCode/chassis/` may use `PascalCase` (`IndLiftMecanum4.cpp`).

Preserve the local control-flow style when editing `UserCode/`:
- Prefer `if constexpr` feature gating around subsystem-specific logic instead of runtime branching when the decision is compile-time.
- Keep namespace-scoped inline globals and pointers consistent with existing code instead of hiding them behind new wrappers unless a refactor is explicitly requested.
- Treat the state-machine enum names in `UserCode/chassis/actions/Step.*` as semantic documentation; do not rename or anglicize them casually.
- Keep configuration constants in `UserCode/chassis/Config.hpp`, `UserCode/grip/Config.hpp`, and `UserCode/project_parts.hpp` instead of scattering magic numbers into behavior code.

## Testing Guidelines
There is no repository-wide `ctest` suite yet. At minimum, rebuild the `Debug` preset after every change. For motion-profile work, also run `uv run python plot_display.py` or `uv run python plot_chassis_display.py` in `Modules/VelocityProfile/SCurve/simulation`. Place new automated tests beside the affected module and wire them into CMake.

When changes touch `UserCode/`, prefer validating the smallest affected behavior first:
- For feature-toggle or startup-path changes, rebuild and re-check that the init sequence still matches `ProjectParts` gating.
- For localization or upper-host protocol changes, verify both the local-init path and the delayed-first-posture path still make sense.
- For lift, grip, or step-action logic changes, inspect 1 kHz / 500 Hz / 100 Hz update placement before changing control code.

## Commit & Pull Request Guidelines
Recent history uses short type prefixes such as `feat:`, `deps:`, `refactor:`, and `ioc:`. Keep commit subjects imperative and specific, for example `feat: adjust lift trajectory parameters`. Pull requests should list touched directories, describe hardware or control-loop impact, and note validation performed. Include plots, logs, or regenerated artifacts when behavior or CubeMX configuration changes.

## Maintaining Agent Instructions
When project formats, workflows, commands, or requirements change, update this `AGENTS.md` in the same change so future agents follow the current project conventions.

## Generated Code & Configuration
Prefer editing `26-v2-r2-ind-mcu-chassis.ioc` and regenerating CubeMX output instead of hand-editing generated HAL files under `Core/`, `Drivers/`, or `Middlewares/`. Use `stm32tool generate` to regenerate CubeMX-managed files after `.ioc` changes. Do not commit `build/` outputs or transient simulation artifacts.
