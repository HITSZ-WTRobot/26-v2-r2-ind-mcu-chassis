# Repository Guidelines

## Project Structure & Module Organization
Firmware integration lives at the repository root. `UserCode/` is the only project-specific application layer; the current project is split into:
- `UserCode/app.cpp` — RTOS init flow and timer-driven update scheduling.
- `UserCode/device.*` — physical device creation and bus registration.
- `UserCode/i2c.*` — shared `I2CBusDMA` / `I2CUpdateManager` ownership and app-level periodic I2C device registration.
- `UserCode/suction/` — reusable suction-cup component and module-local config: `Config.hpp` centralizes shared pressure-sensor parameters plus the shared object-detection Schmitt thresholds; `SuctionCup.*` implements the reusable cup behavior.
- `UserCode/connection.*` — unified connection bitmap refresh and startup connection wait logic.
- `UserCode/project_parts.hpp` — the single source of truth for compile-time feature toggles and derived capabilities.
- `UserCode/chassis/` — the combined wheel-chassis + lift motion object, localization/controller setup, and step action state machine under `UserCode/chassis/actions/`.
- `UserCode/grip/` — the grip mechanism entry point, trajectory control, calibration config, and the two grip action modules under `UserCode/grip/actions/`: `SpearGrab` for taking spearheads and `KfsStore` for temporary roller storage.
- `UserCode/protocol/` — upper-host UART frame parsing and command dispatch.
- `UserCode/system.hpp` and `UserCode/sync/` — initialization handoff and clock alignment helpers.
- `UserCode/arena.cpp` — the global static-allocation arena backing `new` / `delete`.

`UserCode/grip.hpp` and `UserCode/protocol.hpp` are only umbrella includes; real implementation lives in their subdirectories. `Modules/` holds reusable libraries such as `BasicComponents`, `ChassisController`, `MotorDrivers`, `Sensors`, `TrajectoryControl`, and `VelocityProfile`; each module owns its own `CMakeLists.txt`, and module-local `AGENTS.md` files override this guide when present. `Core/`, `Drivers/`, and `Middlewares/` are STM32CubeMX-generated support code. Generated package indexes live in `index.md` and `cpkg_index.json`.

## STM32 + CubeMX Project Conventions
All STM32 + CubeMX projects in this workspace follow these conventions. Application code lives only under `UserCode/`. Referenced driver libraries live under `Modules/*`; each `Modules/${Name}` directory corresponds to the upstream GitHub repository name pulled in as a submodule. Large driver repositories may contain smaller drivers, identified by the presence of a `cpkg.toml` file in that subdirectory. Main projects consume driver libraries by first adding the repository with `add_subdirectory`, which makes all drivers in that repository available, then linking only the required driver targets by driver name; driver names match the names declared in each `cpkg.toml`. The generated driver index is `/home/syhanjin/workspace/robocon2026/references/Packages/cpkg_index.json`, produced from the available `cpkg.toml` files. Do not inspect `Core/`, `Drivers/`, or `Middlewares/` unless explicitly requested; they are unmodified STM32CubeMX-generated default code.

This repository also has several project-specific conventions derived from `UserCode/`:
- Treat `UserCode/project_parts.hpp` as the only place to enable or disable major subsystems. Prefer using derived `ProjectParts::EnableXxx` / `NeedXxx` constants in code; do not re-combine the raw `PROJECT_PART_ENABLE_*` macros elsewhere.
- `Chassis::motion` is intentionally a single `IndLiftMecanum4` object that owns both the mecanum wheelset and the dual lift sides. As long as either wheel chassis or lift is enabled, keep that unified motion-object assumption intact.
- Each `Lift::LiftSide` is now a synchronized dual-motor side built from `trajectory::HomingMotorTrajectory<2>` and two `MotorVelController`s. Keep the “front pair + rear pair” grouping intact unless the hardware contract changes again.
- `Chassis::loc` and `Chassis::ctrl` exist only when wheel chassis support is enabled. Localization mode is selected at compile time: no gyro uses `JustEncoder`, gyro uses `LocEKF`, and upper-host localization delays EKF creation until the first posture packet arrives.
- Suction support is split into two compile-time layers: `ProjectParts::EnableGripSuction` controls whether the grip-side pump hardware exists at all, while `ProjectParts::EnableGripSuctionPressureSensor` controls whether the optional pressure-sensor-backed object-detection capability exists. Without that sensor, the suction cup may still be turned on or off, and `hasObject()` should simply return false.
- `Grip::grip`, `Protocol::pc_rx`, `Chassis::motion`, `Chassis::loc`, and `Chassis::ctrl` are namespace-level singleton-style pointers. The grip suction used in this project is owned by the single `Grip::Action::KfsStore` instance rather than by `Grip` itself. High-level grip actions are exposed through `Grip::Action::SpearGrab::inst()` and `Grip::Action::KfsStore::inst()`; follow that ownership model instead of introducing additional dynamic-lifetime managers.
- `Grip` is responsible for two high-level action groups: `SpearGrab` for spearhead pickup and `KfsStore` for temporary roller storage. Keep them as separate modules under `UserCode/grip/actions/`; do not fold roller temporary-storage logic back into the spear-grab module.
- `Suction::SuctionCup` is a reusable component, not a grip-only type. It stores an externally owned pressure-sensor pointer that may be null; sensor construction and `AppI2C::manager1().registerDevice(...)` must happen before that pointer is passed into the cup. Its public runtime interface should stay limited to suction on, suction off, and “has object”; pressure-sensor access remains an internal implementation detail. When a fresh pressure sample exists, `hasObject()` should apply the configured Schmitt trigger directly on the current pressure so object slip can be observed as soon as the pressure crosses the release threshold; without a sensor or without a fresh sample, it should return false instead of falling back to timing heuristics. Keep only generic suction behavior and shared suction defaults inside `UserCode/suction/`; owner-specific wiring such as pump GPIO, I2C registration phase, and stale-time policy should live with the owning subsystem config.
- `Grip::Action::KfsStore` owns the only suction cup used by this project and a fixed no-parameter store/release flow. When the suction pressure sensor exists, its state machine should still use `Suction::SuctionCup::hasObject()` as the source of truth for “picked” and “released”; when the cup is built without that sensor, the action layer is allowed to fall back to owner-configured “reach pickup pose then delay” / “reach release pose, stop pump, then delay” confirmation to advance the workflow. Keep those fallback delays, together with the suction-pickup / temporary-store / release / standby poses and the owner-side suction config assembly, centralized in `Grip::Config::KfsStore`, and update the config together with the action logic when that hardware sequence changes.
- `Protocol::ActionState::table` is a packed 16-bit feedback field. It currently encodes Step status, chassis mode + trajectory-finished flag, lift status, grip/action status, and a `GripSuctionHasObject` bit. The current chassis-mode subfield uses `Stop / Velocity / Position / Slave`, the current grip subfield uses `Calibrating / TakingSpear / KfsStore / KfsRelease / Idle / Done`, and the suction bit is meaningful only when `ProjectParts::EnableGripSuctionPressureSensor` is enabled; if you change any subfield semantics, update `UserCode/protocol/ActionState.hpp`, the feedback protocol comment, and the upper-host agreement together.
- `UserCode/arena.cpp` overrides global `new` / `delete` with a one-way static arena. Avoid designs that assume `delete` frees memory or that repeatedly allocate and release heap objects at runtime.

## Runtime Flow
The real-time update flow is timer-driven and split by frequency:
- `TIM_Callback_1kHz_1()` in `UserCode/app.cpp` handles the first half-cycle: chassis 1 kHz control plus optional grip 1 kHz / 500 Hz control updates.
- `TIM_Callback_1kHz_2()` handles the second half-cycle, offset by half a timer period: the concentrated DJI CAN current sends for all active groups, followed by connection refresh and watchdog feeding.
- Grip uses a local 500 Hz prescaler inside `TIM_Callback_1kHz_1()` for error updates.
- `Protocol::ActionState::table` is refreshed by its own low-priority 50 Hz task in `UserCode/protocol/ActionState.*`, not from the timer ISR path.
- `TIM_Callback_100Hz()` runs the slower profile updates for chassis and grip.

The startup sequence in `Init()` is also part of the project contract:
- Initialize in order: `Device::init()`, `Chassis::init()`, `Protocol::init()`, then optional `Grip::init()`, then `Connection::init()`.
- Register all shared-I2C periodic devices before calling `AppI2C::start_bus1_manager()`. Do not start the shared I2C manager from an individual device module if other devices may still need to register on the same bus.
- Wait for all enabled devices/protocol links to report connected before proceeding.
- Enable and calibrate motion and grip separately, then wait until all enabled subsystems are ready.
- Call `Chassis::initStandaloneLocCtrl()` only for local-initialization modes; when upper-host localization is enabled, the first posture frame triggers `System::Init::initPostureReceive()` instead.
- Only enable the chassis controller after initialization is complete, and only enable grip after it is calibrated.

When adding a new subsystem, update its init, periodic update hooks, readiness gate, and enable/calibration path together so the startup contract remains coherent.

## Device & Protocol Mapping
The current hardware/software mapping in `UserCode/` is:
- `UART2` (`huart2`) is the yaw gyro (`HWT101CT`); `UART3` (`huart3`) is the upper-host link.
- Wheel motors are DJI motors: front wheel pair on `hcan1` with `id1 = 1, 2`, rear wheel pair on `hcan2` with `id1 = 3, 4`.
- Lift motors are also DJI motors, tracked as `Device::Motor::lift[4]`: front lift pair on `hcan1` with `id1 = 3, 4`, rear lift pair on `hcan2` with `id1 = 5, 6`.
- Grip uses two DJI motors on `hcan2`: arm `id1 = 1`, turn `id1 = 2`.
- Grip suction uses `GRIP_SUCTION` as the pump GPIO. Its optional `XGZP6847D` pressure sensor sits on shared `I2C1` and is gated separately through `PROJECT_PART_ENABLE_GRIP_SUCTION_PRESSURE_SENSOR`.

`Connection::table` and `Connection::Bit` in `UserCode/connection.hpp` define the canonical connection bitmap. The upper-host localization stream uses bit14 and is considered online only while its dedicated watchdog remains fed, currently for 200 watchdog ticks after each valid `LidarPosture` frame; the upper-host UART link itself uses bit15 directly. The four lift motors occupy four independent connection bits, and the grip suction pressure sensor uses bit11 only when `ProjectParts::EnableGripSuctionPressureSensor` is enabled; if you add, remove, or repurpose a device/protocol link, update the enum, required mask, table refresh logic, and connection wait path together.

Upper-host protocol behavior is likewise feature-gated:
- Create `Protocol::PCProtocol` only when `ProjectParts::EnableUpperHostProtocol` is true.
- Treat the first `LidarPosture` frame as the delayed initialization trigger when upper-host localization is enabled.
- Keep step-action commands gated by `ProjectParts::EnableStepAction`, which currently means PC control + wheel chassis + lift all enabled.
- When upper-host command IDs, payload layouts, or feedback layouts change, update `docs/upper_host_command_table.md` and `docs/upper_host_feedback_table.md` in the same change.
- Grip action commands occupy `0x40..0x43`: `0x40 TakeSpear` carries target/end posture as 6 packed `int16` values, `0x41 TakeSpearById` carries `SpearId + endPos`, and `0x42/0x43` trigger `StoreKFS/ReleaseKFS`.
- The six fixed spear target postures live in `Grip::Config::SpearGrab::TargetPoses`; fill them with calibrated world-frame values before using `TakeSpearById`.
- `TakeSpear` uses `Grip::Config::SpearGrab::SafeDistance` as a fixed retreat distance and is gated by `ProjectParts::EnableSpearGrabAction`; `StoreKFS/ReleaseKFS` are gated by `ProjectParts::EnableKfsAction`.

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
- Do not add helper functions that only forward a single call to an already semantically clear API, such as wrapping one member-function call with no additional policy or translation; call the original API directly.
- Treat the state-machine enum names in `UserCode/chassis/actions/Step.*` as semantic documentation; do not rename or anglicize them casually.
- Keep configuration constants in `UserCode/chassis/Config.hpp`, `UserCode/grip/Config.hpp`, `UserCode/suction/Config.hpp`, and `UserCode/project_parts.hpp` instead of scattering magic numbers into behavior code.

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
