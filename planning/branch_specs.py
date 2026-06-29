"""Pure Python branch metadata used by verification only."""

from __future__ import annotations

from dataclasses import dataclass

from config import ENTRY_POINTS, trajectory_name
from geometry import CORRIDORS, Rect, footprint_aabb_over_corridor


@dataclass(frozen=True)
class PhaseSpec:
    corridor_name: str
    nodes: int


@dataclass(frozen=True)
class BranchSpec:
    zone2_yaw_deg: float
    phases: tuple[PhaseSpec, ...]
    anchors: dict[str, tuple[float, float, float, float]]

    @property
    def suffix(self) -> str:
        return f"z2yaw{int(self.zone2_yaw_deg)}"


BRANCHES: tuple[BranchSpec, ...] = (
    BranchSpec(
        zone2_yaw_deg=0.0,
        phases=(
            PhaseSpec("approach", 32),
            PhaseSpec("top0", 42),
            PhaseSpec("right_down0", 22),
            PhaseSpec("low_straight0", 14),
            PhaseSpec("lower_down", 38),
            PhaseSpec("lower_finish", 28),
        ),
        anchors={
            "top_start": (8.55, 5.10, 0.0, 0.3),
            "top_end": (11.36, 5.10, 0.0, 0.3),
        },
    ),
)


def branch_by_suffix(suffix: str) -> BranchSpec:
    for branch in BRANCHES:
        if branch.suffix == suffix:
            return branch
    raise KeyError(f"unknown branch suffix: {suffix}")


def initial_guess_points_for_branch(suffix: str) -> list[tuple[str, float, float, float, float]]:
    branch = branch_by_suffix(suffix)
    return [(name, *state) for name, state in branch.anchors.items()]


def certify_start_corridors(start_idx: int, branch: BranchSpec | None = None) -> list[str]:
    branches = (branch,) if branch is not None else BRANCHES
    messages: list[str] = []
    for branch_spec in branches:
        corridors = _corridors_for_start(start_idx)
        for phase in branch_spec.phases:
            corridor = corridors[phase.corridor_name]
            x_low, x_high, y_low, y_high = footprint_aabb_over_corridor(corridor)
            cert = corridor.certificate
            margins = (
                x_low - cert.x_min,
                cert.x_max - x_high,
                y_low - cert.y_min,
                cert.y_max - y_high,
            )
            min_margin = min(margins)
            if min_margin <= 1e-4:
                raise ValueError(
                    f"{trajectory_name(start_idx)} {branch_spec.suffix} corridor "
                    f"{phase.corridor_name} not certified: min clearance {min_margin:.6f} m"
                )
            messages.append(
                f"{trajectory_name(start_idx)} {branch_spec.suffix} {phase.corridor_name}: "
                f"footprint inside {cert.name}, min clearance {min_margin:.3f} m"
            )
        for from_phase, to_phase in zip(branch_spec.phases, branch_spec.phases[1:]):
            from_corridor = corridors[from_phase.corridor_name]
            to_corridor = corridors[to_phase.corridor_name]
            x_min = max(from_corridor.x_min, to_corridor.x_min)
            x_max = min(from_corridor.x_max, to_corridor.x_max)
            y_min = max(from_corridor.y_min, to_corridor.y_min)
            y_max = min(from_corridor.y_max, to_corridor.y_max)
            yaw_min = max(from_corridor.yaw_min_deg, to_corridor.yaw_min_deg)
            yaw_max = min(from_corridor.yaw_max_deg, to_corridor.yaw_max_deg)
            if x_min > x_max or y_min > y_max or yaw_min > yaw_max:
                raise ValueError(
                    f"{trajectory_name(start_idx)} {branch_spec.suffix} transition "
                    f"{from_phase.corridor_name}->{to_phase.corridor_name} has no overlap"
                )
            transition = type(from_corridor)(
                name=f"{from_phase.corridor_name}_to_{to_phase.corridor_name}",
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                yaw_min_deg=yaw_min,
                yaw_max_deg=yaw_max,
                certificate=_transition_certificate(from_corridor.certificate, to_corridor.certificate),
                requires_h_03=from_corridor.requires_h_03 or to_corridor.requires_h_03,
            )
            x_low, x_high, y_low, y_high = footprint_aabb_over_corridor(transition)
            cert = transition.certificate
            margins = (
                x_low - cert.x_min,
                cert.x_max - x_high,
                y_low - cert.y_min,
                cert.y_max - y_high,
            )
            min_margin = min(margins)
            if min_margin <= 1e-4:
                raise ValueError(
                    f"{trajectory_name(start_idx)} {branch_spec.suffix} transition "
                    f"{transition.name} not certified: min clearance {min_margin:.6f} m"
                )
            messages.append(
                f"{trajectory_name(start_idx)} {branch_spec.suffix} {transition.name}: "
                f"transition footprint inside {cert.name}, min clearance {min_margin:.3f} m"
            )
    return messages


def _transition_certificate(a: Rect, b: Rect) -> Rect:
    x_min = max(a.x_min, b.x_min)
    x_max = min(a.x_max, b.x_max)
    y_min = max(a.y_min, b.y_min)
    y_max = min(a.y_max, b.y_max)
    if x_min <= x_max and y_min <= y_max:
        return Rect(f"{a.name}_and_{b.name}", x_min, x_max, y_min, y_max)
    return b


def _corridors_for_start(start_idx: int) -> dict[str, object]:
    start_y = float(ENTRY_POINTS[start_idx][1])
    corridors = dict(CORRIDORS)
    base = CORRIDORS["approach"]
    corridors["approach"] = type(base)(
        name=base.name,
        x_min=base.x_min,
        x_max=base.x_max,
        y_min=start_y,
        y_max=base.y_max,
        yaw_min_deg=base.yaw_min_deg,
        yaw_max_deg=base.yaw_max_deg,
        certificate=base.certificate,
        requires_h_03=base.requires_h_03,
    )
    return corridors
