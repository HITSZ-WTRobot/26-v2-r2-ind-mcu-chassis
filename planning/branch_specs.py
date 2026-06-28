"""Pure Python branch metadata used by verification only."""

from __future__ import annotations

from dataclasses import dataclass

from config import ENTRY_POINTS, trajectory_name
from geometry import CORRIDORS, footprint_aabb_over_corridor


@dataclass(frozen=True)
class PhaseSpec:
    corridor_name: str
    nodes: int


@dataclass(frozen=True)
class BranchSpec:
    zone2_yaw_deg: float
    phases: tuple[PhaseSpec, ...]
    anchors: dict[str, tuple[float, float, float, float]]
    anchors_are_hard: bool = True

    @property
    def suffix(self) -> str:
        return f"z2yaw{int(self.zone2_yaw_deg)}"


BRANCHES: tuple[BranchSpec, ...] = (
    BranchSpec(
        zone2_yaw_deg=-90.0,
        phases=(
            PhaseSpec("approach", 28),
            PhaseSpec("rotate-90", 20),
            PhaseSpec("top-90", 36),
            PhaseSpec("right_exit-90", 12),
            PhaseSpec("finish-90", 40),
        ),
        anchors={
            "approach_end": (8.55, 5.10, 0.0, 0.412),
            "rotate_end": (8.55, 5.10, -90.0, 0.3),
            "top_end": (11.15, 5.10, -90.0, 0.3),
            "right_exit_end": (11.35, 4.78, -90.0, 0.3),
        },
        anchors_are_hard=False,
    ),
    BranchSpec(
        zone2_yaw_deg=0.0,
        phases=(
            PhaseSpec("approach", 28),
            PhaseSpec("top0", 36),
            PhaseSpec("right_down0", 32),
            PhaseSpec("finish_rotate", 20),
            PhaseSpec("finish", 20),
        ),
        anchors={
            "approach_end": (8.55, 5.10, 0.0, 0.412),
            "top_end": (11.23, 5.645, 0.0, 0.3),
            "right_down_end": (11.23, 2.79, 0.0, 0.412),
            "finish_rotate_end": (11.35, 2.79, -90.0, 0.412),
        },
    ),
)


def branch_by_suffix(suffix: str) -> BranchSpec:
    for branch in BRANCHES:
        if branch.suffix == suffix:
            return branch
    raise KeyError(f"unknown branch suffix: {suffix}")


def anchor_points_for_branch(suffix: str) -> list[tuple[str, float, float, float, float]]:
    branch = branch_by_suffix(suffix)
    if not branch.anchors_are_hard:
        return []
    return [(name, *state) for name, state in branch.anchors.items()]


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
    return messages


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
