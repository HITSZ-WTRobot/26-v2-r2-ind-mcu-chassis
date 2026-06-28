"""离线轨迹规划系统 v3 — C++ CasADi 生成 + Python 验证/绘图入口"""

import csv
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from config import all_trajectory_indices, trajectory_name
import plot


class ExportedTrajectoryResult:
    def __init__(self, name: str, t_array: np.ndarray, s_array: np.ndarray):
        self.name = name
        self.t_array = t_array
        self.s_array = s_array
        self.success = len(t_array) > 0
        self.branch = "z2yaw0"
        self.total_time = float(t_array[-1]) if self.success else 0.0

    @property
    def n_points(self) -> int:
        return len(self.t_array) if self.success else 0


def main():
    t0 = time.time()

    print("=" * 60)
    print("  4-DOF 离线轨迹规划系统 v3 (C++ CasADi OCP + Python Verify)")
    print("=" * 60)

    # ---- [1/5] Costmap 诊断 ----
    _stage_header(1, 5, "Costmap 诊断图")
    t1 = time.time()
    from plot_costmap import plot_costmap
    plot_costmap("costmap_debug.png")
    _stage_footer(t1)

    # ---- [2/5] 3 条轨迹生成 ----
    _stage_header(2, 5, "C++ CasADi 轨迹生成 (3 条)")
    t2 = time.time()
    _run_cpp_generator()
    results = _load_exported_csv_results()
    _stage_footer(t2)

    ok = sum(1 for r in results.values() if r.success)
    print(f"  结果: {ok}/3 成功")
    if ok < 3:
        failed = [n for n, r in results.items() if not r.success]
        print(f"  FAILED: {failed}")
        sys.exit(1)

    # ---- [3/5] 导出检查 ----
    _stage_header(3, 5, "C++ Header/CSV 导出检查")
    t3 = time.time()
    _check_exported_files(results)
    _stage_footer(t3)

    # ---- [4/5] 轨迹验证 ----
    _stage_header(4, 5, "轨迹约束验证")
    t4 = time.time()
    from verify import verify_all
    passed = verify_all(results)
    _stage_footer(t4)

    # ---- [5/5] 可视化 ----
    _stage_header(5, 5, "轨迹可视化 + GIF 生成")
    t5 = time.time()
    plot.plot_all_v3(results, "trajectory_plot_v3.png")
    from plot_gif import generate_all_gifs
    generate_all_gifs(results)
    _stage_footer(t5)

    # ---- 总结 ----
    print("\n" + "=" * 60)
    print(f"  全部完成!  总耗时: {time.time() - t0:.1f}s")
    print("=" * 60)

    if not passed:
        print("  WARNING: 部分验证未通过，请检查上述输出")
        sys.exit(1)


def _stage_header(n: int, total: int, desc: str):
    print(f"\n┌{'─' * 56}┐")
    print(f"│ [{n}/{total}] {desc:<48} │")
    print(f"└{'─' * 56}┘")


def _stage_footer(t_start: float):
    print(f"  ⏱ {time.time() - t_start:.1f}s")


def _run_cpp_generator() -> None:
    build_dir = Path("build")
    build_dir.mkdir(exist_ok=True)

    subprocess.run(["cmake", "-S", ".", "-B", str(build_dir)], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "--target", "planning_cli"], check=True)
    subprocess.run([str(build_dir / "planning_cli")], check=True)


def _load_exported_csv_results() -> dict[str, ExportedTrajectoryResult]:
    results: dict[str, ExportedTrajectoryResult] = {}
    for idx in all_trajectory_indices():
        name = trajectory_name(idx)
        path = Path("dist") / f"{name}.csv"
        t_values: list[float] = []
        states: list[list[float]] = []
        with path.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                t_values.append(float(row["t"]))
                states.append(
                    [
                        float(row["x"]),
                        float(row["y"]),
                        float(row["yaw"]),
                        float(row["h"]),
                        float(row["dx"]),
                        float(row["dy"]),
                        float(row["dyaw"]),
                        float(row["dh"]),
                    ]
                )
        result = ExportedTrajectoryResult(
            name,
            np.array(t_values, dtype=float),
            np.array(states, dtype=float),
        )
        results[name] = result
        print(f"  loaded {path}: {result.total_time:.3f}s, {result.n_points} pts")
    return results


def _check_exported_files(results: dict[str, ExportedTrajectoryResult]) -> None:
    required = [Path("dist") / "trajectory_point.hpp", Path("dist") / "trajectory_all.hpp"]
    for name in results:
        required.extend(
            [
                Path("dist") / f"trajectory_{name}.hpp",
                Path("dist") / f"{name}.csv",
            ]
        )

    missing = [path for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing exported files: " + ", ".join(str(p) for p in missing))

    for path in required:
        print(f"  OK {path} ({path.stat().st_size} bytes)")

    for name, result in results.items():
        path = Path("dist") / f"trajectory_{name}.hpp"
        text = path.read_text()
        expected_ns = f"namespace planning::trajectory::{name} {{"
        if expected_ns not in text:
            raise AssertionError(f"missing namespace in {path}: {expected_ns}")
        for token in ("kSampleHz", "kDt", "kPointCount", "kDuration", "kPoints"):
            if token not in text:
                raise AssertionError(f"missing {token} in {path}")
        if "kTrajectory_" in text:
            raise AssertionError(f"legacy global trajectory symbol still present in {path}")
        if "500.000000" not in text or "0.002000" not in text:
            raise AssertionError(f"500Hz metadata missing in {path}")
        print(f"  OK {path}: {result.n_points} source pts, 500Hz header verified")


if __name__ == "__main__":
    main()
