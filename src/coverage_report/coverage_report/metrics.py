"""Pure-Python run metrics: 'what did the robot do?'

No ROS imports here on purpose: everything is plain data in, plain dict out,
so it is unit-tested with pytest without a ROS installation or a bag file.
Times are in seconds, positions in metres.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

Pose = Tuple[float, float, float]  # (time, x, y)


@dataclass
class RunData:
    """Everything the report needs, already decoded from the bag."""

    # target grid (row-major, origin bottom-left, True = must be treated)
    grid_width: int = 0
    grid_height: int = 0
    resolution: float = 0.0
    target: List[bool] = field(default_factory=list)
    # planned path: (x, y, spray)
    path: List[Tuple[float, float, bool]] = field(default_factory=list)
    # time series; time = header stamp for poses, bag receive time otherwise
    true_poses: List[Pose] = field(default_factory=list)
    meas_poses: List[Pose] = field(default_factory=list)      # stamp = when measured
    meas_arrivals: List[float] = field(default_factory=list)  # when they reached the controller
    est_poses: List[Pose] = field(default_factory=list)
    spray: List[Tuple[float, bool]] = field(default_factory=list)
    states: List[Tuple[float, str]] = field(default_factory=list)
    coverage: List[Tuple[float, float]] = field(default_factory=list)  # (time, percent)
    # last /coverage_grid: 0 not target, 25 sprayed outside target, 50 to treat, 100 treated
    final_map: List[int] = field(default_factory=list)


def state_transitions(states: Sequence[Tuple[float, str]]) -> List[Tuple[float, str]]:
    """Collapse the 2 Hz state heartbeat into a list of changes."""
    out: List[Tuple[float, str]] = []
    for t, s in states:
        if not out or out[-1][1] != s:
            out.append((t, s))
    return out


def distances(true_poses: Sequence[Pose], spray: Sequence[Tuple[float, bool]]) -> Dict[str, float]:
    """Distance travelled with the valve open vs closed (ground-truth positions)."""
    spraying = travelling = 0.0
    spray_times = [s[0] for s in spray]
    for (t0, x0, y0), (_, x1, y1) in zip(true_poses, true_poses[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        i = bisect.bisect_right(spray_times, t0) - 1
        if i >= 0 and spray[i][1]:
            spraying += d
        else:
            travelling += d
    return {"spraying_m": spraying, "travelling_m": travelling}


def max_gap(times: Sequence[float]) -> float:
    """Longest silence between consecutive messages."""
    return max((b - a for a, b in zip(times, times[1:])), default=0.0)


def count_gaps(times: Sequence[float], threshold: float) -> int:
    return sum(1 for a, b in zip(times, times[1:]) if b - a > threshold)


def rms_error(samples: Sequence[Pose], truth: Sequence[Pose]) -> Optional[float]:
    """RMS position error of `samples` against the closest-in-time ground truth."""
    if not samples or not truth:
        return None
    t_truth = [p[0] for p in truth]
    acc = 0.0
    for t, x, y in samples:
        i = bisect.bisect_left(t_truth, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(truth)]
        j = min(cands, key=lambda k: abs(t_truth[k] - t))
        acc += (x - truth[j][1]) ** 2 + (y - truth[j][2]) ** 2
    return math.sqrt(acc / len(samples))


def compute(run: RunData, gap_threshold: float = 0.1) -> Dict[str, object]:
    """All report numbers as a JSON-ready dict."""
    changes = state_transitions(run.states)
    t_start = next((t for t, s in changes if s == "RUNNING"), None)
    t_done = next((t for t, s in changes if s.endswith("path complete")), None)
    t_end = t_done if t_done is not None else (run.states[-1][0] if run.states else None)
    t0 = t_start if t_start is not None else (changes[0][0] if changes else 0.0)

    faults = [s for _, s in changes if s.startswith("FAULT")]
    # A fault counts as recovered once the robot is RUNNING again (auto-recovery
    # goes FAULT -> RUNNING, an operator goes FAULT -> IDLE -> RUNNING).
    recoveries, pending = 0, False
    for _, s in changes:
        if s.startswith("FAULT"):
            pending = True
        elif s == "RUNNING" and pending:
            recoveries += 1
            pending = False

    dist = distances(run.true_poses, run.spray)
    planned = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(run.path, run.path[1:]))
    final_cov = run.coverage[-1][1] if run.coverage else 0.0

    return {
        "start_time": t0,
        "completed": t_done is not None,
        "duration_s": (t_end - t0) if (t_end is not None and t0 is not None) else None,
        "final_coverage_pct": final_cov,
        "target_cells": sum(run.target),
        "target_area_m2": sum(run.target) * run.resolution ** 2,
        "planned_path_m": planned,
        # concrete sprayed OUTSIDE the marked area (on site: removed by mistake)
        "overspray_m2": (sum(1 for v in run.final_map if v == 25) * run.resolution ** 2
                         if run.final_map else None),
        "driven_spraying_m": dist["spraying_m"],
        "driven_travelling_m": dist["travelling_m"],
        "estops": sum(1 for _, s in changes if s == "ESTOP"),
        "faults": len(faults),
        "fault_reasons": sorted(set(s.partition(": ")[2] for s in faults)),
        "recoveries": recoveries,
        "pauses": sum(1 for _, s in changes if s == "PAUSED"),
        "sensor": {
            "messages": len(run.meas_poses),
            # what the controller receives: the reading compared with where the nozzle
            # is when the reading ARRIVES (includes latency)
            "rms_error_at_arrival_mm": _mm(rms_error(
                [(a, x, y) for a, (_, x, y) in zip(run.meas_arrivals, run.meas_poses)],
                run.true_poses)),
            "longest_gap_s": max_gap(run.meas_arrivals),
            "gaps_over_threshold": count_gaps(run.meas_arrivals, gap_threshold),
            "gap_threshold_s": gap_threshold,
            # sensor accuracy alone: reading vs truth at the moment it was taken
            "rms_error_mm": _mm(rms_error(run.meas_poses, run.true_poses)),
        },
        "estimate_rms_error_mm": _mm(rms_error(run.est_poses, run.true_poses)),
        "events": [{"t_s": round(t - t0, 2), "state": s} for t, s in changes],
    }


def _mm(v: Optional[float]) -> Optional[float]:
    return None if v is None else v * 1000.0
