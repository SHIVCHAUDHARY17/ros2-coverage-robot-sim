"""Unit tests for the report metrics. Pure Python: no ROS, no bag files needed."""

import math

import pytest

from coverage_report import html, metrics
from coverage_report.metrics import RunData


def straight_run(with_fault: bool = False) -> RunData:
    """Synthetic 1 m run along x: 0.5 m travel (valve off), then 0.5 m spraying."""
    run = RunData(grid_width=4, grid_height=2, resolution=0.5, target=[True] * 8,
                  path=[(0.0, 0.0, False), (0.5, 0.0, False), (1.0, 0.0, True)])
    for i in range(11):  # t = 0..1 s, x = 0..1 m
        t = float(i) * 0.1
        run.true_poses.append((t, t, 0.0))
        run.meas_poses.append((t, t + 0.003, 0.0))  # sensor reads 3 mm too far
        run.meas_arrivals.append(t)
        run.est_poses.append((t, t + 0.001, 0.0))   # estimate 1 mm off
    run.spray = [(0.0, False), (0.5, True)]
    run.states = [(0.0, "IDLE"), (0.0, "RUNNING"), (0.5, "RUNNING")]
    if with_fault:
        run.states += [(0.6, "FAULT: pose feedback timeout"), (0.7, "RUNNING")]
    run.states += [(1.0, "IDLE: path complete"), (1.5, "IDLE: path complete")]
    run.coverage = [(0.0, 0.0), (0.5, 40.0), (1.0, 100.0)]
    return run


def test_state_transitions_collapse_heartbeat():
    s = [(0, "IDLE"), (1, "IDLE"), (2, "RUNNING"), (3, "RUNNING"), (4, "ESTOP")]
    assert metrics.state_transitions(s) == [(0, "IDLE"), (2, "RUNNING"), (4, "ESTOP")]


def test_distances_split_by_valve_state():
    d = metrics.distances(straight_run().true_poses, straight_run().spray)
    assert d["travelling_m"] == pytest.approx(0.5)
    assert d["spraying_m"] == pytest.approx(0.5)


def test_gaps():
    times = [0.0, 0.02, 0.04, 0.44, 0.46, 0.70]
    assert metrics.max_gap(times) == pytest.approx(0.40)
    assert metrics.count_gaps(times, 0.1) == 2
    assert metrics.max_gap([]) == 0.0


def test_rms_error_against_truth():
    run = straight_run()
    assert metrics.rms_error(run.meas_poses, run.true_poses) == pytest.approx(0.003)
    assert metrics.rms_error([], run.true_poses) is None


def test_compute_clean_run():
    m = metrics.compute(straight_run())
    assert m["completed"] is True
    assert m["duration_s"] == pytest.approx(1.0)
    assert m["final_coverage_pct"] == 100.0
    assert m["target_cells"] == 8
    assert m["target_area_m2"] == pytest.approx(2.0)
    assert m["planned_path_m"] == pytest.approx(1.0)
    assert m["faults"] == 0 and m["recoveries"] == 0 and m["estops"] == 0
    assert m["sensor"]["rms_error_mm"] == pytest.approx(3.0)
    assert m["sensor"]["rms_error_at_arrival_mm"] == pytest.approx(3.0)  # no latency here
    assert m["estimate_rms_error_mm"] == pytest.approx(1.0)
    assert [e["state"] for e in m["events"]] == ["IDLE", "RUNNING", "IDLE: path complete"]


def test_compute_counts_fault_and_recovery():
    m = metrics.compute(straight_run(with_fault=True))
    assert m["faults"] == 1
    assert m["recoveries"] == 1
    assert m["fault_reasons"] == ["pose feedback timeout"]


def test_unrecovered_fault_is_not_counted_as_recovery():
    run = straight_run()
    run.states = [(0.0, "RUNNING"), (0.3, "FAULT: pose feedback timeout"), (0.5, "IDLE")]
    m = metrics.compute(run)
    assert m["faults"] == 1 and m["recoveries"] == 0
    assert m["completed"] is False


def test_html_report_is_self_contained_and_mentions_key_numbers():
    run = straight_run(with_fault=True)
    page = html.render(run, metrics.compute(run), title="Run report: test")
    assert page.startswith("<!DOCTYPE html>")
    assert "100.0 %" in page and "pose feedback timeout" in page
    assert "<script" not in page and "http://" not in page.replace("http://www.w3.org", "")
    assert page.count("<svg") == 2
    assert not math.isnan(metrics.compute(run)["duration_s"])


def test_latency_shows_up_only_in_error_at_arrival():
    run = straight_run()
    # same readings, but each one arrives 0.1 s (= 0.1 m of motion) late
    run.meas_poses = [(t, x, y) for t, x, y in run.true_poses]
    run.meas_arrivals = [t + 0.1 for t, _, _ in run.true_poses]
    s = metrics.compute(run)["sensor"]
    assert s["rms_error_mm"] == pytest.approx(0.0)
    assert s["rms_error_at_arrival_mm"] > 50.0
