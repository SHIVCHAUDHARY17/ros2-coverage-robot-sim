"""Turn a recorded run (rosbag2) into report.json + report.html.

    ros2 run coverage_report report outputs/bags/run_20261124_042421
    ros2 run coverage_report report <bag_dir> --out outputs/reports/my_run
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from . import html, metrics


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run report for the simulated coverage robot")
    ap.add_argument("bag", help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--out", help="output directory (default: outputs/reports/<bag name>)")
    ap.add_argument("--gap-threshold", type=float, default=0.1,
                    help="sensor silence (s) counted as a gap (default 0.1)")
    args = ap.parse_args(argv)

    from .bag import read_bag  # imported here so --help works without ROS sourced

    bag = os.path.normpath(args.bag)
    name = os.path.basename(bag)
    out = args.out or os.path.join("outputs", "reports", name)
    os.makedirs(out, exist_ok=True)

    run = read_bag(bag)
    if not run.states:
        print(f"error: no /controller_state messages in {bag}", file=sys.stderr)
        return 1
    m = metrics.compute(run, gap_threshold=args.gap_threshold)

    with open(os.path.join(out, "report.json"), "w", encoding="utf-8") as f:
        json.dump({"bag": name, **m}, f, indent=2)
    with open(os.path.join(out, "report.html"), "w", encoding="utf-8") as f:
        f.write(html.render(run, m, title=f"Run report: {name}"))

    s = m["sensor"]
    print(f"{name}: coverage {m['final_coverage_pct']:.1f} %, "
          f"{'completed' if m['completed'] else 'NOT completed'} in "
          f"{m['duration_s'] or 0:.1f} s, faults {m['faults']} (recovered {m['recoveries']}), "
          f"e-stops {m['estops']}, sensor RMS {s['rms_error_mm'] or 0:.1f} mm "
          f"({s['rms_error_at_arrival_mm'] or 0:.1f} mm on arrival) -> estimate RMS "
          f"{m['estimate_rms_error_mm'] or 0:.1f} mm")
    print(f"wrote {os.path.join(out, 'report.html')} and report.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
