#!/usr/bin/env bash
# Run the simulated robot headless, check the result, write a run report.
# Runs INSIDE the container (CI and local):
#   bash scripts/run_scenario.sh                     # clean sensors
#   bash scripts/run_scenario.sh site_faults 99      # noisy/late/dropping sensor, min 99 % coverage
# Exit code 0 only if the path was completed with coverage >= threshold.
set -euo pipefail

SCENARIO="${1:-}"          # name of config/scenarios/<name>.yaml, empty = clean
MIN_COVERAGE="${2:-99}"    # percent
TIMEOUT="${3:-150}"        # seconds

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

mkdir -p outputs/bags outputs/reports
BAGS_BEFORE=$(ls outputs/bags 2>/dev/null | wc -l)

# Stop the robot like an operator pressing Ctrl+C (rosbag2 then finishes its file).
# Escalate to SIGTERM / SIGKILL if it hangs, so CI can never get stuck here.
stop_launch() {
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "$LAUNCH_PID" 2>/dev/null || return 0; sleep 0.5; done
  echo "warning: launch still running after SIGINT, sending SIGTERM"
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  sleep 5
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
}

echo "== launching (scenario='${SCENARIO:-clean}', min coverage ${MIN_COVERAGE} %)"
# Job control on: in a script, background jobs otherwise IGNORE SIGINT (Ctrl+C),
# and the launch would never shut down. With -m it gets its own process group.
set -m
ros2 launch coverage_nodes coverage.launch.py record:=true ${SCENARIO:+scenario:=$SCENARIO} \
  > outputs/launch.log 2>&1 &
LAUNCH_PID=$!
set +m
trap stop_launch EXIT  # also stop the robot if this script dies early

set +e
ros2 run coverage_report check --min-coverage "$MIN_COVERAGE" --timeout "$TIMEOUT"
RESULT=$?
set -e

echo "== stopping the robot (SIGINT lets rosbag2 finish the file)"
stop_launch
trap - EXIT

BAG=$(ls -dt outputs/bags/*/ 2>/dev/null | head -1)
if [ "$(ls outputs/bags | wc -l)" -gt "$BAGS_BEFORE" ] && [ -n "$BAG" ]; then
  ros2 run coverage_report report "$BAG"
else
  echo "warning: no new bag recorded, skipping report"
fi

echo "== robot log (warnings and state changes):"
grep -E "WARN|ERROR|State:|Coverage: 100|recovery" outputs/launch.log | tail -25 || true
exit "$RESULT"
