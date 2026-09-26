#!/bin/bash
# Build g1_ws and run its tests. Run INSIDE the project container, in the isolated sim domain:
#   SIM=1 scripts/run_humble.sh scripts/run_tests.sh                 # lint + unit + contract tests (~10 s)
#   SIM=1 scripts/run_humble.sh scripts/run_tests.sh --integration   # + end-to-end runs (~3 min)
# The integration tests publish simulated robot topics (/dog_odom, /lowstate, LiDAR): they refuse
# to run on ROS_DOMAIN_ID 0, the robot's domain (AGENTS.md §19).
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR/g1_ws"

INTEGRATION=0
[ "${1:-}" = "--integration" ] && INTEGRATION=1
if [ "$INTEGRATION" = 1 ] && [ "${ROS_DOMAIN_ID:-0}" = "0" ]; then
  echo "Refusing to run integration tests on ROS_DOMAIN_ID 0. Use SIM=1 scripts/run_humble.sh." >&2
  exit 1
fi

echo "== build"
colcon build --event-handlers console_cohesion- >/tmp/g1_test_build.log 2>&1 \
  || { tail -40 /tmp/g1_test_build.log; exit 1; }
set +u  # ROS setup scripts read unset variables
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "== flake8"
flake8 --max-line-length 100 --extend-ignore E203,W503 src tests

echo "== unit + contract tests"
python3 -m pytest -q -p no:cacheprovider src/*/test tests/test_contracts.py

if [ "$INTEGRATION" = 1 ]; then
  echo "== integration tests (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  python3 -m pytest -q -p no:cacheprovider tests/test_integration.py
fi
echo "== all tests passed"
