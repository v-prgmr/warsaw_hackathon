#!/usr/bin/env bash
# Companion to start_g1_navigation.sh. Stops host-side project processes only.
set -euo pipefail
ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  printf 'Usage: bash scripts/stop_g1_navigation.sh\nStops ROS/Loco processes, project RViz instances and cmd_vel monitors.\nTerminal windows remain open so their output can be inspected.\n'
  exit 0
fi
if (($#)); then
  printf 'Unexpected argument. Use --help.\n' >&2
  exit 2
fi

# Existing project shutdown sends SIGINT first, allowing the Loco client to
# attempt StopMove and RTAB-Map to save its database before any escalation.
ros_status=0
bash "$ROOT/scripts/stop_ros.sh" || ros_status=$?

# Standalone RViz and topic-echo commands may not contain --ros-args and thus
# escape stop_ros.sh. Match their actual argv, not terminal-wrapper shell text.
python3 - "$ROOT" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time

root = sys.argv[1]

def remaining():
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            raw = (entry / 'cmdline').read_bytes()
            args = [s.decode(errors='replace') for s in raw.split(b'\0') if s]
            if not args:
                continue
            project_rviz = (Path(args[0]).name == 'rviz2' and
                            any(a.startswith(root + '/') for a in args[1:]))
            ros_index = next((i for i, a in enumerate(args[:2])
                              if Path(a).name == 'ros2'), None)
            monitor = False
            if ros_index is not None:
                command = args[ros_index + 1:]
                monitor = (command[:2] == ['topic', 'echo'] and
                           any(a == '/cmd_vel' or a.endswith('/cmd_vel') or
                               a.endswith('/cmd_vel_raw') for a in command[2:]))
            if project_rviz or monitor:
                result.append((int(entry.name), args))
        except (OSError, ProcessLookupError):
            continue
    return result

for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 3)):
    processes = remaining()
    if not processes:
        break
    for pid, args in processes:
        try:
            os.kill(pid, sig)
            print(f'{sig.name} -> {pid}: {" ".join(args)}', flush=True)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline and remaining():
        time.sleep(0.1)
left = remaining()
if left:
    print(f'Processes still running: {left}', file=sys.stderr)
    sys.exit(1)
print('Project RViz and velocity monitors stopped.')
PY

if ((ros_status)); then
  printf 'ROS shutdown reported an error; inspect the output above.\n' >&2
  exit "$ros_status"
fi
printf 'G1 navigation shutdown complete. Terminal output remains available.\n'
