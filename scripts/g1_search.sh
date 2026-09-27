#!/usr/bin/env bash
# Ask the G1 to look for an object and wait for the answer (ROBOT_SESSION.md, semantic_query).
#   bash scripts/g1_search.sh red cup           search (up to 20 s); prints "Found ..." + where
#   bash scripts/g1_search.sh stop | clear      stop searching | remove the boxes
# Runs in the robot container of start_g1_session.sh (or any running g1-humble container).
set -euo pipefail
[[ $# -ge 1 ]] || { sed -n 2,4p "$0" | sed 's/^# //'; exit 2; }
QUERY="$*"
C=$(docker ps --format '{{.Names}}' | grep -x g1-robot || docker ps -q --filter ancestor=g1-humble | head -1)
[[ -n $C ]] || { echo "no robot container running: start the session first (scripts/start_g1_session.sh)" >&2; exit 1; }
docker exec -i "$C" bash -c 'source /ros_entrypoint.sh; python3 - "$@"' _ "$QUERY" <<'PY'
import json, sys, time
import rclpy
from std_msgs.msg import String

query = sys.argv[1]
rclpy.init()
node = rclpy.create_node("g1_search_cli")
replies, pois = [], []
node.create_subscription(String, "/ar_glasses/reply", lambda m: replies.append(m.data), 10)
node.create_subscription(String, "/semantic_query/poi", lambda m: pois.append(json.loads(m.data)), 10)
pub = node.create_publisher(String, "/semantic_query/query", 10)
end = time.time() + 10.0
while time.time() < end and (pub.get_subscription_count() == 0
                             or node.count_publishers("/ar_glasses/reply") == 0):
    rclpy.spin_once(node, timeout_sec=0.1)
if pub.get_subscription_count() == 0:
    print("The object search is not running (window 'G1 6', semantic_query poi_node).")
    sys.exit(1)
for _ in range(10):                       # let the search node discover our subscriptions
    rclpy.spin_once(node, timeout_sec=0.05)
pub.publish(String(data=query))
print(f"> {query}")
done = ("Found", "No ", "Stopped", "Cleared")
end = time.time() + (5.0 if query.strip().lower() in ("stop", "clear", "cancel") else 30.0)
shown, finished = 0, False
while time.time() < end and not finished:
    rclpy.spin_once(node, timeout_sec=0.1)
    while shown < len(replies):
        print(replies[shown])
        finished = finished or replies[shown].startswith(done)
        shown += 1
for _ in range(5):
    rclpy.spin_once(node, timeout_sec=0.05)
if pois:
    p = pois[-1]
    b = p.get("box", {})
    print(f"  in map: xyz {p['xyz']}, box {[round(v * 100) for v in b.get('size', [])]} cm, "
          f"confidence {p['confidence']}  (/semantic_query/poi)")
if not finished:
    print("(no final answer yet: the search may still be running; see window 'G1 6')")
PY
