#!/usr/bin/env bash
# R4 read-only sensor discovery for the G1 graph.
# Snapshots node/topic names, per-topic type + QoS + rate, and the TF frames, into a markdown
# report. 100% read-only (topic list/info/hz/echo) -> compliant with the "read-only sensor
# discovery" organizer rule. Run it now for whatever is live, re-run when RealSense comes up.
#
# Usage:
#   ros2 run g1_recorder discover_sensors.sh            # report -> ./sensor_discovery_<ts>.md
#   ros2 run g1_recorder discover_sensors.sh /path.md   # custom report path
#   HZ_SECS=5 ros2 run g1_recorder discover_sensors.sh  # longer rate sampling (default 3s)
set -uo pipefail

REPORT="${1:-./sensor_discovery_$(date +%Y%m%d_%H%M%S).md}"
HZ_SECS="${HZ_SECS:-3}"

# Candidate substrings we care about; discovery still lists everything, this just flags the
# sensor-relevant topics for closer inspection.
PATTERNS='color|image|depth|camera_info|imu|cloud|lidar|scan|odom|/tf|lowstate|joint_states|bmsstate|wirelesscontroller'

log() { echo "$@" | tee -a "$REPORT" >/dev/null; }

: > "$REPORT"
log "# G1 sensor discovery — $(date -Is)"
log ""
log "RMW=\`${RMW_IMPLEMENTATION:-<unset>}\`  ROS_DOMAIN_ID=\`${ROS_DOMAIN_ID:-0}\`"
log ""

log "## Nodes"
log '```'; ros2 node list 2>/dev/null | tee -a "$REPORT" >/dev/null; log '```'
log ""

log "## All topics (name + type)"
log '```'; ros2 topic list -t 2>/dev/null | tee -a "$REPORT" >/dev/null; log '```'
log ""

log "## Sensor-relevant topics — QoS + rate"
mapfile -t TOPICS < <(ros2 topic list 2>/dev/null | grep -E "$PATTERNS")
if [ "${#TOPICS[@]}" -eq 0 ]; then
  log "_No sensor-relevant topics found yet (streams not up?)._"
fi
for t in "${TOPICS[@]}"; do
  log "### \`$t\`"
  log '```'
  echo "-- info -v --" | tee -a "$REPORT" >/dev/null
  ros2 topic info -v "$t" 2>/dev/null | grep -iE 'Type|Reliability|Durability|History|Publisher count' | tee -a "$REPORT" >/dev/null
  echo "-- hz (${HZ_SECS}s) --" | tee -a "$REPORT" >/dev/null
  timeout "$HZ_SECS" ros2 topic hz "$t" 2>/dev/null | grep -iE 'average rate|window' | tail -2 | tee -a "$REPORT" >/dev/null
  log '```'
done
log ""

log "## TF frames"
log "### static (/tf_static, transient_local)"
log '```'
timeout 4 ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --once 2>/dev/null \
  | grep -iE 'frame_id|child_frame_id' | tee -a "$REPORT" >/dev/null
log '```'
log "_For the full tree run: \`ros2 run tf2_tools view_frames\` (writes frames.pdf)._"
log ""

log "## Next: reconcile with config"
log "- Reconcile \`g1_recorder/config/survey.yaml\` and \`live_run.yaml\` with the verified names above."
log "- If any sensor topic is **BEST_EFFORT**, note it; the recorder auto-adopts a single"
log "  publisher's QoS, but add an override if capture drops messages."
log "- Confirm **aligned depth** exists (needs \`align_depth:=true\` on the RealSense launch)."
log "- Update \`keyframe_manager/config/keyframe_params.yaml\` topic names to match."

echo "Report written to: $REPORT"
