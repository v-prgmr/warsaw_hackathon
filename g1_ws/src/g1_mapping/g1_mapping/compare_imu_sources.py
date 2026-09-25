"""Compare g1_mapping's IMU sources (imu_source:=dog vs livox) on one recorded bag.

1. Offline bag analysis (no ROS graph): motion of the three waist joints between the pelvis IMU
   (/dog_imu_raw) and the torso-mounted LiDAR while walking, and raw gyro bias while standing.
2. Replays the bag through g1_mapping once per IMU source and records: ICP tracking (lost scans,
   inlier ratio), loop closures, the final map -> odom correction (accumulated odometry drift),
   floor flatness of the optimized trajectory, and wall/floor thickness in /cloud_map (sharpness).

Replaying publishes robot topics (/dog_odom, /utlidar/..., /lf/lowstate), so this refuses to run
on the robot's DDS domain (0). Run it in the sim container:
    SIM=1 scripts/run_humble.sh
    ros2 run g1_mapping compare_imu_sources bags/<walking_bag> --out bags/<walking_bag>_imu_compare
Output: <out>/report.md and report.json, plus per-run launch logs and RTAB-Map databases.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rtabmap_msgs.msg import Info, OdomInfo
from sensor_msgs.msg import PointCloud2
import tf2_ros
import yaml

LOWSTATE_TOPICS = ("/lf/lowstate", "/lowstate")  # prefer the 20 Hz copy
WAIST_MOTORS = {"yaw": 12, "roll": 13, "pitch": 14}  # G1 29-DoF motor order
MOVING_SPEED = 0.05  # m/s over 0.5 s (/dog_odom is ~2x short, so keep this low)
MOVING_YAW_RATE = 5.0  # deg/s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_topics():
    cfg = os.path.join(get_package_share_directory("g1_mapping"), "config", "g1_mapping.yaml")
    with open(cfg) as f:
        return yaml.safe_load(f)["topics"]


def bag_metadata(uri):
    with open(os.path.join(uri, "metadata.yaml")) as f:
        info = yaml.safe_load(f)["rosbag2_bagfile_information"]
    counts = {t["topic_metadata"]["name"]: t["message_count"]
              for t in info["topics_with_message_count"]}
    return info["storage_identifier"], counts


def read_bag(uri, storage, topics):
    """Yield (topic, message, receive time in s) for the topics whose types are importable."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    for t in topics:
        if t not in types:
            continue
        try:
            classes[t] = get_message(types[t])
        except (AttributeError, ModuleNotFoundError, ValueError):
            print(f"  {t}: type {types[t]} not available here, skipped")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        yield topic, deserialize_message(raw, classes[topic]), t_ns * 1e-9


def range_stats(deg, t):
    rate = np.abs(np.gradient(deg, t)) if len(deg) > 2 else np.zeros(1)
    return {"p1_p99_range_deg": float(np.percentile(deg, 99) - np.percentile(deg, 1)),
            "std_deg": float(deg.std()), "rate_p99_deg_s": float(np.percentile(rate, 99))}


def analyze_bag(uri, storage, counts, topics):
    """Waist motion while walking vs standing, and raw gyro bias while standing."""
    low_topic = next((t for t in LOWSTATE_TOPICS if counts.get(t)), None)
    imu_topics = [t for t in (topics["imu"], topics["imu_livox"]) if counts.get(t)]
    odom, waist, gyro = [], [], {t: [] for t in imu_topics}
    wanted = [topics["dog_odom"]] + imu_topics + ([low_topic] if low_topic else [])
    for topic, m, t in read_bag(uri, storage, wanted):
        if topic == topics["dog_odom"]:
            if not odom or t - odom[-1][0] >= 0.05:  # 20 Hz is plenty to detect motion
                p = m.pose.pose.position
                odom.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))
        elif topic == low_topic:
            waist.append((t, *[m.motor_state[i].q for i in WAIST_MOTORS.values()]))
        else:
            g = m.angular_velocity
            gyro[topic].append((t, g.x, g.y, g.z))

    out = {"lowstate_topic": low_topic, "waist": None, "gyro_bias_standing_deg_s": {}}
    if len(odom) < 20:
        out["note"] = "no /dog_odom: cannot tell walking from standing"
        return out
    o = np.array(odom)
    k = 10  # 0.5 s at 20 Hz
    dt = o[k:, 0] - o[:-k, 0]
    speed = np.hypot(o[k:, 1] - o[:-k, 1], o[k:, 2] - o[:-k, 2]) / dt
    dyaw = np.degrees(np.angle(np.exp(1j * (o[k:, 3] - o[:-k, 3])))) / dt
    t_mid = (o[k:, 0] + o[:-k, 0]) / 2
    moving = ((speed > MOVING_SPEED) | (np.abs(dyaw) > MOVING_YAW_RATE)).astype(float)
    out["walking_s"] = float(moving.mean() * (o[-1, 0] - o[0, 0]))

    def is_moving(ts):
        return np.interp(ts, t_mid, moving) > 0.5

    if waist:
        w = np.array(waist)
        mv = is_moving(w[:, 0])
        out["waist"] = {}
        for j, name in enumerate(WAIST_MOTORS, start=1):
            deg = np.degrees(w[:, j])
            out["waist"][name] = {
                "walking": range_stats(deg[mv], w[mv, 0]) if mv.sum() > 20 else None,
                "standing": range_stats(deg[~mv], w[~mv, 0]) if (~mv).sum() > 20 else None,
            }
    for topic, rows in gyro.items():
        g = np.array(rows)
        still = ~is_moving(g[:, 0])
        if still.sum() > 100:
            bias = np.degrees(g[still, 1:].mean(axis=0))
            out["gyro_bias_standing_deg_s"][topic] = bias.round(3).tolist()
    return out


class Monitor:
    """Collects g1_mapping outputs during one replay."""

    def __init__(self, source):
        self.node = rclpy.create_node(f"imu_compare_{source}",
                                      parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom_info, self.loops, self.proximity = [], 0, 0
        self.path, self.cloud_raw = None, None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self.node)
        n = self.node
        n.create_subscription(OdomInfo, "/odom_info", self.on_odom_info, 100)
        n.create_subscription(Info, "/info", self.on_info, 100)
        n.create_subscription(Path, "/mapPath", lambda m: setattr(self, "path", m), 10)
        n.create_subscription(PointCloud2, "/cloud_map",
                              lambda m: setattr(self, "cloud_raw", m), 1, raw=True)

    def on_odom_info(self, m):
        self.odom_info.append((m.lost, m.icp_inliers_ratio, m.icp_correspondences,
                               m.time_estimation))

    def on_info(self, m):
        self.loops += m.loop_closure_id > 0
        self.proximity += m.proximity_detection_id > 0

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_ready(self, timeout=60.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.count_publishers("/odom_info") and self.node.count_publishers("/info"):
                return True
        return False

    def metrics(self):
        res = {}
        if self.odom_info:
            oi = np.array(self.odom_info, dtype=float)
            ok = oi[:, 0] == 0
            res.update({
                "scans": len(oi), "scans_lost": int((~ok).sum()),
                "icp_inliers_ratio_median": float(np.median(oi[ok, 1])) if ok.any() else None,
                "icp_correspondences_median": float(np.median(oi[ok, 2])) if ok.any() else None,
                "odom_time_ms_median": float(np.median(oi[ok, 3]) * 1000) if ok.any() else None,
            })
        res["loop_closures"] = self.loops
        res["proximity_closures"] = self.proximity
        try:
            tr = self.tf.lookup_transform("map", "odom", Time())
            v = tr.transform.translation
            res["map_to_odom_final_m"] = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            res["map_to_odom_final_yaw_deg"] = abs(math.degrees(yaw_of(tr.transform.rotation)))
        except tf2_ros.TransformException as e:
            res["map_to_odom_error"] = str(e)
        if self.path and self.path.poses:
            p = np.array([[s.pose.position.x, s.pose.position.y, s.pose.position.z]
                          for s in self.path.poses])
            res.update({
                "graph_nodes": len(p),
                "path_length_m": float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
                "path_z_range_m": float(np.percentile(p[:, 2], 95) - np.percentile(p[:, 2], 5)),
            })
        if self.cloud_raw is not None:
            thick, cells = map_thickness(cloud_xyz(deserialize_message(self.cloud_raw,
                                                                       PointCloud2)))
            res["map_thickness_cm_median"] = thick
            res["map_planar_cells"] = cells
        return res


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def map_thickness(xyz, cell=0.3, min_points=10):
    """Median thickness (cm) of planar patches (walls, floor, tables): lower = sharper map."""
    if len(xyz) < min_points:
        return None, 0
    keys = np.floor(xyz / cell).astype(np.int64)
    order = np.lexsort(keys.T)
    keys, xyz = keys[order], xyz[order]
    _, start, count = np.unique(keys, axis=0, return_index=True, return_counts=True)
    thick = []
    for s, c in zip(start, count):
        if c < min_points:
            continue
        ev = np.linalg.eigvalsh(np.cov(xyz[s:s + c].T))  # ascending
        if ev[1] > 10 * ev[0]:  # planar patch
            thick.append(math.sqrt(max(ev[0], 0.0)))
    return (float(np.median(thick) * 100) if thick else None), len(thick)


def replay(bag, source, out_dir, rate, replay_topics, launch_args):
    db = os.path.join(out_dir, f"{source}.db")
    with open(os.path.join(out_dir, f"{source}_launch.log"), "w") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "g1_mapping", "mapping.launch.py", "use_sim_time:=true",
             f"imu_source:={source}", f"database_path:={db}", "delete_db:=true", *launch_args],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        mon = Monitor(source)
        try:
            if not mon.wait_ready():
                raise RuntimeError(f"g1_mapping did not start; see {log.name}")
            play = subprocess.Popen(
                ["ros2", "bag", "play", bag, "--clock", "--rate", str(rate),
                 "--topics", *replay_topics],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            while play.poll() is None:
                mon.spin_for(0.5)
            mon.spin_for(5.0)  # let RTAB-Map process the last scans
            return mon.metrics(), db
        finally:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
            mon.node.destroy_node()


# (metric, better, label)
ROWS = [
    ("scans_lost", "lower", "ICP scans lost"),
    ("icp_inliers_ratio_median", "higher", "ICP inlier ratio (median)"),
    ("graph_nodes", "", "map graph nodes"),
    ("path_length_m", "", "optimized trajectory length (m)"),
    ("loop_closures", "", "loop closures (global)"),
    ("proximity_closures", "", "loop closures (LiDAR proximity)"),
    ("map_to_odom_final_m", "lower", "final map->odom correction (m)"),
    ("map_to_odom_final_yaw_deg", "lower", "final map->odom correction (deg)"),
    ("path_z_range_m", "lower", "trajectory z range p5-p95 (m, flat floor)"),
    ("map_thickness_cm_median", "lower", "wall/floor thickness (cm, median)"),
    ("odom_time_ms_median", "lower", "ICP time per scan (ms)"),
]


def fmt(v):
    if v is None:
        return "-"
    return f"{v:.3f}" if isinstance(v, float) else str(v)


def report(bag, analysis, results):
    lines = [f"# IMU source comparison: `{os.path.basename(os.path.normpath(bag))}`", ""]
    lines += ["## Waist motion (joints between the pelvis IMU and the LiDAR)", ""]
    if analysis.get("waist"):
        lines += [f"From `{analysis['lowstate_topic']}`; walking detected for "
                  f"{analysis.get('walking_s', 0):.0f} s.", "",
                  "| joint | walking p1-p99 (deg) | walking std (deg) | walking rate p99 (deg/s) "
                  "| standing std (deg) |", "|---|---|---|---|---|"]
        walking = [s["walking"] for s in analysis["waist"].values() if s["walking"]]
        for name, s in analysis["waist"].items():
            w, st = s["walking"] or {}, s["standing"] or {}
            lines.append(f"| {name} | {fmt(w.get('p1_p99_range_deg'))} | {fmt(w.get('std_deg'))} "
                         f"| {fmt(w.get('rate_p99_deg_s'))} | {fmt(st.get('std_deg'))} |")
        if walking:
            worst = max(w["p1_p99_range_deg"] for w in walking)
            lines += ["", f"Largest waist range while walking: **{worst:.1f} deg**. Below ~1 deg "
                      "the torso and pelvis rotate together and both IMUs see the same rotation; "
                      "above a few degrees the pelvis IMU misreports the LiDAR's rotation unless "
                      "`/tf` from the joint states is in the bag.", ""]
        else:
            lines += ["", "**No walking detected in this bag**: the waist check needs one.", ""]
    else:
        lines += ["No `/lf/lowstate` or `/lowstate` in the bag: waist motion not available.", ""]
    if analysis.get("gyro_bias_standing_deg_s"):
        lines += ["Raw gyro mean while standing (deg/s, x y z): " + "; ".join(
            f"`{t}` {v}" for t, v in analysis["gyro_bias_standing_deg_s"].items()), ""]
    sources = list(results)
    if not sources:
        return "\n".join(lines)
    lines += ["## Mapping metrics", "", "| metric | better | " + " | ".join(sources) + " |",
              "|---|---|" + "---|" * len(sources)]
    for key, better, label in ROWS:
        lines.append(f"| {label} | {better} | "
                     + " | ".join(fmt(results[s][0].get(key)) for s in sources) + " |")
    lines += ["", "Databases: " + ", ".join(f"`{results[s][1]}`" for s in sources), "",
              "Loop-closure counts only compare fairly if both runs closed the same loops. Also "
              "check the tape measurements against each map (AGENTS.md §10.2) and look at both "
              "maps in `rtabmap-databaseViewer`.", ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--out", help="output directory (default: <bag>_imu_compare)")
    ap.add_argument("--sources", nargs="+", default=["dog", "livox"], choices=["dog", "livox"])
    ap.add_argument("--rate", type=float, default=1.0, help="bag playback rate")
    ap.add_argument("--analyze-only", action="store_true", help="skip the mapping replays")
    ap.add_argument("--launch-arg", action="append", default=[],
                    help="extra mapping.launch.py argument, e.g. static_tf:=false (repeatable)")
    args = ap.parse_args()

    bag = os.path.abspath(args.bag)
    out = os.path.abspath(args.out or bag.rstrip("/") + "_imu_compare")
    os.makedirs(out, exist_ok=True)
    topics = load_topics()
    storage, counts = bag_metadata(bag)

    print("Analyzing the bag offline ...")
    analysis = analyze_bag(bag, storage, counts, topics)
    results = {}
    if not args.analyze_only:
        domain = os.environ.get("ROS_DOMAIN_ID", "0") or "0"
        if domain == "0":
            sys.exit("Refusing to replay on ROS_DOMAIN_ID 0 (the robot's DDS domain): the bag's "
                     "robot topics would reach the live G1. Use `SIM=1 scripts/run_humble.sh` "
                     "(domain 77) or export a non-zero ROS_DOMAIN_ID. --analyze-only is safe.")
        need = {"dog": topics["imu"], "livox": topics["imu_livox"]}
        replay_topics = [t for t in (topics["lidar"], topics["imu"], topics["imu_livox"],
                                     topics["dog_odom"], "/tf", "/tf_static") if counts.get(t)]
        if counts.get("/tf"):
            print("Note: the bag has /tf. If it holds the URDF chain, pass "
                  "--launch-arg static_tf:=false.")
        rclpy.init()
        try:
            for source in args.sources:
                if not counts.get(need[source]):
                    print(f"Skipping imu_source:={source}: {need[source]} is not in the bag")
                    continue
                print(f"Replaying with imu_source:={source} ...")
                results[source] = replay(bag, source, out, args.rate, replay_topics,
                                         args.launch_arg)
        finally:
            rclpy.try_shutdown()

    text = report(bag, analysis, results)
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write(text)
    with open(os.path.join(out, "report.json"), "w") as f:
        json.dump({"analysis": analysis, "runs": {s: r[0] for s, r in results.items()}}, f,
                  indent=2)
    print(text)
    print(f"Written to {out}")


if __name__ == "__main__":
    main()
