"""Record stationary OAK-D depth + Livox on one Ethernet NIC, two isolated DDS domains.

The OAK driver must already be publishing in ROS_DOMAIN_ID=78 on the G1 eth0;
LiDAR/TF stay in domain 0. This command never starts drivers or publishes TF.
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import yaml


TOPICS = {
    "oak": ("/oak/stereo/image_raw", "/oak/stereo/camera_info",
            "/oak/rgb/image_raw", "/oak/rgb/camera_info", "/tf_static"),
    "livox": ("/utlidar/cloud_livox_mid360", "/utlidar/imu_livox_mid360",
              "/lf/lowstate", "/tf_static", "/tf", "/joint_states", "/odom"),
}


def dds_environment(interface, domain):
    env = os.environ.copy()
    env.pop("ROS_LOCALHOST_ONLY", None)
    env.update(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", ROS_DOMAIN_ID=str(domain))
    env["CYCLONEDDS_URI"] = (
        '<CycloneDDS><Domain Id="any"><General><Interfaces>'
        f'<NetworkInterface name="{interface}" priority="default" multicast="default"/>'
        '</Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
    )
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path,
                        default=Path.home() / "g1_camera_calibration")
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--robot-ip", default="192.168.123.164")
    parser.add_argument("--oak-mxid", required=True,
                        help="device MXID from the driver's startup log; prevents mixing cameras")
    args = parser.parse_args()
    if args.duration < 5 or args.duration > 300:
        parser.error("duration must be between 5 and 300 seconds")
    route = subprocess.check_output(["ip", "route", "get", args.robot_ip], text=True).split()
    if "dev" not in route:
        parser.error("No Ethernet route to the G1")
    interface = route[route.index("dev") + 1]
    if not interface.startswith(("en", "eth")):
        parser.error(f"G1 route uses {interface}, not Ethernet")
    envs = {"oak": dds_environment(interface, 78),
            "livox": dds_environment(interface, 0)}
    for name in ("oak", "livox"):
        visible = subprocess.run(["ros2", "topic", "list", "--no-daemon", "--spin-time", "5"],
                                 env=envs[name], capture_output=True, text=True, timeout=15,
                                 check=True).stdout.splitlines()
        required = (TOPICS[name][0], TOPICS[name][1])
        if not all(topic in visible for topic in required):
            parser.error(f"{name} domain missing required topics {required}; start that driver first")
        # A stalled OAK pipeline may still advertise topics and publish /tf_static.
        # Require a real image/cloud message before writing either bag.
        try:
            sample = subprocess.run(
                ["ros2", "topic", "echo", "--once", required[0], "--field", "header"],
                env=envs[name], capture_output=True, text=True, timeout=12)
        except subprocess.TimeoutExpired:
            parser.error(f"{name} topic {required[0]} is advertised but has stopped publishing")
        if sample.returncode or "frame_id:" not in sample.stdout:
            parser.error(f"{name} topic {required[0]} is advertised but not delivering data")

    root = args.output_root.expanduser()
    if not root.is_dir():
        parser.error(f"Output parent must already exist: {root}")
    output = root / f"oak_livox_geometry_{datetime.now():%Y%m%d_%H%M%S}"
    output.mkdir()
    qos_file = Path(get_package_share_directory("g1_sensors")) / "config" / "oak_livox_capture_qos.yaml"
    processes = []
    try:
        for name in ("oak", "livox"):
            log = (output / f"{name}_recorder.log").open("w")
            cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(output / name),
                   "--qos-profile-overrides-path", str(qos_file), *TOPICS[name]]
            try:
                proc = subprocess.Popen(cmd, env=envs[name], stdin=subprocess.DEVNULL,
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                log.close()
                raise
            processes.append((name, proc, log))
        print(f"Recording OAK(domain 78) and Livox(domain 0) via {interface}: {output}", flush=True)
        time.sleep(3)
        for name, proc, _ in processes:
            if proc.poll() is not None:
                raise RuntimeError(f"{name} recorder exited early: {proc.returncode}")
        time.sleep(args.duration)
    finally:
        for _, proc, _ in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for name, proc, log in processes:
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
            finally:
                log.close()
            print(f"{name} recorder exit: {proc.returncode}", flush=True)

    info = {
        "purpose": "Stationary OAK-D depth / MID-360 extrinsic calibration",
        "interface": interface, "duration_requested_s": args.duration,
        "oak": {"bag": "oak", "ros_domain_id": 78, "mxid_reported_by_driver": args.oak_mxid},
        "livox": {"bag": "livox", "ros_domain_id": 0},
        "time_source": "laptop rosbag receipt time, not the unsynchronized sensor header clocks",
        "stationary_capture": "operator must keep the G1 and the room scene still",
    }
    with (output / "capture_info.yaml").open("w") as stream:
        yaml.safe_dump(info, stream, sort_keys=False)
    if any(proc.returncode != 0 for _, proc, _ in processes):
        raise RuntimeError("Recorder exited abnormally; inspect the logs and ros2 bag info")
    for name, topic, minimum in (("oak", "/oak/stereo/image_raw", args.duration * 3),
                                 ("oak", "/oak/rgb/image_raw", args.duration * 3),
                                 ("livox", "/utlidar/cloud_livox_mid360", args.duration * 5)):
        with (output / name / "metadata.yaml").open() as stream:
            bag = yaml.safe_load(stream)["rosbag2_bagfile_information"]
        count = next((item["message_count"] for item in bag["topics_with_message_count"]
                      if item["topic_metadata"]["name"] == topic), 0)
        if count < minimum:
            raise RuntimeError(f"{name} recorded only {count} messages on {topic}; bag is incomplete: {output}")
    print("Finalized:", output, flush=True)


if __name__ == "__main__":
    main()
