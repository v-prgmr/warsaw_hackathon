"""Offline OAK stereo depth / MID-360 extrinsic check from two MCAP bags.

Each bag is recorded on the same laptop clock, but in a different DDS domain.
Never use sensor header stamps to pair the two clocks. This program does not
publish TF or alter the installed G1 extrinsics.
"""

import argparse
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage
import yaml


DEPTH = "/oak/stereo/image_raw"
RGB = "/oak/rgb/image_raw"
RGB_INFO = "/oak/rgb/camera_info"
DEPTH_INFO = "/oak/stereo/camera_info"
LIDAR = "/utlidar/cloud_livox_mid360"


def metadata_span(directory):
    with (directory / "metadata.yaml").open() as stream:
        meta = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    start = meta["starting_time"]["nanoseconds_since_epoch"] / 1e9
    return start, start + meta["duration"]["nanoseconds"] / 1e9


def read_messages(directory):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        yield reader.read_next()


def tf_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def chain(edges, *links):
    transform = np.eye(4)
    for parent, child in zip(links, links[1:]):
        if (parent, child) not in edges:
            raise ValueError(f"Missing static transform {parent} -> {child}")
        transform = transform @ edges[(parent, child)]
    return transform


def recorded_seed(oak_edges, lidar_edges, depth_frame):
    # All these links are fixed to the torso. The old OAK mount is a ROUGH seed
    # only, especially when the connected OAK hardware has a different MXID.
    torso_lidar = chain(lidar_edges, "torso_link", "mid360_link", "livox_frame")
    torso_camera = chain(lidar_edges, "torso_link", "d435_link", "camera_link",
                         "oak-d-base-frame")
    oak_base_depth = chain(oak_edges, "oak-d-base-frame", "oak", "oak_rgb_camera_frame",
                           depth_frame)
    return np.linalg.inv(torso_camera @ oak_base_depth) @ torso_lidar


def depth_points(messages, info, pixel_stride, min_depth, max_depth):
    """Return 3D XYZ and metric depth in the optical frame (Z, not radial range)."""
    if (info.width, info.height) != (messages[0].width, messages[0].height):
        raise ValueError("OAK depth and CameraInfo dimensions do not agree")
    if any(m.encoding != "16UC1" or m.header.frame_id != info.header.frame_id
           for m in messages):
        raise ValueError("Expected 16UC1 depth in the CameraInfo optical frame")
    if any(m.is_bigendian or m.step != m.width * 2 for m in messages):
        raise ValueError("Unsupported padded or big-endian OAK depth image")
    full_depth = np.median(np.stack([
        np.frombuffer(m.data, dtype="<u2").reshape(m.height, m.width)
        for m in messages
    ]), axis=0).astype(np.float32) * 0.001  # Luxonis RAW16 depth in millimetres
    depth = full_depth[::pixel_stride, ::pixel_stride]
    v, u = np.mgrid[:messages[0].height:pixel_stride, :messages[0].width:pixel_stride]
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if fx <= 0 or fy <= 0:
        raise ValueError("OAK CameraInfo has no valid focal length")
    # Avoid the edges of the RGB-aligned stereo image (occlusion/distortion).
    keep = ((depth >= min_depth) & (depth <= max_depth) &
            (u >= info.width * 0.08) & (u < info.width * 0.92) &
            (v >= info.height * 0.08) & (v < info.height * 0.92))
    z = depth[keep]
    points = np.column_stack(((u[keep] - cx) * z / fx,
                              (v[keep] - cy) * z / fy, z))
    return points, full_depth


def cloud_xyz(msg):
    if msg.header.frame_id != "livox_frame" or msg.is_bigendian:
        raise ValueError(f"Unexpected Livox frame/byte order: {msg.header.frame_id}")
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("LiDAR cloud does not contain XYZ")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [fields[k].offset for k in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    # This G1's Livox cloud is unorganized and rows have no extra padding.
    if msg.row_step != msg.width * msg.point_step or msg.height != 1:
        raise ValueError("Unexpected Livox cloud layout")
    points = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
    xyz = np.column_stack([points[k] for k in ("x", "y", "z")])
    radius = np.linalg.norm(xyz, axis=1)
    return xyz[np.isfinite(radius) & (radius >= 0.45) & (radius <= 6.0)]


def voxel(points, size):
    if not len(points):
        return points
    keys = np.floor(points / size).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def crop_livox(points, seed, info, min_depth, max_depth):
    local = transform(points, seed)
    z = local[:, 2]
    valid = (z > min_depth * 0.8) & (z < max_depth * 1.2)
    local = local[valid]
    original = points[valid]
    u = info.k[0] * local[:, 0] / local[:, 2] + info.k[2]
    v = info.k[4] * local[:, 1] / local[:, 2] + info.k[5]
    # Margin accommodates uncertainty in the seed but excludes rear/side scans.
    keep = ((u >= -info.width * 0.08) & (u < info.width * 1.08) &
            (v >= -info.height * 0.08) & (v < info.height * 1.08))
    return original[keep]


def rigid_fit(source, target):
    """Weighted-uniform Kabsch fit; maps source XYZ into target XYZ."""
    src_mean, dst_mean = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - src_mean).T @ (target - dst_mean))
    flip = np.eye(3)
    flip[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ flip @ u.T
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rotation, dst_mean - rotation @ src_mean
    return out


def registration_errors(windows, pose, info, threshold=0.12):
    results = []
    for window in windows:
        distances, _ = cKDTree(window["oak"]).query(transform(window["livox"], pose), workers=-1)
        inlier = distances < threshold
        projected = transform(window["livox"], pose)
        z = projected[:, 2]
        safe_z = np.maximum(z, 1e-6)
        u = np.rint(info.k[0] * projected[:, 0] / safe_z + info.k[2]).astype(int)
        v = np.rint(info.k[4] * projected[:, 1] / safe_z + info.k[5]).astype(int)
        within = ((z > 0) & (u >= 0) & (u < info.width) &
                  (v >= 0) & (v < info.height))
        measured = np.zeros(len(z), dtype=np.float32)
        measured[within] = window["depth"][v[within], u[within]]
        valid = within & (measured > 0.7) & (measured < 3.5)
        delta = z[valid] - measured[valid]
        # Reject rays on occlusion edges, then report their fraction too.
        consistent = np.abs(delta) < 0.20
        depth_errors = np.abs(delta[consistent])
        results.append({"capture": window["capture"], "time_s": round(float(window["time"]), 3),
                        "livox_points": len(window["livox"]), "oak_points": len(window["oak"]),
                        "inlier_fraction": round(float(inlier.mean()), 4),
                        "median_m": round(float(np.median(distances[inlier])), 4) if inlier.any() else None,
                        "p90_m": round(float(np.percentile(distances[inlier], 90)), 4) if inlier.any() else None,
                        "depth_valid_rays": int(valid.sum()),
                        "depth_consistent_fraction": round(float(consistent.mean()), 4) if len(delta) else 0.0,
                        "depth_median_abs_m": round(float(np.median(depth_errors)), 4) if len(depth_errors) else None,
                        "depth_p90_abs_m": round(float(np.percentile(depth_errors, 90)), 4) if len(depth_errors) else None})
    return results


def save_overlay(window, pose, info, filename, color):
    """Project LiDAR points into the held-out OAK RGB image for inspection."""
    image = window.get("rgb")
    if image is None or image.encoding not in ("rgb8", "bgr8"):
        return False
    if (image.width, image.height) != (info.width, info.height):
        return False
    pixels = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)
    pixels = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
    canvas = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if image.encoding == "rgb8" else pixels.copy()
    points = transform(window["livox"], pose)
    points = points[points[:, 2] > 0.1]
    u = np.rint(info.k[0] * points[:, 0] / points[:, 2] + info.k[2]).astype(int)
    v = np.rint(info.k[4] * points[:, 1] / points[:, 2] + info.k[5]).astype(int)
    for x, y in zip(u, v):
        if 0 <= x < image.width and 0 <= y < image.height:
            cv2.circle(canvas, (int(x), int(y)), 3, color, -1)
    cv2.putText(canvas, f"Livox projected into OAK ({window['capture']})",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return cv2.imwrite(str(filename), canvas)


def icp(windows, initial, max_delta_m=0.20, max_delta_deg=12.0):
    """Coarse-to-fine trimmed point-to-point ICP, with one shared pose for all windows."""
    pose = initial.copy()
    trees = [cKDTree(window["oak"]) for window in windows]
    history = []
    for cutoff in (0.25, 0.15, 0.10, 0.07):
        for _ in range(15):
            pairs_src, pairs_dst = [], []
            for window, tree in zip(windows, trees):
                transformed = transform(window["livox"], pose)
                distances, nearest = tree.query(transformed, workers=-1)
                matched = np.flatnonzero(distances < cutoff)
                if len(matched) < 80:
                    continue
                # Drop the worst 25% even inside the gate to suppress occlusion
                # boundaries and OAK stereo depth outliers.
                matched = matched[np.argsort(distances[matched])[:int(len(matched) * 0.75)]]
                pairs_src.append(transformed[matched])
                pairs_dst.append(window["oak"][nearest[matched]])
            if not pairs_src:
                raise ValueError("Insufficient OAK/Livox overlap for ICP")
            src, dst = np.concatenate(pairs_src), np.concatenate(pairs_dst)
            if len(src) < 200:
                raise ValueError("Fewer than 200 point correspondences; not a valid calibration")
            update = rigid_fit(src, dst)
            proposed = update @ pose
            delta_m = np.linalg.norm(proposed[:3, 3] - initial[:3, 3])
            delta_deg = np.degrees(Rotation.from_matrix(
                proposed[:3, :3] @ initial[:3, :3].T).magnitude())
            if delta_m > max_delta_m or delta_deg > max_delta_deg:
                raise ValueError(f"ICP exceeded rough-seed limit: {delta_m:.2f} m / {delta_deg:.1f} deg")
            pose = proposed
            history.append({"gate_m": cutoff, "matches": len(src),
                            "rms_m": round(float(np.sqrt(np.mean(np.sum((transform(src, update) - dst) ** 2, axis=1)))), 4)})
            if np.linalg.norm(update[:3, 3]) < 1e-4 and Rotation.from_matrix(
                    update[:3, :3]).magnitude() < 1e-4:
                break
    return pose, history


def load_windows(root, step, scan_radius, voxel_size):
    oak_dir, livox_dir = root / "oak", root / "livox"
    a0, a1 = metadata_span(oak_dir)
    b0, b1 = metadata_span(livox_dir)
    start, end = max(a0, b0), min(a1, b1)
    centers = np.arange(start + 1.8, end - 1.0, step)
    if len(centers) < 3:
        raise ValueError("Need at least three overlapping stationary windows")
    oak_edges, lidar_edges = {}, {}
    infos = {}
    images = defaultdict(list)
    rgb_images = {}
    scans = defaultdict(list)
    for topic, raw, receipt_ns in read_messages(oak_dir):
        if topic == "/tf_static":
            for item in deserialize_message(raw, TFMessage).transforms:
                oak_edges[(item.header.frame_id, item.child_frame_id)] = tf_matrix(item)
        elif topic in (RGB_INFO, DEPTH_INFO) and topic not in infos:
            infos[topic] = deserialize_message(raw, CameraInfo)
        elif topic == DEPTH:
            receipt = receipt_ns / 1e9
            for index, center in enumerate(centers):
                if abs(receipt - center) < 0.27:
                    images[index].append((abs(receipt - center), deserialize_message(raw, Image)))
        elif topic == RGB:
            receipt = receipt_ns / 1e9
            for index, center in enumerate(centers):
                distance = abs(receipt - center)
                if distance < 0.3 and (index not in rgb_images or distance < rgb_images[index][0]):
                    rgb_images[index] = (distance, deserialize_message(raw, Image))
    for topic, raw, receipt_ns in read_messages(livox_dir):
        if topic == "/tf_static":
            for item in deserialize_message(raw, TFMessage).transforms:
                lidar_edges[(item.header.frame_id, item.child_frame_id)] = tf_matrix(item)
        elif topic == LIDAR:
            receipt = receipt_ns / 1e9
            for index, center in enumerate(centers):
                if abs(receipt - center) <= scan_radius:
                    scans[index].append(deserialize_message(raw, PointCloud2))
    if not {RGB_INFO, DEPTH_INFO}.issubset(infos):
        raise ValueError("OAK bag is missing depth and/or RGB CameraInfo")
    info = infos[RGB_INFO]
    if info.header.frame_id != infos[DEPTH_INFO].header.frame_id or (info.width, info.height) != (
            infos[DEPTH_INFO].width, infos[DEPTH_INFO].height):
        raise ValueError("OAK depth is not aligned to RGB; need the depth optical intrinsics instead")
    seed = recorded_seed(oak_edges, lidar_edges, info.header.frame_id)
    windows = []
    for index, center in enumerate(centers):
        frames = [frame for _, frame in sorted(images[index], key=lambda x: x[0])[:3]]
        if not frames or len(scans[index]) < 5:
            continue
        oak_cloud, depth = depth_points(frames, info, 4, 0.7, 3.5)
        oak = voxel(oak_cloud, voxel_size)
        lidar = voxel(np.concatenate([cloud_xyz(scan) for scan in scans[index]]), voxel_size)
        lidar = crop_livox(lidar, seed, info, 0.7, 3.5)
        if len(oak) < 300 or len(lidar) < 100:
            continue
        windows.append({"capture": root.name, "time": center - start, "oak": oak, "livox": lidar,
                        "depth": depth, "rgb": rgb_images.get(index, (None, None))[1],
                        "depth_frames": len(frames), "scans": len(scans[index])})
    if len(windows) < 3:
        raise ValueError("Too few windows with overlapping OAK depth and Livox points")
    return windows, seed, info


def mount_from_livox(oak_bag, livox_bag, oak_from_livox):
    """Compose the proposed single camera_link -> oak-d-base-frame mount TF."""
    edges = {}
    for key, bag in (("oak", oak_bag), ("livox", livox_bag)):
        collected = {}
        for topic, raw, _ in read_messages(bag):
            if topic != "/tf_static":
                continue
            for item in deserialize_message(raw, TFMessage).transforms:
                collected[(item.header.frame_id, item.child_frame_id)] = tf_matrix(item)
            if key == "oak" and ("oak_rgb_camera_frame", "oak_rgb_camera_optical_frame") in collected:
                break
            if key == "livox" and ("camera_link", "oak-d-base-frame") in collected:
                break
        edges[key] = collected
    robot = edges["livox"]
    torso_camera = chain(robot, "torso_link", "d435_link", "camera_link")
    torso_livox = chain(robot, "torso_link", "mid360_link", "livox_frame")
    base_optical = chain(edges["oak"], "oak-d-base-frame", "oak",
                         "oak_rgb_camera_frame", "oak_rgb_camera_optical_frame")
    return np.linalg.inv(torso_camera) @ torso_livox @ np.linalg.inv(oak_from_livox) @ np.linalg.inv(base_optical)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="+", type=Path,
                        help="one or more independently positioned sessions, each with oak/ and livox/ MCAP bags")
    parser.add_argument("--window-step", type=float, default=4.0)
    parser.add_argument("--scan-radius", type=float, default=0.7)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--candidate", type=Path,
                        help="evaluate a pre-fitted registration on independent captures without refitting")
    parser.add_argument("--output", type=Path, help="default <capture>/registration.yaml")
    args = parser.parse_args()
    datasets = [load_windows(path, args.window_step, args.scan_radius, args.voxel_size)
                for path in args.capture]
    seed, info = datasets[0][1:]
    identities = set()
    for path, (_, candidate, intrinsics) in zip(args.capture, datasets):
        if np.linalg.norm(candidate - seed) > 1e-6:
            raise ValueError(f"The recorded TF seed differs in {path}; do not merge changing mounts")
        if (intrinsics.width, intrinsics.height, intrinsics.header.frame_id) != (
                info.width, info.height, info.header.frame_id) or not np.allclose(intrinsics.k, info.k):
            raise ValueError(f"The OAK intrinsics differ in {path}; do not merge different cameras")
        manifest = path / "capture_info.yaml"
        if manifest.is_file():
            with manifest.open() as stream:
                identities.add(yaml.safe_load(stream).get("oak", {}).get("mxid_reported_by_driver"))
    if len(identities) > 1:
        raise ValueError("Captures report different OAK MXIDs; one rigid extrinsic cannot fit them")
    if len(datasets) == 1:
        train, held_out = datasets[0][0][:-2], datasets[0][0][-2:]
        heldout_note = "same stationary viewpoint: temporal holdout is NOT independent spatial validation"
    else:
        train = [window for windows, _, _ in datasets[:-1] for window in windows]
        held_out = datasets[-1][0]
        heldout_note = "last entire capture reserved for independent spatial validation"
    if args.candidate:
        with args.candidate.open() as stream:
            prior = yaml.safe_load(stream)
        if prior.get("depth_frame") != info.header.frame_id or prior.get("lidar_frame") != "livox_frame":
            raise ValueError("Candidate TF frame names do not match this recording")
        improved = np.asarray(prior["fitted_oak_from_livox_matrix"], dtype=np.float64)
        if improved.shape != (4, 4) or not np.allclose(improved[3], [0, 0, 0, 1]):
            raise ValueError("Invalid candidate transform matrix")
        iterations = []
        heldout_note = "candidate from earlier session; no ICP refitting on validation captures"
    else:
        improved, iterations = icp(train, seed)
    delta_m = float(np.linalg.norm(improved[:3, 3] - seed[:3, 3]))
    delta_deg = float(np.degrees(Rotation.from_matrix(
        improved[:3, :3] @ seed[:3, :3].T).magnitude()))
    report = {
        "captures": [str(path.resolve()) for path in args.capture],
        "fit_mode": "fixed_candidate_validation" if args.candidate else "trimmed_point_to_point_icp",
        "candidate_source": str(args.candidate.resolve()) if args.candidate else None,
        "depth_frame": info.header.frame_id,
        "lidar_frame": "livox_frame", "depth_encoding": "16UC1", "depth_units": "mm",
        "window_note": heldout_note,
        "training_windows": len(train), "heldout_windows": len(held_out),
        "training_before": registration_errors(train, seed, info),
        "training_after": registration_errors(train, improved, info),
        "holdout_before": registration_errors(held_out, seed, info),
        "holdout_after": registration_errors(held_out, improved, info),
        "seed_oak_from_livox_matrix": seed.tolist(),
        "fitted_oak_from_livox_matrix": improved.tolist(),
        "fitted_livox_from_oak_matrix": np.linalg.inv(improved).tolist(),
        "correction_from_seed_m": round(delta_m, 4),
        "correction_from_seed_deg": round(delta_deg, 3),
        "icp_last_iteration": iterations[-1] if iterations else None,
        "proposed_mount_parent": "camera_link", "proposed_mount_child": "oak-d-base-frame",
        "ready_for_tf_publication": False,
        "reasons": (["Need >=3 physically different viewpoints, with the last held out"]
                    if len(datasets) < 3 else ["Inspect independent depth/point-cloud overlays and mount identity before replacing TF"]),
    }
    mount = mount_from_livox(args.capture[0] / "oak", args.capture[0] / "livox", improved)
    report["proposed_mount_xyz_m"] = mount[:3, 3].tolist()
    report["proposed_mount_rpy_rad"] = Rotation.from_matrix(mount[:3, :3]).as_euler("xyz").tolist()
    report["proposed_mount_quaternion_xyzw"] = Rotation.from_matrix(mount[:3, :3]).as_quat().tolist()
    output = args.output or args.capture[0] / "registration.yaml"
    with output.open("w") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    heldout = held_out[0]
    before_image = output.parent / f"{heldout['capture']}_livox_before.png"
    after_image = output.parent / f"{heldout['capture']}_livox_after.png"
    if save_overlay(heldout, seed, info, before_image, (0, 0, 255)):
        save_overlay(heldout, improved, info, after_image, (0, 255, 0))
    if args.candidate and len(datasets) > 2:
        other_view = datasets[-2][0][0]
        save_overlay(other_view, seed, info,
                     output.parent / f"{other_view['capture']}_livox_before.png", (0, 0, 255))
        save_overlay(other_view, improved, info,
                     output.parent / f"{other_view['capture']}_livox_after.png", (0, 255, 0))
    print(yaml.safe_dump({k: report[k] for k in (
        "training_windows", "heldout_windows", "correction_from_seed_m",
        "correction_from_seed_deg", "holdout_before", "holdout_after",
        "ready_for_tf_publication", "reasons")}, sort_keys=False))
    print("Report:", output)


if __name__ == "__main__":
    main()
