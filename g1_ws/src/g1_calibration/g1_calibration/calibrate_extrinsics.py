"""Extrinsic calibration of the chest OAK-D against the G1's head RealSense and MID-360 LiDAR.

    ros2 run g1_calibration calibrate_extrinsics <dataset> [--config calibration.yaml]
        [--oak-intrinsics oak.yaml] [--realsense-intrinsics rs.yaml]

Two independent estimates of T_lidar_oak (LiDAR frame <- OAK optical frame):

1. via_realsense: both cameras see the board in the same sample -> T_realsense_oak
   (cv2.stereoCalibrate, intrinsics fixed), chained with T_lidar_realsense from the URDF /tf
   (g1_sensors tf_chain + the RealSense driver) recorded at capture.
2. lidar_planes: board plane from the OAK (PnP) vs the board points in the accumulated LiDAR
   cloud, over all samples (solvers.calibrate_camera_lidar). Independent of the URDF.

Their agreement is the check. The chosen one becomes the static transform
body (torso_link) -> OAK root frame for g1_sensors/config/g1_sensors.yaml.
Writes <dataset>/results/{extrinsics.yaml, report.md, overlay_*.png}.
"""
import argparse
import datetime
import os

import cv2
import numpy as np
import yaml

from .board import Board, board_plane, board_pose, detect_corners, draw, view_angle_deg
from .dataset import Intrinsics, load_config, load_dataset
from .geometry import (T_BODY_OPTICAL, from_dict, invert, to_dict, transform_difference,
                       average_transforms)
from .lidar_board import extract_board, fraction_inside
from .solvers import CalibrationError, calibrate_camera_lidar, calibrate_camera_pair

CAM, REF = "oak", "realsense"


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("g1_calibration"), "config",
                        "calibration.yaml")


def detect(samples, board, max_px, overrides):
    """detections[sample][camera] = dict(corners, T (camera <- board), rms, intr) + notes."""
    det, notes = {}, []
    for s in samples:
        det[s.name] = {}
        for cam, img in s.images.items():
            intr = overrides.get(cam) or s.intrinsics.get(cam)
            if intr is None:
                notes.append(f"{s.name}/{cam}: no camera_info")
                continue
            corners = detect_corners(img, board)
            if corners is None:
                notes.append(f"{s.name}/{cam}: board not found")
                continue
            tf, rms = board_pose(corners, board, intr.k, intr.d)
            if rms > max_px:
                notes.append(f"{s.name}/{cam}: reprojection {rms:.2f} px > {max_px} px, dropped")
                continue
            det[s.name][cam] = {"corners": corners, "T": tf, "rms": rms, "intr": intr}
    return det, notes


def sample_transform(samples, parent, child, fallback=None):
    """T_parent_child: mean of the per-sample TF recorded at capture, else the fallback."""
    found = [t for t in (s.transform(parent, child) for s in samples) if t is not None]
    if found:
        mean = average_transforms(found)
        spread = max(transform_difference(mean, t)[0] for t in found)
        return mean, f"/tf at capture ({len(found)} samples, spread {spread * 1000:.1f} mm)"
    if fallback is not None:
        return fallback, "config fallback_transforms (URDF)"
    return None, "missing"


def via_realsense(samples, det, board, t_lidar_ref, lidar_frame):
    pairs = [s for s in samples if CAM in det[s.name] and REF in det[s.name]]
    obj = [board.object_points()] * len(pairs)
    a = [det[s.name][CAM] for s in pairs]
    b = [det[s.name][REF] for s in pairs]
    res = calibrate_camera_pair(obj, [x["corners"] for x in a], [x["corners"] for x in b],
                                a[0]["intr"].k, a[0]["intr"].d, b[0]["intr"].k, b[0]["intr"].d,
                                a[0]["intr"].size, [x["T"] for x in a], [x["T"] for x in b])
    res["T_ref_cam"] = res.pop("T_b_a")
    res["T_lidar_cam"] = t_lidar_ref @ res["T_ref_cam"]
    res["samples"] = [s.name for s in pairs]
    return res


def lidar_planes(samples, det, board, t_lidar_cam_init, ext_cfg, rng_seed=0):
    t_lidar_cam = t_lidar_cam_init
    rng = np.random.default_rng(rng_seed)
    used, sol, skipped = [], None, {}
    for stage in ext_cfg["lidar_search"]:
        obs, used, skipped = [], [], {}
        for s in samples:
            d = det[s.name].get(CAM)
            if d is None or s.lidar is None or not len(s.lidar):
                continue
            t_lidar_board = t_lidar_cam @ d["T"]
            found = extract_board(s.lidar, t_lidar_board, board, margin=stage["margin"],
                                  depth_margin=stage["depth_margin"],
                                  threshold=ext_cfg["plane_threshold"],
                                  min_points=ext_cfg["min_board_points"], rng=rng)
            if "error" in found:
                skipped[s.name] = found["error"]
                continue
            n_c, d_c = board_plane(d["T"])
            obs.append({"n_c": n_c, "d_c": d_c, "points": found["points"],
                        "lidar_rms": found["rms"]})
            used.append(s.name)
        sol = calibrate_camera_lidar(obs)
        t_lidar_cam = invert(sol["T_cam_lidar"])
    sol["T_lidar_cam"] = t_lidar_cam
    sol["samples"] = used
    sol["skipped"] = skipped
    sol["board_points"] = {name: len(o["points"]) for name, o in zip(used, obs)}
    sol["inside_fraction"] = {
        name: fraction_inside(o["points"], t_lidar_cam @ det[name][CAM]["T"], board)
        for name, o in zip(used, obs)}
    sol["observations"] = dict(zip(used, obs))
    return sol


def overlay(image, intr, t_cam_lidar, lidar, board_points=None, max_range=6.0):
    """LiDAR points projected into the camera image, coloured by distance; board points green."""
    out = image.copy()
    pts = lidar @ t_cam_lidar[:3, :3].T + t_cam_lidar[:3, 3]
    front = (pts[:, 2] > 0.2) & (pts[:, 2] < max_range)
    uv, _ = cv2.projectPoints(pts[front], np.zeros(3), np.zeros(3), intr.k, intr.d)
    depth = pts[front, 2]
    colors = cv2.applyColorMap((np.clip(depth / max_range, 0, 1) * 255).astype(np.uint8),
                               cv2.COLORMAP_JET).reshape(-1, 3)
    h, w = out.shape[:2]
    for (u, v), c in zip(uv.reshape(-1, 2), colors):
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(out, (int(u), int(v)), 1, tuple(int(x) for x in c), -1)
    if board_points is not None and len(board_points):
        bp = board_points @ t_cam_lidar[:3, :3].T + t_cam_lidar[:3, 3]
        uv, _ = cv2.projectPoints(bp, np.zeros(3), np.zeros(3), intr.k, intr.d)
        for u, v in uv.reshape(-1, 2):
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(out, (int(u), int(v)), 2, (0, 255, 0), -1)
    return out


def run(dataset, cfg, overrides=None):
    """Full extrinsic calibration. Returns the results dict (also the YAML content)."""
    overrides = overrides or {}
    board = Board.from_config(cfg["board"])
    ext_cfg, quality = cfg["extrinsics"], cfg["quality"]
    samples = load_dataset(dataset)
    det, notes = detect(samples, board, ext_cfg["max_reprojection_px"], overrides)

    lidar_frame = cfg["lidar"]["frame"]
    body = cfg["frames"]["body"]
    fb = cfg.get("fallback_transforms") or {}
    t_body_lidar, src_body_lidar = sample_transform(
        samples, body, lidar_frame, from_dict(fb["body_to_lidar"]) if fb.get("body_to_lidar")
        else None)
    if t_body_lidar is None:
        raise CalibrationError(f"no {body} <- {lidar_frame} transform: run g1_sensors tf_chain "
                               "while capturing or set fallback_transforms.body_to_lidar")
    cam_frames = {cam: next((s.frames[cam] for s in samples if s.frames.get(cam)), None)
                  for cam in (CAM, REF)}
    fb_ref = None
    if fb.get("body_to_realsense_optical") and t_body_lidar is not None:
        fb_ref = invert(t_body_lidar) @ from_dict(fb["body_to_realsense_optical"])
    t_lidar_ref, src_lidar_ref = sample_transform(samples, lidar_frame, cam_frames[REF], fb_ref)

    results = {"generated": datetime.datetime.now().isoformat(timespec="seconds"),
               "dataset": os.path.abspath(dataset), "board": cfg["board"],
               "frames": {"lidar": lidar_frame, "body": body, "oak_optical": cam_frames[CAM],
                          "realsense_optical": cam_frames[REF]},
               "samples": len(samples),
               "detections": {cam: sum(cam in d for d in det.values()) for cam in (CAM, REF)},
               "reprojection_px": {cam: [round(d[cam]["rms"], 3) for d in det.values()
                                         if cam in d] for cam in (CAM, REF)},
               "sources": {"body_to_lidar": src_body_lidar,
                           "lidar_to_realsense": src_lidar_ref},
               "notes": list(notes), "methods": {}}

    # 1. OAK -> RealSense -> LiDAR
    rs = None
    if t_lidar_ref is None:
        results["notes"].append("via_realsense skipped: no LiDAR <- RealSense transform "
                                "(run g1_sensors tf_chain + the RealSense driver while capturing)")
    else:
        try:
            rs = via_realsense(samples, det, board, t_lidar_ref, lidar_frame)
            results["methods"]["via_realsense"] = {
                "T_lidar_oak": to_dict(rs["T_lidar_cam"], lidar_frame, cam_frames[CAM]),
                "T_realsense_oak": to_dict(rs["T_ref_cam"], cam_frames[REF], cam_frames[CAM]),
                "stereo_rms_px": round(rs["rms_px"], 3), "views": rs["views"],
                "per_view_spread": {k: round(v, 4) for k, v in rs.get("spread", {}).items()}}
        except CalibrationError as e:
            results["notes"].append(f"via_realsense failed: {e}")

    # 2. OAK <-> LiDAR board planes (seeded by method 1 or the tape-measured guess)
    if rs is not None:
        t_init, init_src = rs["T_lidar_cam"], "via_realsense"
    else:
        guess = from_dict(cfg["initial_guess"]["body_to_oak_body"]) @ T_BODY_OPTICAL
        t_init, init_src = invert(t_body_lidar) @ guess, "initial_guess (tape measure)"
    lp = None
    try:
        lp = lidar_planes(samples, det, board, t_init, ext_cfg)
        results["methods"]["lidar_planes"] = {
            "T_lidar_oak": to_dict(lp["T_lidar_cam"], lidar_frame, cam_frames[CAM]),
            "seeded_by": init_src, "views": len(lp["samples"]),
            "plane_rms_m": round(lp["rms_m"], 4),
            "per_view_rms_m": [round(v, 4) for v in lp["per_view_rms_m"]],
            "normal_singular_values": [round(v, 3) for v in lp["condition"]],
            "board_points": lp["board_points"],
            "inside_board_fraction": {k: round(v, 3) for k, v in lp["inside_fraction"].items()},
            "skipped": lp["skipped"]}
    except CalibrationError as e:
        results["notes"].append(f"lidar_planes failed: {e}")

    # 3. compare + choose
    if rs is not None and lp is not None:
        dt, dr = transform_difference(rs["T_lidar_cam"], lp["T_lidar_cam"])
        results["agreement"] = {"translation_m": round(dt, 4), "rotation_deg": round(dr, 3),
                                "ok": dt <= quality["methods_agree_m"]
                                and dr <= quality["methods_agree_deg"]}
    choice = ext_cfg.get("final", "auto")
    if choice == "auto":
        lp_ok = lp is not None and lp["rms_m"] <= quality["lidar_plane_rms_m"]
        choice = "lidar_planes" if lp_ok else ("via_realsense" if rs is not None else None)
        if lp is not None and not lp_ok:
            results["notes"].append(
                f"lidar_planes RMS {lp['rms_m']:.3f} m above {quality['lidar_plane_rms_m']} m")
    chosen = {"lidar_planes": lp, "via_realsense": rs}.get(choice)
    if chosen is None:
        raise CalibrationError("no method succeeded:\n  " + "\n  ".join(results["notes"]))
    t_lidar_cam = chosen["T_lidar_cam"]
    results["final"] = {"method": choice,
                        "T_lidar_oak": to_dict(t_lidar_cam, lidar_frame, cam_frames[CAM])}

    # 4. the static transform for g1_sensors: body -> OAK root frame (the driver owns the rest)
    oak_root = cfg["frames"].get("oak_root")
    t_root_opt, src_root = sample_transform(samples, oak_root, cam_frames[CAM]) \
        if oak_root else (None, "missing")
    t_body_opt = t_body_lidar @ t_lidar_cam
    if t_root_opt is not None:
        st = to_dict(t_body_opt @ invert(t_root_opt), body, oak_root)
        results["sources"]["oak_root_to_optical"] = src_root
    else:
        st = to_dict(t_body_opt, body, cam_frames[CAM])
        results["notes"].append(
            f"no /tf {oak_root} -> {cam_frames[CAM]} at capture: the static transform below "
            "targets the OPTICAL frame. If the depthai-ros driver also publishes that frame, "
            "re-capture with its TF running so the result targets its root frame instead.")
    results["static_transform_for_g1_sensors"] = {
        "parent": st["parent"], "child": st["child"], "xyz": st["xyz"], "rpy": st["rpy"]}

    quality_rows = []
    for cam in (CAM, REF):
        errs = results["reprojection_px"][cam]
        if errs:
            quality_rows.append((f"{cam} reprojection RMS (max) px", max(errs),
                                 quality["reprojection_px"]))
    if lp is not None:
        quality_rows.append(("LiDAR plane RMS m", lp["rms_m"], quality["lidar_plane_rms_m"]))
    results["quality"] = [{"check": c, "value": round(float(v), 4), "limit": lim,
                           "ok": bool(v <= lim)} for c, v, lim in quality_rows]
    results["_internal"] = {"samples": samples, "det": det, "lp": lp,
                            "t_lidar_cam": t_lidar_cam}
    return results


def report(results):
    fin = results["final"]
    lines = ["# OAK-D extrinsic calibration", "",
             f"Dataset `{results['dataset']}`, {results['samples']} samples, "
             f"{results['generated']}.", "",
             f"Board detections: OAK {results['detections'][CAM]}, "
             f"RealSense {results['detections'][REF]}.", "",
             f"**Final ({fin['method']})** `T_{results['frames']['lidar']}_oak`: "
             f"xyz {fin['T_lidar_oak']['xyz']}, rpy {fin['T_lidar_oak']['rpy']}", "",
             "Static transform for `g1_sensors/config/g1_sensors.yaml` `static_transforms`:", "",
             "```yaml"]
    st = results["static_transform_for_g1_sensors"]
    lines += [f"  - {{parent: {st['parent']}, child: {st['child']}, xyz: {st['xyz']}, "
              f"rpy: {st['rpy']}}}  # g1_calibration {results['generated'][:10]}", "```", ""]
    if "agreement" in results:
        a = results["agreement"]
        lines += [f"Methods agree within **{a['translation_m'] * 100:.1f} cm / "
                  f"{a['rotation_deg']:.2f} deg** ({'OK' if a['ok'] else 'CHECK'}).", ""]
    lines += ["| check | value | limit | ok |", "|---|---|---|---|"]
    lines += [f"| {q['check']} | {q['value']} | {q['limit']} | {'yes' if q['ok'] else 'NO'} |"
              for q in results["quality"]]
    for name, m in results["methods"].items():
        lines += ["", f"## {name}", "", "```yaml",
                  yaml.safe_dump(m, sort_keys=False, default_flow_style=None).rstrip(), "```"]
    lines += ["", "## Sources", ""] + [f"- {k}: {v}" for k, v in results["sources"].items()]
    if results["notes"]:
        lines += ["", "## Notes", ""] + [f"- {n}" for n in results["notes"]]
    lines += ["", "Check `overlay_*.png`: LiDAR points (coloured by distance) must line up with "
              "the image edges; board points are green.", ""]
    return "\n".join(lines)


def write_outputs(dataset, results):
    out = os.path.join(dataset, "results")
    os.makedirs(out, exist_ok=True)
    internal = results.pop("_internal")
    with open(os.path.join(out, "extrinsics.yaml"), "w") as f:
        yaml.safe_dump(results, f, sort_keys=False, default_flow_style=None)
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write(report(results))
    t_cam_lidar = invert(internal["t_lidar_cam"])
    lp = internal["lp"]
    for s in internal["samples"]:
        d = internal["det"][s.name].get(CAM)
        if d is None or s.lidar is None or CAM not in s.images:
            continue
        bp = lp["observations"][s.name]["points"] if lp and s.name in lp["observations"] else None
        img = draw(s.images[CAM], d["corners"], Board.from_config(results["board"]),
                   f"{s.name} {d['rms']:.2f}px {view_angle_deg(d['T']):.0f}deg")
        cv2.imwrite(os.path.join(out, f"overlay_{s.name}.png"),
                    overlay(img, d["intr"], t_cam_lidar, s.lidar, bp))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dataset")
    ap.add_argument("--config", help="calibration.yaml (default: the installed one)")
    ap.add_argument("--oak-intrinsics", help="camera_info YAML to use instead of the driver's")
    ap.add_argument("--realsense-intrinsics", help="camera_info YAML for the RealSense")
    args = ap.parse_args(argv)
    cfg = load_config(args.config or default_config())
    overrides = {}
    if args.oak_intrinsics:
        overrides[CAM] = Intrinsics.from_file(args.oak_intrinsics)
    if args.realsense_intrinsics:
        overrides[REF] = Intrinsics.from_file(args.realsense_intrinsics)
    try:
        results = run(args.dataset, cfg, overrides)
    except (CalibrationError, FileNotFoundError) as e:
        raise SystemExit(f"calibration failed: {e}")
    out = write_outputs(args.dataset, results)
    print(report(results))
    print(f"Written to {out}")
    return results


if __name__ == "__main__":
    main()
