"""Replay a recorded glasses AprilTag registration through the bridge's own code (no ROS).

The bridge records every registration attempt (``record_dir``, default
``/ws/bags/ar_registration`` on the robot laptop, ``./ar_registration`` for ``sim_main``).
This runs the recorded frames through exactly the same detection, multi-view estimate and
commit rules, prints the bridge's diagnosis for every frame, and says why it did or did not
register. Thresholds can be changed to see what would have worked:

    python3 -m g1_ar_bridge.replay_registration bags/ar_registration/20260926-193512
    python3 -m g1_ar_bridge.replay_registration <dir> --max-rms-px 5 --min-baseline-m 0.15
    python3 -m g1_ar_bridge.replay_registration <dir> --overlays /tmp/ov   # annotated frames

Without an anchor in the recording (the robot had not measured the tag), ``--fake-anchor``
puts a wall tag 1.5 m in front of the map origin so the glasses side can still be judged.
"""
import argparse
import asyncio
import dataclasses
import glob
import json
import os
import sys

import cv2
import numpy as np

from .apriltag import project, tag_object_points
from .geometry import FLIP_YZ, inv_T, pose_to_T
from .server import ArBridgeServer, BridgeConfig
from .world import World, wall_tag_pose

OVERRIDES = ("min_views", "min_baseline_m", "max_view_reproj_px", "max_rms_px", "max_tilt_deg",
             "tag_black_size_m")


class ReplayWorld(World):
    def __init__(self, anchor):
        self.T_anchor = None if anchor is None else np.asarray(anchor, float)

    def anchor(self, tag_id):
        return self.T_anchor


def load_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def resolve_dir(path):
    """A registration directory, or the newest one inside a record_dir."""
    if os.path.exists(os.path.join(path, "frames.jsonl")):
        return path
    subdirs = sorted(d for d in glob.glob(os.path.join(path, "*"))
                     if os.path.exists(os.path.join(d, "frames.jsonl")))
    if not subdirs:
        sys.exit(f"no recorded registration (frames.jsonl) in {path}")
    return subdirs[-1]


def recorded_anchor(session, frames):
    for fr in reversed(frames):
        if fr.get("anchor_T_map_tag") is not None:
            return fr["anchor_T_map_tag"]
    return session.get("anchor_T_map_tag")


def draw_overlay(path_in, path_out, fr, detections, tag_id, size_m, T_world_tag):
    img = cv2.imread(path_in)
    if img is None:
        return
    for i, corners in detections:
        c = np.asarray(corners, float).reshape(4, 2)
        colour = (0, 200, 0) if i == tag_id else (0, 200, 200)
        cv2.polylines(img, [c.astype(np.int32)], True, colour, 2)
        cv2.putText(img, f"id {i}", tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    colour, 2)
    if T_world_tag is not None:
        K = np.asarray(fr["K"], float)
        h, w = img.shape[:2]
        if (w, h) != (fr["width"], fr["height"]):
            sx, sy = w / float(fr["width"]), h / float(fr["height"])
            K = np.array([[K[0, 0] * sx, 0, K[0, 2] * sx], [0, K[1, 1] * sy, K[1, 2] * sy],
                          [0, 0, 1.0]])
        hd = fr["header"]
        T_world_cv = pose_to_T(hd["cam_pos"], hd["cam_rot"]) @ FLIP_YZ
        pts = tag_object_points(size_m) @ T_world_tag[:3, :3].T + T_world_tag[:3, 3]
        uv, z = project(K, inv_T(T_world_cv), pts)
        if np.all(z > 0):
            cv2.polylines(img, [uv.astype(np.int32)], True, (0, 0, 255), 1)
    cv2.putText(img, f"{fr['file']}: {fr['result'].get('reason')}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(path_out, img)


async def replay(args):
    folder = resolve_dir(args.dir)
    with open(os.path.join(folder, "session.json")) as f:
        session = json.load(f)
    frames = load_jsonl(os.path.join(folder, "frames.jsonl"))
    events = load_jsonl(os.path.join(folder, "events.jsonl"))
    fields = {f.name for f in dataclasses.fields(BridgeConfig)}
    cfg_dict = {k: (tuple(v) if isinstance(v, list) else v)
                for k, v in session.get("config", {}).items() if k in fields}
    cfg_dict["record_dir"] = ""
    for name in OVERRIDES:
        value = getattr(args, name)
        if value is not None:
            cfg_dict[name] = value
    cfg = BridgeConfig(**cfg_dict)

    anchor = recorded_anchor(session, frames)
    if anchor is None and args.fake_anchor:
        anchor = wall_tag_pose(1.5, 1.0, -cfg.base_height_m)
        print("no robot anchor recorded: using a fake wall tag 1.5 m in front of the map origin")
    print(f"replaying {folder}: {len(frames)} frames, started {session.get('started')}")
    print(f"robot anchor in the recording: {'yes' if recorded_anchor(session, frames) else 'NO'}")
    for ev in events:
        if ev["event"] in ("failed", "stopped", "registered"):
            print(f"recorded outcome: {ev['event']} {ev.get('why', '')}".rstrip())
    print("thresholds: " + ", ".join(f"{k}={getattr(cfg, k)}" for k in OVERRIDES))
    print("-" * 100)

    bridge = ArBridgeServer(ReplayWorld(anchor), cfg, log=print)
    await bridge.on_registration_command(None, {"command": "start", "mode": "april_tag"})
    s = bridge.session
    if args.overlays:
        os.makedirs(args.overlays, exist_ok=True)
    for fr in frames:
        if bridge.session is not s:
            break
        with open(os.path.join(folder, fr["file"]), "rb") as f:
            jpeg = f.read()
        cam = (np.asarray(fr["K"], float), fr["width"], fr["height"])
        s["received"] = s.get("received", 0) + 1
        result = bridge.process_frame(s, fr["header"], jpeg, cam)
        s["diag_logged"] = -1e9                    # print every frame
        await bridge.update_tag_registration(s, result)
    print("-" * 100)
    if bridge.committed:
        print(f"RESULT: registers with these thresholds: {bridge.reg_info}")
    else:
        est = s.get("estimate")
        blocker = bridge.tag_blocker(est, bridge.world.anchor(cfg.tag_id), s.get("solution"))
        print(f"RESULT: does NOT register. Waiting for: {blocker}")
        if est is not None:
            print(f"  views used {est.n_views}/{est.n_total}, rms {est.rms_px:.2f} px, sideways "
                  f"{est.baseline_m:.2f} m, spread {est.pos_spread_m:.3f} m / "
                  f"{est.rot_spread_deg:.1f} deg")
            for v in s["est"].views:
                c = v.cam_centre
                print(f"  view: cam ({c[0]:+.2f}, {c[1]:+.2f}, {c[2]:+.2f}) m, tag at "
                      f"{np.round(v.T_world_tag[:3, 3], 3).tolist()}, {v.distance_m:.2f} m away, "
                      f"{v.side_px:.0f} px, single-view reproj {v.reproj_px:.2f} px")
    if args.overlays:
        est = s.get("estimate")
        T_tag = None if est is None else est.T_world_tag
        for fr in frames:
            img = cv2.imread(os.path.join(folder, fr["file"]), cv2.IMREAD_GRAYSCALE)
            dets = [] if img is None else bridge.detector.detect(img)
            draw_overlay(os.path.join(folder, fr["file"]), os.path.join(args.overlays, fr["file"]),
                         fr, dets, cfg.tag_id, cfg.tag_black_size_m, T_tag)
        print(f"overlays (green: detected tag, red: final estimate reprojected) in "
              f"{args.overlays}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="a recorded registration, or the record_dir (newest is used)")
    for name in OVERRIDES:
        ap.add_argument("--" + name.replace("_", "-"), type=float, default=None,
                        dest=name)
    ap.add_argument("--fake-anchor", action="store_true",
                    help="no robot anchor recorded: assume a wall tag 1.5 m ahead")
    ap.add_argument("--overlays", default="", help="write annotated frames to this directory")
    args = ap.parse_args(argv)
    if args.min_views is not None:
        args.min_views = int(args.min_views)
    asyncio.run(replay(args))


if __name__ == "__main__":
    main()
