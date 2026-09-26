"""Intrinsic calibration of one camera from chessboard views, compared to its factory values.

    ros2 run g1_calibration calibrate_intrinsics <dataset> [--camera oak] [--images DIR]
        [--config calibration.yaml] [--rational]

Uses <camera>.png of every sample (plus any *.png / *.jpg in --images) and OpenCV
calibrateCamera. Writes <dataset>/results/<camera>_intrinsics.yaml (ROS camera_info format, for
the driver's camera_info_url / depthai-ros `i_calibration_file` / set_camera_info) and prints
the reprojection error of the calibrated AND of the factory intrinsics (the driver's
camera_info in the samples) on the same views.

The OAK-D is factory-calibrated, and its on-device depth-to-RGB alignment uses the EEPROM
calibration. Keep the factory intrinsics unless they are clearly worse; to change them on the
device use Luxonis' calibration tool (it rewrites the EEPROM).
"""
import argparse
import glob
import os

import cv2
import yaml

from .board import Board, detect_corners
from .calibrate_extrinsics import default_config
from .dataset import camera_info_to_yaml, load_config, load_dataset
from .solvers import CalibrationError, calibrate_intrinsics, reprojection_rms


def collect(dataset, camera, board, extra_dir=None):
    views, factory, size = [], None, None
    for s in load_dataset(dataset):
        if camera in s.images:
            views.append((s.name, s.images[camera]))
            factory = factory or s.intrinsics.get(camera)
    if extra_dir:
        for p in sorted(glob.glob(os.path.join(extra_dir, "*.png"))
                        + glob.glob(os.path.join(extra_dir, "*.jpg"))):
            views.append((os.path.basename(p), cv2.imread(p, cv2.IMREAD_COLOR)))
    obj, img, names = [], [], []
    for name, image in views:
        if size is None:
            size = (image.shape[1], image.shape[0])
        if (image.shape[1], image.shape[0]) != size:
            raise CalibrationError(f"{name}: image size differs from the first view")
        corners = detect_corners(image, board)
        if corners is not None:
            obj.append(board.object_points())
            img.append(corners)
            names.append(name)
    return obj, img, names, size, factory, len(views)


def run(dataset, cfg, camera="oak", extra_dir=None, rational=False):
    board = Board.from_config(cfg["board"])
    obj, img, names, size, factory, total = collect(dataset, camera, board, extra_dir)
    res = calibrate_intrinsics(obj, img, size, rational=rational)
    out = {"camera": camera, "views_used": len(names), "views_total": total,
           "image_size": list(size), "rms_px": round(res["rms"], 4),
           "per_view_rms_px": dict(zip(names, [round(v, 4) for v in res["per_view_rms"]])),
           "k": res["k"], "d": res["d"]}
    if factory is not None and factory.size == tuple(size):
        out["factory_rms_px"] = round(reprojection_rms(obj, img, factory.k, factory.d), 4)
        out["factory_model"] = factory.distortion_model
    return out


def recommendation(out):
    if "factory_rms_px" not in out:
        return "No factory camera_info in the samples to compare with."
    f, c = out["factory_rms_px"], out["rms_px"]
    if f <= max(1.5 * c, c + 0.15):
        return (f"Keep the FACTORY intrinsics: {f:.3f} px vs {c:.3f} px calibrated. The on-device "
                "depth alignment uses them too.")
    return (f"The factory intrinsics fit worse ({f:.3f} px vs {c:.3f} px). Check the board "
            "(flat? square size measured?) and the views; if it holds, use the calibrated file "
            "(--oak-intrinsics) and consider re-calibrating the device with Luxonis' tool.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dataset")
    ap.add_argument("--camera", default="oak")
    ap.add_argument("--images", help="extra directory of board images (same camera + size)")
    ap.add_argument("--config", help="calibration.yaml (default: the installed one)")
    ap.add_argument("--rational", action="store_true",
                    help="8-coefficient rational model (wide-angle lenses)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config or default_config())
    try:
        out = run(args.dataset, cfg, args.camera, args.images, args.rational)
    except (CalibrationError, FileNotFoundError) as e:
        raise SystemExit(f"calibration failed: {e}")
    res_dir = os.path.join(args.dataset, "results")
    os.makedirs(res_dir, exist_ok=True)
    w, h = out["image_size"]
    info = camera_info_to_yaml(w, h, out["k"], out["d"],
                               "rational_polynomial" if args.rational else "plumb_bob",
                               args.camera)
    path = os.path.join(res_dir, f"{args.camera}_intrinsics.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(info, f, sort_keys=False)
    summary = {k: v for k, v in out.items() if k not in ("k", "d")}
    summary["recommendation"] = recommendation(out)
    with open(os.path.join(res_dir, f"{args.camera}_intrinsics_report.yaml"), "w") as f:
        yaml.safe_dump(summary, f, sort_keys=False)
    print(yaml.safe_dump(summary, sort_keys=False))
    print(f"camera_info written to {path}")
    return out


if __name__ == "__main__":
    main()
