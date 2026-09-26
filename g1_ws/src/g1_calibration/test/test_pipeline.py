"""End-to-end: synthetic dataset on disk -> calibrate_extrinsics / calibrate_intrinsics CLIs."""
import os

import numpy as np
import yaml

from g1_calibration import calibrate_extrinsics, calibrate_intrinsics, make_board
from g1_calibration.dataset import load_dataset
from g1_calibration.geometry import from_dict, invert, transform_difference

from synthetic import (BODY_LIDAR, BOARD, OAK, TRUE_BODY_OAK, TRUE_BODY_OAK_OPT, config,
                       write_dataset)

TRUE_LIDAR_OAK = invert(BODY_LIDAR) @ TRUE_BODY_OAK_OPT


def write_config(tmp_path, cfg=None):
    path = tmp_path / "calibration.yaml"
    path.write_text(yaml.safe_dump(cfg or config()))
    return str(path)


def test_dataset_roundtrip(tmp_path):
    write_dataset(str(tmp_path / "ds"), n=2)
    samples = load_dataset(str(tmp_path / "ds"))
    s = samples[0]
    assert set(s.images) == {"oak", "realsense"} and s.images["oak"].shape == (600, 800, 3)
    np.testing.assert_allclose(s.intrinsics["oak"].k, OAK.k)
    assert s.lidar.shape[1] == 3 and s.lidar_frame == "livox_frame"
    np.testing.assert_allclose(s.transform("torso_link", "livox_frame"), BODY_LIDAR, atol=1e-6)
    assert s.transform("map", "odom") is None


def test_full_extrinsic_calibration(tmp_path):
    ds = write_dataset(str(tmp_path / "ds"), n=12)
    res = calibrate_extrinsics.main([ds, "--config", write_config(tmp_path)])
    out = yaml.safe_load(open(os.path.join(ds, "results", "extrinsics.yaml")))
    assert out["final"]["method"] == "lidar_planes"
    for method in ("lidar_planes", "via_realsense"):
        t = from_dict(out["methods"][method]["T_lidar_oak"])
        dt, dr = transform_difference(t, TRUE_LIDAR_OAK)
        assert dt < 0.01 and dr < 0.3, (method, dt, dr)
    assert out["agreement"]["ok"]
    assert all(q["ok"] for q in out["quality"])
    # the static transform for g1_sensors targets the OAK driver's ROOT frame
    st = out["static_transform_for_g1_sensors"]
    assert (st["parent"], st["child"]) == ("torso_link", "oak-d-base-frame")
    dt, dr = transform_difference(from_dict(st), TRUE_BODY_OAK)
    assert dt < 0.01 and dr < 0.3
    report = open(os.path.join(ds, "results", "report.md")).read()
    assert "g1_sensors" in report and "lidar_planes" in report
    assert len([f for f in os.listdir(os.path.join(ds, "results"))
                if f.startswith("overlay_")]) == 12
    assert res["final"]["method"] == "lidar_planes"


def test_without_tf_or_realsense_uses_the_tape_measured_guess(tmp_path):
    ds = write_dataset(str(tmp_path / "ds"), n=12, seed=3, with_tf=False, with_realsense=False)
    out = calibrate_extrinsics.run(ds, config())
    assert "via_realsense" not in out["methods"]
    assert out["methods"]["lidar_planes"]["seeded_by"].startswith("initial_guess")
    assert out["sources"]["body_to_lidar"].startswith("config fallback")
    dt, dr = transform_difference(from_dict(out["final"]["T_lidar_oak"]), TRUE_LIDAR_OAK)
    assert dt < 0.01 and dr < 0.3
    # no oak_root TF: the result targets the optical frame, with a note
    assert out["static_transform_for_g1_sensors"]["child"] == "oak_rgb_camera_optical_frame"
    assert any("OPTICAL frame" in n for n in out["notes"])


def test_forced_method(tmp_path):
    ds = write_dataset(str(tmp_path / "ds"), n=10, seed=4)
    cfg = config()
    cfg["extrinsics"]["final"] = "via_realsense"
    out = calibrate_extrinsics.run(ds, cfg)
    assert out["final"]["method"] == "via_realsense"


def test_intrinsics_cli_keeps_factory(tmp_path):
    ds = write_dataset(str(tmp_path / "ds"), n=12, seed=5)
    out = calibrate_intrinsics.main([ds, "--camera", "oak", "--config",
                                     write_config(tmp_path)])
    assert out["views_used"] == 12 and out["rms_px"] < 0.5
    assert out["factory_rms_px"] < 0.5
    assert "Keep the FACTORY" in calibrate_intrinsics.recommendation(out)
    info = yaml.safe_load(open(os.path.join(ds, "results", "oak_intrinsics.yaml")))
    assert info["image_width"] == 800 and len(info["camera_matrix"]["data"]) == 9
    assert abs(info["camera_matrix"]["data"][0] - 560.0) < 15.0


def test_make_board(tmp_path):
    out = tmp_path / "board.png"
    make_board.main(["--out", str(out), "--config", write_config(tmp_path), "--dpi", "100"])
    import cv2
    img = cv2.imread(str(out), cv2.IMREAD_GRAYSCALE)
    x0, x1, y0, y1 = BOARD.extent()
    assert abs(img.shape[1] - (x1 - x0) / 0.0254 * 100) <= 1
