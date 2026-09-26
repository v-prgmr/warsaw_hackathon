"""Calibration dataset on disk (written by calib_capture, read by the calibrate_* tools).

    <dataset>/
      sample_000/
        meta.yaml                 # stamps, frame ids, LiDAR scan count, TF looked up at capture
        <camera>.png              # one image per camera (e.g. oak.png, realsense.png), bgr8
        <camera>_camera_info.yaml # the driver's CameraInfo, ROS camera_calibration format
        lidar.npy                 # float32 (N, 3) accumulated LiDAR points in the LiDAR frame
      sample_001/ ...
      results/                    # written by calibrate_extrinsics / calibrate_intrinsics

meta.yaml `tf` is a list of {parent, child, xyz, quat_xyzw} (T_parent_child) looked up on /tf at
capture time, e.g. livox_frame <- camera_color_optical_frame and torso_link <- livox_frame.
"""
from dataclasses import dataclass, field
import os

import cv2
import numpy as np
import yaml

from .geometry import from_dict, to_dict


# --- CameraInfo YAML (camera_calibration_parsers format, loadable via camera_info_url) --------

def camera_info_to_yaml(width, height, k, d, distortion_model="plumb_bob", name="camera",
                        p=None):
    k = np.asarray(k, dtype=float).reshape(3, 3)
    d = np.asarray(d, dtype=float).ravel()
    if p is None:
        p = np.hstack([k, np.zeros((3, 1))])
    return {
        "image_width": int(width), "image_height": int(height), "camera_name": name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in k.ravel()]},
        "distortion_model": distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(d), "data": [float(v) for v in d]},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [float(v) for v in np.eye(3).ravel()]},
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [float(v) for v in np.asarray(p, float).ravel()]},
    }


@dataclass
class Intrinsics:
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    distortion_model: str = "plumb_bob"

    @classmethod
    def from_yaml(cls, data):
        model = data.get("distortion_model", "plumb_bob")
        if model in ("equidistant", "fisheye"):
            raise NotImplementedError("fisheye (equidistant) intrinsics are not supported")
        return cls(int(data["image_width"]), int(data["image_height"]),
                   np.array(data["camera_matrix"]["data"], float).reshape(3, 3),
                   np.array(data["distortion_coefficients"]["data"], float), model)

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls.from_yaml(yaml.safe_load(f))

    @classmethod
    def from_msg(cls, msg):
        return cls(int(msg.width), int(msg.height), np.array(msg.k, float).reshape(3, 3),
                   np.array(msg.d, float), msg.distortion_model or "plumb_bob")

    @property
    def size(self):
        return (self.width, self.height)

    def to_yaml(self, name="camera"):
        return camera_info_to_yaml(self.width, self.height, self.k, self.d,
                                   self.distortion_model, name)


# --- samples ------------------------------------------------------------------------------

@dataclass
class Sample:
    name: str
    path: str
    images: dict = field(default_factory=dict)       # camera -> bgr image
    intrinsics: dict = field(default_factory=dict)   # camera -> Intrinsics
    frames: dict = field(default_factory=dict)       # camera -> optical frame id
    lidar: np.ndarray = None                          # (N, 3) LiDAR frame
    lidar_frame: str = None
    tf: list = field(default_factory=list)

    def transform(self, parent, child):
        """T_parent_child from the TF recorded at capture, or None."""
        for t in self.tf:
            if t["parent"] == parent and t["child"] == child:
                return from_dict(t)
        return None


def write_sample(root, index, images, infos, frames, lidar_points, lidar_frame, transforms,
                 extra=None):
    """images: camera -> bgr; infos: camera -> Intrinsics; transforms: list of
    (parent, child, T_parent_child 4x4)."""
    name = f"sample_{index:03d}"
    path = os.path.join(root, name)
    os.makedirs(path, exist_ok=False)
    for cam, img in images.items():
        cv2.imwrite(os.path.join(path, f"{cam}.png"), img)
    for cam, info in infos.items():
        with open(os.path.join(path, f"{cam}_camera_info.yaml"), "w") as f:
            yaml.safe_dump(info.to_yaml(cam), f, sort_keys=False)
    if lidar_points is not None:
        np.save(os.path.join(path, "lidar.npy"), np.asarray(lidar_points, np.float32))
    meta = {"name": name, "cameras": {cam: {"frame_id": frames.get(cam, "")} for cam in images},
            "lidar": {"frame_id": lidar_frame,
                      "points": 0 if lidar_points is None else int(len(lidar_points))},
            "tf": [to_dict(t, parent, child) for parent, child, t in transforms]}
    if extra:
        meta.update(extra)
    with open(os.path.join(path, "meta.yaml"), "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return path


def load_sample(path):
    with open(os.path.join(path, "meta.yaml")) as f:
        meta = yaml.safe_load(f)
    s = Sample(name=meta.get("name", os.path.basename(path)), path=path, tf=meta.get("tf", []))
    for cam, info in meta.get("cameras", {}).items():
        img_path = os.path.join(path, f"{cam}.png")
        if os.path.isfile(img_path):
            s.images[cam] = cv2.imread(img_path, cv2.IMREAD_COLOR)
        info_path = os.path.join(path, f"{cam}_camera_info.yaml")
        if os.path.isfile(info_path):
            s.intrinsics[cam] = Intrinsics.from_file(info_path)
        s.frames[cam] = info.get("frame_id", "")
    lidar_path = os.path.join(path, "lidar.npy")
    if os.path.isfile(lidar_path):
        s.lidar = np.load(lidar_path).astype(np.float64)
    s.lidar_frame = meta.get("lidar", {}).get("frame_id")
    return s


def load_dataset(root):
    names = sorted(n for n in os.listdir(root)
                   if n.startswith("sample_") and os.path.isdir(os.path.join(root, n)))
    if not names:
        raise FileNotFoundError(f"no sample_* directories in {root}")
    return [load_sample(os.path.join(root, n)) for n in names]


def next_index(root):
    if not os.path.isdir(root):
        return 0
    idx = [int(n.split("_")[1]) for n in os.listdir(root)
           if n.startswith("sample_") and n.split("_")[1].isdigit()]
    return max(idx) + 1 if idx else 0


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)
