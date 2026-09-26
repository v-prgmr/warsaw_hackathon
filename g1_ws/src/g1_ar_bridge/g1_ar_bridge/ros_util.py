"""ROS message <-> numpy helpers for the AR bridge nodes (no cv_bridge, no sensor_msgs_py).

The functions take message objects duck-typed (attribute access only), so they are unit-tested
without ROS; building messages is left to the nodes.
"""
import numpy as np

from .annotations import Annotation, box_polylines
from .geometry import make_T, quat_to_rot, transform_points

# visualization_msgs/Marker constants
ARROW, CUBE, SPHERE, CYLINDER, LINE_STRIP, LINE_LIST = 0, 1, 2, 3, 4, 5
CUBE_LIST, SPHERE_LIST, POINTS, TEXT_VIEW_FACING = 6, 7, 8, 9
ADD, MODIFY, DELETE, DELETEALL = 0, 0, 2, 3
MAX_LIST_MARKERS = 20


def image_to_gray(msg):
    """sensor_msgs/Image (mono8, rgb8, bgr8, rgba8, bgra8, mono16) -> uint8 (H, W) or None."""
    enc = msg.encoding.lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("mono8", "8uc1"):
        return buf.reshape(h, step)[:, :w].copy()
    if enc in ("mono16", "16uc1"):
        img = buf.view(np.uint16).reshape(h, step // 2)[:, :w]
        return (img >> 8).astype(np.uint8)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "8uc3": 3, "8uc4": 4}.get(enc)
    if channels is None:
        return None
    img = buf.reshape(h, step)[:, :w * channels].reshape(h, w, channels).astype(np.float32)
    if enc.startswith("rgb") or enc in ("8uc3", "8uc4"):
        r, g, b = img[..., 0], img[..., 1], img[..., 2]
    else:
        b, g, r = img[..., 0], img[..., 1], img[..., 2]
    return np.clip(0.299 * r + 0.587 * g + 0.114 * b + 0.5, 0, 255).astype(np.uint8)


def depth_image_to_m(msg):
    """sensor_msgs/Image depth (16UC1 mm or 32FC1 m) -> float32 metres, invalid = nan."""
    enc = msg.encoding.upper()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("16UC1", "MONO16"):
        d = buf.view(np.uint16).reshape(h, step // 2)[:, :w].astype(np.float32) / 1000.0
    elif enc == "32FC1":
        d = buf.view(np.float32).reshape(h, step // 4)[:, :w].astype(np.float32)
    else:
        return None
    d[~np.isfinite(d) | (d <= 0.0)] = np.nan
    return d


def camera_info_intrinsics(msg):
    """(K 3x3, distortion coefficients, model) from sensor_msgs/CameraInfo."""
    K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
    D = np.asarray(msg.d, dtype=np.float64).reshape(-1)
    return K, D, msg.distortion_model


_PC2_TYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
              7: np.float32, 8: np.float64}


def pointcloud2_xyz(msg):
    """(N, 3) float64 finite x, y, z from sensor_msgs/PointCloud2."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3))
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for k in ("x", "y", "z"):
        f = fields[k]
        names.append(k)
        formats.append(np.dtype(_PC2_TYPES[f.datatype]).newbyteorder(endian))
        offsets.append(f.offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                      "itemsize": int(msg.point_step)})
    n = int(msg.width) * int(msg.height)
    data = bytes(msg.data)
    if int(msg.height) > 1 and int(msg.row_step) != int(msg.width) * int(msg.point_step):
        rows = [data[r * msg.row_step:r * msg.row_step + msg.width * msg.point_step]
                for r in range(int(msg.height))]
        data = b"".join(rows)
    arr = np.frombuffer(data, dtype=dtype, count=n)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    return pts[np.all(np.isfinite(pts), axis=1)]


def voxel_downsample(points, voxel_m):
    """One point (the first) per voxel."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0 or voxel_m <= 0:
        return pts
    keys = np.floor(pts / voxel_m).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def transform_msg_to_T(tf):
    """geometry_msgs/Transform -> 4x4."""
    t, q = tf.translation, tf.rotation
    return make_T(quat_to_rot([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


def pose_msg_to_T(pose):
    p, q = pose.position, pose.orientation
    return make_T(quat_to_rot([q.x, q.y, q.z, q.w]), [p.x, p.y, p.z])


def marker_key(m):
    ns = (m.ns or "marker").replace(" ", "_")
    return f"{ns}/{int(m.id)}"


def marker_to_annotations(m, T_map_frame):
    """visualization_msgs/Marker (ADD) -> list of Annotation in map.

    TEXT / SPHERE / CYLINDER / ARROW -> labelled marker; CUBE -> 3D box edges (+ label if the
    marker has text); LINE_STRIP / LINE_LIST -> lines; POINTS / SPHERE_LIST -> up to 20 markers.
    """
    key = marker_key(m)
    T = T_map_frame @ pose_msg_to_T(m.pose)
    color = (float(m.color.r), float(m.color.g), float(m.color.b))
    life = m.lifetime.sec + m.lifetime.nanosec * 1e-9
    duration = life if life > 0 else None
    label = m.text or None
    centre = T[:3, 3].reshape(1, 3)
    out = []
    if m.type in (TEXT_VIEW_FACING, SPHERE, CYLINDER, ARROW):
        out.append(Annotation(key, "marker", centre, label or m.ns, color, duration))
    elif m.type == CUBE:
        size = (float(m.scale.x), float(m.scale.y), float(m.scale.z))
        for i, line in enumerate(box_polylines(T[:3, 3], size, T[:3, :3])):
            out.append(Annotation(f"{key}-box{i}", "line", line, None, color, duration))
        if label:
            top = T[:3, 3] + T[:3, :3] @ [0.0, 0.0, size[2] / 2.0 + 0.05]
            out.append(Annotation(key, "marker", top.reshape(1, 3), label, color, duration))
    elif m.type in (LINE_STRIP, LINE_LIST):
        pts = transform_points(T, [[p.x, p.y, p.z] for p in m.points])
        if m.type == LINE_STRIP and len(pts) >= 2:
            out.append(Annotation(key, "line", pts, None, color, duration))
        elif m.type == LINE_LIST:
            for i in range(0, len(pts) - 1, 2):
                out.append(Annotation(f"{key}-{i // 2}", "line", pts[i:i + 2], None, color,
                                      duration))
    elif m.type in (POINTS, SPHERE_LIST, CUBE_LIST):
        pts = transform_points(T, [[p.x, p.y, p.z] for p in m.points])
        for i, p in enumerate(pts[:MAX_LIST_MARKERS]):
            out.append(Annotation(f"{key}-{i}", "marker", p.reshape(1, 3), None, color,
                                  duration))
    return out


def apply_marker_array(current, markers, lookup_T_map):
    """Update ``current`` (dict key -> list[Annotation]) with a MarkerArray's markers.

    ``lookup_T_map(frame_id)`` returns 4x4 map <- frame or None (marker skipped).
    """
    for m in markers:
        if m.action == DELETEALL:
            current.clear()
            continue
        key = marker_key(m)
        if m.action == DELETE:
            current.pop(key, None)
            continue
        T = lookup_T_map(m.header.frame_id)
        if T is None:
            continue
        current[key] = marker_to_annotations(m, T)
    return current
