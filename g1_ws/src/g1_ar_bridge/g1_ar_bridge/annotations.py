"""POIs and 3D boxes as Lens annotations (``draw_world_annotation`` skill, no Lens change).

Annotations are kept in the ROS ``map`` frame and converted to the AR world when sent.
Lens rules (upstream PROTOCOL.md): a marker has exactly one point and an optional label; a line
has >= 2 points, and with exactly 2 the Lens draws a slight curve, so straight segments get a
midpoint here.
"""
from dataclasses import dataclass

import numpy as np

from .geometry import transform_points

POI_COLOR = (1.0, 0.55, 0.1)
BOX_COLOR = (0.1, 0.9, 0.3)


@dataclass
class Annotation:
    id: str
    kind: str                       # "marker" | "line"
    points: np.ndarray              # (N, 3) in map
    label: str = None
    color: tuple = None
    duration_s: float = None


def box_polylines(center, size, R=None):
    """Edges of an oriented box as polylines: bottom loop, top loop, four uprights (3 pts)."""
    c = np.asarray(center, dtype=np.float64)
    hx, hy, hz = (np.asarray(size, dtype=np.float64) / 2.0)
    R = np.eye(3) if R is None else np.asarray(R, dtype=np.float64)
    ring = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy], [-hx, -hy]])
    loops = []
    for z in (-hz, hz):
        pts = np.column_stack([ring, np.full(len(ring), z)])
        loops.append(pts @ R.T + c)
    uprights = []
    for i in range(4):
        a, b = loops[0][i], loops[1][i]
        uprights.append(np.array([a, (a + b) / 2.0, b]))
    return loops + uprights


def poi(poi_id, label, xyz, confidence=None, box=None, color=POI_COLOR, box_color=BOX_COLOR):
    """A labelled POI marker, plus its 3D box when ``box = (center, size, R)`` is given."""
    text = label if confidence is None else f"{label} ({confidence:.2f})"
    out = [Annotation(f"{poi_id}", "marker", np.asarray([xyz], dtype=np.float64), text,
                      tuple(color))]
    if box is not None:
        for i, line in enumerate(box_polylines(*box)):
            out.append(Annotation(f"{poi_id}-box{i}", "line", line, None, tuple(box_color)))
    return out


def to_wire_args(ann, T_ar_map):
    pts = transform_points(T_ar_map, ann.points)
    if ann.kind == "line" and len(pts) == 2:
        pts = np.array([pts[0], (pts[0] + pts[1]) / 2.0, pts[1]])
    points = [[round(float(v), 3) for v in p] for p in pts]
    args = {"id": ann.id, "kind": ann.kind, "points": points}
    if ann.label:
        args["label"] = str(ann.label)
    if ann.color is not None:
        args["color"] = [round(float(v), 3) for v in ann.color]
    if ann.duration_s is not None:
        args["duration_s"] = float(ann.duration_s)
    return args


def signature(ann):
    """Change detector (1 mm, label, colour)."""
    pts = tuple(np.round(np.asarray(ann.points, dtype=np.float64), 3).reshape(-1).tolist())
    return (ann.kind, pts, ann.label, ann.color, ann.duration_s)
