"""Validate the device-to-ROS JSON wire contract."""

import json
from dataclasses import dataclass
from math import isfinite

from .transforms import Transform


@dataclass(frozen=True)
class Observation:
    session_id: str
    timestamp_sec: float
    tracking_valid: bool
    world_device: Transform
    tag_id: object
    device_tag: object


def pose_dict(t):
    return {"position": list(t.position), "orientation": list(t.orientation)}


def parse(payload):
    value = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("Expected schema_version 1 object")
    if value.get("coordinate_system") != "ros_rh_m":
        raise ValueError("Convert coordinates to ROS right-handed metres before sending")
    session = value.get("session_id")
    if not isinstance(session, str) or not 1 <= len(session) <= 64:
        raise ValueError("session_id must be 1-64 characters")
    timestamp = float(value["timestamp_sec"])
    if not isfinite(timestamp):
        raise ValueError("Invalid timestamp")
    valid = value.get("tracking_valid")
    if not isinstance(valid, bool):
        raise ValueError("tracking_valid must be boolean")

    def transform(field):
        obj = value[field]
        if not isinstance(obj, dict):
            raise ValueError(field + " must be a pose")
        return Transform(tuple(obj["position"]), tuple(obj["orientation"]))

    world_device = transform("device_pose")
    tag_id = value.get("tag_id")
    device_tag = transform("tag_pose_device") if value.get("tag_pose_device") is not None else None
    if (tag_id is None) != (device_tag is None):
        raise ValueError("tag_id and tag_pose_device must be paired")
    if tag_id is not None and (not isinstance(tag_id, int) or isinstance(tag_id, bool)):
        raise ValueError("tag_id must be integer")
    return Observation(session, timestamp, valid, world_device, tag_id, device_tag)
