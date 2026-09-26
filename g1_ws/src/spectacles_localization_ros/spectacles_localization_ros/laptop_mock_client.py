"""Exercise the full WebSocket bridge using synthetic laptop/tag geometry."""

import argparse
import asyncio
import json
import os
from math import sin
from uuid import uuid4

import websockets

from .protocol import pose_dict
from .transforms import Transform, compose, inverse


def yaw(angle):
    return (0.0, 0.0, sin(angle / 2), cos(angle / 2))


def packet(index, token, session_id):
    """Tag is laptop origin; both world and device have nonzero offsets/yaw."""
    t = index * 0.05
    laptop_world = Transform((0.7, -0.25, 0.1), yaw(0.22))
    world_device = Transform((0.3 + 0.08*t, 0.05*sin(0.8*t), 1.65),
                             yaw(0.05*sin(t)))
    laptop_tag = Transform((0, 0, 0), (0, 0, 0, 1))
    device_tag = compose(inverse(world_device),
                         compose(inverse(laptop_world), laptop_tag))
    tag_visible = index < 25
    return {
        "schema_version": 1,
        "coordinate_system": "ros_rh_m",
        "session_id": session_id,
        "timestamp_sec": t,
        "tracking_valid": True,
        "device_pose": pose_dict(world_device),
        "tag_id": 0 if tag_visible else None,
        "tag_pose_device": pose_dict(device_tag) if tag_visible else None,
        "token": token,
    }


async def send(uri, token, count):
    session_id = f"laptop_mock_ws_{uuid4().hex}"
    async with websockets.connect(uri, max_size=2 * 1024 * 1024) as socket:
        for index in range(count):
            await socket.send(json.dumps(packet(index, token, session_id)))
            await asyncio.sleep(0.05)
    print(f"Sent {count} synthetic packets ({min(count, 25)} with tag).")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default="ws://127.0.0.1:8765")
    parser.add_argument("--token", default=os.environ.get("SPECTACLES_BRIDGE_TOKEN", ""))
    parser.add_argument("--count", type=int, default=80)
    args = parser.parse_args()
    if args.count < 10:
        parser.error("--count must be at least 10")
    asyncio.run(send(args.uri, args.token, args.count))
