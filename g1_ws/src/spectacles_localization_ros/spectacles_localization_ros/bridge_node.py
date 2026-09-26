"""Local WebSocket JSON ingress; no ROS software runs on the glasses."""

import asyncio
import json
import queue
import threading

import rclpy
import websockets
from rclpy.node import Node
from std_msgs.msg import String

from .protocol import parse, pose_dict


class Bridge(Node):
    def __init__(self):
        super().__init__("spectacles_bridge")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("token", "")
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("tag_size_m", 0.16)
        self.host = self.get_parameter("bind_host").value
        self.port = int(self.get_parameter("port").value)
        self.token = self.get_parameter("token").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        if self.host not in ("127.0.0.1", "::1", "localhost") and not self.token:
            raise ValueError("A token is required when binding beyond localhost")
        self.messages = queue.Queue(maxsize=100)
        self.stop = threading.Event()
        self.publisher = self.create_publisher(String, "/spectacles/observation", 10)
        self.create_timer(0.02, self.drain)
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def drain(self):
        for _ in range(20):
            try:
                item = self.messages.get_nowait()
            except queue.Empty:
                break
            msg = String()
            msg.data = item
            self.publisher.publish(msg)

    async def on_websocket(self, websocket, *_unused):
        async for payload in websocket:
            try:
                if not isinstance(payload, str):
                    raise ValueError("Only UTF-8 JSON text is accepted")
                obj = json.loads(payload)
                if not isinstance(obj, dict):
                    raise ValueError("Expected a JSON object")
                if self.token and obj.get("token") != self.token:
                    raise ValueError("Bad token")
                if "camera_jpeg_base64" in obj:
                    from .tag_detector import detect
                    device_tag = await asyncio.to_thread(detect, obj, self.tag_id,
                                                         self.tag_size_m)
                    obj["tag_id"] = self.tag_id if device_tag is not None else None
                    obj["tag_pose_device"] = (pose_dict(device_tag)
                                              if device_tag is not None else None)
                    # Never forward camera pixels onto the ROS graph.
                    for key in ("camera_jpeg_base64", "camera_intrinsics",
                                "camera_pose_device"):
                        obj.pop(key, None)
                obs = parse(obj)
                clean = {
                    "schema_version": 1,
                    "coordinate_system": "ros_rh_m",
                    "session_id": obs.session_id,
                    "timestamp_sec": obs.timestamp_sec,
                    "tracking_valid": obs.tracking_valid,
                    "device_pose": pose_dict(obs.world_device),
                    "tag_id": obs.tag_id,
                    "tag_pose_device": (pose_dict(obs.device_tag)
                                        if obs.device_tag is not None else None),
                }
                self.messages.put_nowait(json.dumps(clean))
            except (ValueError, KeyError, TypeError, queue.Full,
                    ImportError, RuntimeError) as exc:
                self.get_logger().warn(f"Dropped Spectacles packet: {exc}")

    def serve(self):
        try:
            async def run():
                async with websockets.serve(self.on_websocket, self.host, self.port,
                                            max_size=2 * 1024 * 1024,
                                            ping_interval=20, ping_timeout=20):
                    self.get_logger().info(f"Spectacles WebSocket listening on "
                                           f"{self.host}:{self.port}")
                    while not self.stop.is_set():
                        await asyncio.sleep(0.1)

            asyncio.run(run())
        except Exception as exc:
            self.get_logger().error(f"WebSocket bridge stopped: {exc}")

    def destroy_node(self):
        self.stop.set()
        self.thread.join(timeout=2)
        return super().destroy_node()


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
