"""The AR bridge without ROS: a simulated G1 facing your wall tag (Windows / Ubuntu / macOS).

Tests the Spectacles side of the wall-tag registration at home. Print AprilTag 36h11 ID 0, stick
it on a wall, measure its black square, then:

    pip install -r requirements-sim.txt          # websockets, numpy, opencv-python
    python -m g1_ar_bridge.sim_main --tag-size 0.16

On the glasses: Dimensional OS -> this computer's IP -> Registration: AprilTag -> look at the tag
from 1-2 m and step sideways. The virtual G1 then stands ``--tag-distance`` m in front of the tag,
facing it, in a synthetic room whose front wall is your wall; two demo POIs and a box appear on a
virtual table to its right. Nothing here talks to a robot.
"""
import argparse
import asyncio
import socket

from . import protocol as P
from .server import ArBridgeServer, BridgeConfig, serve
from .world import SimWorld


def local_ips():
    ips = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))   # no packet is sent; picks the default interface
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


async def main_async(args):
    world = SimWorld(tag_id=args.tag_id, tag_distance=args.tag_distance,
                     tag_height=args.tag_height, tag_lateral=args.tag_lateral,
                     base_height=args.base_height, demo_pois=not args.no_demo_pois)
    cfg = BridgeConfig(tag_id=args.tag_id, tag_black_size_m=args.tag_size,
                       base_height_m=args.base_height, min_views=args.min_views,
                       display_name="Unitree G1 (simulated)")
    bridge = ArBridgeServer(world, cfg)
    server = await serve(bridge, args.host, args.port)
    ips = local_ips() or ["<this computer's IP>"]
    print(f"AR bridge (simulated G1, protocol v{P.PROTOCOL_VERSION}) on {args.host}:{args.port}")
    print("Type one of these into the Lens as the Bridge IP: " + ", ".join(ips))
    print(f"Wall tag: AprilTag 36h11 ID {args.tag_id}, black square {args.tag_size * 100:.1f} cm."
          f" The virtual G1 stands {args.tag_distance} m in front of it.")
    print("Ctrl-C to stop.")
    try:
        await bridge.run()
    finally:
        server.close()
        await server.wait_closed()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=P.PORT)
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--tag-size", type=float, default=0.16,
                    help="edge of the tag's BLACK square in metres (measure the print)")
    ap.add_argument("--tag-distance", type=float, default=1.5,
                    help="virtual robot's distance in front of the tag (m)")
    ap.add_argument("--tag-height", type=float, default=1.0,
                    help="height of the tag centre above the floor (m)")
    ap.add_argument("--tag-lateral", type=float, default=0.0,
                    help="tag offset to the robot's left (m)")
    ap.add_argument("--base-height", type=float, default=0.78,
                    help="robot_center (pelvis) height above the floor (m)")
    ap.add_argument("--min-views", type=int, default=6)
    ap.add_argument("--no-demo-pois", action="store_true")
    args = ap.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
