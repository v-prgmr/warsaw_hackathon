# warsaw_hackathon

Unitree G1 side of the Alien Bazaar hackathon project. Architecture and rules: [AGENTS.md](AGENTS.md).

## Clone

The repo has three Unitree submodules under `third_party/` (`unitree_ros`, `unitree_ros2`, `unitree_sdk2`):

```bash
git clone --recurse-submodules <repo-url>
```

Already cloned:

```bash
git submodule update --init --recursive
```

## Packages — [`g1_ws/`](g1_ws/README.md)

ROS 2 capture/keyframe/viz packages (`g1_recorder`, `keyframe_manager`, `scene_server` stub).
Build with `colcon build` in `g1_ws/`. Offline tests need no robot; to record from the robot
see **"Testing from the computer connected to the robot"** in [`g1_ws/README.md`](g1_ws/README.md).

## AR glasses — [`ar_glasses/`](ar_glasses/README.md)

Snap Spectacles (2024) showing the robot, its path, the LiDAR map and semantic POIs in the room.
Windows setup (Lens Studio 5.15.4, USB deploy) and a mock bridge to test without the robot:
[`ar_glasses/README.md`](ar_glasses/README.md).
