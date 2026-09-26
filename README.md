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

## Spectacles display test

[`ar_glasses_test/`](ar_glasses_test/README.md) contains a standalone Snap Lens
smoke test for showing custom text and a world-anchored object on AR glasses.
It is not connected to the robot or mapping stack.

The separate [Spectacles localization prototype](docs/spectacles_localization_architecture.md)
registers the glasses' local tracking frame to the G1 RTAB-Map map using a
known AprilTag; its ROS mock works without glasses, but on-device capture and
networking are not yet tested.
For the G1-free Ubuntu laptop mock and printable tag, follow the
[laptop-anchor test](ar_glasses_test/laptop_anchor/README.md).
For an actual on-glasses bounding-box display test without Lens Studio, see
the [WebXR Browser Lens test](ar_glasses_test/webxr_bbox/README.md).
The [Dimensional OS adaptation review](docs/spectacles_dimensional_os_adaptation.md)
explains which upstream camera/AprilTag ideas apply to our ROS/RTAB-Map G1.
