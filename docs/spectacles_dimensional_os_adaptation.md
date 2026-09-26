# What we can reuse from spectacles-dimensional-os

Reviewed 2026-09-26 at upstream commit `ebf1d38`. Source:
[V4C38/spectacles-dimensional-os](https://github.com/V4C38/spectacles-dimensional-os)
(MIT). Its README calls the Go2 path tested and the G1 path **supported but not
tested**. It is a Dimensional OS stack, not a ROS 2/RTAB-Map integration. Do
not run its launcher against our G1 or let it own navigation/odometry.

## The relevant working pattern

Its [camera stream](https://github.com/V4C38/spectacles-dimensional-os/blob/main/lens-studio/Assets/Scripts/ARBridge/Camera/DeviceCameraStream.ts)
uses Lens Studio `CameraModule` and the tracking camera, obtains frame capture
timestamps, and exposes `DeviceCamera.pose` (camera extrinsic). Its
[camera client](https://github.com/V4C38/spectacles-dimensional-os/blob/main/lens-studio/Assets/Scripts/ARBridge/Camera/CameraClient.ts)
keeps a pose history, interpolates a tracking pose to the image timestamp,
scales camera intrinsics to the transmitted image resolution, JPEG-encodes the
frame, and sends it to a host. The host's
[tag tracker](https://github.com/V4C38/spectacles-dimensional-os/blob/main/dimos-ar/dimos/ar/tag_tracking/tracker.py)
uses OpenCV AprilTag 36h11, pose/reprojection gates, clock-skew checks, and
re-observations for drift correction. These are good implementation references
for a **Lens-to-ROS adapter**, not evidence that our physical device is
already working.

Their Lens is not drop-in compatible with our WebSocket bridge. The upstream
[protocol](https://github.com/V4C38/spectacles-dimensional-os/blob/main/dimos-ar/PROTOCOL.md)
uses a `camera_info` text message and binary `ARF1` JPEG frames, plus
handshake/ping-pong/ack messages; our prototype accepts a smaller JSON packet
with `ros_rh_m`, `T_world_device`, and either `T_device_tag` or a paired JPEG.
Their Lens also exposes navigation/robot commands that are outside our
Spectacles pose-only scope. If reusing MIT code, retain the license notice and
extract only camera/pose/transport and visualization pieces behind a new
read-only adapter. Do not connect its command clients to the G1.

Their Lens has an authored `BoundingBox` debug object scaled from a negotiated
`body_bounds_m`. That can inform a later G1 overlay, but it is tied to their
scene and handshake. Our [standalone WebXR box](../ar_glasses_test/webxr_bbox/README.md)
is the immediate actual-glasses **display-only** test without Lens Studio.

## G1 tag TF in our map

Upstream's [G1 profile](https://github.com/V4C38/spectacles-dimensional-os/blob/main/dimos-ar/dimos/ar/robot_profile/g1.py)
assumes a fixed `base_link` (pelvis)-to-chest/back-tag mount and gives example
offsets. Its G1 path has not been validated on hardware. Our 29-DoF G1 has
moving waist joints, and `AGENTS.md` requires the URDF/`joint_states` chain.
Never copy those offsets or parent a torso tag directly to `robot_center` or
pelvis. Measure the physical tag pose and publish exactly one static
`torso_link -> april_tag_0` TF. Then, while the G1 stands still:

```text
map -> odom -> robot_center -> pelvis -> waist joints -> torso_link
                                                       -> april_tag_0
```

RTAB-Map remains the only `map -> odom` owner. Leave our Spectacles YAML
`tag_pose_map: null` and `publish_tag_tf: false`, so the registration node reads
the dynamic `T_map_tag` through TF. This works for a **stationary** robot with
the current arrival-time lookup. For a moving robot, `T_map_tag` must be looked
up at the image capture time with synchronized clocks or buffered odometry;
otherwise its pose can be wrong. Continuous drift correction from tag revisits
is another future addition; our prototype currently only reports revisit
residuals and does not shift the committed glasses-world registration.

## The exact tag image and sizing

The linked [AprilRobotics tag36_11_00000.png](https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00000.png)
is a **10 × 10 pixel source pattern**, not a print-ready 10 mm tag. Its 8 × 8
inner marker has a one-module white border. OpenCV identifies it as
`DICT_APRILTAG_36h11`, ID 0. We verified that OpenCV's generated ID 0 is the
same pattern rotated 180 degrees; orientation matters when assigning the tag
frame even though the ID is unchanged. Our
[print-scale SVG](../ar_glasses_test/laptop_anchor/apriltag_36h11_id0_160mm.svg)
has now been regenerated to match the AprilRobotics image **pixel-for-pixel in
orientation**. Its black detection square is 160 mm; the white canvas is
180 mm. For the upstream 70 mm *outer* print size, the black detection square
would be 56 mm, which is the value to give the pose solver. Measure the final
print with a ruler and avoid curving the tag over the G1 shell. A different
flat mount/size is acceptable if it respects the G1 mounting constraints and
has a measured `T_torso_link_tag`.

## Recommendation

First confirm the actual glasses can display a local WebXR box. Next use a
supported Lens Studio host to deploy a minimal camera+pose Lens, borrowing the
upstream capture-time interpolation and camera-intrinsics pattern under MIT.
Register to the fixed laptop tag first, then move the same tag logic to a
measured G1 torso mount while the robot stands. Keep RTAB-Map/ROS TF as the
source of truth, with no DimOS locomotion or map ownership.
