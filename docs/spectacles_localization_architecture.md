# Spectacles localization in the G1 RTAB-Map world

Status (2026-09-26): ROS-side prototype and WebSocket mock are implemented and
tested in an isolated Ubuntu ROS 2 Humble container. No physical Spectacles
pose, camera packet, AprilTag observation, or device-to-ROS transport has been
tested. The team confirmed Spectacles (2024); no Lens Studio/SDK is installed
and there is no Lens Studio project in this repo. A G1-free laptop-frame test
and exact commands are in [laptop anchor](../ar_glasses_test/laptop_anchor/README.md).

## What exists already

- `g1_ws/src/g1_mapping` runs RTAB-Map from MID-360 LiDAR + IMU. It owns
  `map -> odom`, `/map`, and the map database; its ICP odometry owns
  `odom -> robot_center` and `/odom` by default. Do not start a second mapper.
- `g1_ws/src/g1_sensors` publishes the 29-DoF G1 URDF/joint-state TF chain
  below `robot_center`. The G1 base frame is **`robot_center`**, not
  `g1/base_link`; the prompt's latter name is conceptual only.
- The laptop host uses ROS 2 Humble (usually `scripts/run_humble.sh`); the G1
  stack uses Foxy/Unitree DDS. `rl_hnav` can be a locomotion consumer but is
  untouched here.
- `ar_glasses_test/` has a display-only Lens script/instructions, **not** a
  Lens Studio project, camera access, networking bridge, or tag detector.
- No Spectacles SDK or Lens Studio installation was found in this workspace.

## TF ownership and direction

Before:

```text
map --[RTAB-Map]--> odom --[g1_mapping odometry]--> robot_center
                                               └── G1 URDF sensor/torso links
```

After registration:

```text
map --[RTAB-Map]--> odom --> robot_center --> G1 URDF links
  ├── april_tag_0          [measured static TF or external tag TF; ONE owner]
  └── spectacles           [this package's dynamic TF]
```

`spectacles_world` is the glasses' private local tracking world. The first
prototype does **not** publish it as a separate ROS TF parent: that avoids a
second path/parent for `spectacles`. Its poses are visible on
`/spectacles/device_pose_local` with `frame_id=spectacles_world`.

Notation: `T_A_B` maps coordinates expressed in B into A. All host-side
transforms are proper right-handed 3D rotations, metres, ROS `xyzw` quaternion.
Given measured `T_map_tag`, synchronized `T_device_tag`, and device tracking
`T_spectaclesWorld_device`, the registration is:

```text
T_map_spectaclesWorld = T_map_tag * inverse(T_device_tag)
                       * inverse(T_spectaclesWorld_device)
T_map_device = T_map_spectaclesWorld * T_spectaclesWorld_device
```

For host-side camera detection, the detector yields `T_cameraOptical_tag`; the
Lens/adapter must supply verified `T_device_cameraOptical`, so
`T_device_tag = T_device_cameraOptical * T_cameraOptical_tag`. A camera frame
cannot be silently substituted for the head/device frame.

## V1 registration and map-tag placement

Use ONE printed AprilTag **36h11** with measured black-square edge length
(`tag.size_m`). `config/spectacles_localization.yaml` intentionally has
`tag_pose_map: null`: no real-world tag pose is hard-coded. For the manual V1
workflow, measure the tag's position **and orientation** in RTAB-Map `map`, set
`tag_pose_map: [x,y,z,qx,qy,qz,qw]`, and `publish_tag_tf: true`. The tag frame
is centered on the black square, X toward its printed right, Y toward its
printed top, Z out of its front. If another node already publishes
`map -> april_tag_0`, leave both YAML pose empty and `publish_tag_tf: false`.
Do not use both sources. Later, a separate G1 camera detector may publish the
same known map-tag transform, but is not a V1 dependency.
Measure the pose against the map database used for the demonstration; a major
RTAB-Map graph correction or a moved tag invalidates the manual pose.

Collect at least 10 paired detections in a configurable 10-second window.
Candidates are formed with the equation above, gross translation/angle
outliers are rejected, translation is averaged, and rotations use a Markley
quaternion mean. The resulting `T_map_spectaclesWorld` is fixed for that
tracking session; device pose packets continue moving `map -> spectacles`
after the tag disappears. A new `session_id` invalidates registration; loss
of device tracking stops pose/TF updates. Re-observing the tag reports a
residual/drift diagnostic but does not silently shift the map registration.

## Device/host contract

The ROS bridge accepts JSON text over WebSocket, then publishes validated
pose-only JSON on `/spectacles/observation` (camera JPEG is stripped). The
minimal tracking packet is:

```json
{
  "schema_version": 1,
  "coordinate_system": "ros_rh_m",
  "session_id": "lens-start-unique-id",
  "timestamp_sec": 1234.56,
  "tracking_valid": true,
  "device_pose": {"position": [0, 0, 1.6], "orientation": [0, 0, 0, 1]},
  "tag_id": null,
  "tag_pose_device": null
}
```

`device_pose` is `T_spectaclesWorld_device` (head pose). When a tag is seen,
set `tag_id` and `tag_pose_device` (`T_device_tag`) in **the same packet**.
`timestamp_sec` is the source capture time in seconds and must increase within
one session; it need not equal ROS wall time. ROS output is currently stamped
at host receipt time, since clock synchronization is not yet measured. The
paired pose must correspond to the image capture time; do not mix the current
tracking pose with a delayed tag detection.

Optional host-side AprilTag input replaces `tag_id`/`tag_pose_device` with:

```json
{
  "camera_jpeg_base64": "<JPEG base64>",
  "camera_intrinsics": {"fx": 500, "fy": 500, "cx": 320, "cy": 240,
                        "distortion": [0, 0, 0, 0, 0]},
  "camera_pose_device": {"position": [0, 0, 0],
                         "orientation": [0, 0, 0, 1]}
}
```

These fields accompany the minimal packet. `camera_pose_device` is
`T_device_cameraOptical`, **not** an unverified identity; above numbers are
schema examples, not hardware calibration. OpenCV detects only 36h11 ID from
config and estimates `T_cameraOptical_tag` from the known tag size and
intrinsics. Its optical axes are X right, Y down, Z forward. Distortion defaults
to zeros only if absent; validate this against real optics before trusting
centimetre-level results. For the prototype, camera frames should be sent only
while registering, not as a permanent video stream.

Lens Studio world positions are in centimetres; this protocol accepts **only
converted metres and right-handed ROS-compatible frames**. The exact
Snap-to-ROS basis/handedness conversion, camera optical extrinsic, and capture
timestamp mapping need on-device verification with axis/rotation tests. The
bridge rejects packets labelled with any other convention; it does not guess.

## Snap API evidence and remaining hardware gate

Official Snap docs describe [World Device Tracking](https://developers.snap.com/lens-studio/features/ar-tracking/world/tracking-modes),
[CameraModule camera frames and camera information](https://developers.snap.com/spectacles/about-spectacles-features/apis/camera-module),
and [WebSocket via InternetModule](https://developers.snap.com/spectacles/about-spectacles-features/apis/web-socket).
The [Marker Tracking guide](https://developers.snap.com/lens-studio/features/ar-tracking/world/marker-tracking)
describes image/Snapcode tracking; it does **not** document an AprilTag
dictionary API. Therefore this package includes a host detector instead of
pretending `MarkerTrackingComponent` is an AprilTag detector. OpenCV's
[marker detection and pose-estimation documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
is the basis for that path.

No installed Lens SDK/project is available to validate actual script names or
camera-frame encoding/streaming on **our** model. Snap's `CameraModule` docs
show frame textures and camera intrinsics; its [media example](https://developers.snap.com/spectacles/about-spectacles-features/snap-cloud/examples/media)
shows JPEG encoding. A physical Lens must still pair each image with the
captured device pose and supply `T_device_cameraOptical`. Build/test that Lens
only after obtaining a compatible Lens Studio host and verifying the available
APIs on the actual device.

Snap also documents a built-in [WebXR Browser Lens](https://developers.snap.com/spectacles/about-spectacles-features/webxr)
for hosted AR pages without Lens Studio. That is a promising route for a
display/local-pose smoke test, but Snap's listed WebXR features do not prove
camera-frame access or AprilTag pose estimation. It is not yet a replacement
for the planned camera-access Lens in this registration pipeline.

Plain `ws://` requires Snap Experimental APIs and is suitable only for an
isolated prototype; Snap says these Lenses cannot be published. The bridge
binds localhost by default. To expose it on a lab LAN, set a nonempty token;
the token and any camera frame still travel in cleartext. Use an isolated LAN,
consent, and later `wss://`/TLS before wider use. Snap's [permission guidance](https://developers.snap.com/spectacles/permission-privacy/transparent-permission)
applies to a Lens combining camera and internet access.

## Acceptance and limits

The offline mock sends 25 tag-bearing packets, then tracking-only packets; it
also publishes ground truth. Unit tests use noncommuting transforms and fail
if inversion/composition order is wrong. The mock verifies the ROS pose against
ground truth and logs `MOCK PASS`/`MOCK FAIL`. This proves only ROS geometry and
topic/TF plumbing, **not** Spectacles tracking or detector accuracy.

For physical acceptance, inspect `/spectacles/localization_status`, check
`map -> spectacles` and the two poses in RViz, move away from the tag, then
revisit it and record the tag residual plus measured position drift. Disable
this package if it publishes a competing `map -> april_tag_0` or another
`map -> spectacles` publisher. It never publishes `map -> odom`, commands, or
anything into G1 navigation/locomotion.

For a robot-mounted tag adaptation and the relevant upstream Spectacles/DimOS
capture code, see [the integration review](spectacles_dimensional_os_adaptation.md).
