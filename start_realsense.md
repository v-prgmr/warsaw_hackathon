# Start And Stop RealSense On The G1

Run these commands on the G1 Orin. The vendor `video_hub_pc4` service and the RealSense ROS node
cannot own the camera devices at the same time.

## 1. Stop The Vendor Video Service

Stop the service before launching `realsense2_camera`:

```bash
sudo /unitree/sbin/mscli stopservice video_hub_pc4
sleep 2
```

Verify that the vendor process released the camera devices:

```bash
ps aux | grep '[v]ideohub'
sudo fuser -v /dev/video*
ls -l /dev/video*
```

Do not start RealSense if `videohub` or another process still owns `/dev/video*`. Resolve that
conflict first; do not kill unrelated Unitree processes.

## 2. Start The RealSense ROS Node

Use the bandwidth-safe settings validated for RTAB-Map recording:

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=false \
  publish_tf:=true
```

Keep this terminal open while recording. Stop the RealSense node cleanly with `Ctrl-C` after the
rosbag has been finalized.

## 3. Verify The Camera

From another ROS terminal, verify the required streams:

```bash
ros2 topic list | grep '^/camera'
ros2 param get /camera/camera rgb_camera.profile
ros2 param get /camera/camera depth_module.profile
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera enable_sync
ros2 param get /camera/camera pointcloud.enable
```

Expected recording topics:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf_static
```

The expected configured profiles are `640x480x15`, alignment and synchronization are `true`, and
the RealSense point cloud is `false`.

## 4. Stop RealSense And Restore The Vendor Service

First stop `realsense2_camera` with `Ctrl-C` in its launch terminal. Confirm that it released the
camera:

```bash
sudo fuser -v /dev/video*
```

Then restore the Unitree video service:

```bash
sudo /unitree/sbin/mscli startservice video_hub_pc4
sleep 2
ps aux | grep '[v]ideohub'
```

If `startservice` reports an error, do not repeatedly stop or kill services. Record the message and
ask the robot owner to restore `video_hub_pc4` with the approved Unitree service procedure.

## Rosbag Capture

See `g1_ws/src/g1_recorder/README.md` for DDS setup, RTAB and full-survey recording profiles,
pause/resume commands, bag validation, and replay.
