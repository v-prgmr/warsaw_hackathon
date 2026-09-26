#!/usr/bin/env python3
"""Patch the TurtleBot3 Waffle SDF (Gazebo Classic) into an RGB-D robot for RTAB-Map sim tests.

Applies the Humble steps listed in rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_demo.launch.py:
  - sensor link camera_rgb_frame -> camera_rgb_optical_frame (image frame_id defaults to the link name)
  - add an empty camera_rgb_frame link + fixed optical joint
  - camera image 1920x1080 -> 640x480
  - sensor type camera -> depth (adds /camera/depth/image_raw + /camera/points)
"""
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
tree = ET.parse(path)
model = tree.getroot().find('model')

sensor_link = model.find("link[@name='camera_rgb_frame']")
if sensor_link is None:
    sys.exit(f'{path}: already patched or unexpected layout (no camera_rgb_frame link)')
sensor_link.set('name', 'camera_rgb_optical_frame')

sensor = sensor_link.find("sensor[@name='camera']")
sensor.set('type', 'depth')
image = sensor.find('camera/image')
image.find('width').text = '640'
image.find('height').text = '480'

model.append(ET.Element('link', name='camera_rgb_frame'))
joint = ET.SubElement(model, 'joint', name='camera_rgb_optical_joint', type='fixed')
ET.SubElement(joint, 'parent').text = 'camera_rgb_frame'
ET.SubElement(joint, 'child').text = 'camera_rgb_optical_frame'
ET.SubElement(joint, 'pose').text = '0 0 0 -1.57079632679 0 -1.57079632679'
ET.SubElement(ET.SubElement(joint, 'axis'), 'xyz').text = '0 0 1'

tree.write(path, encoding='unicode')  # no encoding declaration: spawn_entity.py rejects it
print(f'patched {path}')
