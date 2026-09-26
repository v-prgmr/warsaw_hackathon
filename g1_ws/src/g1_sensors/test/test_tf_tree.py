"""The G1 URDF + g1_sensors glue frames form one valid tree with the expected sensor mounts."""
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(PKG, "urdf", "g1_29dof_rev_1_0.urdf")
CONFIG = os.path.join(PKG, "config", "g1_sensors.yaml")


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch),
                              np.cos(yaw), np.sin(yaw))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def tree():
    """child -> (parent, T_parent_child at zero joint angles), URDF joints + glue frames."""
    edges = {}
    root = ET.parse(URDF).getroot()
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        child = j.find("child").get("link")
        assert child not in edges, f"{child} has two parents in the URDF"
        edges[child] = (j.find("parent").get("link"), transform(xyz, rpy))
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    for tf in cfg["static_transforms"]:
        assert tf["child"] not in edges, f"glue frame {tf['child']} already has a parent"
        edges[tf["child"]] = (tf["parent"], transform(tf["xyz"], tf["rpy"]))
    return edges, cfg


def chain(edges, parent, child):
    t = np.eye(4)
    while child != parent:
        assert child in edges, f"no path from {parent} to {child}"
        p, tpc = edges[child]
        t = tpc @ t
        child = p
    return t


def test_joint_names_match_urdf_revolute_order():
    root = ET.parse(URDF).getroot()
    revolute = [j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"]
    _, cfg = tree()
    assert cfg["joint_names"] == revolute
    assert len(revolute) == 29
    # unitree_sdk2 G1JointIndex: waist yaw/roll/pitch = 12/13/14 (compare_imu_sources relies on it)
    assert revolute[12:15] == ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]


def test_single_rooted_tree_reaching_all_sensor_frames():
    edges, _ = tree()
    roots = {p for p, _ in edges.values()} - set(edges)
    assert roots == {"robot_center"}
    for frame in ("livox_frame", "camera_link", "dog_imu_link", "torso_link"):
        chain(edges, "robot_center", frame)


def test_lidar_mount_is_upside_down_like_the_data():
    edges, _ = tree()
    t = chain(edges, "robot_center", "livox_frame")
    # livox z axis points down (floor along -z in livox_frame), mounted ~0.47 m above the pelvis
    assert t[2, 2] < -0.99
    assert t[2, 3] == pytest.approx(0.47, abs=0.02)
    t_torso = chain(edges, "torso_link", "livox_frame")
    assert np.degrees(np.arccos(np.clip(-t_torso[2, 2], -1, 1))) == pytest.approx(2.93, abs=0.1)


def test_head_camera_points_down_48_deg():
    edges, _ = tree()
    t = chain(edges, "robot_center", "camera_link")
    x_axis = t[:3, 0]  # camera_link: x forward
    assert np.degrees(np.arcsin(-x_axis[2])) == pytest.approx(47.6, abs=1.0)


def test_static_transforms_are_well_formed():
    _, cfg = tree()
    for tf in cfg["static_transforms"]:
        assert set(tf) == {"parent", "child", "xyz", "rpy"}
        assert len(tf["xyz"]) == 3 and len(tf["rpy"]) == 3
