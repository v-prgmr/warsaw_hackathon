"""mapping.launch.py: every option combination yields a consistent node graph.

Checks the wiring without running the nodes: one odometry source owns odom -> base, the cloud
RTAB-Map and icp_odometry consume is produced by someone, the IMU topic exists, and static
fallback extrinsics only appear with static_tf:=true.
"""
import importlib.util
import itertools
import os

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, "config", "g1_mapping.yaml")


def load_launch():
    spec = importlib.util.spec_from_file_location(
        "mapping_launch", os.path.join(PKG, "launch", "mapping.launch.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


LAUNCH = load_launch()
DEFAULTS = {"config": CONFIG, "odom_source": "icp", "use_sim_time": "false",
            "use_imu": "true", "imu_source": "dog", "deskewing": "true", "use_rgbd": "false",
            "static_tf": "true", "database_path": "/tmp/test.db", "delete_db": "true",
            "localization": "false", "rtabmap_viz": "false", "rviz": "false"}


def _text(ctx, subs):
    if isinstance(subs, (list, tuple)):
        return "".join(_text(ctx, s) for s in subs)
    return subs.perform(ctx) if hasattr(subs, "perform") else subs


def _value(ctx, v):
    """Parameter values: strings arrive as tuples of substitutions; bools / numbers as is."""
    if isinstance(v, (list, tuple)) and v and all(hasattr(s, "perform") for s in v):
        return yaml.safe_load(_text(ctx, v))  # launch_ros YAML-encodes them
    return v


def graph(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update({**DEFAULTS, **overrides})
    nodes = []
    for n in LAUNCH._launch_setup(ctx):
        if not isinstance(n, Node):
            continue
        params = {}
        for p in n._Node__parameters:
            params.update({_text(ctx, k): _value(ctx, v) for k, v in p.items()})
        nodes.append({
            "pkg": n.node_package, "exe": n.node_executable,
            "name": _text(ctx, n._Node__node_name) if n._Node__node_name else n.node_executable,
            "remap": {_text(ctx, a): _text(ctx, b) for a, b in n._Node__remappings},
            "params": params, "args": [_text(ctx, a) for a in (n._Node__arguments or [])],
        })
    return nodes


def by_exe(nodes, exe):
    return [n for n in nodes if n["exe"] == exe]


COMBOS = list(itertools.product(["icp", "dog_odom"], ["true", "false"], ["dog", "livox"],
                                ["true", "false"], ["true", "false"], ["true", "false"]))


@pytest.mark.parametrize("odom,use_imu,imu,deskew,rgbd,static", COMBOS)
def test_graph_is_consistent(odom, use_imu, imu, deskew, rgbd, static):
    nodes = graph(odom_source=odom, use_imu=use_imu, imu_source=imu, deskewing=deskew,
                  use_rgbd=rgbd, static_tf=static)
    names = [n["name"] for n in nodes]
    assert len(names) == len(set(names)), f"duplicate node names: {names}"

    # Exactly one owner of odom -> robot_center.
    icp, bridge = by_exe(nodes, "icp_odometry"), by_exe(nodes, "odom_to_tf")
    assert (len(icp), len(bridge)) == ((1, 0) if odom == "icp" else (0, 1))
    (slam,) = by_exe(nodes, "rtabmap")
    assert slam["params"]["map_frame_id"] == "map"
    assert slam["remap"]["odom"] == "/odom"

    # Every consumed cloud is produced: livox_cloud_fix -> [lidar_deskewing] -> consumers.
    (fix,) = by_exe(nodes, "livox_cloud_fix")
    assert fix["remap"]["input"] == "/utlidar/cloud_livox_mid360"
    produced = {fix["remap"]["output"]}
    for d in by_exe(nodes, "lidar_deskewing"):
        assert d["remap"]["input_cloud"] in produced
        produced.add(d["remap"]["input_cloud"] + "/deskewed")  # rtabmap_util's output name
    for consumer in icp + [slam]:
        assert consumer["remap"]["scan_cloud"] in produced

    # Deskewing: via lidar_deskewing in a fixed frame, or inside icp_odometry, never both.
    external = bool(by_exe(nodes, "lidar_deskewing"))
    if deskew == "true":
        internal = bool(icp) and icp[0]["params"]["deskewing"]
        assert external != internal
        if external:
            frame = by_exe(nodes, "lidar_deskewing")[0]["params"]["fixed_frame_id"]
            if odom == "icp":
                (i2tf,) = by_exe(nodes, "imu_to_tf")
                assert i2tf["params"]["fixed_frame_id"] == frame == "robot_center_stabilized"
            else:
                assert frame == "odom"
    else:
        assert not external and not (icp and icp[0]["params"]["deskewing"])

    # The IMU topic handed to the consumers exists.
    imu_consumers = [n for n in icp + [slam] if n["remap"].get("imu", "").startswith("/")]
    if use_imu == "true":
        assert slam["remap"]["imu"] in ("/dog_imu_raw", "/g1_mapping/imu_livox/data")
        if imu == "livox":
            (filt,) = by_exe(nodes, "complementary_filter_node")
            assert filt["remap"]["imu/data"] == slam["remap"]["imu"]
            assert filt["params"]["do_bias_estimation"] is True
            (lfix,) = by_exe(nodes, "livox_imu_fix")
            assert filt["remap"]["imu/data_raw"] == lfix["remap"]["output"]
        for c in imu_consumers:
            assert c["remap"]["imu"] == slam["remap"]["imu"]
    else:
        assert "imu" not in slam["remap"]
        assert not by_exe(nodes, "complementary_filter_node")

    # RGB-D only when asked; static fallback TF only with static_tf.
    assert bool(by_exe(nodes, "rgbd_sync")) == (rgbd == "true")
    assert slam["params"]["subscribe_rgbd"] == (rgbd == "true")
    statics = by_exe(nodes, "static_transform_publisher")
    assert bool(statics) == (static == "true")


def test_static_tf_matches_yaml():
    nodes = graph()
    children = sorted(n["args"][n["args"].index("--child-frame-id") + 1]
                      for n in by_exe(nodes, "static_transform_publisher"))
    assert children == ["camera_link", "dog_imu_link", "livox_frame"]


def test_localization_keeps_database():
    nodes = graph(localization="true")
    (slam,) = by_exe(nodes, "rtabmap")
    assert slam["params"]["Mem/IncrementalMemory"] == "false"
    assert "-d" not in slam["args"]
    (slam,) = by_exe(graph(), "rtabmap")
    assert "-d" in slam["args"]


def test_icp_parameters_derived_from_voxel():
    (icp,) = by_exe(graph(), "icp_odometry")
    assert icp["params"]["Icp/VoxelSize"] == "0.1"
    assert float(icp["params"]["Icp/MaxCorrespondenceDistance"]) == pytest.approx(1.0)


@pytest.mark.parametrize("arg,value", [("odom_source", "fastlio"), ("imu_source", "head")])
def test_rejects_unknown_sources(arg, value):
    with pytest.raises(RuntimeError):
        graph(**{arg: value})
