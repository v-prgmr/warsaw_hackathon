"""Skeleton node's pure helpers: URDF -> bones, bones -> limb chains -> polyline points."""
from g1_ar_bridge.skeleton import chain_point_lists, chains, parse_bones

URDF = """<?xml version="1.0"?>
<robot name="g1">
  <link name="pelvis"/>
  <link name="left_hip"/>
  <link name="left_knee"/>
  <link name="camera_link"/>
  <joint name="j_hip" type="revolute">
    <parent link="pelvis"/><child link="left_hip"/>
  </joint>
  <joint name="j_knee" type="revolute">
    <parent link="left_hip"/><child link="left_knee"/>
  </joint>
  <joint name="j_cam" type="fixed">
    <parent link="pelvis"/><child link="camera_link"/>
  </joint>
</robot>"""


def test_parse_bones_reads_every_joint_in_order():
    assert parse_bones(URDF) == [("pelvis", "left_hip"), ("left_hip", "left_knee"),
                                 ("pelvis", "camera_link")]


def test_parse_bones_skips_joints_missing_a_link():
    urdf = ('<robot name="r"><joint name="j" type="fixed"><parent link="a"/></joint>'
            '<joint name="k" type="fixed"><parent link="a"/><child link="b"/></joint></robot>')
    assert parse_bones(urdf) == [("a", "b")]


def test_chains_are_root_to_leaf_paths_one_per_leaf():
    # pelvis -> left_hip -> left_knee (leaf); pelvis -> camera_link (leaf)
    assert chains(parse_bones(URDF)) == [["pelvis", "left_hip", "left_knee"],
                                         ["pelvis", "camera_link"]]


def test_chain_points_are_the_resolved_positions_along_each_limb():
    bones = [("pelvis", "left_hip"), ("left_hip", "left_knee"), ("pelvis", "camera_link")]
    positions = {"pelvis": (0.0, 0.0, 1.0), "left_hip": (0.0, 0.1, 0.9),
                 "left_knee": (0.0, 0.1, 0.5), "camera_link": (0.1, 0.0, 1.2)}
    assert chain_point_lists(bones, positions) == [
        [(0.0, 0.0, 1.0), (0.0, 0.1, 0.9), (0.0, 0.1, 0.5)],
        [(0.0, 0.0, 1.0), (0.1, 0.0, 1.2)],
    ]


def test_chain_breaks_at_a_missing_frame_instead_of_jumping_across_it():
    bones = [("a", "b"), ("b", "c"), ("c", "d")]         # one chain a-b-c-d
    positions = {"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "d": (3.0, 0.0, 0.0)}  # c missing
    # a-b is a run of 2; d alone is dropped -> no segment jumps b straight to d
    assert chain_point_lists(bones, positions) == [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]]


def test_no_strokes_when_nothing_resolves():
    assert chain_point_lists([("a", "b")], {}) == []
