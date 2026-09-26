"""Cross-package contracts and documentation sync (AGENTS.md §7, §17, §24).

Pure file checks (no ROS graph): recording profiles vs. the mandatory topic list, topic names
shared between packages, the record launch command, and that AGENTS.md / READMEs mention every
package and launch argument. When one of these fails after a change, update the docs or the
contract in the same commit.
"""
import ast
import glob
import importlib.util
import os
import re

import pytest
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(WS)
SRC = os.path.join(WS, "src")
AGENTS = open(os.path.join(REPO, "AGENTS.md")).read()


def load_yaml(*parts):
    with open(os.path.join(SRC, *parts)) as f:
        return yaml.safe_load(f)


def profile(name):
    return load_yaml("g1_recorder", "config", f"{name}.yaml")["topics"]


PACKAGES = sorted(os.path.basename(os.path.dirname(p))
                  for p in glob.glob(os.path.join(SRC, "*", "package.xml")))

# AGENTS.md §7 "Mandatory (a bag without these is not a canonical bag)"
MANDATORY = ["/utlidar/cloud_livox_mid360", "/dog_imu_raw", "/dog_odom", "/lf/lowstate",
             "/tf_static"]
ALSO_RECORD = ["/utlidar/imu_livox_mid360", "/secondary_imu", "/lf/bmsstate", "/tf"]


def test_agents_md_lists_the_mandatory_topics():
    section = AGENTS[AGENTS.index("**Mandatory**"):]
    block = section[section.index("```"):section.index("```", section.index("```") + 3)]
    assert re.findall(r"^(/\S+)", block, re.M) == MANDATORY


def test_survey_profile_is_canonical():
    topics = profile("survey")
    for t in MANDATORY + ALSO_RECORD:
        assert t in topics, f"survey.yaml misses {t} (AGENTS.md §7)"


@pytest.mark.parametrize("name", ["survey", "live_run", "rtab", "full_survey"])
def test_profiles_are_well_formed_and_never_record_lowcmd(name):
    topics = profile(name)
    assert topics and len(topics) == len(set(topics))
    assert all(t.startswith("/") for t in topics)
    assert not any("lowcmd" in t.lower() for t in topics), "no low-level command topic (§25)"


def test_live_run_superset_of_robot_streams():
    live = set(profile("live_run"))
    for t in ["/utlidar/cloud_livox_mid360", "/dog_imu_raw", "/dog_odom", "/lowstate", "/tf",
              "/tf_static", "/cmd_vel", "/joint_states", "/odom"]:
        assert t in live, f"live_run.yaml misses {t}"


def test_mapping_inputs_are_recorded():
    topics = load_yaml("g1_mapping", "config", "g1_mapping.yaml")["topics"]
    survey = profile("survey")
    for key in ("lidar", "imu", "imu_livox", "dog_odom", "rgb", "depth", "camera_info"):
        assert topics[key] in survey, f"g1_mapping input {topics[key]} not in survey.yaml"


def test_camera_topics_agree_between_packages():
    mapping = load_yaml("g1_mapping", "config", "g1_mapping.yaml")["topics"]
    kf = load_yaml("keyframe_manager", "config", "keyframe_params.yaml")["keyframe_node"][
        "ros__parameters"]
    assert (kf["color_topic"], kf["depth_topic"], kf["info_topic"]) == \
        (mapping["rgb"], mapping["depth"], mapping["camera_info"])


def test_sensor_topics_agree_between_packages():
    mapping = load_yaml("g1_mapping", "config", "g1_mapping.yaml")["topics"]
    sensors = load_yaml("g1_sensors", "config", "g1_sensors.yaml")["topics"]
    assert sensors["clock_reference"] == mapping["imu"]


def test_tf_static_is_recorded_transient_local():
    qos = load_yaml("g1_recorder", "config", "qos_override.yaml")
    assert qos["/tf_static"]["durability"] == "transient_local"
    assert qos["/tf_static"]["history"] == "keep_all"


@pytest.mark.parametrize("name", ["survey", "live_run", "rtab", "full_survey"])
def test_record_launch_command(name):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location(
        "record_launch", os.path.join(SRC, "g1_recorder", "launch", "record.launch.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    share = os.path.join(SRC, "g1_recorder")
    ctx.launch_configurations.update({
        "profile": name, "topics_file": os.path.join(share, "config", f"{name}.yaml"),
        "qos_file": os.path.join(share, "config", "qos_override.yaml"), "output": "take"})
    (proc,) = mod._launch_record(ctx)
    cmd = ["".join(s.perform(ctx) for s in part) for part in proc.cmd]
    assert cmd[:3] == ["ros2", "bag", "record"]
    assert cmd[cmd.index("--storage") + 1] == "mcap" and cmd[cmd.index("-o") + 1] == "take"
    assert "--qos-profile-overrides-path" in cmd
    assert cmd[-len(profile(name)):] == profile(name)  # positional topics (--topics rejected)


# --- documentation sync --------------------------------------------------------------------

@pytest.mark.parametrize("pkg", PACKAGES)
def test_every_package_is_documented(pkg):
    assert f"`{pkg}`" in AGENTS, f"AGENTS.md does not mention package {pkg}"
    assert f"`{pkg}`" in open(os.path.join(WS, "README.md")).read(), \
        f"g1_ws/README.md does not list package {pkg}"
    assert f"`{pkg}`" in open(os.path.join(REPO, "README.md")).read(), \
        f"README.md does not list package {pkg}"
    readme = os.path.join(SRC, pkg, "README.md")
    if pkg not in ("keyframe_manager", "scene_server"):  # documented in g1_ws/README.md
        assert os.path.isfile(readme), f"{pkg} has no README.md"


def launch_arguments(path):
    tree = ast.parse(open(path).read())
    return {node.args[0].value for node in ast.walk(tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", "") ==
            "DeclareLaunchArgument"}


@pytest.mark.parametrize("pkg,launch", [("g1_mapping", "mapping.launch.py"),
                                        ("g1_sensors", "tf_chain.launch.py"),
                                        ("g1_calibration", "capture.launch.py")])
def test_launch_arguments_are_documented(pkg, launch):
    path = os.path.join(SRC, pkg, "launch", launch)
    if not os.path.isfile(path):
        pytest.skip(f"{pkg} has no {launch}")
    readme = open(os.path.join(SRC, pkg, "README.md")).read()
    undocumented = [a for a in launch_arguments(path)
                    if a not in ("config",) and f"`{a}" not in readme]
    assert not undocumented, f"{pkg}/README.md does not document {undocumented}"


def test_console_scripts_exist():
    for setup in glob.glob(os.path.join(SRC, "*", "setup.py")):
        pkg_dir = os.path.dirname(setup)
        for mod, fn in re.findall(r"=\s*([\w.]+):(\w+)", open(setup).read()):
            path = os.path.join(pkg_dir, *mod.split(".")) + ".py"
            assert os.path.isfile(path), f"{setup}: {path} missing"
            assert re.search(rf"^def {fn}\(", open(path).read(), re.M), f"{path}: no {fn}()"
