from g1_nav2 import sim_environment
import pytest


def test_ros_dds_precedes_sdk_without_losing_workspace_libraries(monkeypatch):
    monkeypatch.setattr(sim_environment, 'get_package_prefix', lambda _: '/ros/humble')
    monkeypatch.setattr(sim_environment.sysconfig, 'get_config_var', lambda _: 'x86_64-linux-gnu')
    result = sim_environment.ros_library_path('/workspace/rl_sar/lib:/other/lib:/ros/humble/lib:')
    assert result.split(':') == [
        '/ros/humble/lib/x86_64-linux-gnu', '/ros/humble/lib',
        '/workspace/rl_sar/lib', '/other/lib',
    ]


def test_simulation_environment_overrides_shared_robot_graph(monkeypatch):
    monkeypatch.setattr(sim_environment, 'ros_library_path', lambda _: '/ros/lib')
    original = {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
                'CYCLONEDDS_URI': 'hardware.xml', 'DISPLAY': ':1'}
    env = sim_environment.simulation_environment(original)
    assert env['ROS_DOMAIN_ID'] == '76'
    assert env['ROS_LOCALHOST_ONLY'] == '1'
    assert '127.0.0.1' in env['CYCLONEDDS_URI']
    assert '<Interfaces>' not in env['CYCLONEDDS_URI']
    assert env['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert env['DISPLAY'] == ':1'
    assert original['CYCLONEDDS_URI'] == 'hardware.xml'


def test_domain_override_and_invalid_domain(monkeypatch):
    monkeypatch.setattr(sim_environment, 'ros_library_path', lambda _: '/ros/lib')
    assert sim_environment.simulation_environment({'G1_SIM_DOMAIN_ID': '77'})['ROS_DOMAIN_ID'] == '77'
    with pytest.raises(ValueError):
        sim_environment.simulation_environment({'G1_SIM_DOMAIN_ID': '250'})
