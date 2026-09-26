"""Keep ROS simulation processes separate from the SDK's bundled DDS libraries."""

import os
import sysconfig
import sys
import shutil

from ament_index_python.packages import get_package_prefix


# ROS_LOCALHOST_ONLY selects lo in Humble's RMW. Do not also select lo in XML:
# combining the two configurations selects the same interface twice.
# Explicit loopback peers avoid relying on loopback multicast support.
SIM_CYCLONEDDS_URI = '''<CycloneDDS><Domain id="any">
  <General>
    <AllowMulticast>false</AllowMulticast>
  </General>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>80</MaxAutoParticipantIndex>
    <Peers><Peer address="127.0.0.1"/></Peers>
  </Discovery>
</Domain></CycloneDDS>'''


def ros_library_path(inherited):
    """Prefer the CycloneDDS installed alongside the ROS RMW implementation.

    Sourcing rl_sar puts its lib/ first on LD_LIBRARY_PATH. That directory
    also contains the Unitree SDK's libddsc.so.0, which must not override the
    ROS distribution's library in Nav2, Gazebo, RViz, or the ROS bridges.
    This changes only child processes of the simulation launch.
    """
    prefix = get_package_prefix('rmw_cyclonedds_cpp')
    multiarch = sysconfig.get_config_var('MULTIARCH')
    preferred = [os.path.join(prefix, 'lib', multiarch)] if multiarch else []
    preferred.append(os.path.join(prefix, 'lib'))
    paths = preferred + [path for path in inherited.split(os.pathsep) if path]
    return os.pathsep.join(dict.fromkeys(paths))


def simulation_environment(inherited):
    """Return a consistent environment for MuJoCo, navigation, and CLI probes."""
    env = dict(inherited)
    domain = int(env.get('G1_SIM_DOMAIN_ID', '76'))
    if not 0 <= domain <= 101:
        raise ValueError('G1_SIM_DOMAIN_ID must be between 0 and 101')
    env.update(
        ROS_DOMAIN_ID=str(domain),
        ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        CYCLONEDDS_URI=SIM_CYCLONEDDS_URI,
        LD_LIBRARY_PATH=ros_library_path(env.get('LD_LIBRARY_PATH', '')),
        GAZEBO_MASTER_URI='http://127.0.0.1:11476',
    )
    return env


def main():
    """Execute a ros2 simulation command with a private, local-only DDS graph."""
    if len(sys.argv) == 1:
        print('Usage: ros2 run g1_nav2 sim_ros2 <ros2 command and arguments>')
        print('Examples: run rl_sar rl_mujoco g1 scene_29dof; '
              'launch g1_nav2 nav_amcl.launch.py world_name:=square_loop.world')
        return
    executable = shutil.which('ros2')
    if executable is None:
        raise RuntimeError('Source ROS Humble and the workspace before using sim_ros2')
    env = simulation_environment(os.environ)
    print(f"G1 simulation: domain={env['ROS_DOMAIN_ID']}, loopback only, ROS CycloneDDS", flush=True)
    os.execve(executable, [executable, *sys.argv[1:]], env)
