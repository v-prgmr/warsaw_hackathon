import os
from glob import glob

from setuptools import find_packages, setup

package_name = "g1_sensors"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stanislawix",
    maintainer_email="stanislawix@gmail.com",
    description="The G1's /tf chain: /lowstate bridge, URDF, static glue frames.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "lowstate_to_joint_states = g1_sensors.lowstate_to_joint_states:main",
            "bms_to_battery_state = g1_sensors.bms_to_battery_state:main",
        ],
    },
)
