import os
from glob import glob

from setuptools import find_packages, setup

package_name = "g1_ar_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "requirements-sim.txt"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Javgilavi",
    maintainer_email="jgilaviles@gmail.com",
    description="Snap Spectacles AR bridge: wall AprilTag alignment of the glasses with the "
                "RTAB-Map map, robot pose / LiDAR / POIs to the Lens (protocol v19).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "ar_bridge = g1_ar_bridge.bridge_node:main",
            "tag_anchor = g1_ar_bridge.tag_anchor_node:main",
            "publish_demo_pois = g1_ar_bridge.demo_pois_node:main",
            "sim_bridge = g1_ar_bridge.sim_main:main",
        ],
    },
)
