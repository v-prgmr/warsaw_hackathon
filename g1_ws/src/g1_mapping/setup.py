import os
from glob import glob

from setuptools import find_packages, setup

package_name = "g1_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stanislawix",
    maintainer_email="stanislawix@gmail.com",
    description="RTAB-Map LiDAR-inertial mapping for the G1.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "livox_cloud_fix = g1_mapping.livox_cloud_fix:main",
            "odom_to_tf = g1_mapping.odom_to_tf:main",
        ],
    },
)
