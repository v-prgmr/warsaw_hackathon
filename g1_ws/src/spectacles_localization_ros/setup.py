"""ROS 2 Python package metadata."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "spectacles_localization_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alien Bazaar team",
    maintainer_email="team@example.invalid",
    description="Register Snap Spectacles local tracking to the G1 RTAB-Map frame.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "bridge_node = spectacles_localization_ros.bridge_node:main",
        "registration_node = spectacles_localization_ros.registration_node:main",
        "mock_node = spectacles_localization_ros.mock_node:main",
        "mock_verify_node = spectacles_localization_ros.mock_verify_node:main",
        "laptop_mock_client = spectacles_localization_ros.laptop_mock_client:main",
    ]},
)
