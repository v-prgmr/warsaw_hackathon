import os
from glob import glob

from setuptools import find_packages, setup

package_name = "g1_calibration"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jgilaviles",
    maintainer_email="jgilaviles@gmail.com",
    description="Chessboard calibration of the chest OAK-D (intrinsics, OAK-RealSense, "
                "OAK-LiDAR).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "calib_capture = g1_calibration.capture_node:main",
            "calib_trigger = g1_calibration.trigger:main",
            "calibrate_extrinsics = g1_calibration.calibrate_extrinsics:main",
            "calibrate_intrinsics = g1_calibration.calibrate_intrinsics:main",
            "make_board = g1_calibration.make_board:main",
        ],
    },
)
