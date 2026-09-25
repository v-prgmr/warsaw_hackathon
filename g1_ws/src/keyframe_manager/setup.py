import os
from glob import glob

from setuptools import find_packages, setup

package_name = "keyframe_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="inko",
    maintainer_email="inko.inkreate@gmail.com",
    description="Selects and exports RGB-D keyframes for VGGT reconstruction.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "keyframe_node = keyframe_manager.keyframe_node:main",
            "fake_rgbd_pub = keyframe_manager.fake_rgbd_pub:main",
        ],
    },
)
