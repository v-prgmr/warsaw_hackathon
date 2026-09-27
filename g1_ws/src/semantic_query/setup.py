import os
from glob import glob

from setuptools import find_packages, setup

package_name = "semantic_query"

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
    description="NL object detection to 3D POIs (Grounding DINO + SAM2, OAK-D depth).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "poi_node = semantic_query.poi_node:main",
            "fake_oak_pub = semantic_query.fake_oak_pub:main",
        ],
    },
)
