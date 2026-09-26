from glob import glob
from setuptools import setup

package_name = "g1_semantic_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Alien Bazaar team",
    maintainer_email="team@example.com",
    description="OAK-D color and COCO semantics over an RTAB-Map LiDAR cloud",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "semantic_map_node = g1_semantic_map.semantic_map_node:main",
        "synthetic_scene = g1_semantic_map.synthetic_scene:main",
        "synthetic_check = g1_semantic_map.synthetic_check:main",
    ]},
)
