from setuptools import find_packages, setup

package_name = "scene_server"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="inko",
    maintainer_email="inko.inkreate@gmail.com",
    description="STUB viz: synthetic /vggt/scene_cloud in vggt_world for the M3 surface.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "ply_publisher = scene_server.ply_publisher:main",
        ],
    },
)
