from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g1_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # Params files
        (os.path.join('share', package_name, 'params'),
         glob('params/*.yaml')),

        # Lightweight simulation RViz view
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jean',
    maintainer_email='icmayoko18@gmail.com',
    description='G1 Nav2 bringup (fake base)',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_odom_tf = g1_nav2.map_odom_tf:main',
            'sim_ros2 = g1_nav2.sim_environment:main',
            'check_nav_goal = g1_nav2.check_nav_goal:main',
            'navigation_tf_relay = g1_nav2.navigation_tf_relay:main',
        ],
    },
)
