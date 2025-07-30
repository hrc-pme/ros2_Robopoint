from setuptools import setup
import os
from glob import glob

package_name = 'robopoint_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for RoboPoint spatial affordance prediction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robopoint_node = robopoint_ros2.robopoint_node:main',
            'robopoint_client = robopoint_ros2.robopoint_client:main',
        ],
    },
)
