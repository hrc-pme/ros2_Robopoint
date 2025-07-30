from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robopoint_worker'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Create both lib and libexec directories for compatibility
        ('lib/' + package_name, []),
        ('libexec/' + package_name, []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 Model Worker for RoboPoint',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_model_worker = robopoint_worker.ros2_model_worker:main',
        ],
    },
)