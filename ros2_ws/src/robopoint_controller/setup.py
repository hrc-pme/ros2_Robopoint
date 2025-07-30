from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robopoint_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 controller node for managing distributed workers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = robopoint_controller.controller_node:main',
            'constants = robopoint_controller.constants:main',
            'utils = robopoint_controller.utils:main',
            'conversation = robopoint_controller.conversation:main',
        ],
    },
)
