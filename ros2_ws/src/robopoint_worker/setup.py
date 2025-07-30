from setuptools import setup, find_packages
from glob import glob

package_name = 'robopoint_worker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),  # ✅ 自動包含 robopoint_worker 子模組
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
