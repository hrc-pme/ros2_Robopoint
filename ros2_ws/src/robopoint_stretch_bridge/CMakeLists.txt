cmake_minimum_required(VERSION 3.5)
project(robopoint_stretch_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(robopoint_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_package()
