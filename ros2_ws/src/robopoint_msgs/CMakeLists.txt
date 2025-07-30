cmake_minimum_required(VERSION 3.8)
project(robopoint_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GenerateRequest.msg"
  "msg/GenerateResponse.msg"
  "msg/HeartBeat.msg"
  "msg/WorkerStatus.msg"
  "srv/RegisterWorker.srv"
  "srv/GetWorkerAddress.srv"
  "srv/ReceiveHeartBeat.srv"
  "srv/ListModels.srv"
  "srv/WorkerGenerateStream.srv"
  "srv/WorkerGetStatus.srv"
  DEPENDENCIES std_msgs builtin_interfaces sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
