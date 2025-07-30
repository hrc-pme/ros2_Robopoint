# ros2_Robopoint

This repository contains a set of ROS&nbsp;2 packages used to run the **RoboPoint** demo.  The main components are:

- `robopoint_worker` – provides a worker node that loads a multimodal model and exposes services for text/image generation.
- `robopoint_controller` – manages worker registration and dispatches requests.
- `robopoint_ros2` – client node for sending queries and visualizing results.
- `robopoint_interfaces` and `robopoint_msgs` – custom messages and service definitions used by the other packages.

A Docker environment is provided to simplify setup.

## Prerequisites

- Docker and NVIDIA GPU drivers (for GPU acceleration)
- ROS&nbsp;2 Humble (already installed inside the Docker image)

## Building the Docker image

```bash
cd docker
docker build -t ros2-humble-dockeruser -f Dockerfile .
```

## Running the container

Use the helper script which launches the container with GPU and X11 support:

```bash
bash docker/docker_run.sh
```

When the script starts you will be dropped into an interactive shell inside `/workspace`.  Choose `ros2_ws` from the menu or manually enter:

```bash
cd ros2_ws
```

## Building the workspace

Inside the container build all packages with `colcon` and source the environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/local_setup.bash
```

## Launching the nodes

1. **Controller** – starts the worker manager:

   ```bash
   ros2 launch robopoint_controller controller.launch.py
   ```

2. **Worker** – loads a model and connects to the controller:

   ```bash
   ros2 launch robopoint_worker model_worker.launch.py
   ```

   You can override model settings by passing parameters, e.g. `model_path:=wentao-yuan/robopoint-v1-vicuna-v1.5-13b`.

3. **Client** – sends queries and publishes results:

   ```bash
   ros2 launch robopoint_ros2 robopoint_launch.py
   ```

   Use `--ros-args` to set parameters such as `controller_url` or `model_name`.

## Repository layout

```
ros2_ws/            ROS 2 workspace
├── src/
│   ├── robopoint_worker/       Worker node package
│   ├── robopoint_controller/   Controller node package
│   ├── robopoint_ros2/         Client demo node
│   ├── robopoint_interfaces/   Interface definitions
│   └── robopoint_msgs/         Message/service definitions
└── ...
```

`docker/` contains the Dockerfile and `docker_run.sh` helper.

## Notes

The worker node relies on a GPU to run the chosen model.  Make sure your host machine has a compatible NVIDIA GPU and that Docker is configured to access it.
