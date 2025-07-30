FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei

# 安裝基本工具與 GUI 套件
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales sudo \
    build-essential software-properties-common \
    libasound2-dev ffmpeg libsm6 libxext6 libxrender-dev \
    git wget unzip x11-apps \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# 新增非 root 使用者 docker
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 加入 ROS 2 的套件來源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list

# 安裝 ROS 2 Humble desktop-full
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-argcomplete \
    whiptail \
 && rm -rf /var/lib/apt/lists/*

# 環境變數
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# 初始化 rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# 設定 ROS 環境到 docker 使用者 bashrc
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# 安裝 Python 套件
RUN pip3 install --upgrade pip && \
    pip3 install soundfile transformers

# 建立 ros2_ws 並設定權限
WORKDIR /workspace/ros2_ws
RUN mkdir -p src && chown -R $USERNAME:$USERNAME /workspace

# 切換為非 root 使用者 docker
USER $USERNAME

# 初始化 ros2_ws（不報錯就好）
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install || true

# 拷貝內容進容器
WORKDIR /workspace
COPY --chown=$USERNAME:$USERNAME . /workspace

# 預設啟動 bash
CMD ["bash"]
