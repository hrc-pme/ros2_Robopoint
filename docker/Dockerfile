# ==== Base ====
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
# GUI / Qt：使用 X11，強制 xcb 平台
ENV QT_QPA_PLATFORM=xcb
# 指向系統的 Qt 外掛目錄（避免讀到 OpenCV 內建的）
ENV QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms
# 若無硬體 GL，加上這行可避免崩潰（必要時可移除）
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV PYTHONUNBUFFERED=1
SHELL ["/bin/bash", "-c"]

# ==== 基本工具 & GUI 依賴 ====
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales sudo \
    build-essential software-properties-common \
    git wget unzip x11-apps xauth dbus-x11 \
    libasound2-dev ffmpeg libsm6 libxext6 libxrender1 \
    libx11-xcb1 libxcb1 libxcb-render0 libxcb-shape0 libxcb-xfixes0 \
    libxcb-xinerama0 libxcb-randr0 libxcb-image0 libxcb-keysyms1 libxcb-icccm4 \
    libxcb-cursor0 libxkbcommon-x11-0 libgl1-mesa-glx libglu1-mesa mesa-utils \
    libqt5gui5 libqt5widgets5 libqt5x11extras5 \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ==== 非 root 使用者 ====
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ==== ROS 2 套件來源 ====
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list

# ==== 安裝 ROS 2 Humble desktop-full ====
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-argcomplete \
    whiptail \
 && rm -rf /var/lib/apt/lists/*

# ==== rosdep ====
RUN rosdep init 2>/dev/null || true && rosdep update

# ==== ROS 環境給 bashrc ====
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# ---- Global pip constraints: numpy<2 （永久限制 pip 解析）----
RUN printf "numpy<2\n" > /etc/pip-constraints.txt && \
    mkdir -p /etc/pip && \
    printf "[global]\nconstraint = /etc/pip-constraints.txt\n" > /etc/pip/pip.conf
# 讓所有 pip 預設都套用（即使某些工具忽略 pip.conf）
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt

# ==== Python 套件（避免跟 apt 衝突）====
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --ignore-installed "sympy>=1.13.3" && \
    python3 -m pip install --no-cache-dir --ignore-installed \
      "numpy==1.24.3" "scipy==1.11.4" && \
    python3 -m pip install --no-cache-dir \
      soundfile transformers accelerate sentencepiece protobuf && \
    python3 -m pip install --no-cache-dir \
      torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128 && \
    python3 -m pip install --no-cache-dir opencv-python && \
    true

# ==== 移走 OpenCV 內建的 Qt plugins，避免 xcb plugin 衝突 ====
# 有些 OpenCV 輪子會放在 /usr/local/lib/python3.10/dist-packages/cv2/qt/plugins
RUN OPENCV_QT_DIR="/usr/local/lib/python3.10/dist-packages/cv2/qt/plugins" && \
    if [ -d "$OPENCV_QT_DIR" ]; then mv "$OPENCV_QT_DIR" "${OPENCV_QT_DIR}.bak"; fi

# ==== 建立工作區 ====
WORKDIR /workspace/ros2_ws
RUN mkdir -p src && chown -R $USERNAME:$USERNAME /workspace

# ==== 切到非 root ====
USER $USERNAME

# ==== 初次 colcon（空專案不致失敗即可）====
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install || true

# ==== 拷貝你的專案進容器 ====
WORKDIR /workspace
COPY --chown=$USERNAME:$USERNAME . /workspace

# ==== 預設啟動 ====
CMD ["bash"]
