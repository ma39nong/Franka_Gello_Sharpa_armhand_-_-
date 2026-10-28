FROM ros:humble-ros-base

ARG LIBFRANKA_VERSION=0.19.0
ARG FRANKA_DESCRIPTION_TAG=1.0.1
ARG ORBBEC_ROS2_TAG=v2.8.6
ARG ROSDISTRO_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/rosdistro
ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget \
    python3-pip python3-venv python3-yaml python3-numpy python3-prompt-toolkit \
    iproute2 \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-controller-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-ros2controlcli \
    ros-humble-joint-state-broadcaster \
    ros-humble-generate-parameter-library \
    ros-humble-realtime-tools \
    ros-humble-backward-ros \
    ros-humble-xacro \
    ros-humble-control-msgs \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    libgflags-dev nlohmann-json3-dev libdw-dev libssl-dev libgoogle-glog-dev \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport ros-humble-image-publisher \
    ros-humble-camera-info-manager ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs ros-humble-statistics-msgs \
    libpoco-dev libeigen3-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# The vendored Linker Hand driver needs python-can >= 4 for can.exceptions;
# the jammy apt package is 3.3.4, so install the same version the host runs.
RUN python3 -m pip install --no-cache-dir 'python-can==4.3.1'

RUN wget -q \
      "https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VERSION}/libfranka_${LIBFRANKA_VERSION}_jammy_amd64.deb" \
    && apt-get update \
    && apt-get install -y "./libfranka_${LIBFRANKA_VERSION}_jammy_amd64.deb" \
    && rm "libfranka_${LIBFRANKA_VERSION}_jammy_amd64.deb" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/vendor_ws/src
COPY patches/orbbec_ros2_skip_uvc_for_network.patch /tmp/
COPY patches/orbbec_monitor_access_mode.patch /tmp/
RUN git clone --depth 1 --branch v2.0.4 \
        https://github.com/frankarobotics/franka_ros2.git \
    && git clone --depth 1 --branch "${FRANKA_DESCRIPTION_TAG}" \
        https://github.com/frankarobotics/franka_description.git \
    && git clone --depth 1 --branch "${ORBBEC_ROS2_TAG}" \
        https://github.com/orbbec/OrbbecSDK_ROS2.git \
    && git -C OrbbecSDK_ROS2 apply \
        /tmp/orbbec_ros2_skip_uvc_for_network.patch \
    && git -C OrbbecSDK_ROS2 apply \
        /tmp/orbbec_monitor_access_mode.patch \
    && rm -rf \
        franka_ros2/franka_gazebo \
        franka_ros2/franka_fr3_moveit_config \
        franka_ros2/franka_example_controllers \
        franka_ros2/franka_bringup \
        franka_ros2/integration_launch_testing \
        franka_ros2/franka_ros2

ENV ROSDISTRO_INDEX_URL=${ROSDISTRO_MIRROR}/index-v4.yaml
RUN sed -i \
      "s|https://raw.githubusercontent.com/ros/rosdistro/master|${ROSDISTRO_MIRROR}|g" \
      /etc/ros/rosdep/sources.list.d/20-default.list \
    && apt-get update \
    && (rosdep update || rosdep update || rosdep update) \
    && rosdep install --from-paths . --ignore-src -y \
        --dependency-types build \
        --dependency-types buildtool \
        --dependency-types build_export \
        --skip-keys "libfranka pinocchio" \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/vendor_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

# LeRobot 0.6 requires newer Python than Ubuntu 22.04 provides. Keep its
# dataset/video dependencies isolated from the ROS 2 Python runtime.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
RUN python3 -m pip install --no-cache-dir --retries 5 --timeout 60 \
        --index-url https://mirrors.aliyun.com/pypi/simple "uv==0.11.28" \
    && uv python install 3.12.13 \
    && uv venv --python "$(uv python find --managed-python 3.12.13)" \
        /opt/lerobot_venv \
    && uv pip install --python /opt/lerobot_venv/bin/python \
        --index-url https://download.pytorch.org/whl/cpu \
        "torch==2.7.1+cpu" "torchvision==0.22.1+cpu" \
        "torchcodec==0.5" \
    && uv pip install --python /opt/lerobot_venv/bin/python \
        --index-url "${PIP_INDEX_URL}" \
        "lerobot[dataset]==0.6.0" "rosbags==0.11.3" \
        "pyarrow==24.0.0" "av==15.1.0" \
    && ln -s /opt/lerobot_venv/bin/python /usr/local/bin/lerobot-python

# TorchCodec dynamically loads the complete FFmpeg ABI, including
# libavdevice. The ROS base image has only the codec/format subset.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*

COPY patches/franka_ros2_restore_thread_scheduler.patch /tmp/
RUN git -C src/franka_ros2 apply /tmp/franka_ros2_restore_thread_scheduler.patch \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select franka_hardware \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log /tmp/franka_ros2_restore_thread_scheduler.patch

# Optional local integrations are installed late so changes to them do not
# invalidate the expensive libfranka/vendor/LeRobot layers above.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-pilz-industrial-motion-planner \
        ros-humble-joint-trajectory-controller \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        python3-opencv \
    && python3 -m pip install --no-cache-dir 'nicegui>=2.24,<3' \
    && python3 -m pip install --no-cache-dir 'pyarrow==17.0.0' \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace/franka_upper_body_teleop
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
