# ==== БАЗОВЫЙ ОБРАЗ ====
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV CURRENT_ROS_WS=/home/mobile/Q_ground_control

# ==== СОЗДАНИЕ ПОЛЬЗОВАТЕЛЯ ====
ARG USERNAME=mobile
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ==== ОБНОВЛЕНИЕ И БАЗОВЫЕ ПАКЕТЫ ====
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
       curl gnupg2 lsb-release net-tools python3-pip git build-essential \
       squashfs-tools gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
       libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
       libcanberra-gtk-module libcanberra-gtk3-module at-spi2-core \
       x11-apps xauth wget \
       python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ==== ЧИСТИМ ДУБЛИКАТЫ РЕПО ROS2 ====
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list

# ==== ИНИЦ rosdep ====
RUN rosdep init || true && rosdep update

# ==== УСТАНОВКА CAN-UTILS ====
RUN git clone https://github.com/linux-can/can-utils.git /tmp/can-utils \
    && cd /tmp/can-utils && make && make install && rm -rf /tmp/can-utils

# ==== УСТАНОВКА ROS ПАКЕТОВ ====
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-urdfdom \
    ros-${ROS_DISTRO}-urdfdom-headers \
    ros-${ROS_DISTRO}-urdf-tutorial \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-moveit \
    && rm -rf /var/lib/apt/lists/*

# ==== УСТАНОВКА QGroundControl ====
RUN mkdir -p ${CURRENT_ROS_WS} \
    && wget https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage \
        -O ${CURRENT_ROS_WS}/QGroundControl-x86_64.AppImage \
    && chmod +x ${CURRENT_ROS_WS}/QGroundControl-x86_64.AppImage \
    && usermod -aG dialout $USERNAME \
    && mkdir -p /etc/systemd/system \
    && ln -sf /dev/null /etc/systemd/system/ModemManager.service

# ==== УСТАНОВКА PX4 + Micro XRCE-DDS ====
RUN cd /home/$USERNAME \
    && git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
    && bash ./PX4-Autopilot/Tools/setup/ubuntu.sh \
    && git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && sed -i '98s|2.12|2.13|' CMakeLists.txt \
    && sed -i '99s|2.12.x|2.13.3|' CMakeLists.txt \
    && mkdir build && cd build && cmake .. && make && make install && ldconfig /usr/local/lib/

# ==== ДОБАВЛЯЕМ SOURCE В bashrc ====
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /ws_moveit2/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME


USER $USERNAME
CMD ["bash"]

