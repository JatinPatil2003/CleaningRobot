ARG BASE_IMAGE=ros:humble

FROM $BASE_IMAGE as librealsense-builder

SHELL ["/bin/bash", "-c"]

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    python3-pip \
    ca-certificates \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN git clone https://github.com/IntelRealSense/librealsense.git
RUN ln -s /usr/src/librealsense /usr/src/librealsense

FROM ${BASE_IMAGE} as librealsense

COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/


RUN apt-get update \
    && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

COPY naviclean_entrypoint.bash /naviclean_entrypoint.bash

RUN chmod +x /naviclean_entrypoint.bash

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.sh \
    && colcon build --symlink-install
    # && rm -rf log/ build/ src/ \
    # && apt-get autoremove -y \
    # && apt-get autoclean -y \
    # && rm -rf /var/lib/apt/lists/* 

ENTRYPOINT ["/naviclean_entrypoint.bash"]

CMD ["bash"]
