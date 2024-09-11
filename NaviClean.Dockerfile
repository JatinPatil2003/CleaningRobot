FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control* \
    ros-humble-slam-toolbox* \
    ros-humble-example-interfaces* \
    ros-humble-robot-localization* \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cartographer* \
    ros-humble-rviz2* \
    ros-humble-cv-bridge

RUN apt-get update && apt-get install -y \
    python3-pip \
    && python3 -m pip install -U \
    smbus \
    pyserial==3.4 \
    pynput

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

COPY naviclean_entrypoint.bash /naviclean_entrypoint.bash

RUN chmod +x /naviclean_entrypoint.bash

RUN apt-get update && apt-get install -y \
    libserial-dev \
    apt-utils

COPY /naviclean_controller /colcon_ws/src/naviclean_controller

COPY /naviclean_description /colcon_ws/src/naviclean_description

COPY /naviclean_firmware /colcon_ws/src/naviclean_firmware

COPY /naviclean_bringup /colcon_ws/src/naviclean_bringup

COPY /naviclean_mapping /colcon_ws/src/naviclean_mapping

COPY /naviclean_navigation /colcon_ws/src/naviclean_navigation

COPY /naviclean_coverage /colcon_ws/src/naviclean_coverage

COPY /naviclean_utils /colcon_ws/src/naviclean_utils

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.sh \
    && colcon build --symlink-install
    # && rm -rf log/ build/ src/ \
    # && apt-get autoremove -y \
    # && apt-get autoclean -y \
    # && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/naviclean_entrypoint.bash"]

CMD ["bash"]