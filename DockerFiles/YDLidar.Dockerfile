FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-common

COPY naviclean_entrypoint.bash /naviclean_entrypoint.bash

RUN chmod +x /naviclean_entrypoint.bash

COPY /ydlidar_ros2_driver /colcon_ws/src/ydlidar_ros2_driver

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.sh \
    && colcon build --symlink-install
    # && rm -rf log/ build/ src/ \
    # && apt-get autoremove -y \
    # && apt-get autoclean -y \
    # && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/naviclean_entrypoint.bash"]

CMD ["bash"]