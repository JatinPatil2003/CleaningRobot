FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy \
    jstest-gtk \
    # bluez \
    # bluetooth \
    # dbus \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp 

COPY /naviclean_controller /colcon_ws/src/naviclean_controller

WORKDIR /colcon_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install 
    # && rm -rf log/ build/ src/ \
    # && apt-get autoremove -y \
    # && apt-get autoclean -y \
    # && rm -rf /var/lib/apt/lists/*
    
COPY ./naviclean_entrypoint.bash /naviclean_entrypoint.bash
RUN chmod +x /naviclean_entrypoint.bash

ENTRYPOINT ["/naviclean_entrypoint.bash"]
CMD ["bash"]