FROM ros:noetic

ENV DEBIAN_FRONTEND=noninterac

RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial* \
    ros-noetic-cv-bridge* \
    ros-noetic-rospy* \
    ros-noetic-gazebo*

RUN apt-get install -y libudev-dev \
    libusb*

COPY /cleaning_robot_coverage /catkin_ws/src/cleaning_robot_coverage

COPY /cleaning_robot_description /catkin_ws/src/cleaning_robot_description

COPY /cleaning_robot_gazebo /catkin_ws/src/cleaning_robot_gazebo

COPY /cleaning_robot_hardware /catkin_ws/src/cleaning_robot_hardware

COPY /cleaning_robot_navigation /catkin_ws/src/cleaning_robot_navigation

COPY /full_coverage_path_planner /catkin_ws/src/full_coverage_path_planner

WORKDIR /catkin_ws

# RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
#     && catkin_make'

# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD bash