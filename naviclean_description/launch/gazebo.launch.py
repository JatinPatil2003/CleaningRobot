import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():

    naviclean_description = get_package_share_directory("naviclean_description")
    naviclean_description_prefix = get_package_prefix("naviclean_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    world = os.path.join(naviclean_description, "worlds", "cafe.world")

    # Set the environment variable
    model_path = os.path.join(naviclean_description, "models")
    model_path += pathsep + os.path.join(naviclean_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_path = xacro.process_file(
        os.path.join(naviclean_description, "urdf", "naviclean.xacro")
    ).toxml()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description_path}
        ],
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "bumperbot",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(env_var)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)
    ld.add_action(spawn_robot_node)
    ld.add_action(robot_state_publisher_node)

    return ld
