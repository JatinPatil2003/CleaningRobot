import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    naviclean_coverage = get_package_share_directory("naviclean_mapping")

    map_yaml_file = os.path.join(naviclean_coverage, "maps", "cleaning_10x10.yaml")

    map_yaml_file_arg = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_file,  
        description="Full path to the map YAML file",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("naviclean_coverage"), "config", "coverage_params.yaml"]
        ),
        description="Full path to the ROS 2 parameter file",
    )

    # Node definition with the parameter file passed, overriding the yaml_file
    coverage_planner_node = Node(
        package="naviclean_coverage",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("param_file"),  # Load parameters from the param file
            {
                "yaml_file": LaunchConfiguration("map")
            },  
        ],
    )

    # Return the launch description
    return LaunchDescription(
        [
            map_yaml_file_arg,  # Add the map YAML argument
            param_file_arg,  # Add the parameter file argument
            coverage_planner_node,  # Add the node
        ]
    )
