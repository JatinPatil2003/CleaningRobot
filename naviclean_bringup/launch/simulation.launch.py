import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        # os.path.join(get_package_share_directory("naviclean_description"),"launch","gazebo.launch.py")
        os.path.join(
            get_package_share_directory("naviclean_description"),
            "launch",
            "gazebo.launch.py",
        )
    )

    odom_repub = Node(
        package="topic_tools",
        executable="relay",
        name="odom_relay",
        output="screen",
        arguments=["/odom", "/naviclean_controller/odom"],
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("naviclean_controller"),
            "launch",
            "controller.launch.py",
        )
    )

    return LaunchDescription(
        [
            gazebo,
            # controller,
            odom_repub,
        ]
    )
