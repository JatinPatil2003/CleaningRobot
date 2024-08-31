import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false",
    )

    robot_description_file = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("naviclean_description"),
                    "urdf",
                    "naviclean.xacro",
                ),
                " is_sim:=False",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_file}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_file, "use_sim_time": False},
            os.path.join(
                get_package_share_directory("naviclean_controller"),
                "config",
                "naviclean_controller.yaml",
            ),
        ],
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("naviclean_controller"),
            "launch",
            "controller.launch.py",
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("naviclean_bringup"), "rviz", "bringup.rviz"
            ),
        ],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)
    ld.add_action(controller_manager)
    ld.add_action(controller)
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)

    return ld
