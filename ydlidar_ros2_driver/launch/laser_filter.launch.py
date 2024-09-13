import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    naviclean_laser_filter_dir = get_package_share_directory('ydlidar_ros2_driver')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    naviclean_laser_filter_config_file = LaunchConfiguration('config')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='naviclean',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the naviclean_laser_filter config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the sar stack')

    declare_naviclean_laser_filter_config_file_cmd = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(naviclean_laser_filter_dir, 'params', 'laser_filter.yaml'),
        description='Full path to the naviclean_laser_filter config file to use')

    # Launch laser_filters
    start_naviclean_laser_filter_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[naviclean_laser_filter_config_file],
        output='screen')

    namespaced_naviclean_laser_filter_config_file = ReplaceString(
        source_file=naviclean_laser_filter_config_file,
        replacements={'<robot_namespace>': ('/', namespace)})

    start_namespaced_naviclean_laser_filter_cmd = Node(
        condition=IfCondition(use_namespace),
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        namespace=namespace,
        parameters=[namespaced_naviclean_laser_filter_config_file],
        output='screen',
        remappings=[('/scan_filtered', 'scan_filtered')])

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_naviclean_laser_filter_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='naviclean_laser_filter exited'))))

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_naviclean_laser_filter_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='naviclean_laser_filter exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_naviclean_laser_filter_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_naviclean_laser_filter_cmd)
    ld.add_action(start_namespaced_naviclean_laser_filter_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
