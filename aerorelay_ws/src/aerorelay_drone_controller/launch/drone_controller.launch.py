"""
Launches a single drone_controller_node instance inside a given namespace,
matching whatever namespace you launched that drone's MAVROS instance under.

Usage:
    ros2 launch aerorelay_drone_controller drone_controller.launch.py \
        drone_id:=drone1 namespace:=drone1

    ros2 launch aerorelay_drone_controller drone_controller.launch.py \
        drone_id:=drone2 namespace:=drone2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='drone1')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='drone1')

    drone_id = LaunchConfiguration('drone_id')
    namespace = LaunchConfiguration('namespace')

    controller_node = Node(
        package='aerorelay_drone_controller',
        executable='drone_controller_node',
        name='drone_controller_node',
        namespace=namespace,
        output='screen',
        parameters=[{'drone_id': drone_id}],
    )

    return LaunchDescription([
        drone_id_arg,
        namespace_arg,
        controller_node,
    ])
