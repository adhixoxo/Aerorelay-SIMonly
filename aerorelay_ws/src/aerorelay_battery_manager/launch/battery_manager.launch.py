from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone1_id_arg = DeclareLaunchArgument('drone1_id', default_value='drone1')
    drone2_id_arg = DeclareLaunchArgument('drone2_id', default_value='drone2')

    battery_node = Node(
        package='aerorelay_battery_manager',
        executable='battery_manager_node',
        name='battery_manager_node',
        output='screen',
        parameters=[{
            'drone1_id': LaunchConfiguration('drone1_id'),
            'drone2_id': LaunchConfiguration('drone2_id'),
        }],
    )

    return LaunchDescription([drone1_id_arg, drone2_id_arg, battery_node])
