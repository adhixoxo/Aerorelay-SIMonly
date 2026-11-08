from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drone1_id', default_value='drone1'),
        DeclareLaunchArgument('drone2_id', default_value='drone2'),
    ]
    node = Node(
        package='aerorelay_parcel_visualizer',
        executable='parcel_visualizer_node',
        name='parcel_visualizer_node',
        output='screen',
        parameters=[{
            'drone1_id': LaunchConfiguration('drone1_id'),
            'drone2_id': LaunchConfiguration('drone2_id'),
        }],
    )
    return LaunchDescription(args + [node])
