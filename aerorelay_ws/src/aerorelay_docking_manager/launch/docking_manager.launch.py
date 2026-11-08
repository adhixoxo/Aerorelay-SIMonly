from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drone1_id', default_value='drone1'),
        DeclareLaunchArgument('drone2_id', default_value='drone2'),
        DeclareLaunchArgument('drone1_dock_x', default_value='-0.5'),
        DeclareLaunchArgument('drone1_dock_y', default_value='-0.5'),
        DeclareLaunchArgument('drone1_dock_z', default_value='0.0'),
        DeclareLaunchArgument('drone2_dock_x', default_value='0.5'),
        DeclareLaunchArgument('drone2_dock_y', default_value='0.5'),
        DeclareLaunchArgument('drone2_dock_z', default_value='0.0'),
        DeclareLaunchArgument('dock_radius', default_value='1.0'),
    ]

    docking_node = Node(
        package='aerorelay_docking_manager',
        executable='docking_manager_node',
        name='docking_manager_node',
        output='screen',
        parameters=[{
            'drone1_id': LaunchConfiguration('drone1_id'),
            'drone2_id': LaunchConfiguration('drone2_id'),
            'drone1_dock_x': LaunchConfiguration('drone1_dock_x'),
            'drone1_dock_y': LaunchConfiguration('drone1_dock_y'),
            'drone1_dock_z': LaunchConfiguration('drone1_dock_z'),
            'drone2_dock_x': LaunchConfiguration('drone2_dock_x'),
            'drone2_dock_y': LaunchConfiguration('drone2_dock_y'),
            'drone2_dock_z': LaunchConfiguration('drone2_dock_z'),
            'dock_radius': LaunchConfiguration('dock_radius'),
        }],
    )

    return LaunchDescription(args + [docking_node])
