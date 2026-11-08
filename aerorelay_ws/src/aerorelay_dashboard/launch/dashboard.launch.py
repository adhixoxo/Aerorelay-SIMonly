from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone1_id_arg = DeclareLaunchArgument('drone1_id', default_value='drone1')
    drone2_id_arg = DeclareLaunchArgument('drone2_id', default_value='drone2')
    port_arg = DeclareLaunchArgument('port', default_value='8080')

    dashboard_node = Node(
        package='aerorelay_dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=[{
            'drone1_id': LaunchConfiguration('drone1_id'),
            'drone2_id': LaunchConfiguration('drone2_id'),
            'port': LaunchConfiguration('port'),
        }],
    )

    return LaunchDescription([drone1_id_arg, drone2_id_arg, port_arg, dashboard_node])
