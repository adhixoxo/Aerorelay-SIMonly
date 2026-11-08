from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='8090')
    node = Node(
        package='aerorelay_customer_portal',
        executable='customer_portal_node',
        name='customer_portal_node',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}],
    )
    return LaunchDescription([port_arg, node])
