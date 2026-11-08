from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='drone1')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='drone1')

    node = Node(
        package='aerorelay_precision_landing',
        executable='precision_landing_node',
        name='precision_landing_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{'drone_id': LaunchConfiguration('drone_id')}],
    )

    return LaunchDescription([drone_id_arg, namespace_arg, node])
