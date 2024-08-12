from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_world_odom',
            output='screen',
            arguments=['2', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_vectornav',
            output='screen',
            arguments=['0.06', '0', '0', '0.707', '0.707', '0', '0', 'base_link', 'vectornav']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_laser',
            output='screen',
            arguments=['-0.25', '0', '0', '0', '-0.7071', '0', '0.7071', 'base_link', 'base_scan']
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
