import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('leo_nav'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'leo_nav',
        name = 'turtle_pos_controller',
        executable = 'turtle_pos_controller',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
