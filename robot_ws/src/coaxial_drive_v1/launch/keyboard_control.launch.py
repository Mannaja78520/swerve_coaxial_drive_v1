import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # Path to config file for keyboard control node
    speed_config = os.path.join(
        get_package_share_directory('coaxial_drive_v1'),
        'config',
        'speed_control.yaml'
    )

    keyboard_node = Node(
        package='coaxial_drive_v1',
        executable='keyboard_control.py',
        name='Keyboard_Node',
        parameters=[speed_config],
        # output='screen'
    )
    
    ld.add_action(keyboard_node)

    return ld
