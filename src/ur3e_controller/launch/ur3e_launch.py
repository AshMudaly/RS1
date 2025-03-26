import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_controller',
            executable='ur3e_control',  # This will be your executable node name
            name='ur3e_control_node',
            output='screen'
        ),
    ])

