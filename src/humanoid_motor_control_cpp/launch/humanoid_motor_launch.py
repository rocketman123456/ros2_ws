from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_motor_control_cpp',
            namespace='pi',
            executable='humanoid_motor_example',
            name='motor'
        ),
    ])