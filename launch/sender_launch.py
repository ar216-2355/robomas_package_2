from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sender_node = Node(
        package='robomas_package_2',
        executable='sender',
    )
    
    return LaunchDescription([
        sender_node,
    ])
