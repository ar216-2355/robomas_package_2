from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robomas_package_2'

    # パッケージ直下の sender_params.yaml を取得
    pkg_share = get_package_share_directory(pkg_name)
    sender_param_file = os.path.join(pkg_share, 'sender_params.yaml')

    sender_node = Node(
        package=pkg_name,
        executable='sender',
        parameters=[sender_param_file],
        output='screen'
    )

    receiver_node = Node(
        package=pkg_name,
        executable='receiver',
        output='screen'
    )

    return LaunchDescription([
        sender_node,
        receiver_node,
    ])
