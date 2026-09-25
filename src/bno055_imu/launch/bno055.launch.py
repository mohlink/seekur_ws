"""Lance le nœud bno055_imu avec config/bno055_params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bno055_imu'), 'config', 'bno055_params.yaml')

    return LaunchDescription([
        Node(
            package='bno055_imu',
            executable='bno055_serial_node',
            name='bno055_imu',          # doit correspondre à la clé du YAML
            parameters=[params],
            output='screen',
        ),
    ])
