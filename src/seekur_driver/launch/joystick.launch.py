#!/usr/bin/env python3
"""
joystick.launch.py — Téléopération manette pour SeekurJR.

Lance joy_node + teleop_twist_joy_node, qui publient un geometry_msgs/msg/Twist
sur /cmd_vel, directement consommé par seekur_driver_node.

À lancer en parallèle d'un launch de plateforme (ex: sim.launch.py, sim_ekf.launch.py).

Manette utilisée : ShanWan PC/PS3/Android — voir config/joystick.yaml pour la
bascule mode 1 / mode 2 via bouton HOME.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    joystick_params = PathJoinSubstitution([
        FindPackageShare('seekur_driver'),
        'config',
        'joystick.yaml',
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joystick_params],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joystick_params],
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])