#!/usr/bin/env python3
"""
teleop.launch.py — Téléopération manette + arbitrage twist_mux pour SeekurJR.

Chaîne :
    joy_node → teleop_twist_joy_node → /cmd_vel_joy ┐
                                                     ├→ twist_mux → /cmd_vel → seekur_driver
    (Nav2, plus tard) →              /cmd_vel_nav   ┘

twist_mux sort un Twist SIMPLE (use_stamped: false) — voir config/twist_mux.yaml.
Joystick prioritaire (100) sur navigation (10).

À lancer en parallèle d'un launch de plateforme (ex: sim.launch.py).
Pour du joystick nu sans mux, utiliser joystick.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    joystick_params = PathJoinSubstitution([
        FindPackageShare('seekur_driver'), 'config', 'joystick.yaml',
    ])
    twist_mux_params = PathJoinSubstitution([
        FindPackageShare('seekur_driver'), 'config', 'twist_mux.yaml',
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joystick_params],
        output='screen',
    )

    # teleop publie sur /cmd_vel_joy (et non /cmd_vel) pour passer par le mux
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joystick_params],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params],
        # twist_mux publie sur /cmd_vel_out par défaut → remap vers /cmd_vel
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_mux_node,
    ])