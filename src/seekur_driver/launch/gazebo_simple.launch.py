#!/usr/bin/env python3
"""
gazebo_simple.launch.py - Simulation SeekurJR SANS ros2_control

Architecture (identique au robot reel) :
    Twist sur /cmd_vel -> plugin DiffDrive natif de Gazebo -> mouvement
    Gazebo -> /joint_states -> robot_state_publisher -> TF roues -> RViz2
    Gazebo -> /scan (frame renomme par bridge) -> RViz2 / nav2

BRIDGE : configuration centralisee dans config/gz_bridge.yaml.
Le champ 'ros_frame_id' du YAML remplace le frame_id auto-genere par
Gazebo, ce qui remplace elegamment le static_transform_publisher qu'on
utilisait avant pour aliaser 'seekur_jr/base_footprint/laser_scanner'
vers 'laser_frame'.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Nom du modele spawn dans Gazebo (utilise aussi dans le YAML du bridge
# pour construire /world/empty/model/<MODEL_NAME>/joint_state).
# Si tu changes ceci, modifie aussi gz_bridge.yaml.
MODEL_NAME = 'seekur_jr'


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('seekur_driver'), 'urdf', 'seekur_jr_simple.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    bridge_config = PathJoinSubstitution([
        FindPackageShare('seekur_driver'), 'config', 'gz_bridge.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Lancer RViz2'),

        # --- Gazebo Harmonic ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items()
        ),

        # --- Robot State Publisher ------------------------------------------
        # Consomme /joint_states et publie les TF des roues.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen'
        ),

        # --- Spawn du robot dans Gazebo -------------------------------------
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', MODEL_NAME,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

        # --- Pont ROS2 <-> Gazebo -------------------------------------------
        # Toute la config est dans gz_bridge.yaml (topics, types, direction,
        # et ros_frame_id pour renommer le frame du LiDAR au vol).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),

        # --- Alias TF pour le frame_id du LiDAR ------------------------------
        # Gazebo tamponne ses scans avec un frame_id auto-genere :
        #   seekur_jr/base_footprint/laser_scanner
        # alors que l'URDF ne connait que 'laser_frame'.
        # Le champ ros_frame_id de gz_bridge.yaml reglerait ca proprement,
        # mais il est IGNORE par ros_gz_bridge 1.0.x (Jazzy) - teste 2026-08.
        # D'ou cette TF identite en attendant une version du bridge qui le
        # supporte (voir note dans gz_bridge.yaml).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_alias',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'laser_frame',
                '--child-frame-id', f'{MODEL_NAME}/base_footprint/laser_scanner',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # --- RViz2 -----------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('seekur_driver'), 'config', 'seekur_viz.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
