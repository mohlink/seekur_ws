#!/usr/bin/env python3
"""
sim_ekf.launch.py - Chaine de simulation SeekurJR AVEC fusion EKF (P1b)

Variante de sim.launch.py ou la localisation odometrique est produite par
un filtre de Kalman etendu (robot_localization) fusionnant :
    /odom      (odometrie roues, du driver)
  + /imu/data  (gyroscope + accelerometre, du plugin Gazebo IMU)
    -> /odometry/filtered + TF odom->base_footprint

DIFFERENCE CLE avec sim.launch.py :
  - le driver tourne avec publish_tf:=false (il publie /odom mais PAS la TF)
  - l'EKF devient le PROPRIETAIRE UNIQUE de la TF odom->base_footprint
  Ainsi un seul noeud publie cette transform, jamais deux (sinon arbre TF
  qui saute, lecon N2).

Pourquoi un launch separe plutot qu'un flag :
  On garde sim.launch.py intact (driver publie sa TF, chaine simple) et on
  a ce fichier dedie pour la chaine "avec fusion". On peut ainsi comparer
  proprement AVEC vs SANS EKF (utile pour quantifier le gain, notamment sur
  le scenario de patinage).

Usage :
  ros2 launch seekur_driver sim_ekf.launch.py world:=mine_gallery.sdf

Arguments : les memes que sim.launch.py (world, use_sim_time, rviz).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = FindPackageShare('seekur_driver')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Chemin ekf.yaml resolu immediatement (chemin absolu).
    ekf_config = os.path.join(
        get_package_share_directory('seekur_driver'),
        'config', 'ekf.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true en simulation, false sur le vrai robot',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='mine_gallery.sdf',
            description='Monde SDF (defaut mine_gallery pour tests P1)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Lancer RViz2',
        ),

        # --- Chaine sim, mais driver SANS publication TF --------------------
        # publish_tf:=false -> le driver publie /odom mais laisse la TF
        # odom->base_footprint a l'EKF. C'est LE point de bascule.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim.launch.py'
                ])
            ]),
            launch_arguments={
                'publish_tf': 'false',
            }.items()
        ),

        # --- EKF robot_localization -----------------------------------------
        # Delai 4 s : demarrer apres le driver (lance a 3 s dans sim.launch.py)
        # pour que /odom et /imu/data existent quand l'EKF s'abonne.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_config],
                ),
            ]
        ),
    ])
