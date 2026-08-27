#!/usr/bin/env python3
"""
nav_ekf.launch.py - Navigation autonome nav2 par-dessus la chaine EKF

Variante de nav.launch.py qui branche nav2 sur l'odometrie fusionnee
(/odometry/filtered) au lieu de l'odom brute du driver (/odom).

Deux differences avec nav.launch.py :
  1. On inclut sim_ekf.launch.py au lieu de sim.launch.py
     -> le driver publie /odom (topic) mais pas la TF odom->base_footprint
     -> l'EKF (robot_localization) prend en charge la TF et publie
        /odometry/filtered
  2. On remap /odom -> /odometry/filtered sur les noeuds nav2
     -> quand nav2 veut lire /odom (parametre odom_topic du YAML,
        defaut du bt_navigator, etc.), il lit en fait
        /odometry/filtered
     -> le YAML nav2_params.yaml reste INTOUCHE

Pourquoi ce montage :
  - nav.launch.py reste dispo pour comparer (odom brute)
  - nav2_params.yaml reste source de verite unique
  - le remap est cote abonne, pas cote publisher : le driver continue
    de publier /odom (topic) pour d'autres usages (bag pour analyse
    comparative apres coup) sans conflit

Note sur amcl : amcl utilise la TF odom->base_footprint, pas le topic
/odom. Le remap est inoffensif pour lui (pas de subscription /odom).
On le met par uniformite avec les autres noeuds.

Arguments : herites de sim_ekf.launch.py (donc sim.launch.py)
    ros2 launch seekur_driver nav_ekf.launch.py world:=mine_gallery.sdf
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = FindPackageShare('seekur_driver')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params = os.path.join(
        get_package_share_directory('seekur_driver'),
        'config', 'nav2_params.yaml'
    )

    # Remap unique applique a tous les noeuds nav2 qui pourraient
    # s'abonner a /odom (bt_navigator, controller_server surtout).
    # Sans effet sur ceux qui ne s'y abonnent pas.
    ekf_remappings = [('/odom', '/odometry/filtered')]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true en simulation, false sur le vrai robot',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='mine_gallery.sdf',
            description='Monde SDF (defaut: mine_gallery.sdf pour N4/N5)',
        ),

        # --- Chaine sim + EKF complete (via sim_ekf.launch.py) --------------
        # sim_ekf.launch.py inclut lui-meme sim.launch.py avec
        # publish_tf:=false et ajoute le noeud ekf_filter_node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim_ekf.launch.py'
                ])
            ]),
        ),

        # --- Nav2 : delai 3s pour laisser Gazebo + bridge + EKF se ---------
        # stabiliser (l'EKF a besoin de quelques ticks IMU + odom avant
        # de publier une TF stable)
        TimerAction(
            period=3.0,
            actions=[

                # --- LOCALISATION (N4) -------------------------------------
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[nav2_params],
                ),

                # --- NAVIGATION (N5) ---------------------------------------
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params],
                    remappings=ekf_remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[nav2_params],
                ),
            ]
        ),
    ])
