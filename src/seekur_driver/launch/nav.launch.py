#!/usr/bin/env python3
"""
nav.launch.py - Navigation autonome nav2 complete (N5)

Extension de N4 (map_server + AMCL). Ajoute au-dessus :
  - planner_server    : trajectoire globale
  - controller_server : suivi + evitement local
  - behavior_server   : recovery (spin, back_up, wait)
  - bt_navigator      : orchestration behavior tree
  - lifecycle_manager_navigation : active les 4 en batch

Meme pattern qu'avant : on INCLUT sim.launch.py (Gazebo + robot + bridge
+ robot_state_publisher + simulateur + driver + RViz2 avec use_sim_time
cable partout) et on ajoute par-dessus les 6 noeuds nav2 (2 localisation
+ 4 navigation).

L'utilisateur peut alors dans RViz :
  - voir la carte (Map sur /map, Transient Local)
  - voir la localisation AMCL (ParticleCloud sur /particle_cloud, Best Effort)
  - donner un GOAL avec le bouton '2D Goal Pose' -> le robot y va tout seul
  - voir le path global (vert) et le path local (rouge/bleu)
  - voir les costmaps (globale et locale, TRANSIENT_LOCAL)

Arguments : herites de sim.launch.py
    ros2 launch seekur_driver nav.launch.py world:=mine_gallery.sdf
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

        # --- Chaine sim complete (via sim.launch.py) ------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim.launch.py'
                ])
            ]),
        ),

        # --- Nav2 : delai 3s pour laisser Gazebo + bridge se stabiliser ----
        TimerAction(
            period=3.0,
            actions=[

                # --- LOCALISATION (N4, inchange) ---------------------------
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[nav2_params],
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_params],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[nav2_params],
                ),

                # --- NAVIGATION (N5, nouveau) ------------------------------
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params],
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params],
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params],
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params],
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
