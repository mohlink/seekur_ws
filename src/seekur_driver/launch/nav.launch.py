#!/usr/bin/env python3
"""
nav.launch.py - Localisation nav2 (AMCL) sur carte connue - N4

Meme pattern que slam.launch.py : on INCLUT sim.launch.py (Gazebo +
robot + bridge + robot_state_publisher + simulateur + driver + RViz2
avec use_sim_time cable partout) et on ajoute par-dessus les noeuds
nav2 de localisation.

Ce que ce launch fait EN PLUS de sim.launch.py :
  - lance map_server (sert la carte statique sur /map)
  - lance amcl (localise le robot dans la carte, publie map -> odom)
  - lance lifecycle_manager_localization qui les active tous les deux

L'utilisateur peut alors :
  - voir la carte dans RViz (display Map sur /map)
  - voir le nuage de particules AMCL (display PoseArray sur /particle_cloud)
  - donner une pose initiale au robot via '2D Pose Estimate' de RViz
  - piloter le robot en teleop -> voir la localisation converger

En N5 on ajoutera par-dessus le planner + controller + BT navigator
pour transformer 'localisation' en 'navigation autonome'.

Arguments : herites de sim.launch.py + le YAML nav2
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

    # Chemin nav2_params.yaml resolu IMMEDIATEMENT (chemin en dur).
    # Meme lecon que slam.launch.py : plusieurs noeuds nav2 ignorent
    # silencieusement un params-file passe comme substitution non
    # resolue dans parameters=[]. Toujours passer un chemin absolu.
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
            description='Monde SDF (defaut: mine_gallery.sdf pour N4)',
        ),

        # --- Chaine de simulation complete (via sim.launch.py) --------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim.launch.py'
                ])
            ]),
        ),

        # --- MAP SERVER -----------------------------------------------------
        # Lifecycle node : lance ici, active par le lifecycle_manager plus bas.
        # Delai 3 s pour laisser Gazebo + bridge se stabiliser (meme delai
        # que le driver dans sim.launch.py, pour la meme raison).
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[nav2_params],
                ),

                # --- AMCL ---------------------------------------------------
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_params],
                ),

                # --- LIFECYCLE MANAGER --------------------------------------
                # Configure + active map_server puis amcl (ordre du YAML).
                # autostart:true dans le YAML declenche tout au demarrage,
                # pas besoin des EmitEvent qu'on avait dans slam.launch.py.
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[nav2_params],
                ),
            ]
        ),
    ])
