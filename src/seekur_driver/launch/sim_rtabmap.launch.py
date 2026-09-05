#!/usr/bin/env python3
"""
sim_rtabmap.launch.py - Chaine de simulation SeekurJR AVEC SLAM 3D RTAB-Map (P4)

Variante de sim_ekf.launch.py ou l'on ajoute RTAB-Map en mode SLAM 3D pur
(Option A) pour construire une carte 3D dense + occupancy grid + graph de
poses avec detection de fermeture de boucle.

CHAINE COMPLETE :
  Gazebo + driver + EKF (via sim_ekf.launch.py)
    -> /odometry/filtered + TF odom->base_footprint
  + camera D455 sim -> /camera/color/*, /camera/depth/*
  + LiDAR sim -> /scan
    -> RTAB-Map consomme tout ca
      -> /map (occupancy grid 2D), /mapData, /mapGraph, TF map->odom

CHOIX ARCHITECTURAL - Option A1 (odometrie EKF externe) :
  RTAB-Map consomme /odometry/filtered de l'EKF, ne calcule PAS sa propre
  odometrie visuelle. L'EKF reste la source unique de verite pour la position.
  Alternative A2 (ICP odometry interne) reservee pour usage sur robot reel
  si l'EKF s'avere insuffisant en galerie de mine reelle.

REMPLACE slam_toolbox :
  RTAB-Map devient le SLAM de reference. slam.launch.py reste disponible
  pour comparaison mais ne doit pas tourner en meme temps que ce launch.

Usage :
  ros2 launch seekur_driver sim_rtabmap.launch.py
  ros2 launch seekur_driver sim_rtabmap.launch.py world:=mine_gallery.sdf

Visualisation SLAM detaillee (optionnelle, dans un autre terminal) :
  ros2 run rtabmap_viz rtabmap_viz --ros-args \\
    -p subscribe_depth:=true -p subscribe_rgb:=true -p subscribe_scan:=true \\
    -p approx_sync:=true -p frame_id:=base_link -p odom_frame_id:=odom \\
    -r rgb/image:=/camera/color/image_raw \\
    -r rgb/camera_info:=/camera/color/camera_info \\
    -r depth/image:=/camera/depth/image_rect_raw \\
    -r odom:=/odometry/filtered -r scan:=/scan

Arguments : les memes que sim_ekf.launch.py (world, use_sim_time, rviz).
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

    # Chemin rtabmap_params.yaml resolu immediatement (chemin absolu).
    rtabmap_config = os.path.join(
        get_package_share_directory('seekur_driver'),
        'config', 'rtabmap_params.yaml'
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
            description='Monde SDF (defaut mine_gallery pour tests P1/P4)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Lancer RViz2',
        ),

        # --- Chaine sim + EKF (reutilisation complete) ----------------------
        # sim_ekf.launch.py inclut deja sim.launch.py avec publish_tf:=false
        # et demarre l'EKF a 4s. Rien a re-configurer ici.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim_ekf.launch.py'
                ])
            ]),
        ),

        # --- RTAB-Map SLAM 3D -----------------------------------------------
        # Delai 6 s : demarrer apres l'EKF (lance a 4 s dans sim_ekf.launch.py)
        # pour que /odometry/filtered existe quand RTAB-Map s'abonne.
        # Remaps : les topics attendus par le noeud sont mappes vers ceux
        # reellement publies par le stack (camera D455, EKF, LiDAR).
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    parameters=[
                        rtabmap_config,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=[
                        ('rgb/image',       '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image',     '/camera/depth/image_rect_raw'),
                        ('odom',            '/odometry/filtered'),
                        ('scan',            '/scan'),
                    ],
                    # --delete_db_on_start : demarre chaque session avec une
                    # base vide (comportement equivalent a un slam_toolbox
                    # frais). Sans ce flag, RTAB-Map continue la session
                    # precedente stockee dans ~/.ros/rtabmap.db.
                    arguments=['--delete_db_on_start'],
                ),
            ]
        ),
    ])

