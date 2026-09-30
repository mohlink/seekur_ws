#!/usr/bin/env python3
"""
real_rtabmap.launch.py - SLAM RTAB-Map SUR LE VRAI ROBOT SeekurJR (LiDAR seul)

Pendant reel de sim_rtabmap.launch.py. Construit sur real_ekf.launch.py :
  driver + LiDAR LMS111 + IMU BNO055 + EKF
    -> /odometry/filtered + TF odom->base_footprint (EKF)
  + RTAB-Map (odometrie EKF externe, option A1 comme en sim)
    -> /map (occupancy grid depuis le LiDAR), /mapData, /mapGraph, TF map->odom

LIDAR SEUL, POUR L'INSTANT :
  En sim, RTAB-Map consomme aussi la camera (RGB + depth). Sur le vrai
  robot la D435 est encore en USB 2 (15 Hz) et sa configuration est dans
  la branche feat/camera-front, non fusionnee. On demarre donc sans
  camera : config/rtabmap_params.yaml est reutilise, et ce launch
  surcharge les abonnements (pas de RGB ni de depth).

  Sans images, pas de detection visuelle des fermetures de boucle. On
  passe a la configuration LiDAR des demos RTAB-Map :
    Reg/Strategy 1            recalage ICP sur les scans (0 = visuel)
    Reg/Force3DoF true        x, y, lacet seulement (robot au sol)
    RGBD/NeighborLinkRefining affine chaque lien odometrique par ICP
    RGBD/ProximityBySpace     fermetures de boucle par proximite (scans)
  Quand la camera sera prete, ces surcharges disparaissent et on revient
  a la configuration de la sim.

LE LMS111 N'EST PAS PRET JUSTE APRES L'ALLUMAGE DU ROBOT :
  attendre ~1 min apres la mise sous tension avant de lancer.

Usage (Jetson ; RViz sur le ROG) :
  ros2 launch seekur_driver real_rtabmap.launch.py rviz:=false joy:=false
La base est effacee a chaque lancement (--delete_db_on_start) ; retirer
l'argument pour continuer une session.

Ne pas lancer en meme temps que real_slam.launch.py (deux proprietaires
de map->odom).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share_dir = get_package_share_directory('seekur_driver')
    rtabmap_config = os.path.join(pkg_share_dir, 'config', 'rtabmap_params.yaml')

    # Surcharges materiel : LiDAR seul. Parametres RTAB-Map core en string
    # (piege documente dans rtabmap_params.yaml).
    lidar_only = {
        'use_sim_time': False,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan': True,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Grid/RangeMin': '0.5',         # min_laser_range du LMS111 (comme slam_toolbox)
        'Optimizer/GravitySigma': '0',  # pas de contrainte de gravite en 3DoF
    }

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='true', description='Lancer RViz2'),
        DeclareLaunchArgument('joy', default_value='true',
                              description='Lancer la chaine manette'),

        # --- Chaine reelle + EKF ---------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, 'launch', 'real_ekf.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'joy': LaunchConfiguration('joy'),
            }.items(),
        ),

        # --- RTAB-Map ----------------------------------------------------------
        # Delai 10 s : le driver reel met ~9 s a s'initialiser (demarrage a
        # 2 s + SYNC/OPEN/ENABLE/SETO), l'EKF demarre a 4 s. RTAB-Map
        # s'abonne ainsi a une odometrie deja publiee.
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    parameters=[rtabmap_config, lidar_only],
                    remappings=[
                        ('odom', '/odometry/filtered'),
                        ('scan', '/scan'),
                    ],
                    arguments=['--delete_db_on_start'],
                ),
            ],
        ),
    ])
