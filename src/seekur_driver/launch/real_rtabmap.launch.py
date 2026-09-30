#!/usr/bin/env python3
"""
real_rtabmap.launch.py - SLAM RTAB-Map SUR LE VRAI ROBOT SeekurJR

Pendant reel de sim_rtabmap.launch.py. Construit sur real_ekf.launch.py :
  driver + LiDAR LMS111 + IMU BNO055 + camera D435 + EKF
    -> /odometry/filtered + TF odom->base_footprint (EKF)
  + RTAB-Map (odometrie EKF externe, option A1 comme en sim)
    -> /map (occupancy grid depuis le LiDAR), /mapData, /mapGraph, TF map->odom

DEUX MODES (argument camera) :

  camera:=true (defaut) - COMME LA SIM
    config/rtabmap_params.yaml tel quel : RGB + profondeur alignee + LiDAR,
    fermetures de boucle visuelles, carte 3D. Topics camera_front identiques
    a la sim (real.launch.py lance la D435, USB 3, 848x480 a 30 Hz).

  camera:=false - LIDAR SEUL (repli)
    real.launch.py ne lance pas la camera (l'argument lui est transmis),
    et ce launch surcharge les abonnements (pas de RGB ni de depth) avec
    la configuration LiDAR des demos RTAB-Map :
      Reg/Strategy 1            recalage ICP sur les scans (0 = visuel)
      Reg/Force3DoF true        x, y, lacet seulement (robot au sol)
      RGBD/NeighborLinkRefining affine chaque lien odometrique par ICP
      RGBD/ProximityBySpace     fermetures de boucle par proximite (scans)
      Kp/MaxFeatures -1         pas de mots visuels : supprime le
                                "Missing visual features" a chaque seconde
    Essai au lab 2026-09-30 : carte correcte mais murs plus epais que
    slam_toolbox ; en 2D LiDAR seul, preferer real_slam.launch.py.

LE LMS111 N'EST PAS PRET JUSTE APRES L'ALLUMAGE DU ROBOT :
  attendre ~1 min apres la mise sous tension avant de lancer.

Usage (Jetson ; RViz sur le ROG) :
  ros2 launch seekur_driver real_rtabmap.launch.py rviz:=false joy:=false
  ros2 launch seekur_driver real_rtabmap.launch.py rviz:=false joy:=false camera:=false
La base est effacee a chaque lancement (--delete_db_on_start) ; retirer
l'argument pour continuer une session.

Ne pas lancer en meme temps que real_slam.launch.py (deux proprietaires
de map->odom).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share_dir = get_package_share_directory('seekur_driver')
    rtabmap_config = os.path.join(pkg_share_dir, 'config', 'rtabmap_params.yaml')

    real_common = {'use_sim_time': False}

    # Mode camera : memes topics que sim_rtabmap.launch.py.
    camera_remaps = [
        ('rgb/image',       '/camera_front/color/image_raw'),
        ('rgb/camera_info', '/camera_front/color/camera_info'),
        ('depth/image',     '/camera_front/aligned_depth_to_color/image_raw'),
        ('odom',            '/odometry/filtered'),
        ('scan',            '/scan'),
    ]

    # Mode LiDAR seul. Parametres RTAB-Map core en string (piege documente
    # dans rtabmap_params.yaml).
    lidar_only = {
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
        'Kp/MaxFeatures': '-1',         # pas de mots visuels sans images
    }

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='true', description='Lancer RViz2'),
        DeclareLaunchArgument('joy', default_value='true',
                              description='Lancer la chaine manette'),
        # Transmis implicitement a real.launch.py (via real_ekf.launch.py) :
        # camera:=false n'y lance pas non plus la D435.
        DeclareLaunchArgument('camera', default_value='true',
                              description='RTAB-Map avec camera (true) ou LiDAR seul (false)'),

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
                    condition=IfCondition(LaunchConfiguration('camera')),
                    parameters=[rtabmap_config, real_common],
                    remappings=camera_remaps,
                    arguments=['--delete_db_on_start'],
                ),
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration('camera')),
                    parameters=[rtabmap_config, real_common, lidar_only],
                    remappings=[
                        ('odom', '/odometry/filtered'),
                        ('scan', '/scan'),
                    ],
                    arguments=['--delete_db_on_start'],
                ),
            ],
        ),
    ])
