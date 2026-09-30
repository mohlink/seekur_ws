#!/usr/bin/env python3
"""
real_slam.launch.py - SLAM 2D slam_toolbox SUR LE VRAI ROBOT SeekurJR

Pendant reel de slam.launch.py. Construit sur real_ekf.launch.py :
  driver + LiDAR LMS111 + IMU BNO055 + EKF
    -> /odometry/filtered + TF odom->base_footprint (EKF)
  + slam_toolbox (online_async)
    -> /map + TF map->odom

MEME CONFIGURATION QUE LA SIM :
  config/slam_toolbox_params.yaml est reutilise tel quel ; seul
  use_sim_time est surcharge ici (false sur le vrai robot).

LIDAR INCLINE (6,5 deg vers le haut, mesure au lab 2026-09-29) :
  slam_toolbox travaille en 2D et ignore l'inclinaison du TF. Chaque
  distance est donc lue ~0,6 % trop longue (1 / cos 6,5 deg). Acceptable
  pour une premiere carte ; a garder en tete pour les mesures precises.

LE LMS111 N'EST PAS PRET JUSTE APRES L'ALLUMAGE DU ROBOT :
  attendre ~1 min apres la mise sous tension avant de lancer, sinon lms1xx
  peut se bloquer ("Laser not ready", 2026-09-28).

Usage (Jetson ; RViz sur le ROG) :
  ros2 launch seekur_driver real_slam.launch.py rviz:=false joy:=false
Sauvegarde de la carte (sur la Jetson) :
  ros2 run nav2_map_server map_saver_cli -f ~/seekur_ws/maps/<nom>

Ne pas lancer en meme temps que real_rtabmap.launch.py (deux proprietaires
de map->odom).
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():

    pkg_share_dir = get_package_share_directory('seekur_driver')
    # Chemin resolu immediatement : slam_toolbox ignore un params-file
    # passe en substitution non resolue (lecon N2).
    slam_config = os.path.join(pkg_share_dir, 'config', 'slam_toolbox_params.yaml')

    # slam_toolbox 2.8.x est un lifecycle node : CONFIGURE puis ACTIVATE,
    # automatises comme dans slam.launch.py.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            slam_config,
            {'use_sim_time': False},
        ],
        output='screen',
    )

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

        # --- slam_toolbox ------------------------------------------------------
        slam_node,

        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        ),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            )
        ),
    ])
