#!/usr/bin/env python3
"""
real_ekf.launch.py - Chaine du VRAI robot SeekurJR AVEC fusion EKF

Pendant reel de sim_ekf.launch.py. La localisation odometrique est produite
par robot_localization, qui fusionne :
    /odom      (driver : vitesse d'avance vx)
  + /imu/data  (BNO055 : vitesse angulaire Z, 100 Hz)
    -> /odometry/filtered + TF odom->base_footprint

POURQUOI (mesures au lab 2026-09-29) :
  La vitesse angulaire de /odom (ROTVEL du firmware) vaut 0,96 x le gyro
  et arrive avec 0,25 a 0,5 s de retard (SIP a 10 Hz sur une liaison
  9600 bauds). En rotation, les scans etaient places avec un cap en retard :
  les murs pivotaient dans RViz puis revenaient. Le gyro du BNO055, a
  100 Hz et sans retard notable, donne un cap qui suit la rotation.

MEME CONFIGURATION QUE LA SIM :
  On reutilise config/ekf.yaml (celui de sim_ekf.launch.py) et on ne
  surcharge ici que use_sim_time (false). Depuis le 2026-10-05, ekf.yaml
  ne fusionne plus l'acceleration de l'IMU, en sim comme en reel (vyaw du
  gyro seul) : la surcharge imu0_config qui existait ici pour le BNO055
  n'est plus necessaire. Tout reglage EKF se fait dans ekf.yaml et vaut
  pour les deux.

PROPRIETAIRE UNIQUE de la TF odom->base_footprint : l'EKF. real.launch.py
est inclus avec publish_tf:=false (le driver publie /odom sans la TF).

Usage :
  ros2 launch seekur_driver real_ekf.launch.py rviz:=false joy:=false   # Jetson

Arguments : rviz (defaut true), joy (defaut true), transmis a real.launch.py.
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
    ekf_config = os.path.join(pkg_share_dir, 'config', 'ekf.yaml')

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='true', description='Lancer RViz2'),
        DeclareLaunchArgument('joy', default_value='true',
                              description='Lancer la chaine manette'),

        # --- Chaine reelle, IMU active, driver SANS publication TF ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, 'launch', 'real.launch.py')),
            launch_arguments={
                'publish_tf': 'false',
                'imu': 'true',
                'rviz': LaunchConfiguration('rviz'),
                'joy': LaunchConfiguration('joy'),
            }.items(),
        ),

        # --- EKF robot_localization -----------------------------------------
        # Delai 4 s : apres le driver (lance a 2 s dans real.launch.py) et
        # le redemarrage de la Nano de l'IMU (~2 s a l'ouverture du port),
        # pour que /odom et /imu/data existent quand l'EKF s'abonne.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[
                        ekf_config,
                        {'use_sim_time': False},
                    ],
                ),
            ],
        ),
    ])
