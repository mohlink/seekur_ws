#!/usr/bin/env python3
"""
real.launch.py - Chaine de pilotage SUR LE VRAI ROBOT SeekurJR

Pendant reel de sim.launch.py :
  - PAS de Gazebo
  - PAS de simulateur SeekurOS TCP
  - PAS de bridge Gazebo <-> ROS2
  - Driver seekur en SERIE sur /dev/seekur au lieu de tcp://localhost:9999

CHAINE ACTIVE :
  robot_state_publisher (URDF calibre datasheet + lab)
    -> TF base_footprint <-> base_link <-> roues, LiDAR, camera_front_link, IMU
  bno055_imu (Nano + BNO055 sur /dev/imu, 100 Hz)
    -> /imu/data (frame imu_link)
  seekur_driver (serie 9600 baud, DTR/RTS)
    -> /odom + TF odom -> base_footprint (publish_tf:=true par defaut)
    -> /battery_state, /diagnostics
    <- /cmd_vel
  joy_node + teleop_twist_joy + twist_mux
    -> pilotage manette avec dead-man
  rviz2 avec seekur_viz.rviz

FUSION EKF - dans real_ekf.launch.py, pas ici :
  Par defaut le driver publie lui-meme la TF odom->base_footprint.
  real_ekf.launch.py inclut ce launch avec publish_tf:=false et demarre
  robot_localization, qui devient le proprietaire unique de cette TF
  (meme schema que sim_ekf.launch.py par-dessus sim.launch.py).

PREREQUIS PHYSIQUES :
  - Robot SeekurJR allume, bouton MOTORS relache (bleu clignotant) pour
    autoriser le mouvement. MOTORS enfonce = arret d'urgence, aucune
    commande moteur ne passe (le driver et l'odometrie fonctionnent quand
    meme, seul le mouvement est bloque).
  - Cable serie PL2303 branche, /dev/seekur present (regle udev 99-seekur-devices.rules), utilisateur dans le
    groupe dialout (verifiable : groups | grep dialout).
  - Manette de jeu branchee et reconnue (verifiable : ls /dev/input/js*).
    La manette maintient un DEAD-MAN : sans bouton enfonce, aucune commande
    n'atteint le driver. Sans manette branchee, /joy_node echouera au
    demarrage - le driver continuera de tourner mais rien ne pilotera.

SEQUENCE DE DEMARRAGE :
  - t=0 : robot_state_publisher + bno055_imu + lms1xx + joy_node +
          teleop_twist_joy + twist_mux + rviz2
  - t=2 : seekur_driver (delai pour laisser DDS s'etablir avant d'ouvrir le
          port serie - evite les timings serres si le driver demarre avant
          que /cmd_vel ait des subscribers)

Usage :
  ros2 launch seekur_driver real.launch.py
  ros2 launch seekur_driver real.launch.py rviz:=false      # sans RViz (headless)
  ros2 launch seekur_driver real.launch.py joy:=false       # sans manette
  ros2 launch seekur_driver real.launch.py rviz:=false joy:=false   # Jetson

Arguments : use_sim_time (defaut false, ce launch est pour le REEL),
            rviz (defaut true), joy (defaut true), imu (defaut true),
            publish_tf (defaut true ; false quand l'EKF publie la TF).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    pkg_share = FindPackageShare('seekur_driver')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Chemins absolus resolus immediatement (pas de substitutions imbriquees
    # dans les configs et arguments RViz -> plus simple a debugger si un
    # fichier est absent).
    pkg_share_dir = get_package_share_directory('seekur_driver')
    seekur_params = os.path.join(pkg_share_dir, 'config', 'seekur_params.yaml')
    rviz_config   = os.path.join(pkg_share_dir, 'config', 'seekur_viz_real.rviz')
    xacro_file    = os.path.join(pkg_share_dir, 'urdf',   'seekur_jr_simple.urdf.xacro')

    return LaunchDescription([

        # --- Arguments ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description="false sur le vrai robot (defaut). true seulement pour test avec bag rejoue.",
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Lancer RViz2 avec la config seekur_viz.rviz',
        ),
        DeclareLaunchArgument(
            'joy',
            default_value='true',
            description='Lancer la chaine manette (joy_node + teleop_twist_joy + twist_mux)',
        ),
        DeclareLaunchArgument(
            'imu',
            default_value='true',
            description='Lancer le noeud IMU BNO055 (/imu/data)',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description="Le driver publie la TF odom->base_footprint. false quand l'EKF la publie.",
        ),

        # ================================================================
        # Robot description : URDF calibre datasheet Rev B + mesures lab
        # ================================================================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # Le xacro est resolu au lancement via la substitution Command.
                # Genere l'URDF final avec toutes les proprietes ${...} evaluees.

                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file,
                             ' lidar_pitch_deg:=6.5',
                             ' mesh_uri:=package://seekur_driver/description/meshes']),
                    value_type=str,
                ),               
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ================================================================
        # Chaine manette : joy_node -> teleop_twist_joy -> twist_mux
        # Meme setup que celui deja valide en interactif (session 2026-09).
        # ================================================================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('joy')),
            parameters=[
                os.path.join(pkg_share_dir, 'config', 'joystick.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('joy')),
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
            parameters=[
                os.path.join(pkg_share_dir, 'config', 'joystick.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            condition=IfCondition(LaunchConfiguration('joy')),
            remappings=[('cmd_vel_out', 'cmd_vel')],           
            parameters=[
                # Config : /cmd_vel_joy (prio 100, manette) et /cmd_vel_nav
                # (prio 10, nav2 futur) fusionnes vers /cmd_vel. use_stamped:
                # false coherent avec seekur_driver qui attend du Twist simple.
                os.path.join(pkg_share_dir, 'config', 'twist_mux.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ================================================================
        # IMU BNO055 (Nano sur /dev/imu) : /imu/data a 100 Hz, frame imu_link.
        # Reutilise le launch et la config du package bno055_imu.
        # Au lab 2026-09-29 : orientation X avant / Z haut validee, position
        # au centre a 11,5 cm du sol (conforme au xacro).
        # ================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('bno055_imu'), 'launch', 'bno055.launch.py')),
            condition=IfCondition(LaunchConfiguration('imu')),
        ),

        # ================================================================
        # LiDAR SICK LMS111-10100 sur Ethernet
        # Driver Clearpath 'lms1xx', validé sur firmware V1.31 (2011).
        # sick_scan_xd 3.9.0 crashait au premier télégramme (parsing binaire
        # incompatible avec ce firmware d'origine).
        # ================================================================
        Node(
            package='lms1xx',
            executable='lms1xx',
            name='lms1xx',
            output='screen',
            parameters=[
                os.path.join(pkg_share_dir, 'config', 'lms111.yaml'),
            ],
        ),

        # ================================================================
        # Driver SeekurJR sur port SERIE (le vrai robot).
        # Demarre a t=2s : laisse le temps aux autres nodes de s'inscrire
        # sur /cmd_vel et au TF tree de se former, evite les warnings
        # "no subscribers" au demarrage.
        # ================================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='seekur_driver',
                    executable='seekur_driver_node',
                    name='seekur_driver',
                    output='screen',
                    parameters=[
                        seekur_params,
                        # publish_tf : true sans EKF (le driver publie
                        # odom->base_footprint) ; real_ekf.launch.py passe false.
                        {'use_sim_time': use_sim_time,
                         'publish_tf': ParameterValue(
                             LaunchConfiguration('publish_tf'), value_type=bool),
                         # Rayon de roulement effectif : le firmware sous-estime
                         # les distances de 1,5 % (1 m -> 0,985 ; 6 m -> 5,907).
                         'linear_scale': 1.0155},
                    ],
                    # Pas de remap : /cmd_vel, /odom, /battery_state,
                    # /diagnostics vont directement sur les topics standards.
                ),
            ],
        ),

        # ================================================================
        # RViz2
        # ================================================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
