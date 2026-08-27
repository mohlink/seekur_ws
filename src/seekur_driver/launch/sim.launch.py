#!/usr/bin/env python3
"""
sim.launch.py - Chaine de simulation complete SeekurJR (N2 validee)

Un seul ros2 launch demarre TOUTE la chaine sim-fidele :
    Gazebo + robot + bridge + robot_state_publisher
        + simulateur de protocole SeekurOS (TCP:9999)
        + seekur_driver_node (client TCP du simulateur)
        + RViz2 (optionnel)

Le driver herite automatiquement de use_sim_time=true : plus besoin de
le passer a la main a chaque relance (c'etait la source de bugs
recurrents en debugging). Le principe : use_sim_time est UN SEUL argument
de launch qui se propage a TOUS les nodes.

Le jour du vrai robot :
  - ce launch ne sert pas (Gazebo/simulateur/bridge disparaissent)
  - on lance uniquement le driver avec serial_port:=/dev/ttyUSB0 et
    use_sim_time:=false, plus le driver LiDAR reel separement
  - nav2, robot_state_publisher, RViz2 sont dans un launch separe
    (celui-la a venir en N4-N5) qui marche a l'identique sim/reel

Arguments :
  world       fichier .sdf du monde (defaut: warehouse_simple.sdf)
  use_sim_time  vrai/faux (defaut: true - c'est de la sim ici)
  rviz        lancer RViz2 (defaut: true)
  driver      lancer le driver (defaut: true - a false pour debug isole)

Usage :
  # Lancement standard :
  ros2 launch seekur_driver sim.launch.py

  # Changer de monde :
  ros2 launch seekur_driver sim.launch.py world:=mine_gallery.sdf

  # Sans RViz (utile en CI ou pour tester juste la chaine data) :
  ros2 launch seekur_driver sim.launch.py rviz:=false

  # Sans driver (pour lancer un driver custom en dehors du launch) :
  ros2 launch seekur_driver sim.launch.py driver:=false
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

MODEL_NAME = 'seekur_jr'


def generate_launch_description():

    # --- Arguments launch ----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_arg = LaunchConfiguration('world')
    rviz_arg = LaunchConfiguration('rviz')
    driver_arg = LaunchConfiguration('driver')
    publish_tf_arg = LaunchConfiguration('publish_tf')
    pkg_share = FindPackageShare('seekur_driver')

    xacro_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'seekur_jr_simple.urdf.xacro'
    ])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    bridge_config = PathJoinSubstitution([
        pkg_share, 'config', 'gz_bridge.yaml'
    ])

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', world_arg
    ])

    return LaunchDescription([

        # --- Declarations d'arguments ---------------------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true en simulation, false sur le vrai robot',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_simple.sdf',
            description='Nom du fichier de monde dans le dossier worlds/',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Lancer RViz2',
        ),
        DeclareLaunchArgument(
            'driver',
            default_value='true',
            description='Lancer le driver seekur (le simulateur est toujours lance)',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Le driver publie-t-il odom->base_footprint. '
                        'true en solo, false quand l EKF prend le relais '
                        '(sim_ekf.launch.py).',
        ),
        # --- Gazebo Harmonic avec le monde selectionne ----------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': [world_file, ' -r']}.items()
        ),

        # --- Robot State Publisher ------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen'
        ),

        # --- Spawn du robot dans Gazebo -------------------------------------
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', MODEL_NAME,
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
            ],
            output='screen'
        ),

        # --- Bridge ROS2 <-> Gazebo (config YAML centralisee) ---------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),

        # --- Simulateur de protocole SeekurOS (TCP 9999) --------------------
        # Fait la boucle Gazebo <-> protocole. Doit demarrer AVANT le driver
        # (sinon le driver ne trouve pas le serveur TCP), mais grace au
        # comportement 'accept en attente' du serveur TCP, on peut les
        # lancer quasi simultanement -- le driver reessaiera implicitement
        # via son propre timeout de connexion. Le TimerAction ci-dessous
        # ajoute juste une petite marge de securite (2 s) pour laisser
        # Gazebo se stabiliser avant que le simulateur ne commence a
        # publier sur /sim/cmd_vel.
        Node(
            package='seekur_driver',
            executable='seekur_protocol_simulator',
            name='seekur_protocol_simulator',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # --- Driver SeekurJR ------------------------------------------------
        # Delai 3 s pour laisser le simulateur TCP demarrer son serveur.
        # use_sim_time est propage via l'argument global (regle du bug
        # recurrent en N2 : le driver le manquait, timestamps en heure
        # murale, chainage TF impossible cote RViz2).
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='seekur_driver',
                    executable='seekur_driver_node',
                    name='seekur_driver',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'serial_port': 'tcp://localhost:9999',
                        'publish_tf': publish_tf_arg,
                    }],
                    condition=IfCondition(driver_arg),
                    output='screen'
                ),
            ]
        ),

        # --- RViz2 -----------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                pkg_share, 'config', 'seekur_viz.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz_arg),
            output='screen'
        ),
    ])
