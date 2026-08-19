#!/usr/bin/env python3
"""
slam.launch.py - SeekurJR + slam_toolbox en mode online_async

Lance la meme chaine que gazebo_simple.launch.py, mais avec :
  - le monde warehouse_simple.sdf (piece 10x10 m avec obstacles)
  - le noeud slam_toolbox par-dessus, qui publie map -> odom et /map

Pour cartographier :
  1. Lancer ce launch
  2. Dans un autre terminal, lancer le simulateur de protocole
     puis seekur_interactive1_tcp.py pour conduire le robot
  3. Dans RViz2, ajouter un affichage 'Map' sur le topic /map
  4. Une fois satisfait de la carte : sauvegarder via
     ros2 run nav2_map_server map_saver_cli -f ~/seekur_ws/maps/warehouse
"""

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os

MODEL_NAME = 'seekur_jr'


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # Chemin du YAML slam resolu IMMEDIATEMENT (chemin en dur), pas via
    # PathJoinSubstitution : slam_toolbox ignore silencieusement un
    # params-file fourni comme substitution non resolue dans parameters=[].
    slam_config = os.path.join(
        get_package_share_directory('seekur_driver'),
        'config', 'slam_toolbox_params.yaml'
    )

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'warehouse_simple.sdf'
    ])

    # Noeud slam_toolbox declare comme LifecycleNode pour pouvoir cibler ses
    # transitions d'etat depuis le launch (voir plus bas dans la description).
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # --- Gazebo Harmonic avec notre monde de test ------------------------
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
        # Origine (0,0,0.1) : au centre de la piece, hors de tout obstacle.
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

        # --- Alias TF pour le frame_id du LiDAR ------------------------------
        # Cf. commentaire dans gz_bridge.yaml : ros_frame_id ignore sur Jazzy.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_alias',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'laser_frame',
                '--child-frame-id', f'{MODEL_NAME}/base_footprint/laser_scanner',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # --- SLAM Toolbox (online_async, lifecycle) --------------------------
        # slam_toolbox 2.8.x est un lifecycle node : il demarre 'unconfigured'
        # et ne fait RIEN tant qu'on ne l'a pas configure puis active.
        # Le parametre autostart est ignore par cette version (teste 2026-08).
        # On automatise donc les transitions ici :
        #   1. EmitEvent CONFIGURE des le lancement
        #   2. OnStateTransition 'configuring'->'inactive' => EmitEvent ACTIVATE
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

        # --- RViz2 -----------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                pkg_share, 'config', 'seekur_viz.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
