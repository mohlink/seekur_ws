#!/usr/bin/env python3
"""
gazebo_simple.launch.py - Simulation SeekurJR SANS ros2_control

Architecture (identique au robot réel) :
    Twist sur /cmd_vel  ->  plugin DiffDrive natif de Gazebo  ->  mouvement
    Gazebo  ->  /joint_states  ->  robot_state_publisher  ->  TF roues  ->  RViz2

CORRECTION PRINCIPALE par rapport a la version precedente :
    ajout du pont /joint_states (il manquait, d'ou les roues figees dans RViz2).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# --- Parametres a ajuster si besoin -----------------------------------------
# Le topic Gazebo des joint states depend du nom du MONDE et du nom du MODELE.
# Monde 'empty.sdf'  -> nom de monde 'empty'
# Spawn '-name seekur_jr' -> nom de modele 'seekur_jr'
# A verifier au premier lancement avec :  gz topic -l | grep joint_state
WORLD_NAME = 'empty'
MODEL_NAME = 'seekur_jr'
GZ_JOINT_STATE_TOPIC = f'/world/{WORLD_NAME}/model/{MODEL_NAME}/joint_state'
# ----------------------------------------------------------------------------


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('seekur_driver'), 'urdf', 'seekur_jr_simple.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Lancer RViz2'),

        # --- Gazebo Harmonic ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items()
        ),

        # --- Robot State Publisher ------------------------------------------
        # Consomme /joint_states et publie les TF des roues.
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
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

        # --- Pont ROS2 <-> Gazebo -------------------------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                # Horloge de simulation (indispensable avec use_sim_time:=true)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                # Commande de vitesse : ROS2 -> Gazebo, en Twist NON stamped
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # Retours Gazebo -> ROS2
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

                # *** LA LIGNE QUI MANQUAIT ***
                # Etats des joints (roues) : Gazebo -> ROS2
                GZ_JOINT_STATE_TOPIC + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[
                # robot_state_publisher ecoute /joint_states
                (GZ_JOINT_STATE_TOPIC, '/joint_states'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # --- RViz2 -----------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('seekur_driver'), 'config', 'seekur_viz.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
