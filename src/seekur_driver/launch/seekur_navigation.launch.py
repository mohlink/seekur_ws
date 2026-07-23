from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Récupération des chemins de packages
    pkg_share = FindPackageShare('seekur_driver')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Arguments de lancement
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Chemin complet vers le fichier de carte (.yaml)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Utiliser le temps de simulation'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']),
        description='Chemin vers le fichier de paramètres nav2'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Démarrer automatiquement nav2'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition', 
        default_value='true',
        description='Utiliser la composition des nodes'
    )
    
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='false',
        description='Redémarrer les nodes si ils crashent'
    )
    
    # 1. Lancement du driver SeekurJR
    seekur_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'seekur_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # 2. Transform statique base_link -> laser_frame (ajustez selon votre LiDAR)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.15', '0', '0.25',  # x, y, z (position du LiDAR)
            '0', '0', '0',        # roll, pitch, yaw
            'base_link', 'laser_frame'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 3. Map server (si carte fournie)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('map'))
    )
    
    # 4. Nav2 Bringup (localization + navigation)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': LaunchConfiguration('use_respawn'),
        }.items()
    )
    
    # 5. AMCL (localization) si carte fournie
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(LaunchConfiguration('map'))
    )
    
    # 6. Lifecycle manager pour gérer les nodes nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
        ]
    )
    
    return LaunchDescription([
        # Arguments
        map_arg,
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        
        # Nodes
        seekur_bringup,
        static_tf_base_to_laser,
        map_server,
        nav2_bringup,
        amcl,
        lifecycle_manager,
    ])