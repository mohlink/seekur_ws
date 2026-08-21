#!/usr/bin/env python3
"""
slam.launch.py - SLAM par-dessus la chaine de simulation SeekurJR

Refactorisation post-N2 : au lieu de dupliquer la chaine sim, on
INCLUT sim.launch.py (qui contient deja Gazebo + robot + bridge +
robot_state_publisher + simulateur + driver + RViz2 avec use_sim_time
cable partout) et on ajoute simplement le noeud slam_toolbox par
dessus. Ce meme pattern sera utilise pour nav.launch.py plus tard.

Ce que ce launch fait EN PLUS de sim.launch.py :
  - lance slam_toolbox en mode online_async
  - gere le cycle de vie du lifecycle node (CONFIGURE puis ACTIVATE)
  - charge le monde warehouse_simple.sdf par defaut (surchargable)

Pour cartographier :
  1. Lancer ce launch (Gazebo + toute la chaine + slam_toolbox se lance)
  2. Piloter le robot :
     ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
     (ou avec seekur_interactive1_tcp.py sur tcp://localhost:9999,
     mais dans ce cas ne pas lancer le driver : --driver=false)
  3. Dans RViz2, ajouter un affichage 'Map' sur le topic /map si absent
  4. Sauvegarder :
     ros2 run nav2_map_server map_saver_cli -f ~/seekur_ws/maps/warehouse

Arguments : herites de sim.launch.py (world, use_sim_time, rviz, driver)
    ros2 launch seekur_driver slam.launch.py world:=mine_gallery.sdf
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    EmitEvent, RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = FindPackageShare('seekur_driver')

    # Chemin du YAML slam resolu IMMEDIATEMENT (chemin en dur), pas via
    # PathJoinSubstitution : slam_toolbox ignore silencieusement un
    # params-file fourni comme substitution non resolue dans parameters=[].
    # C'etait une des lecons de la mise en place N2 initiale.
    slam_config = os.path.join(
        get_package_share_directory('seekur_driver'),
        'config', 'slam_toolbox_params.yaml'
    )

    # slam_toolbox 2.8.x est un lifecycle node : demarre 'unconfigured'
    # et ne fait rien tant qu'on ne l'a pas configure puis active. Le
    # parametre autostart est ignore par cette version. On automatise
    # les transitions : EmitEvent CONFIGURE au demarrage, puis un
    # OnStateTransition qui declenche ACTIVATE des que CONFIGURE reussit.
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

        # sim.launch.py declare deja tous les arguments (world, rviz,
        # driver, use_sim_time). On n'a pas besoin de les redeclarer :
        # ils sont transmis automatiquement quand on inclut le launch.
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # --- Chaine de simulation complete (via sim.launch.py) --------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim.launch.py'
                ])
            ]),
            # Pas besoin de repasser les arguments : ils remontent
            # naturellement depuis la ligne de commande vers l'include.
        ),

        # --- slam_toolbox par-dessus ----------------------------------------
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
