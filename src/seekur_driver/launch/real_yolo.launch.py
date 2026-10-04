#!/usr/bin/env python3
"""
real_yolo.launch.py - VRAI robot SeekurJR + detection YOLO (2D par defaut, 3D en option)

Pendant reel de sim_yolo.launch.py : meme chaine de perception, posee
sur real.launch.py au lieu de sim.launch.py.

Ce qui est lance :
  - real.launch.py complet (driver serie, IMU, LiDAR, camera D435, TF)
  - yolo_bringup : yolo_node, tracking_node, debug_node
                   (+ detect_3d_node seulement avec use_3d:=True)
  - image_republisher : /yolo/dbg_image compresse en JPEG 30 pour le WiFi

Topics produits par la perception :
  /yolo/detections               : bbox en pixels
  /yolo/detections_3d            : bbox3d en metres, frame base_link (use_3d:=True)
  /yolo/tracking                 : detections avec id persistant
  /yolo/dbg_image                : image annotee BRUTE (~34 Mo/s, rester local)
  /yolo/dbg_image/compressed     : image annotee JPEG 30 (a regarder a distance)
  /yolo/dgb_bb_markers           : markers RViz des boites 3D (use_3d:=True)


2D PAR DEFAUT, 3D EN OPTION
---------------------------
    ros2 launch seekur_driver real_yolo.launch.py rviz:=false joy:=false
    ros2 launch seekur_driver real_yolo.launch.py rviz:=false joy:=false use_3d:=True

Pourquoi 2D par defaut (mesure au lab 2026-10-04) : sur la Jetson,
detect_3d_node sature un coeur CPU (100 %). py-spy : tout le temps passe
dans convert_bb_to_3d (histogrammes et quantiles numpy sur TOUS les pixels
de profondeur de chaque boite, ~84 000 pour une personne proche). Resultat
en 3D : /yolo/detections_3d a 5-6 Hz avec 0,70 s de retard, contre
/yolo/detections a 22,6 Hz et 0,12 s. Le retard vient de la file d'attente
du noeud qui ne suit pas. Le cout est par detection et par taille de boite,
pas par image : passer la camera a 15 i/s ne change rien.
Pour la securite, la distance d'une personne sera obtenue a moindre cout
par un noeud dedie (mediane de profondeur sur un petit patch au centre de
la boite 2D), sans detect_3d_node.


PREREQUIS : DEUX WORKSPACES SOURCES (sur la Jetson)
---------------------------------------------------
    source /opt/ros/jazzy/setup.bash
    source ~/yolo_ws/install/setup.bash
    source ~/seekur_ws/install/setup.bash
    ros2 launch seekur_driver real_yolo.launch.py rviz:=false joy:=false

Sans yolo_ws source, le launch echoue au demarrage, y compris la partie
robot. Sur la Jetson, ~/.config/uv/uv.toml doit pointer vers l'index
PyTorch cu132 (voir DEPS.md), sinon le uv sync de yolo_ros installe un
torch qui ne cible pas l'Orin.

Arguments de real.launch.py (rviz, joy, imu, publish_tf...) : transmis
tels quels depuis la ligne de commande. camera doit rester a true : sans
camera, YOLO tourne a vide.


VISUALISATION A DISTANCE (ROG)
------------------------------
Pas de rqt_image_view ici (Jetson sans ecran, contrairement a la sim).
Sur le ROG :
    ros2 run rqt_image_view rqt_image_view
    -> choisir /yolo/dbg_image/compressed
Ne jamais ouvrir /yolo/dbg_image (brut) a travers le WiFi.


DIFFERENCE SIM / REEL
---------------------
depth_image_units_divisor = 1000 ici (realsense2_camera publie le depth
en 16UC1, MILLIMETRES), 1 en simulation (Gazebo, 32FC1, metres).
C'est la seule difference de la chaine de perception.


VALIDATION (lab 2026-10-03 et 2026-10-04)
-----------------------------------------
YOLOv8n sur GPU Orin (torch 2.14.1+cu132) : ~28,8 Hz sur la camera a
30 i/s, GPU 23-43 %, RAM robot + YOLO 3,65/7,5 Go. Un coeur CPU a ~98 %
(probablement le thread Python de yolo_node) : premier goulot probable
quand RTAB-Map tournera en parallele.
2D : person 0,91, chair 0,85, tv 0,81 (lumiere allumee ; dans le noir,
luminosite 6,6/255, aucune detection : la D435 couleur n'a pas
d'eclairage propre).
3D : personne centree suivie (2,84 m puis 2,38 m, y < 0,1 m) ; ecran
statique repetable a 1 cm lateral et ~16 cm en profondeur a ~3,7 m.
Le z d'une personne qui remplit l'image tombe a la hauteur de la camera
(~0,47 m) : centre de la boite 2D sur la ligne d'horizon, pas une erreur
de profondeur.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('seekur_driver')
    yolo_share = FindPackageShare('yolo_bringup')

    return LaunchDescription([

        # --- Parametres YOLO ------------------------------------------------
        DeclareLaunchArgument(
            'model',
            default_value='yolov8n.pt',
            description='Modele YOLO. yolov8n valide sur Jetson Orin Nano (~28,8 Hz).',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='cuda:0 pour le GPU de la Jetson, cpu en repli',
        ),
        DeclareLaunchArgument(
            'depth_units_divisor',
            default_value='1000',
            description=(
                '1000 sur le vrai robot (realsense2_camera, 16UC1 en mm). '
                '1 en simulation. SEULE difference sim/reel de la perception.'
            ),
        ),
        DeclareLaunchArgument(
            'use_3d',
            default_value='False',
            description=(
                "False (defaut) : 2D seulement, detect_3d_node n'est pas lance. "
                "True : boites 3D, mais ~6 Hz et 0,7 s de retard sur la Jetson "
                "(detect_3d_node sature un coeur CPU). Majuscule obligatoire : "
                "yolo.launch.py fait eval() de la valeur, 'true' echoue."
            ),
        ),
        DeclareLaunchArgument(
            'dbg_jpeg_quality',
            default_value='30',
            description=(
                "Qualite JPEG de /yolo/dbg_image/compressed. 30 : ~20-25 Ko "
                "par image, supportable par le WiFi (95 par defaut le sature)."
            ),
        ),

        # --- Chaine reelle complete (via real.launch.py) --------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'real.launch.py'])
            ]),
        ),

        # --- YOLO : delai 8 s -------------------------------------------------
        # Plus long que les 5 s de la sim : la D435 fait un initial_reset et
        # n'est prete qu'environ 6,5 s apres le lancement ("RealSense Node Is
        # Up"). Sans ce delai, detect_3d_node s'active sans camera_info.
        # Au premier lancement, le uv sync de yolo_ros ajoute 1-2 min.
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([yolo_share, 'launch', 'yolo.launch.py'])
                    ]),
                    launch_arguments={
                        'model': LaunchConfiguration('model'),
                        'device': LaunchConfiguration('device'),

                        # Memes topics que la sim (conventions realsense2_camera).
                        'input_image_topic': '/camera_front/color/image_raw',
                        'input_depth_topic': '/camera_front/aligned_depth_to_color/image_raw',
                        'input_depth_info_topic': '/camera_front/color/camera_info',

                        # False par defaut : yolo.launch.py ne lance alors
                        # pas detect_3d_node (IfCondition sur use_3d).
                        'use_3d': LaunchConfiguration('use_3d'),
                        'depth_image_units_divisor':
                            LaunchConfiguration('depth_units_divisor'),
                    }.items(),
                ),

                # --- Image annotee compressee pour la vue a distance --------
                # debug_node publie /yolo/dbg_image en brut (~1,2 Mo/image).
                # On la compresse ici, sur la Jetson, avant le WiFi.
                Node(
                    package='image_transport',
                    executable='republish',
                    name='image_republisher',
                    output='screen',
                    parameters=[{
                        'in_transport': 'raw',
                        'out_transport': 'compressed',
                        'out.compressed.jpeg_quality':
                            LaunchConfiguration('dbg_jpeg_quality'),
                    }],
                    remappings=[
                        ('in', '/yolo/dbg_image'),
                        ('out/compressed', '/yolo/dbg_image/compressed'),
                    ],
                ),
            ],
        ),
    ])
