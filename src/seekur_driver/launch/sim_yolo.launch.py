#!/usr/bin/env python3
"""
sim_yolo.launch.py - Simulation complete + detection YOLO 3D (P3)

Variante de sim.launch.py qui ajoute la chaine de perception YOLO
par-dessus la simulation.

Ce qui est lance :
  - sim.launch.py complet (Gazebo + robot + bridge + driver + RViz2)
  - yolo_bringup : yolo_node, tracking_node, detect_3d_node, debug_node

Topics produits par la perception :
  /yolo/detections      : Detection2DArray (bbox pixels)
  /yolo/detections_3d   : detections avec bbox3d en metres, frame base_link
  /yolo/tracking        : detections avec id persistant
  /yolo/dbg_image       : image annotee (boites + labels + scores)
  /yolo/dgb_bb_markers  : markers RViz des boites 3D


PREREQUIS : DEUX WORKSPACES SOURCES
-----------------------------------
yolo_ros n'est pas dans seekur_ws (code tiers, installe separement).
Le terminal doit sourcer les deux avant de lancer :

    conda deactivate
    source /opt/ros/jazzy/setup.bash
    source ~/yolo_ws/install/setup.bash
    source ~/seekur_ws/install/setup.bash
    ros2nv launch seekur_driver sim_yolo.launch.py

Sans yolo_ws source, le launch echoue au demarrage - y compris la
partie simulation, puisque l'include de yolo_bringup est evalue en
premier. C'est le compromis du launch combine.

Installation de yolo_ros (rappel) :
    mkdir -p ~/yolo_ws/src && cd ~/yolo_ws/src
    git clone https://github.com/mgonzs13/yolo_ros.git
    cd yolo_ros && uv sync
    cd ~/yolo_ws && colcon build
  Note : ne PAS faire rosdep install - les deps Python (torch,
  ultralytics) sont gerees par uv dans un venv dedie. rosdep voudrait
  les reinstaller globalement, avec un risque de version CPU-only.


DIFFERENCE SIM / REEL A CONNAITRE
---------------------------------
depth_image_units_divisor = 1 en simulation, 1000 sur le vrai robot.

  Gazebo rgbd_camera publie le depth en 32FC1, unite = METRE
  realsense2_camera publie le depth en 16UC1, unite = MILLIMETRE

yolo_ros divise la valeur depth par ce diviseur pour obtenir des
metres. Avec la valeur par defaut (1000) sur du depth Gazebo, toutes
les detections se retrouvaient a ~3 mm de la camera : la position 3D
tombait exactement sur l'origine de la camera (0.3, 0, 0.6 dans
base_link), ce qui a servi de signature au diagnostic.

C'est la SEULE difference sim/reel de toute la chaine de perception :
les topics camera ont ete nommes selon les conventions
realsense2_camera des P2 pour eviter les autres.


PRECISION MESUREE (validation 2026-09, mine_gallery.sdf)
--------------------------------------------------------
Personne a x=2.602 m (verite terrain Gazebo), detectee a x=2.450 m.
Ecart 15 cm, explique : le depth mesure la surface AVANT (poitrine),
la pose Gazebo est le CENTRE du modele. Un torse fait ~25-30 cm
d'epaisseur, donc ~13-15 cm entre surface et centre. L'erreur va dans
le sens conservateur (personne percue plus proche qu'elle ne l'est).
Ecart lateral : 2 mm.
Score de detection : 0.906 de face, 0.725 de profil (COCO est domine
par des vues frontales).

Arguments : herites de sim.launch.py, plus les parametres YOLO
    ros2 launch seekur_driver sim_yolo.launch.py model:=yolov8s.pt
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

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true en simulation, false sur le vrai robot',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='mine_gallery.sdf',
            description='Monde SDF (contient person_1 pour tester YOLO)',
        ),

        # --- Parametres YOLO ------------------------------------------------
        DeclareLaunchArgument(
            'model',
            default_value='yolov8n.pt',
            description=(
                'Modele YOLO. yolov8n = nano, choisi pour la compatibilite '
                'future avec la Jetson Orin Nano (~40 TOPS). yolov8s '
                'possible sur RTX 4060 mais laisse moins de marge.'
            ),
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='cuda:0 pour GPU NVIDIA, cpu en repli',
        ),
        DeclareLaunchArgument(
            'depth_units_divisor',
            default_value='1',
            description=(
                '1 en simulation (Gazebo publie en metres, 32FC1). '
                '1000 sur le vrai robot (realsense2_camera publie en '
                'millimetres, 16UC1). SEULE difference sim/reel de la '
                'chaine de perception.'
            ),
        ),

        # --- Chaine sim complete (via sim.launch.py) ------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'sim.launch.py'
                ])
            ]),
        ),

        # --- YOLO : delai 5s -------------------------------------------------
        # Plus long que les 3s de nav.launch.py : il faut que Gazebo ait
        # demarre le rendu ET que le bridge ait cree les ponts camera,
        # sinon detect_3d_node s'active sans camera_info et reste muet.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            yolo_share, 'launch', 'yolo.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'model': LaunchConfiguration('model'),
                        'device': LaunchConfiguration('device'),

                        # Topics camera : conventions realsense2_camera,
                        # etablies en P2 pour la fidelite sim-to-real.
                        'input_image_topic': '/camera/color/image_raw',
                        'input_depth_topic': '/camera/depth/image_rect_raw',
                        'input_depth_info_topic': '/camera/color/camera_info',

                        # Mode 3D : exploite le depth pour donner une
                        # position metrique aux detections. Indispensable
                        # pour l'usage securite (savoir a quelle distance
                        # se trouve la personne, pas seulement qu'elle
                        # est dans l'image).
                        'use_3d': 'True',
                        'depth_image_units_divisor':
                            LaunchConfiguration('depth_units_divisor'),
                    }.items(),
                ),

                # --- Visualisation des detections annotees -------------------
                # rqt_image_view ouvert directement sur l'image annotee.
                # C'est la vue la plus utile en debug perception : boites,
                # labels, scores et ids de tracking superposes sur le flux
                # camera. RViz reste dispo pour les markers 3D
                # (/yolo/dgb_bb_markers).
                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='yolo_image_view',
                    output='screen',
                    arguments=['/yolo/dbg_image'],
                ),

            ]
        ),
    ])
