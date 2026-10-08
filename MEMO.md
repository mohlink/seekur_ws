# MEMO — Commandes de lancement SeekurJR

Aide-mémoire des commandes courantes. Voir `README.md` pour le contexte.

---

## Travaux en cours

A mettre a jour a chaque creation ou fusion de branche.
Regle : branches courtes. Une branche qui attend recoit `git merge main --no-edit`
apres chaque fusion dans main, pour que les conflits restent petits
(lecon de `feat/camera-front`, 2026-09-30).
Fusions et push depuis le ROG ; la Jetson ne fait que LIRE GitHub
(`fetch`, `switch`, `pull`), sauf pour ce qui est cree sur la Jetson (une carte).
Avant un commit : `git branch --show-current` (commit tombe sur main par erreur
le 2026-09-30 quand `git switch -c` a echoue sur une branche deja existante).

| Branche | But | Fichiers touches | Etat | Bloque par |
|---|---|---|---|---|
| `feat/real-slam` | SLAM sur le vrai robot | `launch/real_slam.launch.py`, `launch/real_rtabmap.launch.py` (nouveaux) | slam_toolbox valide au lab (2026-09-30, carte `maps/lab_slam_toolbox`) ; RTAB-Map LiDAR seul teste (murs plus epais que slam_toolbox, avertissements visuels) | real_rtabmap a passer a la camera (camera maintenant dans real.launch.py) ; LiDAR seul en repli avec `Kp/MaxFeatures: -1` |
| `feat/drivetrain-switch` | Train de roulement sim interchangeable : `drivetrain:=tripod` (defaut, inchange) ou `center` (roues au centre + 2 roulettes suspendues, ressort k=3000 N/m, precharge 15 mm, amortissement 300) | `urdf/seekur_jr_simple.urdf.xacro`, `urdf/drivetrain_tripod.xacro` et `urdf/drivetrain_center.xacro` (nouveaux), `launch/sim.launch.py` | URDF des 2 variantes valide (`check_urdf`) ; `tripod` identique a main. Pas encore lance dans Gazebo | Test Gazebo de `center` sur le ROG (tangage IMU, debattement `/joint_states`, mine_polycam) |

### A faire, sans branche pour l'instant
- Driver : cumul des deplacements (limite des +-32,7 m du firmware).
- Superviseur LiDAR (attente LMS111 pret, relance si /scan se tait).
- EKF en 3D (roulis, tangage) : d'abord en sim.
- Deceleration firmware reglable (SETA/SETRA) ; liaison serie 115 200 bauds
  (outil SeekurOSParamsManager a retrouver).
- Alimentation embarquee de la Jetson (DC-DC sur le 24 V ou batterie separee).

---

## Prérequis à chaque session

Trois lignes à faire dans tout nouveau terminal, dans cet ordre :

    conda deactivate
    source /opt/ros/jazzy/setup.bash
    source ~/yolo_ws/install/setup.bash     # si perception (YOLO)
    source ~/seekur_ws/install/setup.bash

Rappels :
- `ros2nv` = alias pour rendu Gazebo sur GPU NVIDIA (voir `.bashrc`)
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` doit être dans `.bashrc`
- Ne PAS utiliser `colcon build --symlink-install` (casse les métadonnées Python)

---

## Build

    cd ~/seekur_ws
    colcon build --packages-select seekur_driver

---

## Simulation

### Pour lancer la mannette

ros2 launch seekur_driver joystick.launch.py
ros2 launch seekur_driver teleop.launch.py   ## avec mux

### Pour visionner la camera
ros2 run rqt_image_view rqt_image_view

rtabmap-databaseViewer ~/.ros/rtabmap.db

### Simulation de base (sans EKF, sans perception)

    ros2nv launch seekur_driver sim.launch.py

Gazebo + robot + bridge + simulateur SeekurOS TCP + driver + RViz2.
Le driver publie /odom et la TF odom→base_footprint directement.

### Simulation + fusion EKF

    ros2nv launch seekur_driver sim_ekf.launch.py

Ajoute robot_localization fusionnant IMU + odométrie roues.
Le driver ne publie plus la TF (publish_tf:=false), l'EKF le fait à sa place.
Base pour tout ce qui suit.

### Simulation + perception YOLO

    ros2nv launch seekur_driver sim_yolo.launch.py

sim + EKF + détection YOLO 3D (personnes, véhicules).
Requiert ~/yolo_ws sourcé.

### Simulation + SLAM 3D RTAB-Map

    ros2nv launch seekur_driver sim_rtabmap.launch.py

sim + EKF + RTAB-Map en SLAM 3D dense avec fermeture de boucle.
Remplace slam_toolbox comme SLAM de référence.
Base sauvegardée dans ~/.ros/rtabmap.db (écrasée à chaque lancement).

### SLAM 2D classique (slam_toolbox)

    ros2nv launch seekur_driver slam.launch.py

Cartographie 2D via slam_toolbox. Alternative à RTAB-Map, conservée
pour comparaison. Ne PAS lancer en même temps que sim_rtabmap.

### Navigation nav2

    ros2nv launch seekur_driver nav.launch.py         # sur odométrie brute
    ros2nv launch seekur_driver nav_ekf.launch.py     # sur odométrie fusionnée

Nav2 complet avec AMCL. La variante _ekf utilise l'odométrie fusionnée
et est celle à privilégier.

### Choix du monde

Argument commun à tous les launches :

    world:=mine_gallery.sdf      # défaut, galerie de test
    world:=warehouse_simple.sdf  # entrepôt simple

Exemple :

    ros2nv launch seekur_driver nav_ekf.launch.py world:=warehouse_simple.sdf

---

## Téléopération pour tester

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false

Le driver attend du Twist (pas TwistStamped), d'où le `stamped:=false`.
Contrôles : i/,/j/l (avant/arrière/gauche/droite), k (stop), q/z (vitesse).

---

## Outils de troubleshoot (voir `src/seekur_driver/tools/README.md`)

### Envoyer des commandes SeekurOS à la main au simulateur

    ros2 run seekur_driver seekur_protocol_simulator                     # terminal 1
    python3 src/seekur_driver/tools/interactive/seekur_interactive_tcp.py \
      --port tcp://localhost:9999                                        # terminal 2

### Envoyer des commandes SeekurOS à la main au vrai robot

    python3 src/seekur_driver/tools/interactive/seekur_interactive_serial.py \
      --port /dev/seekur

### Détecter le baudrate d'un port série inconnu

    cd src/seekur_driver/tools/low_level && make
    ./seekur_baudrate_detector /dev/ttyUSB0

### Inspecter une base RTAB-Map après un run

    rtabmap-databaseViewer ~/.ros/rtabmap.db

---

## Modèle de simulation (v10.2)

Trépied : 2 roues motrices **avant** (x = +L/4) + 1 roulette **arrière** tirée
(x = -L/3, sans écart), maillage SeekurJR, LiDAR incliné de 5° [SIM].
Les roues qui attaquent une bosse sont motrices : franchit `mine_polycam`.
Limite : pivote autour de l'essieu avant, pas au centre comme le vrai robot.

Pourquoi pas les 2 roulettes de v10.1 : le châssis basculait d'une roulette à
l'autre (±3.3° mesuré à l'IMU sim), ce qui a effondré les fermetures RTAB-Map.

| mine_gallery, RTAB-Map 0.23.7 (2026-09-27) | Globales | Proximité |
|---|---|---|
| v10.1 : roues au centre + 2 roulettes, gap 2 cm | 1 | 0 |
| Trépied roues arrière, LiDAR horizontal | 9 | 3 |

Conduite manuelle non reproductible : pour comparer finement deux variantes,
rejouer un bag `/cmd_vel` de référence (à faire).

### Mesurer le tangage (sim ou vrai robot)

    python3 src/seekur_driver/tools/imu_pitch_probe.py
    # attendu sur sol plat : ~0° au repos et à l'accélération

### Statistiques d'une base après un run

    rtabmap-info ~/.ros/rtabmap.db | grep -iE "closure|odometry length|LTM"
    # la base est écrasée au prochain lancement : la copier avant

## Robot réel

Repartition : tout tourne sur la **Jetson** (`jetson-seekur`), manette et
RViz sur le **ROG**. Meme domaine DDS (0) : arreter les noeuds de la Jetson
avant de lancer une sim sur le ROG (sinon /cmd_vel, /tf, /scan se melangent,
et piloter la sim peut faire bouger le vrai robot).

### Avant de lancer
- Robot allume depuis **~1 min** : le LMS111 n'est pas pret juste apres
  l'allumage et `lms1xx` peut se bloquer (« Laser not ready »).
- Bouton MOTORS relache (bleu clignotant) pour autoriser le mouvement.
- Robot sur cales pour les premiers essais d'une session.
- Liens udev : `/dev/seekur` (PL2303, robot), `/dev/imu` (Nano CH340, BNO055).

### Lancements (sur la Jetson, dans tmux)

    ros2 launch seekur_driver real.launch.py rviz:=false joy:=false        # driver + LiDAR + IMU + camera
    ros2 launch seekur_driver real_ekf.launch.py rviz:=false joy:=false    # + EKF (base conseillee)
    ros2 launch seekur_driver real_slam.launch.py rviz:=false joy:=false   # + slam_toolbox  [feat/real-slam]
    ros2 launch seekur_driver real_rtabmap.launch.py rviz:=false joy:=false # + RTAB-Map (LiDAR seul) [feat/real-slam]

Camera D435 : lancee par real.launch.py (argument `camera:=false` pour s'en
passer). Deux avertissements `pointcloud__neon_ ... not supported` au demarrage
sont normaux (liste fixe de rs_launch.py ; le nuage est bien publie a 30 Hz).
La camera se reinitialise au demarrage (~6 s, `initial_reset`).

Sur le ROG : `ros2 launch seekur_driver teleop.launch.py`, puis
`rviz2 -d $(ros2 pkg prefix seekur_driver)/share/seekur_driver/config/seekur_viz_real.rviz`.
Ne pas afficher le nuage de la camera dans RViz via le WiFi (trop lourd).

Sauvegarder une carte (slam_toolbox) :

    ros2 run nav2_map_server map_saver_cli -f ~/seekur_ws/maps/<nom>

### Differences sim -> reel
Un seul fichier de config partage ; les valeurs propres au materiel sont
passees par les launch reels :
- `real.launch.py` : `serial_port` (yaml), `linear_scale: 1.0155`,
  xacro `lidar_pitch_deg:=6.5` et `mesh_uri:=package://...`, `publish_tf`.
- `real_ekf.launch.py` : reutilise `ekf.yaml`, surcharge `use_sim_time`
  et `imu0_config` (vyaw du BNO055 seul, acceleration non fusionnee).

### Valeurs mesurees au lab (2026-09-28/29)
| Grandeur | Valeur |
|---|---|
| Odometrie en translation | firmware -1,5 % (1 m -> 0,985 ; 6 m -> 5,907) -> `linear_scale` 1.0155 |
| Cap THPOS (gyro SAG) | juste a ~1 % ; ROTVEL = 0,96 x gyro BNO055, retard 0,25-0,5 s |
| LiDAR LMS111 | faisceau a 40 cm du sol, incline de 6,5 deg vers le haut, ~50 Hz |
| BNO055 | centre du chassis, 11,5 cm du sol, X avant / Z haut |
| Camera D435 | USB 3, 848x480 a 30 Hz (couleur, profondeur alignee, nuage) |
| Limites de vitesse de depart | 0,5 m/s ; 0,7 rad/s (firmware : 1,0 m/s ; 1,75 rad/s) |

### Outils sur le vrai robot (driver arrete : port exclusif)

    python3 src/seekur_driver/tools/interactive/seekur_config_reader.py           # CONFIGpac
    python3 src/seekur_driver/tools/interactive/seekur_gyro_test.py --port /dev/seekur --rvel 15
    python3 src/seekur_driver/tools/imu_pitch_probe.py                            # roulis/tangage/cap

---

## Debug rapide

### Voir les topics ROS2

    ros2 topic list
    ros2 topic hz /odometry/filtered
    ros2 topic echo /tf_static --once

### Voir la chaîne TF

    ros2 run tf2_tools view_frames
    # ouvre frames.pdf dans le dossier courant

### Voir la config d'un nœud

    ros2 param list /rtabmap
    ros2 param get /rtabmap Rtabmap/DetectionRate
    
### note    
ros2 topic echo /scan --once | head -20

ros2 topic echo /cmd_vel_nav --once
ros2 topic hz /cmd_vel_nav

ros2 topic echo /odom nav_msgs/msg/Odometry  |grep -A3 'position:'


ros2 run plotjuggler plotjuggler
