# MEMO — Commandes de lancement SeekurJR

Aide-mémoire des commandes courantes. Voir `README.md` pour le contexte.

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
      --port /dev/ttyUSB0

### Détecter le baudrate d'un port série inconnu

    cd src/seekur_driver/tools/low_level && make
    ./seekur_baudrate_detector /dev/ttyUSB0

### Inspecter une base RTAB-Map après un run

    rtabmap-databaseViewer ~/.ros/rtabmap.db

---

## Robot réel (à compléter)

À faire quand le robot physique sera disponible :

- Vérifier le port série effectif : `ls -l /dev/ttyUSB*`
- Vérifier le baudrate avec `seekur_baudrate_detector` si doute
- Vérifier que DTR/RTS sont bien activés côté driver (déjà géré par
  seekur_driver_node.py, mais bon à savoir en cas de silence radio)

Différence sim → réel : deux paramètres seulement dans `seekur_params.yaml`
(voir tableau dans README.md).

Séquence type prévue :

    # 1. Appuyer sur le bouton MOTORS (LED bleue) sur le robot
    # 2. Lancer le driver + EKF sans Gazebo
    ros2 launch seekur_driver <launch_real.launch.py à créer>
    # 3. Éventuellement teleop pour valider

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
