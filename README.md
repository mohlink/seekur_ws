# SeekurJR — Navigation autonome et perception pour environnement minier

Pile ROS2 complète pour le robot **Adept MobileRobots SeekurJR** : driver
SeekurOS, jumeau virtuel Gazebo, navigation autonome nav2, fusion inertielle
et perception multimodale. Développé dans le cadre d'un projet de recherche à
l'**UQAT**, avec pour objectif la navigation autonome en galerie minière
souterraine.

Le projet suit une approche **simulation-vers-réel** : toute la chaîne est
développée et validée sur un jumeau virtuel, puis portée sur le robot physique
en changeant deux paramètres.

---

## Contexte et architecture

### Le robot

Le SeekurJR est une plateforme différentielle (skid-steer) de 77 kg pilotée par
un microcontrôleur exécutant le firmware **SeekurOS**, protocole série
client-serveur hérité de la lignée PSOS / P2OS / ARCOS de MobileRobots.

### Décision d'architecture : nœud simple, pas `ros2_control`

Contrairement au Volet A (voir plus bas), ce projet **n'utilise pas
`ros2_control`**. Ce choix est délibéré :

- Le firmware SeekurOS réalise déjà la cinématique différentielle en interne
- Il n'expose pas les encodeurs individuels des roues
- Il fournit directement une odométrie fusionnée (encodeurs + gyroscope) via
  les paquets SIP

Un `diff_drive_controller` serait donc redondant : il recalculerait une
cinématique déjà faite et n'aurait pas les entrées nécessaires. Un nœud ROS2
simple qui traduit `/cmd_vel` ↔ protocole SeekurOS est plus direct et
pleinement compatible avec l'écosystème nav2/SLAM.

### Frontière simulation / réel

```
                    ┌─── TOPICS PUBLICS (identiques sim et réel) ───┐
                    │  /cmd_vel   /odom   TF odom→base_footprint    │
                    └───────────────────┬───────────────────────────┘
                                        │
                            seekur_driver_node.py
                                        │
                    ┌───────────────────┴───────────────────┐
                    │                                       │
              [SIMULATION]                            [ROBOT RÉEL]
                    │                                       │
         TCP localhost:9999                        série /dev/ttyUSB0
                    │                                       │
      seekur_protocol_simulator.py                  microcontrôleur
                    │                                    SeekurOS
      /sim/cmd_vel  │  /sim/odom
                    │
                 Gazebo
```

Les topics `/sim/*` restent internes à la simulation. Tout ce qui est en
amont du driver ne voit aucune différence entre les deux modes.

**Passage au robot réel : deux paramètres seulement**

| Paramètre | Simulation | Robot réel |
|---|---|---|
| `serial_port` | `tcp://localhost:9999` | `/dev/ttyUSB0` |
| `use_sim_time` | `true` | `false` |

Une seule exception à cette règle, côté perception : `depth_image_units_divisor`
(voir la section Pièges).

---

## Prérequis

### Plateforme

| | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz sim 8.x) |
| Python | 3.12 |

### Configuration d'environnement obligatoire

À placer dans `~/.bashrc` :

```bash
# CycloneDDS : OBLIGATOIRE pour la caméra
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Rendu Gazebo sur GPU NVIDIA (laptop Optimus / PRIME offload)
alias ros2nv='__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2'
```

**CycloneDDS n'est pas optionnel.** Avec FastDDS (le défaut de Jazzy), les
topics caméra s'effondrent de 27 Hz à 6-13 Hz avec des blocages de près d'une
seconde : FastDDS échoue à reconstituer les messages fragmentés sur loopback.
Diagnostic complet documenté dans `urdf/seekur_jr_simple.urdf.xacro`.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### Séquence de sourcing

`conda deactivate` est **requis** avant tout : un environnement conda actif
entre en conflit avec les bibliothèques Python de ROS2.

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/yolo_ws/install/setup.bash      # si perception
source ~/seekur_ws/install/setup.bash
```

---

## Installation

```bash
git clone https://github.com/mohlink/seekur_ws.git ~/seekur_ws
cd ~/seekur_ws
colcon build --packages-select seekur_driver
source install/setup.bash
```

> **Ne pas utiliser `--symlink-install`.** Sur ce package Python en Jazzy, le
> symlink invalide les métadonnées et provoque un `PackageNotFoundError` au
> lancement des nœuds. Le build normal prend une seconde.

### Perception (optionnel, requis pour `sim_yolo.launch.py`)

`yolo_ros` est du code tiers, installé dans un workspace séparé :

```bash
mkdir -p ~/yolo_ws/src && cd ~/yolo_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
cd yolo_ros && uv sync
cd ~/yolo_ws && colcon build
```

> **Ne pas lancer `rosdep install`.** Les dépendances Python (PyTorch,
> Ultralytics) sont gérées par `uv` dans un venv dédié. `rosdep` voudrait les
> réinstaller globalement, avec un risque d'obtenir une version CPU-only de
> PyTorch.

Vérification GPU :

```bash
~/yolo_ws/src/yolo_ros/.venv/bin/python -c "import torch; print(torch.cuda.is_available())"
# doit afficher True
```

---

## Utilisation

Un fichier de lancement par mode, plutôt que des drapeaux à l'exécution. Les
variantes coexistent pour permettre la comparaison directe.

| Launch | Contenu |
|---|---|
| `sim.launch.py` | Gazebo + robot + bridge + simulateur + driver + RViz2 |
| `sim_ekf.launch.py` | idem + fusion EKF (IMU + odométrie roues) |
| `slam.launch.py` | cartographie SLAM Toolbox |
| `nav.launch.py` | navigation nav2 sur odométrie brute |
| `nav_ekf.launch.py` | navigation nav2 sur odométrie fusionnée |
| `sim_yolo.launch.py` | simulation + détection YOLO 3D + rqt_image_view |

```bash
ros2nv launch seekur_driver sim_yolo.launch.py
ros2nv launch seekur_driver nav_ekf.launch.py world:=mine_gallery.sdf
```

### Mondes disponibles

- `mine_gallery.sdf` — labyrinthe de galeries (22 murs, ~20 × 13 m), contient
  une cible humaine à x=2.6 pour tester la détection
- `warehouse_simple.sdf` — environnement d'entrepôt

---

## Structure

```
src/seekur_driver/
├── config/
│   ├── gz_bridge.yaml           # ponts ROS2 ↔ Gazebo
│   ├── nav2_params.yaml         # paramètres nav2 (source unique)
│   ├── slam_toolbox_params.yaml
│   ├── ekf.yaml                 # robot_localization
│   ├── seekur_params.yaml       # paramètres driver
│   └── seekur_viz.rviz
├── launch/                      # un fichier par mode (voir tableau)
├── urdf/
│   └── seekur_jr_simple.urdf.xacro
├── worlds/
│   ├── mine_gallery.sdf
│   └── warehouse_simple.sdf
└── seekur_driver/
    ├── seekur_driver_node.py         # driver ROS2 (série ou TCP)
    ├── seekur_protocol.py            # protocole SeekurOS
    └── seekur_protocol_simulator.py  # simulateur TCP du robot
```

---

## État du projet

### Validé

| Jalon | Tag | Contenu |
|---|---|---|
| Localisation | `v4.0-amcl-ready` | map_server + AMCL |
| Dimensions réelles | `v5.1-real-dimensions` | géométrie datasheet Adept Rev B |
| Fusion inertielle | `v6.0-ekf-fusion` | EKF IMU + odométrie, TF propre |
| Caméra RGB-D | `v7.0-camera-d455` | D455 simulée, 848×480 @ 27 Hz |
| Perception | `v8.0-yolo-perception` | YOLO 3D, détection personnes/véhicules |

**Navigation + EKF** — TF `map→odom` (AMCL, 25 Hz) et `odom→base_footprint`
(EKF, 30 Hz), sans conflit. Navigation autonome fonctionnelle, dérive réduite.

**Perception** — détection stable à 28,8 Hz. Score 0,906 de face, 0,725 de
profil. Position 3D mesurée : personne à 2,602 m (vérité terrain Gazebo)
détectée à 2,450 m, soit 15 cm d'écart correspondant à la demi-épaisseur du
torse (le depth mesure la surface avant, la pose Gazebo le centre). Écart
latéral 2 mm. Consommation : 216 Mo VRAM, 9 % GPU sur RTX 4060.

### En cours

- **P4 — RTAB-Map** : cartographie 3D dense de galerie, prochaine brique

### En attente de mesures physiques

Ces valeurs sont estimées et fonctionnent en simulation, mais devront être
mesurées sur le robot avant déploiement :

| Paramètre | Valeur actuelle | Statut |
|---|---|---|
| `wheel_separation` | 0.68 m | estimé (largeur hors-tout − largeur pneu) |
| Position caméra | `xyz="0.3 0 0.6"` | estimé (mât) |
| Position LiDAR | `xyz="0.55 0 0.10"` | d'après photo, à affiner |
| IMU interne (0x9A) | non confirmé | seuls des SIP 0x32 observés |

### Perspectives

- **Jetson Orin Nano** embarquée pour la perception et RTAB-Map. Point ouvert :
  JetPack 6.x repose sur Ubuntu 22.04 (ROS2 Humble), alors que ce projet est en
  Jazzy. À vérifier si JetPack 7 lève la contrainte, sinon conteneur ou repli
  sur laptop embarqué.
- **Dataset minier** : COCO couvre `person` et les véhicules de surface, mais
  pas les engins miniers (scooptram, jumbo) ni la signalisation souterraine. Un
  dataset annoté spécifique constituerait un volet de recherche à part.
- **Batterie** : remplacement du pack NiMH d'origine par du LiFePO4 24V 50Ah.
  Connecteur de charge à inspecter avant commande du chargeur.

---

## Spécifications matérielles

D'après le datasheet officiel *Adept SeekurJr Rev B* (2011).

| | Valeur |
|---|---|
| Dimensions châssis | 1051 × 494 × 425 mm |
| Roues | pneumatiques 16″ (rayon 203 mm) |
| Masse | 77 kg |
| Empattement | ~0.68 m (à mesurer) |

**Capteurs**

- LiDAR SICK LMS1xx — 270°, 541 points, portée 20 m, 25 Hz
- Caméra Intel RealSense D455 — 848×480 @ 30 Hz, depth 0.6–6.0 m, FOV 90°
- IMU — gyro + accéléromètre 3 axes, 100 Hz

---

## Protocole SeekurOS — points essentiels

Communication série 9600 8N1, trames `0xFA 0xFB [count] [cmd] [type] [arg] [checksum]`.
Séquence d'initialisation : SYNC0/1/2 → OPEN → ENABLE. Watchdog PULSE toutes
les 1,5 s, sinon le robot s'arrête après 2 s. SIP standard toutes les 100 ms.

### Bug firmware : valeurs négatives

Le firmware interprète mal les entiers signés. Une commande `RVEL -10` encodée
en complément à deux (`0xF6 0xFF`) est lue comme 65526 °/s — le robot part à
vitesse maximale au lieu de tourner lentement en sens inverse.

**Contournement validé sur le robot** : encoder la direction dans le *type
d'argument*, avec une valeur toujours positive.

| Commande | Direction | Type | Valeur |
|---|---|---|---|
| `VEL` | avant | `0x3B` (INT_POS) | absolue |
| `VEL` | arrière | `0x1B` (INT_SIGNED) | absolue |
| `RVEL` | CCW | `0x3B` | absolue |
| `RVEL` | CW | `0x1B` | absolue |

Ce comportement contredit la documentation, qui décrit `0x1B` comme le type
signé standard.

---

## Pièges connus

Chaque point ci-dessous a coûté du temps de diagnostic. Les détails complets
sont en commentaire dans les fichiers concernés.

### Middleware et environnement

- **FastDDS ne reconstitue pas les gros messages fragmentés sur loopback.**
  Symptôme : cadence caméra qui s'effondre par à-coups, avec des trous d'une
  seconde. Ni le GPU, ni Gazebo, ni RViz, ni la résolution, ni les buffers UDP
  ne sont en cause. Solution : CycloneDDS.
- **`--symlink-install` casse les métadonnées Python** du package
  (`PackageNotFoundError` au lancement). Build normal.
- **conda doit être désactivé** avant de sourcer ROS2.

### Gazebo / URDF

- **`gz-sim-sensors-system` ne doit pas être déclaré dans le SDF du monde**
  s'il est déjà chargé par le xacro du robot — sinon crash Ogre2.
- **Le nom du monde dans le SDF doit être `empty`** : le chemin `joint_state`
  est codé en dur dans `gz_bridge.yaml`.
- **`publish_wheel_tf` doit être à `false`** dans le plugin DiffDrive, sinon
  double publication des TF de roues avec `robot_state_publisher`.
- **`ros_frame_id` est ignoré par `ros_gz_bridge` 1.0.x** (Jazzy). Utiliser
  `gz_frame_id` à la source, dans le xacro.

### nav2 / ROS2 Jazzy

- **`enable_stamped_cmd_vel: false`** requis dans `controller_server` : Jazzy
  utilise `TwistStamped` par défaut, le driver attend `Twist`.
- **`inflation_radius` ≥ rayon inscrit du robot**, sinon le planificateur rase
  les murs.
- **QoS dans RViz** : `/map` nécessite Transient Local, `/particle_cloud` Best
  Effort.

### Perception

- **`depth_image_units_divisor` : 1 en simulation, 1000 sur le robot réel.**
  Gazebo publie le depth en `32FC1` (mètres), `realsense2_camera` en `16UC1`
  (millimètres). Avec la valeur par défaut sur du depth Gazebo, toutes les
  détections se retrouvent à 3 mm de la caméra.

---

## Volet A — NaviBot

Ce dépôt constitue le **Volet B** du projet. Le Volet A
([`mohlink/robot_ws`](https://github.com/mohlink/robot_ws)) développe une pile
de navigation générique autour d'un driver `diffdrive_generic` modulaire
(Factory Pattern, interface `BaseDriver`, intégration `ros2_control` complète)
sur plateforme Raspberry Pi + Arduino.

Les deux volets partagent un cadre conceptuel et des objectifs de navigation
autonome, mais **aucun code** : l'architecture `ros2_control` du Volet A est
délibérément absente ici, pour les raisons exposées en tête de ce document.
