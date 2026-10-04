# SeekurJR — Navigation autonome et perception pour environnement minier

Pile ROS2 complète pour le robot **Adept MobileRobots SeekurJR** : driver
SeekurOS, jumeau virtuel Gazebo, navigation autonome nav2, fusion inertielle
et perception multimodale. Développé dans le cadre d'un projet de recherche à
l'**UQAT**, avec pour objectif la navigation autonome en galerie minière
souterraine.

Le projet suit une approche **simulation-vers-réel** : toute la chaîne est
développée et validée sur un jumeau virtuel, puis portée sur le robot physique
en changeant deux paramètres.

> Pour les commandes de lancement quotidiennes, voir `MEMO.md`.
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

# Rendu Gazebo sur GPU NVIDIA (laptop Optimus / PRIME offload) + garde-fou
# sim/reel : ros2nv est une FONCTION (plus un alias), definie dans le bloc
# « Isolation simulation / robot réel » ci-dessous.
```

**CycloneDDS n'est pas optionnel.** Avec FastDDS (le défaut de Jazzy), les
topics caméra s'effondrent de 27 Hz à 6-13 Hz avec des blocages de près d'une
seconde : FastDDS échoue à reconstituer les messages fragmentés sur loopback.
Diagnostic complet documenté dans `urdf/seekur_jr_simple.urdf.xacro`.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### Isolation simulation / robot réel (ROG)

La simulation et le robot réel utilisent les mêmes noms de topics. Sur un même
domaine DDS, un nœud lancé pour la sim peut se brancher sans prévenir sur le
vrai robot. Vécu le 2026-10-04 : `person_distance`, lancé pour la sim puis
oublié, s'est abonné à la profondeur du robot à travers le WiFi et a fait
chuter toute la caméra à ~13 i/s. Convention :

| Mode | Domaine DDS | Prompt | Usage |
|---|---|---|---|
| normal (défaut) | 0 | `user@ROG-Strix:~$` | dialoguer avec le robot (Jetson, domaine 0) |
| `sim` | 42 | `[SIM 42] user@ROG-Strix:~$` (rouge) | Gazebo et tout ce qui s'y connecte |

- Taper `sim` dans **chaque** terminal de simulation (launch, téléop, RViz,
  `ros2 topic echo`...) ; `real` pour revenir. Un nouveau terminal démarre
  en mode normal.
- `ros2nv launch` **refuse** une simulation (`sim*`, `slam`, `nav`, `nav_ekf`,
  `gazebo_simple`, `seekur_navigation`) hors mode sim, et un launch `real*`
  en mode sim. `ros2 launch` sans `nv` n'est pas contrôlé : toujours lancer
  la sim avec `ros2nv`.
- Le Jetson n'a rien de particulier : il reste au domaine 0.
- Le fichier CycloneDDS doit garder `<Domain id="any">`, sinon il ignore
  `ROS_DOMAIN_ID`.
- Vérification : en mode normal, `ros2 node list` montre les nœuds du robot ;
  en mode `sim`, il ne les montre plus.

Bloc à ajouter à la fin de `~/.bashrc` sur toute machine de développement
(il remplace l'ancien `alias ros2nv`, à supprimer) :

```bash
# ---------------------------------------------------------------------------
# Isolation simulation / robot reel (2026-10-04)
# Robot reel (Jetson) et terminaux normaux : domaine DDS 0 (ROS_DOMAIN_ID vide).
# Simulation : domaine 42, SEULEMENT dans les terminaux ou l'on a tape `sim`.
# ---------------------------------------------------------------------------
SEEKUR_SIM_DOMAIN=42
_SEEKUR_PS1_BASE="$PS1"

sim() {
    export ROS_DOMAIN_ID=$SEEKUR_SIM_DOMAIN
    PS1="\[\e[1;31m\][SIM $SEEKUR_SIM_DOMAIN]\[\e[0m\] $_SEEKUR_PS1_BASE"
    echo "Mode SIM : ROS_DOMAIN_ID=$SEEKUR_SIM_DOMAIN (robot reel invisible depuis ce terminal)"
}

real() {
    unset ROS_DOMAIN_ID
    PS1="$_SEEKUR_PS1_BASE"
    echo "Mode REEL : domaine 0 (robot visible, simulation invisible)"
}

# ros2nv : rendu NVIDIA (comme l'ancien alias) + garde-fou sur le domaine.
unalias ros2nv 2>/dev/null
ros2nv() {
    if [ "$1" = "launch" ]; then
        local a base=""
        for a in "$@"; do
            case "$a" in *.launch.py) base="${a##*/}" ;; esac
        done
        case "$base" in
            real*.launch.py)
                if [ "$ROS_DOMAIN_ID" = "$SEEKUR_SIM_DOMAIN" ]; then
                    echo "STOP : $base est un launch du ROBOT REEL, mais ce terminal est en mode SIM. Tapez : real" >&2
                    return 1
                fi ;;
            sim*.launch.py|slam.launch.py|nav.launch.py|nav_ekf.launch.py|gazebo_simple.launch.py|seekur_navigation.launch.py)
                if [ "$ROS_DOMAIN_ID" != "$SEEKUR_SIM_DOMAIN" ]; then
                    echo "STOP : $base est une SIMULATION, mais ce terminal est en mode REEL (robot visible). Tapez : sim" >&2
                    return 1
                fi ;;
        esac
    fi
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 "$@"
}
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

> **Premier lancement : environ 4 Go à télécharger.** `yolo_ros` exécute ses
> nœuds avec `uv run` depuis le dossier installé : l'environnement d'exécution
> est créé au premier `sim_yolo.launch.py` dans
> `~/yolo_ws/install/yolo_ros/share/yolo_ros/.venv` (PyTorch, CUDA, Ultralytics).
> Les lancements suivants démarrent directement.

Vérification GPU (après un premier lancement) :

```bash
~/yolo_ws/install/yolo_ros/share/yolo_ros/.venv/bin/python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# attendu : 2.13.0+cu130 True
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
| `sim_rtabmap.launch.py` | simulation + EKF + RTAB-Map SLAM 3D (remplace slam_toolbox) |

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
│   ├── rtabmap_params.yaml      # paramètres RTAB-Map SLAM 3D
│   ├── ekf.yaml                 # robot_localization
│   ├── seekur_params.yaml       # paramètres driver
│   └── seekur_viz.rviz
├── launch/                      # un fichier par mode (voir tableau)
├── urdf/
│   └── seekur_jr_simple.urdf.xacro
├── worlds/
│   ├── build_mine_gallery.py    # générateur du SDF ci-dessous
│   ├── mine_gallery.sdf
│   └── warehouse_simple.sdf
├── tools/                       # utilitaires troubleshoot (voir tools/README.md)
│   ├── interactive/             # contrôleurs SeekurOS Python
│   └── low_level/               # diagnostic série C++
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
| SLAM 3D | `v9.0-rtabmap` | RTAB-Map dense + fermeture de boucle (593 keyframes / 73m / 16 loop closures sur mine_gallery) |

**Navigation + EKF** — TF `map→odom` (AMCL, 25 Hz) et `odom→base_footprint`
(EKF, 30 Hz), sans conflit. Navigation autonome fonctionnelle, dérive réduite.

**Perception** — détection stable à 28,8 Hz. Score 0,906 de face, 0,725 de
profil. Position 3D mesurée : personne à 2,602 m (vérité terrain Gazebo)
détectée à 2,450 m, soit 15 cm d'écart correspondant à la demi-épaisseur du
torse (le depth mesure la surface avant, la pose Gazebo le centre). Écart
latéral 2 mm. Consommation : 216 Mo VRAM, 9 % GPU sur RTX 4060.

### En cours


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
- **Sim et robot réel sur le même domaine DDS.** Symptômes : nœuds en double
  dans `ros2 node list`, valeurs incohérentes, caméra du robot qui ralentit
  sans raison apparente. Solution : mode `sim` (voir Prérequis).
- **Ne jamais s'abonner depuis le ROG aux images brutes du robot**
  (`/camera_front/*/image_raw`, `/yolo/dbg_image`). Un seul abonné distant
  (0,8 à 1,2 Mo/image) sature le WiFi, bloque les envois du nœud RealSense et
  fait perdre des images à la source, pour tous les abonnés (constaté :
  couleur 30 → 13 i/s, profondeur alignée 30 → 4 i/s). Regarder uniquement
  les topics `/compressed`.
- **Mesurer la cadence d'images avec `ros2 bag record`** (C++), pas avec
  `ros2 topic hz` (Python : sature un cœur sur les images) ni via
  `camera_info` (realsense2_camera 4.58 le publie en double ou découplé des
  images). Compter les messages dans `ros2 bag info`.

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
- **`~/yolo_ws/install/` n'est pas un simple artefact de build.** Il contient
  l'environnement d'exécution de `yolo_ros` (voir Installation). Le supprimer
  lors d'un nettoyage disque force le retéléchargement d'environ 4 Go au
  lancement suivant. Le `.venv` de `src/yolo_ros` créé par `uv sync` n'est pas
  utilisé à l'exécution (vérifié : `sim_yolo` à 29,7 Hz sans lui).

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
