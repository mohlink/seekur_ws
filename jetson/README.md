# Jetson Orin Nano — mise en place pour le SeekurJR

Procédure complète pour passer d'une Jetson vierge à une machine embarquée prête
pour `seekur_ws` (ROS 2 Jazzy). Chaque étape a été validée sur la cible le
2026-09-26. Les fichiers de `config/` sont les originaux : on les copie, on ne
les retape pas.

| Élément | Valeur |
|---|---|
| Carte | Jetson Orin Nano Developer Kit 8 Go (module P3767-0005) |
| Stockage | NVMe 465 Go (système) ; carte SD non utilisée |
| JetPack | 7.2.1 (L4T R39.2.1), Ubuntu 24.04, noyau `6.8.12-1021-tegra` |
| CUDA / TensorRT / cuDNN | 13.2 / 10.16.2 / 9.20 |
| ROS 2 | Jazzy, `ros-base` (pas de desktop : la Jetson tourne sans écran) |
| Hostname / utilisateur | `jetson-seekur` / `moh` |
| Mode d'alimentation | 25 W (mode 1) |

Répartition des rôles : la Jetson exécute le driver, l'IMU, le LiDAR, l'EKF,
RTAB-Map, Nav2 et `twist_mux`. Le ROG-Strix garde la manette (`joy`,
`teleop_twist_joy`), RViz et la simulation Gazebo. Les deux communiquent en
CycloneDDS sur le WiFi, domaine 0.

---

## 1. Installation de JetPack 7.2.1

JetPack 7 est nécessaire : JetPack 6.x est limité à Ubuntu 22.04, donc à Humble.

Méthode retenue : **ISO sur clé USB** (`jetsoninstaller-r39.2.1-…-arm64.iso`,
page JetPack de NVIDIA), écrite avec `dd`, balenaEtcher ou Rufus en mode DD,
puis démarrage de la Jetson sur la clé et installation sur le **NVMe**.
Il n'existe plus d'image carte SD pour l'Orin Nano Dev Kit en JP7.

SDK Manager a échoué depuis le ROG : il faut plus de 40 Go libres côté hôte
(le dossier `Linux_for_Tegra` seul a dépassé 27 Go). Si on le réutilise :
carte **P3767-0005** (dev kit), **Storage Device : NVMe** (défaut SD Card),
décocher Host Machine.

Vérification :

```bash
cat /etc/nv_tegra_release        # R39, REVISION: 2.1
lsb_release -ds                  # Ubuntu 24.04.x
df -h /                          # /dev/nvme0n1p1
sudo nvpmodel -q                 # 25W
```

Retirer la clé USB après installation.

## 2. Mise à jour et composants NVIDIA

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y nvidia-jetpack
```

> **Ne jamais lancer `sudo apt autoremove`.** Apt propose de retirer
> `initramfs-tools` et `linux-base` parce que le noyau tegra ne les déclare pas
> comme dépendances. Les retirer peut casser la mise à jour de L4T, voire le
> démarrage. Les protéger :
>
> ```bash
> sudo apt-mark manual initramfs-tools initramfs-tools-bin initramfs-tools-core klibc-utils libklibc linux-base
> ```

Vérification :

```bash
/usr/local/cuda/bin/nvcc --version        # release 13.2
dpkg -l | grep -E "libnvinfer10|cudnn9"
```

## 3. Swap

8 Go de RAM ne suffisent pas pour certaines compilations (`colcon`, venv PyTorch).

```bash
sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 4. ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
echo "ros-apt-source : $ROS_APT_SOURCE_VERSION"   # si vide : l'API GitHub a échoué, s'arrêter
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-demo-nodes-cpp ros-jazzy-rmw-cyclonedds-cpp
```

## 5. Environnement shell et CycloneDDS

```bash
cp ~/seekur_ws/jetson/config/cyclonedds.xml ~/
cat ~/seekur_ws/jetson/config/bashrc_jetson.sh >> ~/.bashrc
cp ~/seekur_ws/jetson/config/tmux.conf ~/.tmux.conf     # facultatif
```
**Gros messages DDS** (nuage de points de la caméra, plusieurs Mo par message) :
sans ce réglage, la mémoire tampon UDP par défaut d'Ubuntu (208 Ko) empêche le
nuage d'arriver, sans aucun message d'erreur. `cyclonedds.xml` demande 10 Mo.

```bash
sudo cp ~/seekur_ws/jetson/config/99-robot-dds.conf /etc/sysctl.d/
sudo sysctl --system | grep -E "rmem_max|ipfrag"
```

(La ligne `source ~/seekur_ws/install/setup.bash` du bloc échoue tant que le
workspace n'est pas compilé à l'étape 10 : sans conséquence.)

**CycloneDDS est obligatoire**, comme sur le ROG : avec FastDDS (défaut de
Jazzy), les gros messages fragmentés (images) s'effondrent. DDS est épinglé
sur l'interface WiFi `wlP1p1s0` par nom, pas par IP : il suit les changements
de réseau.

Test entre machines (talker sur l'une, listener sur l'autre, dans les deux
sens) :

```bash
ros2 run demo_nodes_cpp talker      # Jetson
ros2 run demo_nodes_cpp listener    # ROG ; la découverte prend quelques secondes
```

## 6. Réseau

```bash
sudo cp ~/seekur_ws/jetson/config/99-robot-arp.conf /etc/sysctl.d/
sudo sysctl --system | grep arp

sudo cp ~/seekur_ws/jetson/config/zz-wifi-powersave-off.conf /etc/NetworkManager/conf.d/
NetworkManager --print-config | grep powersave     # wifi.powersave=2
sudo iw dev wlP1p1s0 set power_save off            # effet immédiat
```

- **ARP** : sans ce réglage, avec l'Ethernet et le WiFi sur le même
  sous-réseau, la Jetson perd son IP WiFi pendant environ 5 minutes à un
  renouvellement DHCP (voir le commentaire du fichier).
- **Économie d'énergie WiFi** : elle ajoutait une latence irrégulière. Le nom
  du fichier commence par `zz-` pour être lu **après**
  `default-wifi-powersave-on.conf`.

## 7. Aucune mise en veille

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo mkdir -p /etc/systemd/logind.conf.d
sudo cp ~/seekur_ws/jetson/config/99-robot-no-sleep.conf /etc/systemd/logind.conf.d/
sudo cp ~/seekur_ws/jetson/config/50-usb-no-autosuspend.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=usb --action=add
```

Volontairement **non modifiés** : le mode 25 W et la régulation de fréquence
(pas de `jetson_clocks`), et le démarrage du noyau (`extlinux.conf`).

## 8. Pilotes série ch341 et pl2303

Le noyau JP7.2.1 **ne contient ni `ch341` (IMU) ni `pl2303` (robot)** :
l'USB est énuméré, mais aucun `/dev/ttyUSB*` n'apparaît. Il n'y a pas de
paquet `linux-modules-extra`. On les compile hors arbre avec les en-têtes
fournis :

```bash
sudo apt install -y build-essential
~/seekur_ws/jetson/scripts/build_usb_serial.sh
```

Puis **bloquer le noyau** pour qu'un `apt upgrade` ne fasse pas disparaître
les modules :

```bash
sudo apt-mark hold $(dpkg -l | awk '/^ii  nvidia-l4t-kernel/{print $2}')
apt-mark showhold      # 9 paquets nvidia-l4t-kernel*
```

### Procédure de mise à jour du noyau (JetPack)

Une opération planifiée, au lab, jamais sur le terrain :

```bash
sudo apt-mark unhold $(apt-mark showhold | grep nvidia-l4t-kernel)
sudo apt update && sudo apt upgrade
sudo reboot
~/seekur_ws/jetson/scripts/build_usb_serial.sh
sudo apt-mark hold $(dpkg -l | awk '/^ii  nvidia-l4t-kernel/{print $2}')
ls -l /dev/imu /dev/seekur
```

Si ça tourne mal, la Jetson démarre quand même : seuls `/dev/imu` et
`/dev/seekur` manquent.

## 9. Ports série stables

```bash
sudo usermod -aG dialout moh          # effectif à la prochaine connexion
sudo cp ~/seekur_ws/jetson/config/99-seekur-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

`brltty`, qui s'approprie les CH340, n'est pas installé sur l'image JP7 : à
retirer s'il apparaît (`sudo apt remove brltty`).

Vérification, adaptateurs branchés :

```bash
ls -l /dev/imu /dev/seekur                        # liens vers ttyUSB*
udevadm info -q property -n /dev/imu | grep ID_MM_DEVICE_IGNORE   # =1
```

## 10. Workspace `seekur_ws`

Accès GitHub (dépôt privé) :

```bash
ssh-keygen -t ed25519 -C "moh@jetson-seekur" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub    # à ajouter dans GitHub > Settings > SSH keys
ssh -T git@github.com
```

La clé est sans phrase de passe : si la Jetson est perdue, la révoquer dans GitHub.

```bash
git clone git@github.com:mohlink/seekur_ws.git ~/seekur_ws
sudo rosdep init && rosdep update
cd ~/seekur_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build              # jamais --symlink-install ; toujours depuis ~/seekur_ws
```

### Paquets du robot réel

Les `package.xml` ne déclarent pas les paquets utilisés par les fichiers de
lancement : `rosdep` ne les installe donc pas. Liste explicite :

```bash
NAV=$(apt-cache depends ros-jazzy-navigation2 | awk '/Depends:/{print $2}' | grep -v rviz)
RTAB="ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-util ros-jazzy-rtabmap-sync ros-jazzy-rtabmap-msgs ros-jazzy-rtabmap-conversions"
BASE="ros-jazzy-joint-state-publisher ros-jazzy-xacro ros-jazzy-lms1xx ros-jazzy-robot-localization ros-jazzy-twist-mux"
PKGS="$BASE $RTAB $NAV ros-jazzy-slam-toolbox"
sudo apt install -y --no-install-recommends $PKGS    # ~470 paquets, ~2 Go
```

**Ne pas installer les métapaquets** `navigation2`, `nav2_bringup` ni
`rtabmap_ros` : ils tirent Gazebo complet, RViz et les démonstrations
(660 paquets). Il reste deux dépendances lourdes, inévitables mais inertes :
Qt5/VTK/JDK (via RTAB-Map → PCL) et les bibliothèques RViz (via le panneau
intégré de `slam_toolbox`). Rien ne tourne tant qu'on ne lance pas RViz.

Pas sur la Jetson : `ros_gz_*`, `rviz2`, `rqt_image_view`, `joy`,
`teleop_twist_joy`. Plus tard : `realsense2_camera` et `yolo_ws` (roues
PyTorch spécifiques Jetson, pas celles du `uv.lock` x86_64).

## 11. Validation

```bash
ros2 run bno055_imu bno055_serial_node --ros-args \
  --params-file ~/seekur_ws/src/bno055_imu/config/bno055_params.yaml
ros2 topic hz /imu/data           # ~100 Hz, écart-type < 1 ms (validé)
```

Après chaque redémarrage :

```bash
systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target  # masked x4
iw dev wlP1p1s0 get power_save                     # off
lsmod | grep -E "ch341|pl2303"
ls -l /dev/imu
```

---

## Pièges rencontrés

| Symptôme | Cause | Correction |
|---|---|---|
| Nano détecté par `lsusb`, pas de `/dev/ttyUSB*` | Pas de `ch341`/`pl2303` dans le noyau JP7 | §8 |
| WiFi repasse en économie d'énergie au démarrage | `99-…` lu avant `default-…` | Préfixe `zz-` (§6) |
| SSH coupé ~5 min, IP inchangée | Conflit ARP entre les deux cartes de la Jetson | `99-robot-arp.conf` (§6) |
| `apt` veut retirer `initramfs-tools` | Non déclaré par le noyau tegra | `apt-mark manual` (§2) |
| Installation de 660 paquets dont Gazebo | Métapaquets Nav2/RTAB-Map | Paquets individuels (§10) |
| Aucun octet sur `/dev/imu` pendant 2 s | Chaque ouverture du port réinitialise le Nano (DTR) | Attendre quelques secondes |
| `"` introuvable pour découper tmux | Clavier CA-FR | `tmux.conf` : Ctrl+b puis `-` |
| Nuage de points RealSense jamais reçu, aucune erreur | Tampon UDP de 208 Ko, messages de plusieurs Mo | `99-robot-dds.conf` + `SocketReceiveBufferSize` (§5) |

## Points ouverts

- **`/dev/seekur`** : à valider au lab avec le PL2303 (module chargé, règle en place).
- **Délai d'expiration `/cmd_vel` dans `seekur_driver_node`** : avec la manette
  sur le ROG, une coupure WiFi laisse le robot à la dernière vitesse (le PULSE
  continue). À ajouter avant tout essai au sol.
- **Réseau du bureau / de la mine** : le multicast peut être bloqué
  (isolation des clients). Solutions : pairs unicast dans `cyclonedds.xml`
  ou routeur WiFi dédié au robot.
- **Ethernet** : à passer en IP statique 192.168.0.x pour le LMS111.
- **RTAB-Map** : 0.23.7 sur la Jetson contre 0.22.1 sur le ROG (bases `.db`,
  valeurs par défaut) : aligner le ROG avant le premier SLAM réel.
- **NVMe** : un timeout isolé vu dans `dmesg` ; surveiller
  (`sudo dmesg | grep -c "nvme.*timeout"`), piste APST si ça se répète.
- **Interface graphique** : GNOME consomme 0,5 à 1 Go ;
  `sudo systemctl set-default multi-user.target` si la RAM manque.
- **DKMS** pour recompiler automatiquement `ch341`/`pl2303`.
