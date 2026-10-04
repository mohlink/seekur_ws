# Reconstruction des dépendances externes

Le stack YOLO 3D vit dans un workspace séparé `~/yolo_ws` pour isoler ses
dépendances Python GPU (PyTorch CUDA, Ultralytics) du workspace ROS2
principal. Ce fichier documente sa reconstruction, sur le ROG (x86_64) et
sur la Jetson Orin Nano (aarch64).

## Fonctionnement du venv d'exécution (à comprendre avant tout)

- `yolo_bringup/launch/yolo.launch.py` exécute, À CHAQUE LANCEMENT,
  `uv sync --project <install>/share/yolo_ros`, puis ajoute le
  `site-packages` de ce venv en tête du `PYTHONPATH`.
- Le venv réellement utilisé est donc
  `~/yolo_ws/install/yolo_ros/share/yolo_ros/.venv`, créé au premier
  lancement. Un `uv sync` dans `src/yolo_ros` n'est PAS nécessaire.
- `uv sync` rend ce venv identique au projet : un paquet installé à la main
  dedans est remplacé au lancement suivant. Pour changer une version, il faut
  agir sur la résolution (index uv), pas sur le venv.
- Pas de `uv.lock` upstream au commit épinglé. torch n'est pas déclaré
  directement : il arrive comme dépendance de `ultralytics==8.4.6`.

## Prérequis (les deux machines)

- ROS2 Jazzy sourcé
- `uv` installé (https://docs.astral.sh/uv/)
- `python3-vcstool` installé (`sudo apt install python3-vcstool`)
- AUCUN venv ni conda actif dans le shell (voir Pièges)

## Procédure commune

    mkdir -p ~/yolo_ws/src && cd ~/yolo_ws
    vcs import src < ~/seekur_ws/yolo.repos
    which python3            # doit afficher /usr/bin/python3
    source /opt/ros/jazzy/setup.bash
    colcon build

Le premier `ros2 launch yolo_bringup yolo.launch.py ...` construit le venv
(une à deux minutes, le launch semble figé pendant ce temps).

Ne PAS lancer `rosdep install` dans ~/yolo_ws : il réinstallerait des
paquets Python CPU-only en global.

## ROG-Strix (x86_64, RTX 4060)

Aucune configuration uv particulière : torch est résolu depuis PyPI.
Versions observées : torch 2.13.0+cu130, Ultralytics 8.4.6.
Validé en simulation : 29,7 Hz sur `/yolo/detections_3d`.

## Jetson Orin Nano (aarch64, JetPack 7.2.1, CUDA 13.2)

Le torch aarch64 de PyPI vise les GPU datacenter et ne cible pas l'Orin
(capacité 8.7). Il faut le torch de l'index PyTorch cu132, imposé par une
configuration uv propre à l'utilisateur de la Jetson, AVANT le premier
lancement :

    mkdir -p ~/.config/uv
    cat > ~/.config/uv/uv.toml <<'UVEOF'
    [[index]]
    name = "pytorch-cu132"
    url = "https://download.pytorch.org/whl/cu132"
    UVEOF

Cette configuration est hors du dépôt, survit aux `colcon build` et ne
concerne que la Jetson.

Versions attendues dans le venv d'exécution : torch 2.14.1+cu132,
torchvision 0.29.1+cu132, numpy 1.26.4. Vérification :

    ~/yolo_ws/install/yolo_ros/share/yolo_ros/.venv/bin/python -c \
      "import torch; print(torch.__version__, torch.cuda.is_available())"

Attendu : `2.14.1+cu132 True`. `get_arch_list()` ne contient pas sm_87,
c'est normal : les binaires sm_80 tournent sur l'Orin (convolution et NMS
CUDA vérifiées).

Validé le 2026-10-03 : YOLOv8n à ~28,8 Hz sur `/camera_front/color/image_raw`
(caméra réelle), GPU 23-43 %, RAM totale du robot 3,65/7,5 Go.

## Pièges connus

- `colcon build` avec un venv actif : CMake prend le Python du venv
  (variable VIRTUAL_ENV) et échoue avec `No module named 'em'`. Corriger :
  `deactivate`, vérifier `which python3`, puis `rm -rf build install log`
  (le cache CMake a mémorisé le mauvais Python) et recompiler.
- tmux hérite de l'environnement du shell qui l'a démarré : ne jamais lancer
  `tmux new` depuis un shell où un venv est actif.
- Téléchargements interrompus sur le WiFi de la Jetson (wheels CUDA de
  plusieurs centaines de Mo) :
  `export UV_HTTP_TIMEOUT=300 UV_CONCURRENT_DOWNLOADS=2`, puis relancer ;
  uv reprend depuis son cache.

## Mise à jour de la version épinglée

Éditer `yolo.repos`, changer le champ `version` vers un nouveau SHA du
dépôt upstream, puis relancer la procédure. Sur la Jetson, revérifier la
version de torch obtenue après le premier lancement.

