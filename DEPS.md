# Reconstruction des dépendances externes

Le stack YOLO 3D vit dans un workspace séparé `~/yolo_ws` pour isoler
ses dépendances Python GPU (PyTorch CUDA, Ultralytics) du workspace ROS2
principal. Ce fichier documente sa reconstruction à l'identique.

## Prérequis
- ROS2 Jazzy sourcé, `conda deactivate` fait
- `uv` installé (https://docs.astral.sh/uv/)
- `python3-vcstool` installé (`sudo apt install python3-vcstool`)

## Procédure

    mkdir -p ~/yolo_ws/src && cd ~/yolo_ws
    vcs import src < ~/seekur_ws/yolo.repos
    cd src/yolo_ros && uv sync
    cd ~/yolo_ws && colcon build

Le `uv sync` reconstruit le venv `.venv/` avec les versions exactes du
`uv.lock` upstream (PyTorch 2.13.0+cu130, Ultralytics 8.4.6).

Ne PAS lancer `rosdep install` dans ~/yolo_ws — il réinstallerait des
paquets Python CPU-only en global et écraserait le venv GPU.

## Mise à jour de la version épinglée

Éditer `yolo.repos`, changer le champ `version` vers un nouveau SHA du
dépôt upstream, puis relancer la procédure ci-dessus.
