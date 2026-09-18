#!/usr/bin/env python3
"""
build_mine_gallery.py - Generateur de monde SDF galeries de mine

METHODE B validee en N3 : au lieu d'ecrire ~500 lignes de XML SDF a la
main, on decrit le monde en une trentaine de lignes Python lisibles et
le script pond le SDF. Facile a iterer : on change 3 chiffres, on
regenere, on retest. Un apercu ASCII s'affiche avant generation pour
validation visuelle rapide.

Convention du monde :
  - Repere : X vers l'est, Y vers le nord, Z vers le haut (ROS standard)
  - Robot spawn a (0, 0, 0.1) dans la chambre centrale
  - Galeries de largeur 3.0 m, hauteur 2.5 m
  - Murs de 0.2 m d'epaisseur

Topologie generee (approuvee en debut N3) :

              +---------+
              |         |
              | CHAMBRE |     6x6 m au centre
              |         |
              +--+---+--+
                 |   |
                 |   +-> cul-de-sac lateral (3 m)
                 |
              +--+-------+
              |          |
              | Interse. |     T-junction 5x4 m
              |    T     |
              +--+----+--+
                 |    |
             couloir couloir
              droit  avec
              10 m   virage

Usage :
  # Preview seul (pas de fichier ecrit) :
  python3 build_mine_gallery.py --preview

  # Generation vers l'emplacement standard :
  python3 build_mine_gallery.py

  # Chemin de sortie personnalise :
  python3 build_mine_gallery.py -o /tmp/test.sdf
"""

import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple


# ============================================================================
# Constantes du monde
# ============================================================================

GALLERY_WIDTH = 3.0      # largeur des galeries (m)
WALL_THICKNESS = 0.2     # epaisseur des murs (m)
WALL_HEIGHT = 2.5        # hauteur des murs (m)
FLOOR_MATERIAL_AMBIENT = "0.3 0.3 0.3 1"
FLOOR_MATERIAL_DIFFUSE = "0.4 0.4 0.4 1"
WALL_MATERIAL_AMBIENT = "0.5 0.4 0.3 1"    # teinte terreuse
WALL_MATERIAL_DIFFUSE = "0.6 0.5 0.4 1"


# ============================================================================
# Description haut niveau du monde (a editer pour changer la topologie)
# ============================================================================

@dataclass
class Rect:
    """Zone rectangulaire (chambre, segment de galerie).
    x, y = centre. w, l = largeur (X), longueur (Y)."""
    name: str
    x: float
    y: float
    w: float
    l: float


# Layout du monde. Chaque Rect est une zone OUVERTE (sans mur interne).
# Les murs sont generes automatiquement autour du contour de l'union
# de toutes les zones : un mur n'est pose que sur les segments qui
# ne sont PAS partages avec une autre zone.
#
# Modifie librement cette liste pour changer la topologie. Le preview
# ASCII te dira tout de suite si le layout tient debout.

ZONES: List[Rect] = [
    # Chambre centrale 6x6 m, robot spawne au centre (0,0)
    Rect(name="chambre",         x=0.0,   y=0.0,   w=6.0, l=6.0),

    # Galerie principale vers le sud
    Rect(name="gal_sud",         x=0.0,   y=-6.0,  w=3.0, l=6.0),

    # Cul-de-sac lateral vers l'est, au MILIEU de gal_sud
    Rect(name="cul_de_sac",      x=3.5,   y=-6.0,  w=4.0, l=3.0),

    # Intersection en T au sud de la galerie principale
    Rect(name="intersection_T",  x=0.0,   y=-11.5, w=8.0, l=5.0),

    # Couloir droit vers le sud, sortant du bord ouest du T
    Rect(name="couloir_droit",   x=-2.5,  y=-19.0, w=3.0, l=10.0),

    # Couloir en L : segment vertical qui descend depuis le T,
    # puis segment horizontal qui tourne vers l'est.
    # Le bord ouest du horizontal (x=4.0) touche exactement le bord est
    # du vertical -> vrai virage propre au lieu d'un T inverse.
    Rect(name="virage_vert",     x=2.5,   y=-17.0, w=3.0, l=6.0),
    Rect(name="virage_horiz",    x=6.5,   y=-18.5, w=5.0, l=3.0),
]
# ============================================================================
# Preview ASCII
# ============================================================================

def preview_ascii(zones: List[Rect], scale: float = 0.5) -> str:
    """Rendu ASCII vue de dessus, X vers la droite, Y vers le haut.
    scale = combien de metres par caractere. Plus petit = plus detaille."""

    if not zones:
        return "(monde vide)"

    # Bounding box de toutes les zones
    min_x = min(z.x - z.w/2 for z in zones)
    max_x = max(z.x + z.w/2 for z in zones)
    min_y = min(z.y - z.l/2 for z in zones)
    max_y = max(z.y + z.l/2 for z in zones)

    # Marge
    min_x -= 2
    max_x += 2
    min_y -= 2
    max_y += 2

    cols = int((max_x - min_x) / scale) + 1
    rows = int((max_y - min_y) / scale) + 1

    # Grille : ' ' = exterieur, '.' = interieur galerie
    grid = [[' '] * cols for _ in range(rows)]

    def to_cell(x, y):
        col = int((x - min_x) / scale)
        row = rows - 1 - int((y - min_y) / scale)  # Y inverse pour ASCII
        return row, col

    # Remplir les zones
    for z in zones:
        r1, c1 = to_cell(z.x - z.w/2, z.y + z.l/2)
        r2, c2 = to_cell(z.x + z.w/2, z.y - z.l/2)
        for r in range(min(r1, r2), max(r1, r2) + 1):
            for c in range(min(c1, c2), max(c1, c2) + 1):
                if 0 <= r < rows and 0 <= c < cols:
                    grid[r][c] = '.'

    # Marquer l'origine (spawn du robot)
    r0, c0 = to_cell(0, 0)
    if 0 <= r0 < rows and 0 <= c0 < cols:
        grid[r0][c0] = 'R'

    lines = []
    lines.append(f"Preview ASCII (echelle: {scale} m/char, R = spawn robot)")
    lines.append(f"Bounding box: X=[{min_x:.1f}..{max_x:.1f}] Y=[{min_y:.1f}..{max_y:.1f}]")
    lines.append("")
    for row in grid:
        lines.append(''.join(row))
    return '\n'.join(lines)


# ============================================================================
# Generation SDF
# ============================================================================

def sdf_header() -> str:
    return """<?xml version="1.0" ?>
<sdf version="1.9">
  
  <!-- name="empty" impose par gz_bridge.yaml (chemin joint_state en dur) -->
  <world name="empty">

    <!-- Physique standard -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins Gazebo essentiels -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- gz-sim-sensors-system est charge par le xacro du robot -->

    <!-- Lumiere du soleil (mine avec eclairage indirect) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -1</direction>
    </light>

    <!-- Lumiere ambiante douce (mine sombre) -->
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.2 0.2 0.2 1</background>
      <shadows>true</shadows>
    </scene>
"""


def sdf_floor(min_x: float, max_x: float, min_y: float, max_y: float) -> str:
    """Un grand sol qui couvre toute la bounding box + marge."""
    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    sx = (max_x - min_x) + 4
    sy = (max_y - min_y) + 4
    return f"""
    <!-- Sol -->
    <model name="floor">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} 0.1</size></box></geometry>
          <material>
            <ambient>{FLOOR_MATERIAL_AMBIENT}</ambient>
            <diffuse>{FLOOR_MATERIAL_DIFFUSE}</diffuse>
          </material>
        </visual>
        <pose>0 0 -0.05 0 0 0</pose>
      </link>
    </model>
"""


def sdf_wall(name: str, x: float, y: float, sx: float, sy: float) -> str:
    """Un mur = un box statique. sx et sy sont les dimensions."""
    return f"""
    <model name="wall_{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
          <material>
            <ambient>{WALL_MATERIAL_AMBIENT}</ambient>
            <diffuse>{WALL_MATERIAL_DIFFUSE}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def sdf_footer() -> str:
    return """
  </world>
</sdf>
"""


# ============================================================================
# Extraction des segments de murs
# ============================================================================
#
# Algorithme : on utilise une grille discrete (pas = 0.25 m) qui couvre
# la bounding box. Pour chaque cellule, on marque 'interieur' si elle
# est dans au moins une Rect. Puis pour chaque cellule interieure, on
# cree un segment de mur sur chaque de ses 4 aretes qui touche une
# cellule exterieure. Enfin, on fusionne les segments contigus pour
# obtenir des murs plus longs (evite d'avoir 100 petits murs alignes).
#
# Ce systeme donne "gratuitement" les intersections propres : si deux
# Rect se touchent, l'arete commune n'a pas de mur.


GRID_STEP = 0.25  # metres par cellule


def build_wall_segments(zones: List[Rect]) -> List[Tuple[str, float, float, float, float]]:
    """Retourne une liste de murs (name, cx, cy, sx, sy)."""

    if not zones:
        return []

    min_x = min(z.x - z.w/2 for z in zones) - 1
    max_x = max(z.x + z.w/2 for z in zones) + 1
    min_y = min(z.y - z.l/2 for z in zones) - 1
    max_y = max(z.y + z.l/2 for z in zones) + 1

    cols = int((max_x - min_x) / GRID_STEP) + 1
    rows = int((max_y - min_y) / GRID_STEP) + 1

    inside = [[False] * cols for _ in range(rows)]

    def to_grid(x, y):
        return int((y - min_y) / GRID_STEP), int((x - min_x) / GRID_STEP)

    def to_world(row, col):
        return min_x + col * GRID_STEP, min_y + row * GRID_STEP

    # Marquer les cellules interieures
    for z in zones:
        r1, c1 = to_grid(z.x - z.w/2, z.y - z.l/2)
        r2, c2 = to_grid(z.x + z.w/2, z.y + z.l/2)
        for r in range(min(r1, r2), max(r1, r2) + 1):
            for c in range(min(c1, c2), max(c1, c2) + 1):
                if 0 <= r < rows and 0 <= c < cols:
                    inside[r][c] = True

    # Extraire les aretes-murs : une arete est un mur si elle separe
    # interieur et exterieur. On enregistre chaque arete comme un
    # segment horizontal ou vertical dans le monde reel.
    #
    # h_edges[r][c] = mur horizontal sur l'arete SUD de la cellule (r,c)
    # v_edges[r][c] = mur vertical sur l'arete EST de la cellule (r,c)

    h_edges = [[False] * cols for _ in range(rows)]
    v_edges = [[False] * cols for _ in range(rows)]

    for r in range(rows):
        for c in range(cols):
            if not inside[r][c]:
                continue
            # Arete sud : cellule au sud est-elle exterieure ?
            if r == 0 or not inside[r-1][c]:
                h_edges[r][c] = True
            # Arete nord de la cellule = arete sud de la cellule au nord
            if r == rows-1 or not inside[r+1][c]:
                if r+1 < rows:
                    h_edges[r+1][c] = True
                else:
                    # Bord de grille : on met le mur ici-meme decale
                    h_edges[r][c] = True  # sera dedoublonne plus tard
            # Arete ouest
            if c == 0 or not inside[r][c-1]:
                v_edges[r][c] = True
            # Arete est
            if c == cols-1 or not inside[r][c+1]:
                if c+1 < cols:
                    v_edges[r][c+1] = True

    # Fusionner les segments contigus. Pour les murs horizontaux,
    # on scanne ligne par ligne les runs de True consecutifs dans h_edges.
    walls = []
    idx = 0

    # Horizontaux
    for r in range(rows):
        c = 0
        while c < cols:
            if h_edges[r][c]:
                c_start = c
                while c < cols and h_edges[r][c]:
                    c += 1
                c_end = c - 1
                # Ce segment couvre les colonnes c_start..c_end sur la ligne r
                # (arete sud de la ligne r = bord y = min_y + r*step)
                length = (c_end - c_start + 1) * GRID_STEP
                cx = min_x + (c_start + c_end + 1) / 2 * GRID_STEP
                cy = min_y + r * GRID_STEP
                walls.append((f"h{idx}", cx, cy, length, WALL_THICKNESS))
                idx += 1
            else:
                c += 1

    # Verticaux
    for c in range(cols):
        r = 0
        while r < rows:
            if v_edges[r][c]:
                r_start = r
                while r < rows and v_edges[r][c]:
                    r += 1
                r_end = r - 1
                length = (r_end - r_start + 1) * GRID_STEP
                cx = min_x + c * GRID_STEP
                cy = min_y + (r_start + r_end + 1) / 2 * GRID_STEP
                walls.append((f"v{idx}", cx, cy, WALL_THICKNESS, length))
                idx += 1
            else:
                r += 1

    return walls


# ============================================================================
# Main
# ============================================================================

def default_output_path() -> str:
    """Emplacement standard dans le workspace."""
    return os.path.expanduser(
        "~/seekur_ws/src/seekur_driver/worlds/mine_gallery.sdf"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('-o', '--output', default=default_output_path(),
                        help='Chemin du fichier .sdf a generer')
    parser.add_argument('--preview', action='store_true',
                        help='Afficher seulement l\'apercu ASCII, sans generer')
    parser.add_argument('--preview-scale', type=float, default=0.5,
                        help='Metres par caractere pour l\'apercu ASCII (defaut 0.5)')
    args = parser.parse_args()

    # Preview
    print(preview_ascii(ZONES, args.preview_scale))
    print()

    if args.preview:
        print("(--preview specifie : pas de generation du SDF)")
        return

    # Bounding box (pour dimensionner le sol)
    min_x = min(z.x - z.w/2 for z in ZONES) - 2
    max_x = max(z.x + z.w/2 for z in ZONES) + 2
    min_y = min(z.y - z.l/2 for z in ZONES) - 2
    max_y = max(z.y + z.l/2 for z in ZONES) + 2

    # Construction des murs
    walls = build_wall_segments(ZONES)
    print(f"Zones definies    : {len(ZONES)}")
    print(f"Murs generes      : {len(walls)}")
    print(f"Bounding box (m)  : X=[{min_x:.1f}..{max_x:.1f}]  Y=[{min_y:.1f}..{max_y:.1f}]")

    # Generation SDF
    parts = [sdf_header()]
    parts.append(sdf_floor(min_x, max_x, min_y, max_y))
    for name, x, y, sx, sy in walls:
        parts.append(sdf_wall(name, x, y, sx, sy))
    parts.append(sdf_footer())
    sdf = ''.join(parts)

    # Ecriture
    output = os.path.expanduser(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, 'w') as f:
        f.write(sdf)
    print(f"\nGenere : {output}")
    print(f"Taille : {os.path.getsize(output)} octets")
    print(f"\nTest :")
    print(f"  ros2 launch seekur_driver sim.launch.py world:=mine_gallery.sdf")


if __name__ == '__main__':
    main()
