#!/usr/bin/env python3
"""
person_distance_node.py - Distance des personnes (et classes choisies) a partir
de la detection YOLO 2D et de la profondeur alignee de la D435.

POURQUOI CE NOEUD
-----------------
Sur la Jetson, detect_3d_node de yolo_ros sature un coeur CPU (histogrammes
et quantiles numpy sur TOUS les pixels de chaque boite) : 3D a 5-6 Hz avec
0,7 s de retard, contre 2D a ~25 Hz et 0,12 s (mesures lab 2026-10-04).
Pour la securite, une seule valeur suffit par personne : sa distance. Ce
noeud la calcule sur un PETIT patch au centre de la boite 2D (quelques
centaines de pixels au plus), au rythme des detections.

CE QUE FAIT LE NOEUD (mesure seulement, aucun arret du robot)
------------------------------------------------------------
Pour chaque detection d'une classe retenue (classes, defaut "person") :
  1. patch central de la boite (patch_fraction de sa largeur et hauteur),
     echantillonne a max_patch_pixels pixels au plus ;
  2. mediane des profondeurs valides (zeros, NaN et inf ignores) ;
  3. pixel central + intrinseques camera -> point 3D dans le repere optique ;
  4. TF vers target_frame (base_link) : x avant, y gauche.

ENTREES
  detections_topic  (defaut /yolo/tracking)   yolo_msgs/DetectionArray
      /yolo/tracking plutot que /yolo/detections : meme contenu + id
      persistant par personne (utile pour lisser ou suivre plus tard).
  depth_topic       (defaut /camera_front/aligned_depth_to_color/image_raw)
      16UC1 en mm (reel, diviseur 1000) ou 32FC1 en m (sim, diviseur 1).
  camera_info_topic (defaut /camera_front/color/camera_info)
      La profondeur alignee partage les intrinseques de la couleur.
  Synchronisation : yolo_ros recopie l'horodatage de l'image dans ses
  detections ; on garde les dernieres images de profondeur et on prend celle
  dont l'horodatage est le plus proche (tolerance sync_tolerance).

SORTIES
  /person_distance/detections  yolo_msgs/DetectionArray
      Detections retenues, avec SEULEMENT bbox3d.center rempli (dans
      target_frame), bbox3d.size a zero. Profondeur indisponible : center
      en NaN (voir ci-dessous).
  /person_distance/nearest     std_msgs/Float32
      Distance horizontale sqrt(x^2 + y^2) de la PERSONNE (classe "person")
      la plus proche :
        +inf  : personne detectee (aucune detection retenue)
        0.0   : au moins une personne SANS profondeur valide -> a traiter
                comme TRES PROCHE (la D435 ne mesure pas sous ~0,3 m : une
                personne collee au robot donne une grande boite sans
                profondeur). Choix volontairement prudent.
        sinon : distance en metres
      Publie a chaque message de detections. Un consommateur doit appliquer
      un delai de garde : plus de message = plus d'information, pas "rien".
  /person_distance/nearest_vehicle  std_msgs/Float32
      Meme chose pour toutes les AUTRES classes retenues (vehicules : car,
      truck, bus...). Separe de nearest : la regle d'arret pourra appliquer
      une distance de securite differente aux personnes et aux vehicules.

LIMITES CONNUES
  - ZONE AVEUGLE PROCHE (lab 2026-10-04) : en dessous d'environ 0,7 m du
    centre du robot (~0,3 m de la camera, minimum de la D435), YOLO cesse
    de reconnaitre la personne (elle remplit l'image) et nearest passe a
    +inf, en alternance puis en continu. La regle "0.0 = tres proche" ne se
    declenche donc presque jamais en pratique : CE NOEUD NE COUVRE PAS LE
    CHAMP PROCHE. Le LiDAR doit le couvrir, et la regle d'arret doit tenir
    un delai de maintien (un +inf soudain apres une mesure proche = personne
    probablement toujours la).
  - Vehicules miniers (chargeuses, foreuses...) : absents de COCO. YOLO peut
    les classer "truck"/"car" ou pas du tout ; ne pas compter dessus sans
    reentrainement.
  - Dans le noir, la camera couleur ne voit rien : aucune detection, donc
    +inf. "Personne detectee" ne veut pas dire "personne presente". La
    couche de securite principale reste le LiDAR.
  - La profondeur mesure la SURFACE avant (poitrine), pas le centre du
    corps : ~13-15 cm plus proche (constate en sim avec yolo_ros 3D).
    L'erreur va dans le sens prudent.
  - Au-dela de max_range, la profondeur de la D435 est trop bruitee : la
    detection est ignoree.

VALIDATION (lab 2026-10-04, Jetson)
  ~29 Hz, retard 0,10 s, ~27 % d'un coeur (detect_3d_node : 5-6 Hz, 0,70 s,
  100 %). Lineaire de 1 a 4 m (decalage constant, aucune erreur d'echelle) ;
  profondeur camera verifiee sur une table : 1,032 m pour 1,00 m au ruban.
  Lateral correct (y < 0 a droite du robot). ~2 % de mesures manquees
  (profondeur synchronisee absente), sans consequence. En sim : 3,05 m pour
  3,23 m (centre a centre), ecart = epaisseur du torse + derive d'odometrie.

DEPENDANCE : yolo_msgs vient de ~/yolo_ws -> sourcer yolo_ws avant de lancer.
NE JAMAIS lancer ce noeud sur le ROG contre le robot reel : il s'abonne a la
profondeur brute (0,8 Mo/image) a travers le WiFi, ce qui fait chuter TOUTE
la camera (couleur incluse) a ~13 i/s (constate le 2026-10-04).

Test isole :
  ros2 run seekur_driver person_distance_node                      # reel
  ros2 run seekur_driver person_distance_node --ros-args \\
      -p depth_units_divisor:=1.0 -p use_sim_time:=true             # sim
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformException
from yolo_msgs.msg import DetectionArray


# ---------------------------------------------------------------------------
# Fonctions pures (testables sans ROS)
# ---------------------------------------------------------------------------

_DTYPES = {'16UC1': np.uint16, 'mono16': np.uint16, '32FC1': np.float32}


def depth_to_array(msg: Image) -> np.ndarray:
    """Image ROS de profondeur -> tableau numpy (vue, sans copie si possible)."""
    if msg.encoding not in _DTYPES:
        raise ValueError(f"encodage de profondeur non gere : {msg.encoding}")
    dtype = np.dtype(_DTYPES[msg.encoding])
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    row = msg.step // dtype.itemsize
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row)
    return arr[:, :msg.width]


def patch_depth(depth: np.ndarray, cx: float, cy: float, w: float, h: float,
                fraction: float, max_pixels: int, divisor: float):
    """Mediane de profondeur (m) sur le patch central d'une boite.

    Retourne (profondeur_m ou None, nombre_de_pixels_valides).
    """
    H, W = depth.shape
    half_w = max(1.0, 0.5 * w * fraction)
    half_h = max(1.0, 0.5 * h * fraction)
    u0, u1 = int(max(0, cx - half_w)), int(min(W, cx + half_w + 1))
    v0, v1 = int(max(0, cy - half_h)), int(min(H, cy + half_h + 1))
    if u1 <= u0 or v1 <= v0:
        return None, 0

    n = (u1 - u0) * (v1 - v0)
    stride = max(1, math.ceil(math.sqrt(n / max_pixels)))
    vals = depth[v0:v1:stride, u0:u1:stride].astype(np.float32) / divisor
    vals = vals[np.isfinite(vals) & (vals > 0.0)]
    if vals.size == 0:
        return None, 0
    return float(np.median(vals)), int(vals.size)


def pixel_to_point(u: float, v: float, z: float, k):
    """Pixel (u, v) + profondeur z -> point dans le repere optique.
    k = (fx, fy, cx, cy). Repere optique : x droite, y bas, z avant."""
    fx, fy, cx, cy = k
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def transform_point(p, t, q):
    """Applique une transformation (translation t, quaternion q=(x,y,z,w))."""
    qx, qy, qz, qw = q
    px, py, pz = p
    # v' = v + 2w(u x v) + 2 u x (u x v), u = (qx, qy, qz)
    cx1 = qy * pz - qz * py
    cy1 = qz * px - qx * pz
    cz1 = qx * py - qy * px
    cx2 = qy * cz1 - qz * cy1
    cy2 = qz * cx1 - qx * cz1
    cz2 = qx * cy1 - qy * cx1
    return (px + 2.0 * (qw * cx1 + cx2) + t[0],
            py + 2.0 * (qw * cy1 + cy2) + t[1],
            pz + 2.0 * (qw * cz1 + cz2) + t[2])


def parse_classes(text: str):
    """'person, truck' -> {'person', 'truck'} ; 'all' ou '' -> None (toutes)."""
    items = {c.strip() for c in text.split(',') if c.strip()}
    if not items or items == {'all'}:
        return None
    return items


# ---------------------------------------------------------------------------
# Noeud
# ---------------------------------------------------------------------------

class PersonDistanceNode(Node):

    def __init__(self):
        super().__init__('person_distance')

        self.declare_parameter('detections_topic', '/yolo/tracking')
        self.declare_parameter('depth_topic',
                               '/camera_front/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_front/color/camera_info')
        self.declare_parameter('classes', 'person')
        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('patch_fraction', 0.2)
        self.declare_parameter('max_patch_pixels', 400)
        self.declare_parameter('min_valid_pixels', 20)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('sync_tolerance', 0.02)

        gp = self.get_parameter
        self.classes = parse_classes(gp('classes').value)
        self.divisor = float(gp('depth_units_divisor').value)
        self.fraction = float(gp('patch_fraction').value)
        self.max_pixels = int(gp('max_patch_pixels').value)
        self.min_valid = int(gp('min_valid_pixels').value)
        self.target_frame = gp('target_frame').value
        self.max_range = float(gp('max_range').value)
        self.sync_tol_ns = int(float(gp('sync_tolerance').value) * 1e9)

        self.depth_buf = deque(maxlen=10)   # (stamp_ns, Image)
        self.k = None                       # (fx, fy, cx, cy)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS "sensor data" (best effort) : compatible avec un editeur
        # reliable ou best effort.
        self.create_subscription(Image, gp('depth_topic').value,
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, gp('camera_info_topic').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(DetectionArray, gp('detections_topic').value,
                                 self.on_detections, qos_profile_sensor_data)

        self.pub_dets = self.create_publisher(
            DetectionArray, '/person_distance/detections', 10)
        self.pub_nearest = self.create_publisher(
            Float32, '/person_distance/nearest', 10)
        self.pub_nearest_vehicle = self.create_publisher(
            Float32, '/person_distance/nearest_vehicle', 10)

        self.get_logger().info(
            f"classes={'toutes' if self.classes is None else sorted(self.classes)} "
            f"diviseur={self.divisor:g} patch={self.fraction:g} "
            f"max_pixels={self.max_pixels} target={self.target_frame} "
            f"max_range={self.max_range:g} m")

    # --- Callbacks ----------------------------------------------------------

    def on_depth(self, msg: Image):
        self.depth_buf.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))

    def on_info(self, msg: CameraInfo):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def find_depth(self, stamp_ns: int):
        best, best_dt = None, None
        for t, msg in self.depth_buf:
            dt = abs(t - stamp_ns)
            if best_dt is None or dt < best_dt:
                best, best_dt = msg, dt
        if best is None or best_dt > self.sync_tol_ns:
            return None
        return best

    def on_detections(self, msg: DetectionArray):
        nearest = math.inf           # classe "person"
        nearest_vehicle = math.inf   # toutes les autres classes retenues
        out = DetectionArray()
        out.header = msg.header

        selected = [d for d in msg.detections
                    if self.classes is None or d.class_name in self.classes]

        if selected:
            ready = self.prepare(msg)
            if ready is not None:
                depth, t, q = ready
                for det in selected:
                    dist = self.measure(det, depth, t, q)
                    if dist is None:
                        continue                 # au-dela de max_range
                    out.detections.append(det)
                    if det.class_name == 'person':
                        nearest = min(nearest, dist)
                    else:
                        nearest_vehicle = min(nearest_vehicle, dist)

        self.pub_dets.publish(out)
        self.pub_nearest.publish(Float32(data=float(nearest)))
        self.pub_nearest_vehicle.publish(Float32(data=float(nearest_vehicle)))

    # --- Calcul -------------------------------------------------------------

    def prepare(self, msg: DetectionArray):
        """Profondeur synchronisee + TF. None si une donnee manque (journalise)."""
        if self.k is None:
            self.get_logger().warn('camera_info pas encore recu',
                                   throttle_duration_sec=5.0)
            return None
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        depth_msg = self.find_depth(stamp_ns)
        if depth_msg is None:
            self.get_logger().warn('aucune image de profondeur synchronisee',
                                   throttle_duration_sec=5.0)
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'TF indisponible : {e}',
                                   throttle_duration_sec=5.0)
            return None
        tr, rot = tf.transform.translation, tf.transform.rotation
        return (depth_to_array(depth_msg),
                (tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w))

    def measure(self, det, depth, t, q):
        """Remplit det.bbox3d et retourne la distance horizontale (m).
        0.0 si profondeur indisponible, None si au-dela de max_range."""
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        z, n_valid = patch_depth(depth, cx, cy, det.bbox.size.x, det.bbox.size.y,
                                 self.fraction, self.max_pixels, self.divisor)

        box = det.bbox3d
        box.frame_id = self.target_frame
        box.size.x = box.size.y = box.size.z = 0.0
        box.center.orientation.x = box.center.orientation.y = 0.0
        box.center.orientation.z = 0.0
        box.center.orientation.w = 1.0

        if z is None or n_valid < self.min_valid:
            nan = float('nan')
            box.center.position.x = box.center.position.y = nan
            box.center.position.z = nan
            return 0.0                       # prudent : traite comme tres proche

        if z > self.max_range:
            return None

        x, y, zz = transform_point(pixel_to_point(cx, cy, z, self.k), t, q)
        box.center.position.x, box.center.position.y = x, y
        box.center.position.z = zz
        return math.hypot(x, y)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
