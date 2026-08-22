#!/usr/bin/env python3
"""
seekur_driver_node.py - Node ROS2 pour robot SeekurJR
Basé sur le protocole série SeekurOS avec corrections firmware bidirectionnelles

VERSION N2 (2026-08) - Modifications par rapport à la version initiale :

1. SerialTCPAdapter ajouté (porté depuis seekur_interactive1_tcp.py) :
   le paramètre serial_port accepte désormais 'tcp://host:port' pour se
   connecter au simulateur de protocole, ou '/dev/ttyUSBx' pour le vrai
   robot. C'est LE point de bascule sim <-> réel : un seul paramètre.

2. base_frame par défaut : 'base_link' -> 'base_footprint'.
   L'URDF publie base_footprint -> base_link (via robot_state_publisher).
   Le driver publie odom -> base_footprint. Chaîne TF cohérente :
   map -> odom -> base_footprint -> base_link -> capteurs/roues.

3. _io_lock (mutex) protégeant TOUS les accès au port :
   trois threads écrivent/lisent (callback cmd_vel, watchdog PULSE,
   monitoring SIP). Sans verrou, trames entrelacées = corruption
   intermittente. Leçon apprise sur seekur_interactive (navbot11).

4. PROPRIÉTÉ EXCLUSIVE : ce node est le SEUL à publier /odom et la TF
   odom -> base_footprint. Gazebo ne les publie plus (publish_odom_tf
   false dans le xacro, /odom rerouté en /sim/odom dans le bridge).

Note DTR/RTS : on reproduit exactement le comportement du script
interactif validé (setDTR(False), setRTS(False) après ouverture).
L'historique du projet mentionne que l'ACTIVATION de ces signaux était
nécessaire (navbot8) — à revalider sur le vrai robot lors du passage
au hardware. En TCP, la question ne se pose pas.
"""

import rclpy
from rclpy.node import Node

# Messages ROS2
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TF2
import tf2_ros

def yaw_to_quaternion(yaw: float):
    """Quaternion (x,y,z,w) pour une rotation plane autour de Z.
    Remplace tf_transformations.quaternion_from_euler(0, 0, yaw) :
    seule conversion dont on a besoin, evite la dependance externe."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

import serial
import socket
import threading
import time
import math
from typing import Optional
from enum import Enum

# =============================================================================
# Protocole SeekurOS (repris du code validé)
# =============================================================================

HDR0, HDR1 = 0xFA, 0xFB


class ArgType(Enum):
    NONE = None
    INT_POS = 0x3B
    INT_SIGNED = 0x1B
    STRING = 0x2B


def aria_checksum(data_bytes: bytes) -> int:
    """Calcul du checksum SeekurOS/ARIA"""
    c = 0
    i = 0
    n = len(data_bytes)
    while n > 1:
        c = (c + (((data_bytes[i] << 8) | data_bytes[i + 1]) & 0xFFFF)) & 0xFFFF
        i += 2
        n -= 2
    if n > 0:
        c ^= data_bytes[i]
    return c & 0xFFFF


def build_cmd(cmd: int, arg_type: Optional[int] = None, arg_val: Optional[int] = None) -> bytes:
    """Construit une trame complète SeekurOS"""
    body = bytearray([cmd & 0xFF])

    if arg_type is not None and arg_val is not None:
        if not (0 <= arg_val <= 0xFFFF):
            if arg_val < 0:
                arg_val = arg_val & 0xFFFF
        lo = arg_val & 0xFF
        hi = (arg_val >> 8) & 0xFF
        body += bytes([arg_type & 0xFF, lo, hi])

    count = len(body) + 2
    chk = aria_checksum(body)
    frame = bytearray([HDR0, HDR1, count & 0xFF]) + body + bytes([(chk >> 8) & 0xFF, chk & 0xFF])
    return bytes(frame)


# =============================================================================
# Adaptateur TCP (porté depuis seekur_interactive1_tcp.py)
# =============================================================================

class SerialTCPAdapter:
    """Adaptateur pour utiliser un socket TCP comme un port série.

    Expose la même interface minimale que serial.Serial (write/read/
    flush/reset_input_buffer/close/is_open) pour que le reste du driver
    soit strictement identique en mode simulateur et en mode robot réel.
    """

    def __init__(self, host, port, timeout=1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.is_open = False

    def open(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(self.timeout)
        self.socket.connect((self.host, self.port))
        self.is_open = True

    def close(self):
        if self.socket:
            self.socket.close()
        self.is_open = False

    def write(self, data):
        if self.socket:
            self.socket.send(data)

    def read(self, size=1):
        if self.socket:
            try:
                return self.socket.recv(size)
            except socket.timeout:
                return b''
        return b''

    def flush(self):
        pass  # TCP n'a pas de flush explicite

    def reset_input_buffer(self):
        # Vider le buffer en lisant tout ce qui traîne
        try:
            while True:
                data = self.socket.recv(1024)
                if not data:
                    break
        except socket.timeout:
            pass


# =============================================================================
# Node driver
# =============================================================================

class SeekurDriverNode(Node):
    def __init__(self):
        super().__init__('seekur_driver')

        # Paramètres ROS2
        self.declare_parameters(
            namespace='',
            parameters=[
                # 'tcp://localhost:9999' = simulateur | '/dev/ttyUSB0' = vrai robot
                ('serial_port', 'tcp://localhost:9999'),
                ('baud_rate', 9600),
                ('timeout', 0.5),
                ('base_frame', 'base_footprint'),   # cf. note d'en-tête (2)
                ('odom_frame', 'odom'),
                ('max_linear_vel', 0.5),    # m/s  - prudent pour debut nav2
                ('max_angular_vel', 0.7),   # rad/s
                ('wheel_separation', 0.68), # m (SeekurJR)
                ('publish_tf', True),
                ('odom_freq', 20.0),        # Hz (SIP arrivent a 10 Hz)
                ('battery_freq', 1.0),      # Hz
                ('diagnostics_freq', 1.0),  # Hz
            ]
        )

        # Récupération des paramètres
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Communication série / TCP
        self.ser = None                     # serial.Serial OU SerialTCPAdapter
        self.connected = False
        self.initialized = False
        self.watchdog_active = False
        self.monitor_active = False

        # Verrou d'accès au port (cf. note d'en-tête (3))
        self._io_lock = threading.Lock()

        # Threads
        self.watchdog_thread: Optional[threading.Thread] = None
        self.monitor_thread: Optional[threading.Thread] = None
        self.last_pulse = 0.0

        # État du robot (mis à jour par les SIP)
        self.robot_x = 0.0        # m
        self.robot_y = 0.0        # m
        self.robot_theta = 0.0    # rad
        self.robot_vx = 0.0       # m/s
        self.robot_vtheta = 0.0   # rad/s
        self.battery_voltage = 0.0
        self.battery_level = 0

        # Publishers — ce node est le PROPRIÉTAIRE de /odom (cf. note (4))
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 1)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 1)

        # TF2 Broadcaster — propriétaire de odom -> base_footprint
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber — /cmd_vel vient de nav2 (ou d'un teleop)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # Timers
        odom_freq = self.get_parameter('odom_freq').value
        battery_freq = self.get_parameter('battery_freq').value
        diagnostics_freq = self.get_parameter('diagnostics_freq').value

        self.odom_timer = self.create_timer(1.0 / odom_freq, self.publish_odometry)
        self.battery_timer = self.create_timer(1.0 / battery_freq, self.publish_battery)
        self.diagnostics_timer = self.create_timer(1.0 / diagnostics_freq, self.publish_diagnostics)

        # Initialisation
        self.get_logger().info(f'SeekurJR Driver démarré - Port: {self.serial_port}')
        self.connect_robot()

    # -------------------------------------------------------------------------
    # Connexion et initialisation
    # -------------------------------------------------------------------------

    def connect_robot(self):
        """Établit la connexion avec le robot (série OU TCP simulateur)"""
        try:
            if self.serial_port.startswith('tcp://'):
                # --- Mode simulateur : tcp://host:port ---
                url = self.serial_port.replace('tcp://', '')
                host, port = url.split(':')
                self.get_logger().info(f'Connexion TCP au simulateur {host}:{port}...')
                self.ser = SerialTCPAdapter(host, int(port), self.timeout)
                self.ser.open()
            else:
                # --- Mode robot réel : port série ---
                self.get_logger().info(f'Connexion série {self.serial_port}...')
                self.ser = serial.Serial(
                    self.serial_port, self.baud_rate, bytesize=8,
                    parity=serial.PARITY_NONE, stopbits=1,
                    timeout=self.timeout, xonxoff=False, rtscts=False
                )
                # Comportement identique au script interactif validé.
                # NOTE : à revalider sur le vrai robot (cf. note d'en-tête).
                self.ser.setDTR(False)
                self.ser.setRTS(False)

            self.ser.reset_input_buffer()
            self.connected = True

            if self.initialize_robot():
                self.get_logger().info('Robot initialisé avec succès')
                self.start_watchdog()
                self.start_monitoring()
            else:
                self.get_logger().error("Échec de l'initialisation du robot")

        except Exception as e:
            self.get_logger().error(f'Erreur de connexion: {e}')
            self.connected = False

    def initialize_robot(self) -> bool:
        """Séquence d'initialisation : SYNC0/1/2 -> OPEN -> ENABLE"""
        if not self.ser or not self.connected:
            return False

        try:
            with self._io_lock:
                self.ser.reset_input_buffer()
            time.sleep(0.1)

            # Séquence SYNC0, SYNC1, SYNC2 avec vérification des échos
            for cmd_id in [0, 1, 2]:
                frame = build_cmd(cmd_id)
                with self._io_lock:
                    self.ser.write(frame)
                    self.ser.flush()
                time.sleep(self.timeout)
                with self._io_lock:
                    rx = self.ser.read(4096)
                if rx != frame:
                    self.get_logger().warn(f'Écho SYNC{cmd_id} différent (reçu {len(rx)} octets)')

            time.sleep(0.5)

            # OPEN - Démarrage des serveurs
            frame = build_cmd(1)
            with self._io_lock:
                self.ser.write(frame)
                self.ser.flush()
            time.sleep(self.timeout)
            with self._io_lock:
                self.ser.read(4096)

            time.sleep(0.5)

            # ENABLE - Activation des moteurs
            frame = build_cmd(4, 0x3B, 1)
            with self._io_lock:
                self.ser.write(frame)
                self.ser.flush()
            time.sleep(self.timeout)
            with self._io_lock:
                self.ser.read(4096)

            self.initialized = True
            return True

        except Exception as e:
            self.get_logger().error(f'Erreur initialisation: {e}')
            return False

    # -------------------------------------------------------------------------
    # Commandes de mouvement (nav2 -> protocole SeekurOS)
    # -------------------------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        """Traduit /cmd_vel (Twist) en trames VEL + RVEL SeekurOS"""
        if not self.connected or not self.initialized:
            return

        # Saturation aux limites configurées
        linear_vel_ms = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel_rads = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # Conversion vers unités SeekurOS
        linear_vel_mms = int(linear_vel_ms * 1000)                    # mm/s
        angular_vel_degs = int(angular_vel_rads * 180.0 / math.pi)    # deg/s

        self.send_velocity_command(linear_vel_mms)
        self.send_rotation_command(angular_vel_degs)

    def send_velocity_command(self, vel_mms: int):
        """Trame VEL avec correction bidirectionnelle firmware.

        Bug firmware SeekurJR : la direction s'encode par le TYPE
        d'argument, pas par le signe de la valeur.
          avant   : type 0x3B (INT_POS)    + valeur positive
          arrière : type 0x1B (INT_SIGNED) + valeur ABSOLUE
        """
        if not self.ser:
            return
        try:
            if vel_mms >= 0:
                frame = build_cmd(11, 0x3B, vel_mms)
            else:
                frame = build_cmd(11, 0x1B, abs(vel_mms))
            with self._io_lock:
                self.ser.write(frame)
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'Erreur envoi VEL: {e}')

    def send_rotation_command(self, rvel_degs: int):
        """Trame RVEL avec correction bidirectionnelle firmware.
          CCW : type 0x3B + valeur positive
          CW  : type 0x1B + valeur ABSOLUE
        """
        if not self.ser:
            return
        try:
            if rvel_degs >= 0:
                frame = build_cmd(21, 0x3B, rvel_degs)
            else:
                frame = build_cmd(21, 0x1B, abs(rvel_degs))
            with self._io_lock:
                self.ser.write(frame)
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'Erreur envoi RVEL: {e}')

    # -------------------------------------------------------------------------
    # Watchdog (PULSE toutes les 1.5 s)
    # -------------------------------------------------------------------------

    def start_watchdog(self):
        if self.watchdog_active:
            return
        self.watchdog_active = True
        self.last_pulse = time.time()
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        self.get_logger().info('Watchdog démarré')

    def _watchdog_loop(self):
        while self.watchdog_active and self.connected:
            try:
                current_time = time.time()
                if current_time - self.last_pulse >= 1.5:
                    if self.initialized and self.ser:
                        frame = build_cmd(0)  # PULSE
                        with self._io_lock:
                            self.ser.write(frame)
                            self.ser.flush()
                        self.last_pulse = current_time
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Erreur watchdog: {e}')
                break

    # -------------------------------------------------------------------------
    # Monitoring des SIP (réception odométrie/batterie)
    # -------------------------------------------------------------------------

    def start_monitoring(self):
        if self.monitor_active:
            return
        self.monitor_active = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.get_logger().info('Monitoring SIP démarré')

    def _monitor_loop(self):
        buf = bytearray()
        while self.monitor_active and self.ser and self.ser.is_open:
            try:
                with self._io_lock:
                    chunk = self.ser.read(1024)
                if chunk:
                    buf += chunk
                    self._parse_sip_packets(buf)
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Erreur monitoring: {e}')
                break

    def _parse_sip_packets(self, buf: bytearray):
        while True:
            i = buf.find(b'\xFA\xFB')
            if i < 0:
                if len(buf) > 8192:
                    del buf[:-2]
                break

            if i > 0:
                del buf[:i]

            if len(buf) < 3:
                break

            count = buf[2]
            frame_len = 3 + count

            if len(buf) < frame_len:
                break

            frame = bytes(buf[:frame_len])
            del buf[:frame_len]

            # Vérification checksum
            body = frame[3:-2]
            chk = (frame[-2] << 8) | frame[-1]
            comp = aria_checksum(body)

            if chk == comp:
                self._decode_sip_packet(frame)

    def _decode_sip_packet(self, frame: bytes):
        """Décode les SIP standards et met à jour l'état du robot"""
        if len(frame) < 4:
            return

        packet_type = frame[3]

        if packet_type in [0x32, 0x33] and len(frame) >= 20:
            xpos = int.from_bytes(frame[4:6], 'little', signed=True)    # mm
            ypos = int.from_bytes(frame[6:8], 'little', signed=True)    # mm
            thpos = int.from_bytes(frame[8:10], 'little', signed=True)  # unités
            lvel = int.from_bytes(frame[10:12], 'little', signed=True)  # mm/s
            rvel = int.from_bytes(frame[12:14], 'little', signed=True)  # mm/s
            battery = frame[14]

            # Conversion vers unités SI
            self.robot_x = xpos / 1000.0
            self.robot_y = ypos / 1000.0
            self.robot_theta = thpos * 0.001534   # AngleConvFactor (rad/unité)

            linear_vel_mms = (lvel + rvel) / 2.0
            self.robot_vx = linear_vel_mms / 1000.0

            if self.wheel_separation > 0:
                self.robot_vtheta = (rvel - lvel) / (self.wheel_separation * 1000.0)

            self.battery_voltage = battery * 0.2
            self.battery_level = battery

    # -------------------------------------------------------------------------
    # Publication ROS2 (odom, TF, batterie, diagnostics)
    # -------------------------------------------------------------------------

    def publish_odometry(self):
        if not self.initialized:
            return

        current_time = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0

        quat = yaw_to_quaternion(self.robot_theta)       
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = self.robot_vx
        odom.twist.twist.angular.z = self.robot_vtheta

        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.2

        self.odom_pub.publish(odom)

        # TF odom -> base_footprint (PROPRIÉTAIRE UNIQUE : ce node)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.robot_x
            tf_msg.transform.translation.y = self.robot_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_battery(self):
        if not self.initialized:
            return
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = self.battery_voltage
        battery_msg.percentage = (self.battery_level / 255.0) * 100.0
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery_msg.present = True
        self.battery_pub.publish(battery_msg)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'seekur_connection'
        status.hardware_id = 'SeekurJR'

        if self.connected and self.initialized:
            status.level = DiagnosticStatus.OK
            status.message = 'Robot connecté et opérationnel'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Robot déconnecté ou non initialisé'

        status.values = [
            KeyValue(key='port', value=self.serial_port),
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='initialized', value=str(self.initialized)),
            KeyValue(key='battery_voltage', value=f'{self.battery_voltage:.1f}V'),
        ]

        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

    # -------------------------------------------------------------------------
    # Arrêt propre
    # -------------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Arrêt du driver SeekurJR')

        self.watchdog_active = False
        self.monitor_active = False

        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=2)
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)

        if self.ser and self.ser.is_open:
            try:
                # STOP puis CLOSE avant de couper
                with self._io_lock:
                    self.ser.write(build_cmd(29))   # STOP
                    self.ser.flush()
                    time.sleep(0.1)
                    self.ser.write(build_cmd(2))    # CLOSE
                    self.ser.flush()
                    time.sleep(0.1)
            except Exception:
                pass
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SeekurDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
