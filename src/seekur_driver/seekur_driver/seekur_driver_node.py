#!/usr/bin/env python3
"""
seekur_driver_node.py - Node ROS2 pour robot SeekurJR
Basé sur le protocole série SeekurOS avec corrections firmware bidirectionnelles
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Messages ROS2
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage

# TF2
import tf2_ros
from tf_transformations import quaternion_from_euler

import serial
import threading
import time
import math
from typing import Optional
from enum import Enum

# Configuration protocole SeekurOS (réutilisé du code fonctionnel)
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

class SeekurDriverNode(Node):
    def __init__(self):
        super().__init__('seekur_driver')
        
        # Paramètres ROS2
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 9600),
                ('timeout', 0.5),
                ('base_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('max_linear_vel', 2.0),  # m/s
                ('max_angular_vel', 3.14),  # rad/s
                ('wheel_separation', 0.33),  # m (SeekurJR)
                ('publish_tf', True),
                ('odom_freq', 10.0),  # Hz
                ('battery_freq', 1.0),  # Hz
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
        
        # Communication série
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.initialized = False
        self.watchdog_active = False
        self.monitor_active = False
        
        # Threads
        self.watchdog_thread: Optional[threading.Thread] = None
        self.monitor_thread: Optional[threading.Thread] = None
        self.last_pulse = 0.0
        
        # État du robot
        self.robot_x = 0.0  # m
        self.robot_y = 0.0  # m
        self.robot_theta = 0.0  # rad
        self.robot_vx = 0.0  # m/s
        self.robot_vtheta = 0.0  # rad/s
        self.battery_voltage = 0.0  # V
        self.battery_level = 0
        self.last_odom_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 1)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 1)
        
        # TF2 Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber
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
    
    def connect_robot(self):
        """Établit la connexion avec le robot"""
        try:
            self.get_logger().info('Connexion au robot SeekurJR...')
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate, bytesize=8, 
                parity=serial.PARITY_NONE, stopbits=1, 
                timeout=self.timeout, xonxoff=False, rtscts=False
            )
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.ser.reset_input_buffer()
            self.connected = True
            
            if self.initialize_robot():
                self.get_logger().info('Robot initialisé avec succès')
                self.start_watchdog()
                self.start_monitoring()
            else:
                self.get_logger().error('Échec de l\'initialisation du robot')
                
        except Exception as e:
            self.get_logger().error(f'Erreur de connexion: {e}')
            self.connected = False
    
    def initialize_robot(self) -> bool:
        """Séquence d'initialisation du robot"""
        if not self.ser or not self.connected:
            return False
        
        try:
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            
            # Séquence SYNC0, SYNC1, SYNC2
            for cmd_id in [0, 1, 2]:
                frame = build_cmd(cmd_id)
                self.ser.write(frame)
                self.ser.flush()
                time.sleep(self.timeout)
                
                # Lire l'écho
                rx = self.ser.read(4096)
                if rx != frame:
                    self.get_logger().warn(f'Écho SYNC{cmd_id} différent')
            
            time.sleep(0.5)
            
            # OPEN - Ouverture des serveurs
            frame = build_cmd(1)
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(self.timeout)
            self.ser.read(4096)  # Lire réponse
            
            time.sleep(0.5)
            
            # ENABLE - Activation des moteurs
            frame = build_cmd(4, 0x3B, 1)
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(self.timeout)
            self.ser.read(4096)  # Lire réponse
            
            self.initialized = True
            return True
            
        except Exception as e:
            self.get_logger().error(f'Erreur initialisation: {e}')
            return False
    
    def cmd_vel_callback(self, msg: Twist):
        """Callback pour les commandes de vitesse"""
        if not self.connected or not self.initialized:
            return
        
        # Conversion des vitesses ROS (m/s, rad/s) vers SeekurOS (mm/s, deg/s)
        linear_vel_ms = msg.linear.x  # m/s
        angular_vel_rads = msg.angular.z  # rad/s
        
        # Limiter les vitesses
        linear_vel_ms = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel_ms))
        angular_vel_rads = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel_rads))
        
        # Conversion vers unités SeekurOS
        linear_vel_mms = int(linear_vel_ms * 1000)  # mm/s
        angular_vel_degs = int(angular_vel_rads * 180.0 / math.pi)  # deg/s
        
        # Envoi des commandes avec correction bidirectionnelle
        self.send_velocity_command(linear_vel_mms)
        self.send_rotation_command(angular_vel_degs)
    
    def send_velocity_command(self, vel_mms: int):
        """Envoie commande VEL avec correction bidirectionnelle"""
        if not self.ser:
            return
        
        try:
            if vel_mms >= 0:
                # Avant: type 3B + valeur positive
                frame = build_cmd(11, 0x3B, vel_mms)
            else:
                # Arrière: type 1B + valeur absolue
                frame = build_cmd(11, 0x1B, abs(vel_mms))
            
            self.ser.write(frame)
            self.ser.flush()
            
        except Exception as e:
            self.get_logger().error(f'Erreur envoi VEL: {e}')
    
    def send_rotation_command(self, rvel_degs: int):
        """Envoie commande RVEL avec correction bidirectionnelle"""
        if not self.ser:
            return
        
        try:
            if rvel_degs >= 0:
                # CCW: type 3B + valeur positive
                frame = build_cmd(21, 0x3B, rvel_degs)
            else:
                # CW: type 1B + valeur absolue
                frame = build_cmd(21, 0x1B, abs(rvel_degs))
            
            self.ser.write(frame)
            self.ser.flush()
            
        except Exception as e:
            self.get_logger().error(f'Erreur envoi RVEL: {e}')
    
    def start_watchdog(self):
        """Démarre le watchdog automatique"""
        if self.watchdog_active:
            return
        
        self.watchdog_active = True
        self.last_pulse = time.time()
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        self.get_logger().info('Watchdog démarré')
    
    def _watchdog_loop(self):
        """Boucle du watchdog - PULSE toutes les 1.5s"""
        while self.watchdog_active and self.connected:
            try:
                current_time = time.time()
                if current_time - self.last_pulse >= 1.5:
                    if self.initialized and self.ser:
                        frame = build_cmd(0)  # PULSE
                        self.ser.write(frame)
                        self.ser.flush()
                        self.last_pulse = current_time
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Erreur watchdog: {e}')
                break
    
    def start_monitoring(self):
        """Démarre le monitoring des SIP"""
        if self.monitor_active:
            return
        
        self.monitor_active = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.get_logger().info('Monitoring SIP démarré')
    
    def _monitor_loop(self):
        """Boucle de monitoring des paquets SIP"""
        buf = bytearray()
        
        while self.monitor_active and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(1024)
                if chunk:
                    buf += chunk
                    self._parse_sip_packets(buf)
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Erreur monitoring: {e}')
                break
    
    def _parse_sip_packets(self, buf: bytearray):
        """Parse les paquets SIP"""
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
        """Décode les paquets SIP et met à jour l'état du robot"""
        if len(frame) < 4:
            return
        
        packet_type = frame[3]
        
        if packet_type in [0x32, 0x33] and len(frame) >= 20:
            # SIP Standard - extraction des données
            xpos = int.from_bytes(frame[4:6], 'little', signed=True)  # mm
            ypos = int.from_bytes(frame[6:8], 'little', signed=True)  # mm
            thpos = int.from_bytes(frame[8:10], 'little', signed=True)  # unités
            lvel = int.from_bytes(frame[10:12], 'little', signed=True)  # mm/s
            rvel = int.from_bytes(frame[12:14], 'little', signed=True)  # mm/s
            battery = frame[14]
            
            # Conversion vers unités SI
            self.robot_x = xpos / 1000.0  # m
            self.robot_y = ypos / 1000.0  # m
            self.robot_theta = thpos * 0.001534  # rad
            
            # Calcul des vitesses
            linear_vel_mms = (lvel + rvel) / 2.0  # mm/s
            self.robot_vx = linear_vel_mms / 1000.0  # m/s
            
            # Vitesse angulaire (approximation différentielle)
            if self.wheel_separation > 0:
                angular_vel_rads = (rvel - lvel) / (self.wheel_separation * 1000.0)  # rad/s
                self.robot_vtheta = angular_vel_rads
            
            # Batterie
            self.battery_voltage = battery * 0.2  # V
            self.battery_level = battery
    
    def publish_odometry(self):
        """Publie l'odométrie"""
        if not self.initialized:
            return
        
        current_time = self.get_clock().now()
        
        # Message Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion depuis angle)
        quat = quaternion_from_euler(0, 0, self.robot_theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Vitesses
        odom.twist.twist.linear.x = self.robot_vx
        odom.twist.twist.angular.z = self.robot_vtheta
        
        # Covariances (estimées)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.2  # theta
        odom.twist.covariance[0] = 0.1  # vx
        odom.twist.covariance[35] = 0.2 # vtheta
        
        self.odom_pub.publish(odom)
        
        # Transform odom -> base_link
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
        """Publie l'état de la batterie"""
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
        """Publie les diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Diagnostic de connexion
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
            KeyValue(key='baud', value=str(self.baud_rate)),
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='initialized', value=str(self.initialized)),
            KeyValue(key='battery_voltage', value=f'{self.battery_voltage:.1f}V'),
        ]
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)
    
    def destroy_node(self):
        """Nettoyage lors de l'arrêt"""
        self.get_logger().info('Arrêt du driver SeekurJR')
        
        # Arrêter les threads
        self.watchdog_active = False
        self.monitor_active = False
        
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=2)
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
        
        # Fermer la connexion série
        if self.ser and self.ser.is_open:
            try:
                # Envoyer CLOSE
                frame = build_cmd(2)
                self.ser.write(frame)
                self.ser.flush()
                time.sleep(0.1)
            except:
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