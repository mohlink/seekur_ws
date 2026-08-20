#!/usr/bin/env python3
"""
seekur_protocol_simulator.py - Simulateur du protocole série SeekurOS
Émule un robot SeekurJR pour tester les drivers sans hardware
"""

import socket
import threading
import time
import struct
import math
from typing import Optional, Dict, List
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Configuration protocole SeekurOS (repris de votre code)
HDR0, HDR1 = 0xFA, 0xFB

def aria_checksum(data_bytes: bytes) -> int:
    """Calcul du checksum SeekurOS/ARIA - FONCTION MANQUANTE"""
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

class SeekurState(Enum):
    DISCONNECTED = 0
    WAITING_SYNC = 1
    SYNC0_RECEIVED = 2
    SYNC1_RECEIVED = 3
    SYNC2_RECEIVED = 4
    OPENED = 5
    RUNNING = 6
    
    def __ge__(self, other):
        return self.value >= other.value
    
    def __le__(self, other):
        return self.value <= other.value

    def __lt__(self, other):
        return self.value < other.value

    def __gt__(self, other):
        return self.value > other.value
    
def build_response(cmd: int, arg_type: Optional[int] = None, arg_val: Optional[int] = None) -> bytes:
    """Construit une réponse SeekurOS"""
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

def build_sip_packet(x_mm: int, y_mm: int, theta_units: int, 
                    lvel_mms: int, rvel_mms: int, battery: int) -> bytes:
    """Construit un paquet SIP standard"""
    body = bytearray([0x32])  # Type SIP standard
    
    # Position (little endian, signed)
    body += struct.pack('<h', x_mm)      # X position (mm)
    body += struct.pack('<h', y_mm)      # Y position (mm)  
    body += struct.pack('<h', theta_units) # Orientation (unités)
    body += struct.pack('<h', lvel_mms)   # Vitesse roue gauche (mm/s)
    body += struct.pack('<h', rvel_mms)   # Vitesse roue droite (mm/s)
    body += bytes([battery])              # Niveau batterie
    
    # Padding pour compléter le SIP (format réel)
    body += bytes([0] * 20)  # Autres champs SIP
    
    count = len(body) + 2
    chk = aria_checksum(body)
    frame = bytearray([HDR0, HDR1, count & 0xFF]) + body + bytes([(chk >> 8) & 0xFF, chk & 0xFF])
    return bytes(frame)

class SeekurProtocolSimulator(Node):
    """Simulateur du protocole série SeekurOS avec interface ROS2"""
    
    def __init__(self):
        super().__init__('seekur_protocol_simulator')
        
        # État du simulateur
        self.state = SeekurState.DISCONNECTED
        self.motors_enabled = False
        self.current_vel = 0      # mm/s
        self.current_rvel = 0     # deg/s
        self.last_pulse = time.time()
        
        # Position simulée (comme dans vos SIP réels)
        self.x_mm = 0
        self.y_mm = 0
        self.theta_units = 0
        self.battery_level = 175  # Comme observé (35V)
        
        # Interface ROS2 avec Gazebo — topics PRIVES de simulation (/sim/*)
        self.cmd_vel_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/sim/odom', self.odom_callback, 10)
        
        # Timer pour publication continue des commandes
        self.cmd_vel_timer = self.create_timer(0.05, self.publish_cmd_vel_continuous)  # 20Hz
        
        # Variables pour conversion
        self.wheel_separation = 0.33  # m
        self.last_odom_time = time.time()
        
        # Socket TCP pour émulation port série
        self.server_socket = None
        self.client_socket = None
        self.server_thread = None
        self.sip_thread = None
        self.running = False
        
        # Démarrer le serveur TCP
        self.start_tcp_server()
        
        self.get_logger().info("Simulateur SeekurOS démarré sur port 9999")
        self.get_logger().info("Connectez votre driver avec: --port tcp://localhost:9999")
    
    def start_tcp_server(self):
        """Démarre le serveur TCP qui émule le port série"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.listen(1)
        
        self.running = True
        self.server_thread = threading.Thread(target=self.accept_connections, daemon=True)
        self.server_thread.start()
    
    def accept_connections(self):
        """Accepte les connexions client"""
        while self.running:
            try:
                self.get_logger().info("En attente de connexion...")
                self.client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Client connecté: {addr}")
                
                self.state = SeekurState.WAITING_SYNC
                self.handle_client()
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Erreur serveur: {e}")
    
    def handle_client(self):
        """Gère la communication avec un client"""
        try:
            # Démarrer l'envoi des SIP
            self.sip_thread = threading.Thread(target=self.sip_sender, daemon=True)
            self.sip_thread.start()
            
            while self.running and self.client_socket:
                data = self.client_socket.recv(1024)
                if not data:
                    break
                
                self.process_received_data(data)
                
        except Exception as e:
            self.get_logger().error(f"Erreur client: {e}")
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
            self.state = SeekurState.DISCONNECTED
            self.get_logger().info("Client déconnecté")
    
    def process_received_data(self, data: bytes):
        """Traite les données reçues du client"""
        for i, byte in enumerate(data):
            if i + 5 < len(data) and data[i] == HDR0 and data[i+1] == HDR1:
                # Extraire la commande
                count = data[i+2]
                if i + 3 + count <= len(data):
                    frame = data[i:i+3+count]
                    self.process_command(frame)
    
    def process_command(self, frame: bytes):
        """Traite une commande SeekurOS"""
        if len(frame) < 6:
            return
        
        cmd_id = frame[3]
        
        # Debug: afficher le frame reçu
        self.get_logger().info(f"Frame reçu: {[hex(b) for b in frame]}")
        
        # Vérifier le checksum
        body = frame[3:-2]
        received_chk = (frame[-2] << 8) | frame[-1]
        computed_chk = aria_checksum(body)
        
        if received_chk != computed_chk:
            self.get_logger().warn(f"Checksum invalide pour commande {cmd_id}")
            return
        
        response = None
        
        if cmd_id == 0 and self.state == SeekurState.WAITING_SYNC:  # SYNC0
            self.state = SeekurState.SYNC0_RECEIVED
            response = frame  # Écho
            self.get_logger().info("SYNC0 reçu")
            
        elif cmd_id == 1 and self.state == SeekurState.SYNC0_RECEIVED:  # SYNC1
            self.state = SeekurState.SYNC1_RECEIVED
            response = frame  # Écho
            self.get_logger().info("SYNC1 reçu")
            
        elif cmd_id == 2 and self.state == SeekurState.SYNC1_RECEIVED:  # SYNC2
            self.state = SeekurState.SYNC2_RECEIVED
            response = frame  # Écho
            self.get_logger().info("SYNC2 reçu")
            
        elif cmd_id == 1 and self.state >= SeekurState.SYNC2_RECEIVED:  # OPEN
            self.state = SeekurState.OPENED
            response = build_response(1)
            self.get_logger().info("OPEN reçu")
            
        elif cmd_id == 0 and self.state >= SeekurState.OPENED:  # PULSE (après SYNC)
            self.last_pulse = time.time()
            # Pas de réponse pour PULSE
            
        elif cmd_id == 4:  # ENABLE
            self.get_logger().info(f"Frame ENABLE complet: {[hex(b) for b in frame]}")
            if len(frame) >= 7:  # Minimum pour ENABLE avec argument
                # CORRECTION CRITIQUE: L'argument commence à frame[5], pas frame[6] !
                # Structure: [FA FB COUNT CMD TYPE ARG_LO ARG_HI CHKSUM_HI CHKSUM_LO]
                #           [0  1  2     3   4    5      6      7         8        ]
                
                arg_val = frame[5] | (frame[6] << 8)  # Argument à position 5-6
                self.get_logger().info(f"Argument parsé: {arg_val} (bytes: {hex(frame[5])}, {hex(frame[6])})")
                
                self.motors_enabled = (arg_val == 1)
                self.state = SeekurState.RUNNING if self.motors_enabled else SeekurState.OPENED
                response = build_response(4)
                self.get_logger().info(f"ENABLE {arg_val} - Moteurs: {'ON' if self.motors_enabled else 'OFF'}")
            else:
                self.get_logger().warn(f"Frame ENABLE trop court: {len(frame)} bytes")
        
        elif cmd_id == 11:  # VEL
            if len(frame) >= 7 and self.motors_enabled:
                arg_type = frame[4]
                # CORRECTION: argument à position 5-6
                arg_val = frame[5] | (frame[6] << 8)
                
                # Gestion des valeurs signées (conversion depuis unsigned vers signed)
                if arg_val > 32767:
                    arg_val = arg_val - 65536
                
                # Appliquer correction bidirectionnelle
                if arg_type == 0x3B:  # Avant
                    self.current_vel = arg_val
                elif arg_type == 0x1B:  # Arrière
                    self.current_vel = -arg_val
                
                self.send_cmd_vel_to_gazebo()
                self.get_logger().info(f"VEL: {self.current_vel} mm/s (type: 0x{arg_type:02X})")
        
        elif cmd_id == 21:  # RVEL
            if len(frame) >= 7 and self.motors_enabled:
                arg_type = frame[4]
                # CORRECTION: argument à position 5-6
                arg_val = frame[5] | (frame[6] << 8)
                
                # Gestion des valeurs signées
                if arg_val > 32767:
                    arg_val = arg_val - 65536
                
                # Appliquer correction bidirectionnelle
                if arg_type == 0x3B:  # CCW
                    self.current_rvel = arg_val
                elif arg_type == 0x1B:  # CW
                    self.current_rvel = -arg_val
                
                self.send_cmd_vel_to_gazebo()
                self.get_logger().info(f"RVEL: {self.current_rvel} deg/s (type: 0x{arg_type:02X})")
        
        elif cmd_id == 29:  # STOP
            self.current_vel = 0
            self.current_rvel = 0
            # Publication immédiate pour arrêt rapide
            twist = Twist()  # Toutes les vitesses à 0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("STOP reçu - Robot arrêté")
        
        elif cmd_id == 2:  # CLOSE
            self.state = SeekurState.DISCONNECTED
            self.motors_enabled = False
            response = build_response(2)
            self.get_logger().info("CLOSE reçu")
        
        # Envoyer la réponse si nécessaire
        if response and self.client_socket:
            try:
                self.client_socket.send(response)
            except Exception as e:
                self.get_logger().error(f"Erreur envoi réponse: {e}")
    
    def publish_cmd_vel_continuous(self):
        """Publie continuellement la dernière commande de vitesse"""
        if self.motors_enabled and self.state >= SeekurState.RUNNING:
            twist = Twist()
            twist.linear.x = self.current_vel / 1000.0  # mm/s -> m/s
            twist.angular.z = math.radians(self.current_rvel)  # deg/s -> rad/s
            self.cmd_vel_pub.publish(twist)
    
    def send_cmd_vel_to_gazebo(self):
        """Mise à jour des commandes (appelée lors de nouveaux ordres)"""
        # La publication continue est gérée par le timer
        self.get_logger().debug(f"Commandes mises à jour: linear={self.current_vel} mm/s, angular={self.current_rvel} deg/s")
    
    def odom_callback(self, msg: Odometry):
        """Récupère l'odométrie de Gazebo pour les SIP"""
        # Convertir en format SeekurOS
        self.x_mm = int(msg.pose.pose.position.x * 1000)
        self.y_mm = int(msg.pose.pose.position.y * 1000)
        
        # Convertir quaternion en angle SeekurOS
        quat = msg.pose.pose.orientation
        yaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y), 
                        1 - 2*(quat.y*quat.y + quat.z*quat.z))
        self.theta_units = int(yaw / 0.001534)
    
    def scan_callback(self, msg: LaserScan):
        """Traite les données LiDAR si nécessaire"""
        pass
    
    def sip_sender(self):
        """Envoie les paquets SIP toutes les 100ms. (cf. correctif N2)"""
        sip_started = False
        while self.running and self.client_socket:
            try:
                if self.state < SeekurState.OPENED:
                    time.sleep(0.05)
                    continue

                if not sip_started:
                    self.get_logger().info("Flux SIP démarré (100ms)")
                    sip_started = True

                linear_ms = self.current_vel / 1000.0
                angular_rads = math.radians(self.current_rvel)

                v_left = linear_ms - (angular_rads * self.wheel_separation / 2.0)
                v_right = linear_ms + (angular_rads * self.wheel_separation / 2.0)

                lvel_mms = int(v_left * 1000)
                rvel_mms = int(v_right * 1000)

                sip = build_sip_packet(self.x_mm, self.y_mm, self.theta_units,
                                     lvel_mms, rvel_mms, self.battery_level)

                self.client_socket.send(sip)
                time.sleep(0.1)

            except Exception as e:
                # JAMAIS de break muet : c'est ce qui a masqué deux bugs de suite.
                self.get_logger().error(f"sip_sender arrêté: {e}")
                break
    
    def destroy_node(self):
        """Nettoyage à l'arrêt"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        if self.client_socket:
            self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        simulator = SeekurProtocolSimulator()
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        if 'simulator' in locals():
            simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()