#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
seekur_interactive.py — Contrôleur interactif pour SeekurJR/SeekurOS
Interface interactive pour envoyer des commandes de mouvement et contrôler le robot.
Basé sur le protocole de communication série SeekurOS.
VERSION CORRIGÉE : Gestion bidirectionnelle VEL et RVEL avec affichage lisible
"""

import socket
import argparse
import time
import sys
import threading
import queue
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    import serial
except ImportError as e:
    print("PySerial manquant. Installez-le avec:\n  sudo apt install python3-serial", file=sys.stderr)
    raise


class SerialTCPAdapter:
    """Adaptateur pour utiliser socket TCP comme port série"""
    
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
        # Vider le buffer en lisant tout
        try:
            while True:
                data = self.socket.recv(1024)
                if not data:
                    break
        except socket.timeout:
            pass
    
    @property
    def in_waiting(self):
        # Approximation - toujours dire qu'il y a des données
        return 1


#####################################################


# Configuration protocole SeekurOS
HDR0, HDR1 = 0xFA, 0xFB

# Types d'arguments pour les commandes
class ArgType(Enum):
    NONE = None
    INT_POS = 0x3B      # Entier positif
    INT_SIGNED = 0x1B   # Entier signé
    STRING = 0x2B       # String

# Commandes SeekurOS définies
@dataclass
class Command:
    id: int
    name: str
    description: str
    arg_type: ArgType
    arg_range: Optional[Tuple[int, int]] = None

# Dictionnaire des commandes disponibles
COMMANDS = {
    # Commandes de synchronisation
    'sync0': Command(0, 'SYNC0', 'Démarrage de connexion (étape 1)', ArgType.NONE),
    'sync1': Command(1, 'SYNC1', 'Démarrage de connexion (étape 2)', ArgType.NONE),
    'sync2': Command(2, 'SYNC2', 'Démarrage de connexion (étape 3)', ArgType.NONE),
    
    # Commandes de base
    'pulse': Command(0, 'PULSE', 'Reset watchdog du serveur', ArgType.NONE),
    'open': Command(1, 'OPEN', 'Démarrer les serveurs', ArgType.NONE),
    'close': Command(2, 'CLOSE', 'Fermer serveurs et connexion', ArgType.NONE),
    'enable': Command(4, 'ENABLE', 'Activer (1) ou désactiver (0) les moteurs', ArgType.INT_POS, (0, 1)),
    
    # Commandes de mouvement - CORRIGÉES avec bidirectionnel
    'vel': Command(11, 'VEL', 'Vitesse translation (mm/s) avant(+)/arrière(-)', ArgType.INT_SIGNED, (-2000, 2000)),
    'rvel': Command(21, 'RVEL', 'Vitesse rotation (deg/s) CCW(+)/CW(-)', ArgType.INT_SIGNED, (-180, 180)),
    'head': Command(12, 'HEAD', 'Tourner vers orientation absolue (degrés)', ArgType.INT_SIGNED, (-180, 180)),
    'dhead': Command(13, 'DHEAD', 'Tourner relativement (degrés)', ArgType.INT_SIGNED, (-180, 180)),
    'rotate': Command(9, 'ROTATE', 'Rotation continue (deg/s)', ArgType.INT_SIGNED, (-180, 180)),
    
    # Commandes de configuration
    'setv': Command(6, 'SETV', 'Vitesse max translation (mm/s)', ArgType.INT_POS, (0, 2000)),
    'setrv': Command(10, 'SETRV', 'Vitesse max rotation (deg/s)', ArgType.INT_POS, (0, 180)),
    'seta': Command(5, 'SETA', 'Accélération translation (mm/s²)', ArgType.INT_SIGNED, (-1000, 1000)),
    'setra': Command(23, 'SETRA', 'Accélération rotation (deg/s²)', ArgType.INT_SIGNED, (-360, 360)),
    
    # Commandes utilitaires
    'stop': Command(29, 'STOP', 'Arrêter le robot (moteurs restent activés)', ArgType.NONE),
    'estop': Command(55, 'E_STOP', 'Arrêt d\'urgence (freinage brutal)', ArgType.NONE),
    'seto': Command(7, 'SETO', 'Reset position à origine (0,0,0)', ArgType.NONE),
    'config': Command(18, 'CONFIG', 'Demander packet de configuration', ArgType.NONE),
    
    # Commandes accessoires
    'bumpstall': Command(44, 'BUMPSTALL', 'Config bumpers: 0=off, 1=avant, 2=arrière, 3=tous', ArgType.INT_POS, (0, 3)),
    'joydrive': Command(47, 'JOYDRIVE', 'Autoriser joystick (1) ou non (0)', ArgType.INT_POS, (0, 1)),
    'lrfpower': Command(96, 'LRFPOWER', 'Alimenter LRF: 1=ON, 0=OFF', ArgType.INT_POS, (0, 1)),
}

COMMANDS.update({
    # SIP supplémentaires
    'imu': Command(26, 'IMU', 'Demander données IMU: 1=une fois, 2=continu', ArgType.INT_POS, (0, 2)),
    'joyrequest': Command(17, 'JOYREQUEST', 'Demander données joystick', ArgType.INT_POS, (0, 2)),
    
    # Orientation avancée
    'dchead': Command(22, 'DCHEAD', 'Ajuster orientation relative', ArgType.INT_SIGNED, (-180, 180)),
    'rotvelmaxdir': Command(20, 'ROTVELMAXDIR', 'Vitesse rotation directionnelle', ArgType.INT_SIGNED),
    
    # Système Seekur spécifique (peut ne pas fonctionner sur SeekurJR)
    'latvel': Command(110, 'LATVEL', 'Vitesse latérale (mm/s)', ArgType.INT_SIGNED, (-1000, 1000)),
    'lataccel': Command(113, 'LATACCEL', 'Accélération latérale', ArgType.INT_SIGNED),
    'cntrwheels': Command(120, 'CNTRWHEELS', 'Recentrer roues', ArgType.NONE),
    
    # Alimentation
    'seekurpower': Command(116, 'SEEKURPOWER', 'Contrôle ports alimentation', ArgType.INT_POS),
    'seekuroff': Command(119, 'SEEKUROFF', 'Arrêt alimentation principale', ArgType.NONE),
    'auxpcpower': Command(125, 'AUXPCPOWER', 'PC secondaire 0=OFF, 1=ON', ArgType.INT_POS, (0, 1)),
    'ptzpower': Command(127, 'PTZPOWER', 'Caméra PTZ 0=OFF, 1=ON', ArgType.INT_POS, (0, 1)),
    'lrf2power': Command(129, 'LRF2POWER', 'LRF secondaire 0=OFF, 1=ON', ArgType.INT_POS, (0, 1)),
    
    # Test/Debug
    'battest': Command(250, 'BATTEST', 'Test batterie (pourcentage)', ArgType.INT_POS, (0, 100)),
    'debug': Command(251, 'DEBUG', 'Debug packets 1=ON, 0=OFF', ArgType.INT_POS, (0, 1)),
    'reset': Command(253, 'RESET', 'Reset microcontrôleur', ArgType.NONE),
    'maintenance': Command(255, 'MAINTENANCE', 'Mode maintenance', ArgType.NONE),
})

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
            # Convertir les valeurs négatives en complément à 2 sur 16 bits
            if arg_val < 0:
                arg_val = arg_val & 0xFFFF
        lo = arg_val & 0xFF
        hi = (arg_val >> 8) & 0xFF
        body += bytes([arg_type & 0xFF, lo, hi])
    
    count = len(body) + 2
    chk = aria_checksum(body)
    frame = bytearray([HDR0, HDR1, count & 0xFF]) + body + bytes([(chk >> 8) & 0xFF, chk & 0xFF])
    return bytes(frame)

def format_readable_frame(label: str, frame: bytes, cmd_name: str = "", arg_val: Optional[int] = None, direction: str = ""):
    """Affichage lisible des trames avec interprétation"""
    if not frame:
        print(f"{label}: (vide)")
        return
    
    # Affichage basique de la trame
    hex_data = " ".join(f"{x:02X}" for x in frame)
    print(f"{label}: {hex_data}")
    
    # Interprétation du contenu
    if len(frame) >= 6 and frame[0] == 0xFA and frame[1] == 0xFB:
        count = frame[2]
        cmd_id = frame[3] if len(frame) > 3 else 0
        
        interpretation = f"  → Commande #{cmd_id}"
        if cmd_name:
            interpretation += f" ({cmd_name.upper()})"
        
        if len(frame) >= 7:  # Commande avec argument
            arg_type = frame[4]
            arg_value = (frame[6] << 8) | frame[5]  # Little endian
            
            type_name = {0x3B: "INT_POS", 0x1B: "INT_SIGNED", 0x2B: "STRING"}.get(arg_type, f"0x{arg_type:02X}")
            interpretation += f", argument: {arg_value} (type: {type_name})"
            
            if direction:
                interpretation += f", direction: {direction}"
        
        print(interpretation)

class SeekurController:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.5):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.initialized = False
        self.monitoring = False
        self.watchdog_active = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.watchdog_thread: Optional[threading.Thread] = None
        self.response_queue = queue.Queue()
        self.last_pulse = 0
        self.verbose = True  # Mode verbeux par défaut
        
    def set_verbose(self, verbose: bool):
        """Active/désactive l'affichage détaillé"""
        self.verbose = verbose
        
    def connect(self) -> bool:
        """Établit la connexion série avec le robot et lance l'initialisation automatique"""
        try:
            print("🔌 Ouverture du port série...")
            
            # Détecter si c'est une URL TCP
            if self.port.startswith('tcp://'):
                # Parser l'URL TCP
                url_parts = self.port.replace('tcp://', '').split(':')
                host = url_parts[0]
                port = int(url_parts[1])
                
                # Utiliser l'adaptateur TCP
                self.ser = SerialTCPAdapter(host, port, self.timeout)
                self.ser.open()
            else:
                # Port série classique
                self.ser = serial.Serial(
                    self.port, self.baud, bytesize=8, parity=serial.PARITY_NONE,
                    stopbits=1, timeout=self.timeout, xonxoff=False, rtscts=False
                )
                self.ser.setDTR(False)
                self.ser.setRTS(False)
                
            self.ser.reset_input_buffer()
            self.connected = True
            print(f"✅ Connexion établie sur {self.port}")
            
            # Initialisation automatique obligatoire
            print("🚀 Lancement de l'initialisation automatique...")
            if self.initialize_robot():
                print("✅ Initialisation réussie!")
                self.start_watchdog()
                return True
            else:
                print("❌ Échec de l'initialisation automatique")
                print("Vous pouvez essayer manuellement avec 'init' ou 'sync'")
                return True
                
        except Exception as e:
            print(f"❌ Erreur de connexion: {e}")
            return False
    
    def disconnect(self):
        """Ferme la connexion série"""
        self.stop_watchdog()
        self.stop_monitoring()
        if self.ser and self.ser.is_open:
            try:
                self.send_command('close')
                time.sleep(0.1)
            except:
                pass
            self.ser.close()
        self.connected = False
        self.initialized = False
        print("🔌 Connexion fermée")
    
    def send_command(self, cmd_name: str, arg_val: Optional[int] = None, skip_watchdog: bool = False) -> bool:
        """Envoie une commande au robot - AVEC CORRECTION BIDIRECTIONNELLE VEL ET RVEL"""
        if not self.connected or not self.ser:
            print("❌ Pas de connexion active")
            return False
        
        if cmd_name not in COMMANDS:
            print(f"❌ Commande '{cmd_name}' inconnue")
            return False
        
        cmd = COMMANDS[cmd_name]
        
        # === CORRECTION BIDIRECTIONNELLE VEL ET RVEL ===
        if cmd_name in ['vel', 'rvel', 'rotate'] and arg_val is not None:
            return self._send_movement_command(cmd.id, cmd_name, arg_val, skip_watchdog)
        
        # Validation des arguments pour les autres commandes
        if cmd.arg_type == ArgType.NONE and arg_val is not None:
            print(f"❌ La commande '{cmd_name}' ne prend pas d'argument")
            return False
        
        if cmd.arg_type != ArgType.NONE and arg_val is None:
            print(f"❌ La commande '{cmd_name}' nécessite un argument")
            return False
        
        if cmd.arg_range and arg_val is not None:
            if not (cmd.arg_range[0] <= arg_val <= cmd.arg_range[1]):
                print(f"❌ Argument hors limites [{cmd.arg_range[0]}, {cmd.arg_range[1]}]")
                return False
        
        # Construction et envoi de la trame normale
        return self._send_standard_command(cmd, cmd_name, arg_val, skip_watchdog)
    
    def _send_movement_command(self, cmd_id: int, cmd_name: str, arg_val: int, skip_watchdog: bool) -> bool:
        """Gestion spécialisée des commandes de mouvement avec correction bidirectionnelle"""
        try:
            # Déterminer le type d'argument selon la direction et la commande
            if arg_val >= 0:
                # Valeur positive = direction normale = Type 3B (INT_POS)
                arg_type = 0x3B
                abs_val = arg_val
                if cmd_name == 'vel':
                    direction = "AVANT"
                    direction_symbol = "→"
                else:  # rvel, rotate
                    direction = "CCW (sens antihoraire)"
                    direction_symbol = "↺"
            else:
                # Valeur négative = direction inverse = Type 1B (INT_SIGNED) 
                arg_type = 0x1B
                abs_val = abs(arg_val)  # Utiliser valeur absolue
                if cmd_name == 'vel':
                    direction = "ARRIÈRE"
                    direction_symbol = "←"
                else:  # rvel, rotate
                    direction = "CW (sens horaire)"
                    direction_symbol = "↻"
            
            if self.verbose and not skip_watchdog:
                print(f"🎯 Mouvement {direction} {direction_symbol}: {arg_val} -> valeur={abs_val}, type=0x{arg_type:02X}")
            
            # Construction de la trame
            frame = build_cmd(cmd_id, arg_type, abs_val)
            
            # Envoi
            self.ser.write(frame)
            self.ser.flush()
            
            # Mise à jour du watchdog seulement pour PULSE
            if cmd_name == 'pulse':
                self.last_pulse = time.time()
            
            # Affichage selon le contexte
            if self.verbose and not skip_watchdog:
                format_readable_frame(f"📤 TX {cmd_name.upper()}", frame, cmd_name, arg_val, direction)
                print(f"✅ Commande '{cmd_name}' {direction} envoyée")
            
            # Lecture de la réponse éventuelle
            if not skip_watchdog and cmd_name != 'pulse':
                time.sleep(0.1)
                if self.ser.in_waiting > 0:
                    rx = self.ser.read(self.ser.in_waiting)
                    if rx and self.verbose:
                        format_readable_frame("📥 RX Réponse", rx)
            
            return True
            
        except Exception as e:
            print(f"❌ Erreur envoi commande mouvement: {e}")
            return False
    
    def _send_standard_command(self, cmd: Command, cmd_name: str, arg_val: Optional[int], skip_watchdog: bool) -> bool:
        """Envoi des commandes standard (non-mouvement)"""
        try:
            # Construction de la trame
            if cmd.arg_type == ArgType.NONE:
                frame = build_cmd(cmd.id)
            else:
                frame = build_cmd(cmd.id, cmd.arg_type.value, arg_val)
            
            # Envoi
            self.ser.write(frame)
            self.ser.flush()
            
            # Mise à jour du watchdog seulement pour PULSE
            if cmd_name == 'pulse':
                self.last_pulse = time.time()
            
            # Affichage selon le contexte
            if (self.verbose and not skip_watchdog) or cmd_name in ['open', 'enable']:
                format_readable_frame(f"📤 TX {cmd_name.upper()}", frame, cmd_name, arg_val)
                print(f"✅ Commande '{cmd_name}' envoyée")
            
            # Lecture de la réponse éventuelle
            if not skip_watchdog and cmd_name != 'pulse':
                time.sleep(0.1)
                if self.ser.in_waiting > 0:
                    rx = self.ser.read(self.ser.in_waiting)
                    if rx and self.verbose:
                        format_readable_frame("📥 RX Réponse", rx)
            
            return True
            
        except Exception as e:
            print(f"❌ Erreur envoi commande: {e}")
            return False
    
    def initialize_robot(self) -> bool:
        """Séquence d'initialisation complète du robot"""
        print("🤖 Initialisation du robot SeekurJR...")
        
        if not self.ser:
            print("❌ Pas de connexion série")
            return False
        
        if not self.connected:
            print("❌ Port série non connecté")
            return False
        
        try:
            # Vider le buffer d'entrée
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            
            # Séquence de synchronisation
            print("🔄 Synchronisation (SYNC0, SYNC1, SYNC2)...")
            
            # SYNC0, SYNC1, SYNC2 avec réception des échos
            sync_commands = [
                (0, "SYNC0"),
                (1, "SYNC1"), 
                (2, "SYNC2")
            ]
            
            for cmd_id, cmd_name in sync_commands:
                print(f"📡 Envoi {cmd_name}...")
                
                # Construction de la trame
                frame = build_cmd(cmd_id)
                format_readable_frame(f"📤 TX {cmd_name}", frame, cmd_name.lower())
                
                # Envoi
                self.ser.write(frame)
                self.ser.flush()
                
                # Attendre l'écho avec timeout
                time.sleep(self.timeout)
                
                # Lire la réponse
                rx = self.ser.read(4096)
                if rx:
                    format_readable_frame(f"📥 ECHO {cmd_name}", rx)
                    
                    # Vérifier que c'est bien l'écho attendu
                    if rx == frame:
                        print(f"✅ Écho {cmd_name} correct!")
                    else:
                        print(f"⚠️  Écho {cmd_name} différent de l'attendu")
                else:
                    print(f"⚠️  Pas d'écho reçu pour {cmd_name}")
            
            # Attendre un peu après les SYNC
            time.sleep(0.5)
            
            # OPEN - Ouverture des serveurs
            print("🚪 Ouverture des serveurs (OPEN)...")
            frame = build_cmd(1)  # OPEN = commande 1
            format_readable_frame("📤 TX OPEN", frame, "open")
            self.ser.write(frame)
            self.ser.flush()
            
            # Lire réponse OPEN
            time.sleep(self.timeout)
            rx = self.ser.read(4096)
            if rx:
                format_readable_frame("📥 RX OPEN", rx)
            
            time.sleep(0.5)
            
            # ENABLE - Activation des moteurs
            print("⚡ Activation des moteurs (ENABLE 1)...")
            frame = build_cmd(4, 0x3B, 1)  # ENABLE = commande 4, arg_type 0x3B, valeur 1
            format_readable_frame("📤 TX ENABLE", frame, "enable", 1)
            self.ser.write(frame)
            self.ser.flush()
            
            # Lire réponse ENABLE
            time.sleep(self.timeout)
            rx = self.ser.read(4096)
            if rx:
                format_readable_frame("📥 RX ENABLE", rx)
            
            self.initialized = True
            print("✅ Robot initialisé et prêt!")
            print("🔄 Watchdog automatique démarré (PULSE toutes les 1.5s)")
            return True
            
        except Exception as e:
            print(f"❌ Erreur durant l'initialisation: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_watchdog(self):
        """Démarre le watchdog automatique (PULSE toutes les 1.5s)"""
        if self.watchdog_active:
            return
        
        self.watchdog_active = True
        self.last_pulse = time.time()
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
    
    def stop_watchdog(self):
        """Arrête le watchdog"""
        self.watchdog_active = False
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=2)
    
    def _watchdog_loop(self):
        """Boucle du watchdog - envoie PULSE toutes les 1.5 secondes"""
        while self.watchdog_active and self.connected:
            try:
                current_time = time.time()
                if current_time - self.last_pulse >= 1.5:  # Envoyer PULSE toutes les 1.5s
                    if self.initialized and self.ser:
                        # PULSE silencieux
                        frame = build_cmd(0)  # PULSE = commande 0
                        self.ser.write(frame)
                        self.ser.flush()
                        self.last_pulse = current_time
                time.sleep(0.5)  # Vérifier toutes les 0.5s
            except Exception as e:
                if self.watchdog_active:
                    print(f"❌ Erreur watchdog: {e}")
                break
    
    def start_monitoring(self):
        """Démarre le monitoring des paquets SIP"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("📊 Monitoring des SIP démarré")
    
    def stop_monitoring(self):
        """Arrête le monitoring"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)
        print("⏹️  Monitoring arrêté")
    
    def _monitor_loop(self):
        """Boucle de monitoring des paquets SIP"""
        buf = bytearray()
        
        while self.monitoring and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(1024)
                if chunk:
                    buf += chunk
                    self._parse_sip_packets(buf)
                time.sleep(0.05)
            except Exception as e:
                if self.monitoring:
                    print(f"❌ Erreur monitoring: {e}")
                break
    
    def _parse_sip_packets(self, buf: bytearray):
        """Parse les paquets SIP reçus"""
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
            
            # Vérification du checksum
            body = frame[3:-2]
            chk = (frame[-2] << 8) | frame[-1]
            comp = aria_checksum(body)
            
            if chk == comp:
                self._decode_sip_packet(frame)
    
##############
# remplacez la méthode existante _decode_sip_packet
#  par cette version améliorée :


    def _decode_sip_packet(self, frame: bytes):
        """Décode et affiche un paquet SIP en format lisible"""
        if len(frame) < 4:
            return
        
        packet_type = frame[3]
        
        if packet_type in [0x32, 0x33]:
            if len(frame) >= 20:
                # Extraction des données
                xpos = int.from_bytes(frame[4:6], 'little', signed=True)
                ypos = int.from_bytes(frame[6:8], 'little', signed=True)  
                thpos = int.from_bytes(frame[8:10], 'little', signed=True)
                lvel = int.from_bytes(frame[10:12], 'little', signed=True)
                rvel = int.from_bytes(frame[12:14], 'little', signed=True)
                battery = frame[14]
                
                # Conversions
                angle_deg = thpos * 0.001534 * 180 / 3.14159
                linear_vel = (lvel + rvel) / 2.0
                angular_vel_deg = (rvel - lvel) / 330.0 * 1000 * 180 / 3.14159
                
                print(f"\n📊 Robot en temps réel:")
                print(f"  📍 Position: X={xpos}mm, Y={ypos}mm, Angle={angle_deg:.1f}°")
                print(f"  🏃 Vitesses roues: G={lvel}mm/s, D={rvel}mm/s")
                print(f"  ➡️  Vitesse: {linear_vel:.0f}mm/s linéaire, {angular_vel_deg:.1f}°/s rotation")
                print(f"  🔋 Batterie: {battery*0.2:.1f}V ({battery}/255)")
        
        elif packet_type == 0x9A:
            print(f"\n📐 Données IMU reçues")
        elif packet_type == 0x20:
            print(f"\n⚙️ Configuration reçue")
        else:
            print(f"\n❓ Paquet type 0x{packet_type:02X}")

def print_help():
    """Affiche l'aide des commandes disponibles"""
    print("\n📖 COMMANDES DISPONIBLES:")
    print("=" * 60)
    print("🔧 CORRECTION APPLIQUÉE: vel/rvel/rotate bidirectionnels")
    print("   • Valeurs positives → direction normale (avant/CCW)")
    print("   • Valeurs négatives → direction inverse (arrière/CW)")
    
    categories = {
        '🔌 Connexion': ['sync0', 'sync1', 'sync2', 'open', 'close', 'pulse'],
        '⚡ Moteurs': ['enable', 'stop', 'estop'],
        '🚗 Mouvement': ['vel', 'rvel', 'head', 'dhead', 'rotate'],
        '⚙️ Configuration': ['setv', 'setrv', 'seta', 'setra', 'seto'],
        '🔧 Accessoires': ['bumpstall', 'joydrive', 'lrfpower', 'config'],
        '📊 Données': ['imu', 'joyrequest', 'debug'],
        '🔋 Alimentation': ['auxpcpower', 'ptzpower', 'lrf2power', 'seekuroff'],
        '🛠️ Système': ['reset', 'maintenance', 'battest']
    }
    
    for category, cmd_list in categories.items():
        print(f"\n{category}:")
        for cmd_name in cmd_list:
            if cmd_name in COMMANDS:
                cmd = COMMANDS[cmd_name]
                arg_info = ""
                if cmd.arg_type != ArgType.NONE:
                    if cmd.arg_range:
                        arg_info = f" <{cmd.arg_range[0]}..{cmd.arg_range[1]}>"
                    else:
                        arg_info = " <valeur>"
                print(f"  {cmd_name:12} {arg_info:15} - {cmd.description}")

def interactive_mode(controller: SeekurController):
    """Mode interactif principal"""
    print("\n🎮 MODE INTERACTIF SEEKUR CONTROLLER Version ascii")
    print("Tapez 'help' pour voir les commandes, 'quit' pour quitter")
    print("Format: <commande> [argument]")
    print("Exemples: vel 500, vel -500, rvel 30, rvel -30, head 90, stop")
    print("🔧 VEL/RVEL corrigés: positif=avant/CCW, négatif=arrière/CW")
    print("🔧 Commandes spéciales: status, monitor, nomonitor, sync, verbose, quiet")
    
    while True:
        try:
            user_input = input("\nSeekur> ").strip().lower()
            
            if not user_input:
                continue
                
            if user_input in ['quit', 'exit', 'q']:
                break
                
            if user_input == 'help':
                print_help()
                continue
                
            if user_input == 'verbose':
                controller.set_verbose(True)
                print("🔊 Mode verbeux activé")
                continue
                
            if user_input == 'quiet':
                controller.set_verbose(False)
                print("🔇 Mode silencieux activé")
                continue
                
            if user_input == 'sync':
                print("🔄 Test de synchronisation manuelle...")
                if controller.initialize_robot():
                    if not controller.watchdog_active:
                        controller.start_watchdog()
                continue
                
            if user_input == 'init':
                print("🚀 Initialisation manuelle...")
                if controller.connected:
                    if controller.initialize_robot():
                        if not controller.watchdog_active:
                            controller.start_watchdog()
                else:
                    print("❌ Pas de connexion active")
                continue
                
            if user_input == 'status':
                print(f"📊 État du système:")
                print(f"  🔌 Connecté: {'✅' if controller.connected else '❌'}")
                print(f"  🤖 Initialisé: {'✅' if controller.initialized else '❌'}")
                print(f"  🔄 Watchdog: {'✅ Actif' if controller.watchdog_active else '❌ Inactif'}")
                print(f"  📊 Monitoring: {'✅ Actif' if controller.monitoring else '❌ Inactif'}")
                print(f"  🔊 Mode: {'Verbeux' if controller.verbose else 'Silencieux'}")
                if controller.watchdog_active:
                    since_pulse = time.time() - controller.last_pulse
                    print(f"  💓 Dernier PULSE: {since_pulse:.1f}s")
                continue
                
            if user_input == 'monitor':
                controller.start_monitoring()
                continue
                
            if user_input == 'nomonitor':
                controller.stop_monitoring()
                continue
            
            # Parse commande et argument
            parts = user_input.split()
            cmd_name = parts[0]
            arg_val = None
            
            if len(parts) > 1:
                try:
                    arg_val = int(parts[1])
                except ValueError:
                    print("❌ Argument doit être un nombre entier")
                    continue
            
            # Exécution de la commande
            controller.send_command(cmd_name, arg_val)
            
        except KeyboardInterrupt:
            print("\n\n⚠️ Interruption utilisateur")
            break
        except Exception as e:
            print(f"❌ Erreur: {e}")

def main():
    parser = argparse.ArgumentParser(description="Contrôleur interactif SeekurJR/SeekurOS")
    parser.add_argument("--port", required=True, help="Port série (ex: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=9600, help="Vitesse de transmission (défaut: 9600)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Timeout lecture (défaut: 0.5s)")
    parser.add_argument("--quiet", action="store_true", help="Mode silencieux (moins de messages)")
    
    args = parser.parse_args()
    
    print("🤖 SEEKUR JR INTERACTIVE CONTROLLER - VERSION LISIBLE")
    print("=" * 60)
    print("🔧 CORRECTION: Mouvement bidirectionnel VEL/RVEL/ROTATE")
    print("   • Positif (+) = avant/CCW (type 3B)")
    print("   • Négatif (-) = arrière/CW (type 1B)")
    print("⚠️ ATTENTION: Initialisation automatique au démarrage!")
    print("🔄 Watchdog automatique pour maintenir la connexion")
    
    controller = SeekurController(args.port, args.baud, args.timeout)
    
    if args.quiet:
        controller.set_verbose(False)
        print("🔇 Mode silencieux activé")
    
    try:
        # Connexion avec initialisation automatique
        print("🔌 Tentative de connexion et initialisation...")
        if not controller.connect():
            print("❌ Impossible de se connecter au port série")
            sys.exit(1)
        
        interactive_mode(controller)
        
    except Exception as e:
        print(f"❌ Erreur inattendue: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        controller.disconnect()
        print("\n👋 Au revoir!")

if __name__ == "__main__":
    main()