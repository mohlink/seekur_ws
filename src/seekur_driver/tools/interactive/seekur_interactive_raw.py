#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
seekur_interactive_raw.py — Contrôleur interactif SeekurJR (trames brutes)

Version DE DIAGNOSTIC BAS NIVEAU. Envoie des trames SeekurOS arbitraires
avec calcul de CRC automatique — utile pour tester une commande non
documentée, reproduire un bug protocole, ou envoyer des séquences
d'octets précises que les commandes standard ne permettent pas.

Ne remplace pas seekur_interactive_serial.py ou seekur_interactive_tcp.py
pour l'usage normal — c'est un outil de troubleshoot spécialisé.
"""

import argparse
import time
import sys
import threading
import queue
import struct
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    import serial
except ImportError as e:
    print("PySerial manquant. Installez-le avec:\n  sudo apt install python3-serial", file=sys.stderr)
    raise

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
    
    # Commandes de mouvement
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
    'halt': Command(21, 'HALT', 'Arrêt complet: rvel 0 + vel 0', ArgType.NONE),
    'estop': Command(55, 'E_STOP', 'Arrêt d\'urgence (freinage brutal)', ArgType.NONE),
    'seto': Command(7, 'SETO', 'Reset position à origine (0,0,0)', ArgType.NONE),
    'config': Command(18, 'CONFIG', 'Demander packet de configuration', ArgType.NONE),
    
    # Commandes accessoires
    'bumpstall': Command(44, 'BUMPSTALL', 'Config bumpers: 0=off, 1=avant, 2=arrière, 3=tous', ArgType.INT_POS, (0, 3)),
    'joydrive': Command(47, 'JOYDRIVE', 'Autoriser joystick (1) ou non (0)', ArgType.INT_POS, (0, 1)),
    'lrfpower': Command(96, 'LRFPOWER', 'Alimenter LRF: 1=ON, 0=OFF', ArgType.INT_POS, (0, 1)),
    'rotvelmaxdir': Command(20, 'ROTVELMAXDIR', 'Direction rotation max', ArgType.INT_SIGNED, (-180, 180)),
}


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
        if arg_type == 0x1B:  # INT_SIGNED
            # Convertir en little-endian signed 16-bit
            arg_bytes = struct.pack('<h', arg_val)
            lo, hi = arg_bytes[0], arg_bytes[1]
            print(f"Valeur signée {arg_val} -> bytes: LO=0x{lo:02X}, HI=0x{hi:02X}")
        else:  # INT_POS ou autres
            if arg_val < 0:
                print(f"Valeur négative {arg_val} pour type d'argument non signé!")
                arg_val = 0
            if arg_val > 0xFFFF:
                arg_val = 0xFFFF
            lo = arg_val & 0xFF
            hi = (arg_val >> 8) & 0xFF
            
        body += bytes([arg_type & 0xFF, lo, hi])
    
    count = len(body) + 2
    chk = aria_checksum(body)
    frame = bytearray([HDR0, HDR1, count & 0xFF]) + body + bytes([(chk >> 8) & 0xFF, chk & 0xFF])
    return bytes(frame)


def build_rvel_test_frame(angular_velocity: int) -> bytes:
    """Construction manuelle de trame RVEL pour test diagnostique"""
    
    # Header
    hdr = bytes([0xFA, 0xFB])
    
    # Command RVEL = 21 = 0x15
    cmd = 0x15
    
    # Arg type = 0x1B (INT_SIGNED)
    arg_type = 0x1B
    
    # Argument en little-endian signed 16-bit
    arg_bytes = struct.pack('<h', angular_velocity)
    lo, hi = arg_bytes[0], arg_bytes[1]
    
    # Body = cmd + arg_type + lo + hi
    body = bytes([cmd, arg_type, lo, hi])
    
    # CORRECTION: Byte count = len(body) + 2 (pour le checksum)
    # Donc pour body de 4 bytes: count = 4 + 2 = 6
    count = len(body) + 2
    
    # Calcul checksum sur le body seulement
    checksum = aria_checksum(body)
    
    # Checksum en MSB-first (big-endian)
    chk_hi = (checksum >> 8) & 0xFF
    chk_lo = checksum & 0xFF
    
    # Trame complète
    frame = hdr + bytes([count]) + body + bytes([chk_hi, chk_lo])
    
    print(f"=== DIAGNOSTIC RVEL {angular_velocity} ===")
    print(f"Header     : {hdr.hex().upper()}")
    print(f"Byte count : {count:02X} (body={len(body)}, checksum=2)")
    print(f"Command    : {cmd:02X} (RVEL)")
    print(f"Arg type   : {arg_type:02X} (INT_SIGNED)")
    print(f"Argument   : {lo:02X} {hi:02X} (little-endian)")
    print(f"Checksum   : {chk_hi:02X} {chk_lo:02X} (MSB-first)")
    print(f"Trame complète: {frame.hex().upper()}")
    
    return frame


def parse_hex_frame(hex_string: str) -> bytes:
    """Parse une chaîne hexadécimale en bytes"""
    # Nettoyer la chaîne : enlever espaces, tirets, etc.
    clean_hex = ''.join(c for c in hex_string.upper() if c in '0123456789ABCDEF')
    
    # Vérifier que la longueur est paire
    if len(clean_hex) % 2 != 0:
        raise ValueError("Nombre impair de caractères hexadécimaux")
    
    # Convertir en bytes
    return bytes.fromhex(clean_hex)


def send_raw_frame_auto_crc(controller, hex_string: str) -> bool:
    """Envoie une trame brute avec calcul automatique du CRC"""
    if not controller.connected or not controller.ser:
        print("Pas de connexion active")
        return False
    
    try:
        # Parser la trame hexadécimale (sans CRC)
        frame_no_crc = parse_hex_frame(hex_string)
        
        if len(frame_no_crc) < 3:
            print("Trame trop courte (minimum 3 bytes: header + count)")
            return False
        
        # Extraire header et count
        header = frame_no_crc[0:2]
        count = frame_no_crc[2]
        
        # Le body commence après count
        body = frame_no_crc[3:]
        
        # Calculer le CRC sur le body
        checksum = aria_checksum(body)
        chk_hi = (checksum >> 8) & 0xFF
        chk_lo = checksum & 0xFF
        
        # Construire la trame complète
        complete_frame = header + bytes([count]) + body + bytes([chk_hi, chk_lo])
        
        print(f"=== ENVOI TRAME BRUTE AVEC CRC AUTO ===")
        print(f"Entrée (sans CRC) : {hex_string}")
        print(f"Body              : {body.hex().upper()}")
        print(f"CRC calculé       : {chk_hi:02X}{chk_lo:02X}")
        print(f"Trame complète    : {complete_frame.hex().upper()}")
        hexdump("Trame finale", complete_frame)
        
        # Analyser la trame
        if len(complete_frame) >= 5:
            print(f"Analyse :")
            print(f"  Header : {header[0]:02X} {header[1]:02X}")
            print(f"  Count  : {count:02X}")
            if len(body) > 0:
                print(f"  Command: {body[0]:02X}")
            if len(body) >= 3:
                print(f"  ArgType: {body[1]:02X}")
                print(f"  Arg    : {body[2]:02X} {body[3]:02X}")
        
        # Envoyer la trame
        controller.ser.write(complete_frame)
        controller.ser.flush()
        print("Trame envoyée")
        
        # Attendre une réponse
        time.sleep(0.2)
        if controller.ser.in_waiting > 0:
            rx = controller.ser.read(controller.ser.in_waiting)
            hexdump("Réponse", rx)
        else:
            print("Aucune réponse")
        
        return True
        
    except ValueError as e:
        print(f"Erreur de parsing: {e}")
        return False
    except Exception as e:
        print(f"Erreur envoi: {e}")
        return False


def test_rvel_diagnostics(controller):
    """Test diagnostique des trames RVEL"""
    print("\n=== TEST DIAGNOSTIQUE RVEL ===")
    
    test_values = [10, -10]
    
    for vel in test_values:
        print(f"\nTest {vel}°/s:")
        
        # Version originale
        print("1. Trame via build_cmd():")
        frame_orig = build_cmd(21, 0x1B, vel)
        hexdump(f"   Original", frame_orig)
        
        # Version diagnostique
        print("2. Trame via build_rvel_test_frame():")
        frame_test = build_rvel_test_frame(vel)
        
        # Comparaison
        if frame_orig == frame_test:
            print("   Trames identiques")
        else:
            print("   Trames différentes!")
            print(f"   Diff: orig={frame_orig.hex().upper()}")
            print(f"         test={frame_test.hex().upper()}")
        
        # Envoi de la trame de test
        if controller.ser and controller.connected:
            print("3. Envoi trame de test...")
            controller.ser.write(frame_test)
            controller.ser.flush()
            time.sleep(0.1)
            
            if controller.ser.in_waiting > 0:
                rx = controller.ser.read(controller.ser.in_waiting)
                hexdump("   Réponse", rx)
            else:
                print("   Aucune réponse")
        
        print("-" * 40)


def hexdump(label: str, b: bytes):
    """Affichage hexadécimal des données"""
    print(f"{label} ({len(b)} B): " + (" ".join(f"{x:02X}" for x in b) if b else "(vide)"))


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
        
    def connect(self) -> bool:
        """Établit la connexion série avec le robot et lance l'initialisation automatique"""
        try:
            print("Ouverture du port série...")
            self.ser = serial.Serial(
                self.port, self.baud, bytesize=8, parity=serial.PARITY_NONE,
                stopbits=1, timeout=self.timeout, xonxoff=False, rtscts=False
            )
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.ser.reset_input_buffer()
            self.connected = True
            print(f"Connexion établie sur {self.port} à {self.baud} baud")
            
            # Initialisation automatique obligatoire
            print("Lancement de l'initialisation automatique...")
            if self.initialize_robot():
                print("Initialisation réussie!")
                self.start_watchdog()
                return True
            else:
                print("Échec de l'initialisation automatique")
                print("Vous pouvez essayer manuellement avec 'init' ou 'sync'")
                return True
                
        except Exception as e:
            print(f"Erreur de connexion: {e}")
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
        print("Connexion fermée")
    
    def send_command(self, cmd_name: str, arg_val: Optional[int] = None, skip_watchdog: bool = False) -> bool:
        """Envoie une commande au robot"""
        if not self.connected or not self.ser:
            print("Pas de connexion active")
            return False
        
        if cmd_name not in COMMANDS:
            print(f"Commande '{cmd_name}' inconnue")
            return False
        
        cmd = COMMANDS[cmd_name]
        
        # Validation des arguments
        if cmd.arg_type == ArgType.NONE and arg_val is not None:
            print(f"La commande '{cmd_name}' ne prend pas d'argument")
            return False
        
        if cmd.arg_type != ArgType.NONE and arg_val is None:
            print(f"La commande '{cmd_name}' nécessite un argument")
            return False
        
        if cmd.arg_range and arg_val is not None:
            if not (cmd.arg_range[0] <= arg_val <= cmd.arg_range[1]):
                print(f"Argument hors limites [{cmd.arg_range[0]}, {cmd.arg_range[1]}]")
                return False
        
        # Construction de la trame
        try:
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
            if not skip_watchdog or cmd_name in ['open', 'enable']:
                hexdump(f"TX {cmd_name.upper()}", frame)
                print(f"Commande '{cmd_name}' envoyée")
            
            # Lecture de la réponse éventuelle
            if not skip_watchdog and cmd_name != 'pulse':
                time.sleep(0.1)
                if self.ser.in_waiting > 0:
                    rx = self.ser.read(self.ser.in_waiting)
                    if rx:
                        hexdump("RX", rx)
            
            return True
            
        except Exception as e:
            print(f"Erreur envoi commande: {e}")
            return False
    
    def initialize_robot(self) -> bool:
        """Séquence d'initialisation complète du robot"""
        print("Initialisation du robot SeekurJR...")
        
        if not self.ser or not self.connected:
            print("Pas de connexion série")
            return False
        
        try:
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            
            print("Synchronisation (SYNC0, SYNC1, SYNC2)...")
            
            sync_commands = [(0, "SYNC0"), (1, "SYNC1"), (2, "SYNC2")]
            
            for cmd_id, cmd_name in sync_commands:
                print(f"Envoi {cmd_name}...")
                frame = build_cmd(cmd_id)
                hexdump(f"TX {cmd_name}", frame)
                
                self.ser.write(frame)
                self.ser.flush()
                time.sleep(self.timeout)
                
                rx = self.ser.read(4096)
                if rx:
                    hexdump(f"ECHO {cmd_name}", rx)
                    if rx == frame:
                        print(f"Écho {cmd_name} correct!")
                    else:
                        print(f"Écho {cmd_name} différent de l'attendu")
                else:
                    print(f"Pas d'écho reçu pour {cmd_name}")
            
            time.sleep(0.5)
            
            print("Ouverture des serveurs (OPEN)...")
            frame = build_cmd(1)
            hexdump("TX OPEN", frame)
            self.ser.write(frame)
            self.ser.flush()
            
            time.sleep(self.timeout)
            rx = self.ser.read(4096)
            if rx:
                hexdump("RX OPEN", rx)
            
            time.sleep(0.5)
            
            print("Activation des moteurs (ENABLE 1)...")
            frame = build_cmd(4, 0x3B, 1)
            hexdump("TX ENABLE", frame)
            self.ser.write(frame)
            self.ser.flush()
            
            time.sleep(self.timeout)
            rx = self.ser.read(4096)
            if rx:
                hexdump("RX ENABLE", rx)
            
            self.initialized = True
            print("Robot initialisé et prêt!")
            return True
            
        except Exception as e:
            print(f"Erreur durant l'initialisation: {e}")
            return False
    
    def start_watchdog(self):
        if self.watchdog_active:
            return
        self.watchdog_active = True
        self.last_pulse = time.time()
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
    
    def stop_watchdog(self):
        self.watchdog_active = False
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=2)
    
    def _watchdog_loop(self):
        while self.watchdog_active and self.connected:
            try:
                current_time = time.time()
                if current_time - self.last_pulse >= 1.5:
                    if self.initialized and self.ser:
                        frame = build_cmd(0)
                        self.ser.write(frame)
                        self.ser.flush()
                        self.last_pulse = current_time
                time.sleep(0.5)
            except Exception as e:
                if self.watchdog_active:
                    print(f"Erreur watchdog: {e}")
                break


def interactive_mode(controller: SeekurController):
    """Mode interactif principal"""
    print("\nMODE INTERACTIF SEEKUR CONTROLLER")
    print("Commandes spéciales:")
    print("  'test_rvel_diag' - Test diagnostique RVEL")
    print("  'raw <hex>'      - Envoyer trame (CRC calculé auto)")
    print("  'halt'           - Arrêt complet")
    print("  'help'           - Afficher aide")
    print("  'quit'           - Quitter")
    
    while True:
        try:
            user_input = input("\nSeekur> ").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() in ['quit', 'exit', 'q']:
                break
                
            if user_input.lower() == 'test_rvel_diag':
                test_rvel_diagnostics(controller)
                continue
            
            if user_input.lower().startswith('raw '):
                hex_data = user_input[4:].strip()
                if hex_data:
                    send_raw_frame_auto_crc(controller, hex_data)
                else:
                    print("Usage: raw <données_hex_sans_crc>")
                    print("Exemple: raw FAFB06151BF6FF (CRC calculé automatiquement)")
                continue
                
            if user_input.lower() == 'halt':
                print("Arrêt complet (rvel 0 + vel 0)...")
                controller.send_command('rvel', 0)
                time.sleep(0.1)
                controller.send_command('vel', 0)
                continue
            
            if user_input.lower() == 'help':
                print("\nCOMMANDES DISPONIBLES:")
                print("Movement:")
                print("  rvel <deg/s>   - Rotation")
                print("  vel <mm/s>     - Translation")
                print("  stop           - Arrêt")
                print("  halt           - Arrêt complet")
                print("Configuration:")
                print("  setv <mm/s>    - Vitesse max translation")
                print("  setrv <deg/s>  - Vitesse max rotation")
                print("  config         - Info configuration")
                print("Tests:")
                print("  test_rvel_diag - Diagnostic RVEL")
                print("  raw <hex>      - Trame brute (CRC auto)")
                print("Exemples trames brutes:")
                print("  raw FAFB06151BF6FF  (RVEL -10, count=06)")
                print("  raw FAFB05151BF6FF  (RVEL -10, count=05)")
                continue
            
            # Parse commande et argument
            parts = user_input.split()
            cmd_name = parts[0].lower()
            arg_val = None
            
            if len(parts) > 1:
                try:
                    arg_val = int(parts[1])
                except ValueError:
                    print("Argument doit être un nombre entier")
                    continue
            
            # Exécution de la commande
            controller.send_command(cmd_name, arg_val)
            
        except KeyboardInterrupt:
            print("\n\nInterruption utilisateur")
            break
        except Exception as e:
            print(f"Erreur: {e}")


def main():
    parser = argparse.ArgumentParser(description="Contrôleur SeekurJR avec CRC automatique")
    parser.add_argument("--port", required=True, help="Port série (ex: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=9600, help="Vitesse de transmission (défaut: 9600)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Timeout lecture (défaut: 0.5s)")
    
    args = parser.parse_args()
    
    print("SEEKUR JR INTERACTIVE CONTROLLER - CRC AUTOMATIQUE")
    print("=" * 50)
    print("NOUVEAU: Commande 'raw' calcule automatiquement le CRC")
    print("Exemple: raw FAFB06151BF6FF")
    
    controller = SeekurController(args.port, args.baud, args.timeout)
    
    try:
        if not controller.connect():
            print("Impossible de se connecter au port série")
            sys.exit(1)
        
        interactive_mode(controller)
        
    except Exception as e:
        print(f"Erreur inattendue: {e}")
        
    finally:
        controller.disconnect()
        print("\nAu revoir!")


if __name__ == "__main__":
    main()