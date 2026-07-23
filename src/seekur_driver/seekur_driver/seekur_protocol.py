"""
Module pour le protocole de communication SeekurOS
Contient les fonctions utilitaires pour la communication série
"""

from enum import Enum
from typing import Optional

# Configuration protocole SeekurOS
HDR0, HDR1 = 0xFA, 0xFB

class ArgType(Enum):
    NONE = None
    INT_POS = 0x3B      # Entier positif
    INT_SIGNED = 0x1B   # Entier signé
    STRING = 0x2B       # String

class SeekurCommands:
    """Définitions des commandes SeekurOS"""
    
    # Commandes de base
    PULSE = 0
    OPEN = 1
    CLOSE = 2
    ENABLE = 4
    
    # Commandes de mouvement
    VEL = 11
    RVEL = 21
    HEAD = 12
    DHEAD = 13
    ROTATE = 9
    STOP = 29
    ESTOP = 55
    
    # Configuration
    SETV = 6
    SETRV = 10
    SETA = 5
    SETRA = 23
    SETO = 7
    
    # Accessoires
    CONFIG = 18
    IMU = 26
    LRFPOWER = 96

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

def build_velocity_cmd(vel_mms: int) -> bytes:
    """Construit commande VEL avec correction bidirectionnelle"""
    if vel_mms >= 0:
        # Avant: type 3B + valeur positive
        return build_cmd(SeekurCommands.VEL, ArgType.INT_POS.value, vel_mms)
    else:
        # Arrière: type 1B + valeur absolue
        return build_cmd(SeekurCommands.VEL, ArgType.INT_SIGNED.value, abs(vel_mms))

def build_rotation_cmd(rvel_degs: int) -> bytes:
    """Construit commande RVEL avec correction bidirectionnelle"""
    if rvel_degs >= 0:
        # CCW: type 3B + valeur positive
        return build_cmd(SeekurCommands.RVEL, ArgType.INT_POS.value, rvel_degs)
    else:
        # CW: type 1B + valeur absolue
        return build_cmd(SeekurCommands.RVEL, ArgType.INT_SIGNED.value, abs(rvel_degs))

class SIPDecoder:
    """Décodeur pour les paquets SIP SeekurOS"""
    
    @staticmethod
    def decode_standard_sip(frame: bytes) -> dict:
        """Décode un paquet SIP standard"""
        if len(frame) < 20:
            return {}
        
        # Extraction des données (little endian, signed)
        xpos = int.from_bytes(frame[4:6], 'little', signed=True)  # mm
        ypos = int.from_bytes(frame[6:8], 'little', signed=True)  # mm
        thpos = int.from_bytes(frame[8:10], 'little', signed=True)  # unités
        lvel = int.from_bytes(frame[10:12], 'little', signed=True)  # mm/s
        rvel = int.from_bytes(frame[12:14], 'little', signed=True)  # mm/s
        battery = frame[14]  # niveau batterie
        
        return {
            'x_mm': xpos,
            'y_mm': ypos,
            'theta_units': thpos,
            'left_vel_mms': lvel,
            'right_vel_mms': rvel,
            'battery_level': battery,
            'x_m': xpos / 1000.0,
            'y_m': ypos / 1000.0,
            'theta_rad': thpos * 0.001534,
            'battery_voltage': battery * 0.2
        }