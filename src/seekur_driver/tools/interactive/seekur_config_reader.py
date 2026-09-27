#!/usr/bin/env python3
"""
seekur_config_reader.py — Lit et décode le CONFIGpac (type 0x20) du SeekurJR.

Séquence : SYNC0/1/2 -> OPEN -> PULSE -> CONFIG -> lecture (jusqu'à 3 s) -> CLOSE.
Aucun ENABLE : les moteurs restent désactivés.

Ordre des champs : manuel SeekurJr (Table 11) / ARIA ArRobotConfigPacketReader.
Le décodage s'arrête proprement si le paquet est plus court que prévu.

Usage : python3 seekur_config_reader.py --port /dev/seekur
"""

import argparse
import sys
import time

import serial

HDR = b'\xFA\xFB'


def aria_checksum(data: bytes) -> int:
    c, i, n = 0, 0, len(data)
    while n > 1:
        c = (c + ((data[i] << 8) | data[i + 1])) & 0xFFFF
        i += 2
        n -= 2
    if n > 0:
        c ^= data[i]
    return c & 0xFFFF


def build_cmd(cmd: int) -> bytes:
    body = bytes([cmd & 0xFF])
    chk = aria_checksum(body)
    return HDR + bytes([len(body) + 2]) + body + bytes([chk >> 8, chk & 0xFF])


def extract_packets(buf: bytes):
    """Renvoie la liste des paquets complets à checksum valide trouvés dans buf."""
    out, i = [], 0
    while True:
        i = buf.find(HDR, i)
        if i < 0 or i + 3 > len(buf):
            return out
        end = i + 3 + buf[i + 2]
        if end > len(buf):
            return out
        frame = buf[i:end]
        body = frame[3:-2]
        if len(frame) >= 6 and aria_checksum(body) == ((frame[-2] << 8) | frame[-1]):
            out.append(frame)
            i = end
        else:
            i += 1


class Reader:
    def __init__(self, data: bytes):
        self.d, self.i = data, 0

    def byte(self):
        v = self.d[self.i]
        self.i += 1
        return v

    def u16(self):
        v = self.d[self.i] | (self.d[self.i + 1] << 8)
        self.i += 2
        return v

    def s16(self):
        v = self.u16()
        return v - 0x10000 if v & 0x8000 else v

    def string(self):
        end = self.d.index(0, self.i)
        s = self.d[self.i:end].decode('ascii', errors='replace')
        self.i = end + 1
        return s


# (nom, type, commentaire) — ordre du CONFIGpac
FIELDS = [
    ('Type', 'str', ''), ('Subtype', 'str', ''), ('SerialNum', 'str', ''),
    ('FourMot (NU)', 'byte', ''),
    ('RotVelTop', 'u16', 'deg/s'), ('TransVelTop', 'u16', 'mm/s'),
    ('RotAccTop', 'u16', 'deg/s²'), ('TransAccTop', 'u16', 'mm/s²'),
    ('PWMMax', 'u16', ''), ('Name', 'str', ''),
    ('SipCycle', 'byte', 'ms'), ('HostBaud', 'byte', '0=9600 1=19200 2=38400 4=115200'),
    ('AuxBaud', 'byte', ''), ('Gripper (NU)', 'u16', ''),
    ('FrontSonar (NU)', 'u16', ''), ('RearSonar (NU)', 'byte', ''),
    ('LowBattery', 'u16', 'V x10'), ('RevCount (NU)', 'u16', ''),
    ('Watchdog', 'u16', 'ms'), ('P2Mpacs (NU)', 'byte', ''),
    ('StallVal', 'u16', ''), ('StallCount', 'u16', 'ms'),
    ('JoyVel', 'u16', 'mm/s'), ('JoyRVel', 'u16', 'deg/s'),
    ('RotVelMax', 'u16', 'deg/s'), ('TransVelMax', 'u16', 'mm/s'),
    ('RotAcc', 'u16', 'deg/s²'), ('RotDecel', 'u16', 'deg/s²'),
    ('RotKP', 'u16', ''), ('RotKV', 'u16', ''), ('RotKI', 'u16', ''),
    ('TransAcc', 'u16', 'mm/s²'), ('TransDecel', 'u16', 'mm/s²'),
    ('TransKP', 'u16', ''), ('TransKV', 'u16', ''), ('TransKI', 'u16', ''),
    ('FrontBumps', 'byte', 'segments'), ('RearBumps', 'byte', 'segments'),
    ('Charger', 'byte', ''), ('SonarCycle (NU)', 'byte', ''),
    ('Autobaud', 'byte', ''),
    ('HasGyro', 'byte', '0=aucun 3=SAG 4=IMU'),
    ('DriftFactor (NU)', 's16', ''), ('Aux2Baud (NU)', 'byte', ''),
    ('Aux3Baud (NU)', 'byte', ''), ('TicksMM (NU)', 'u16', ''),
    ('ShutdownVolts', 'u16', 'V x10'),
    ('VersionMajor', 'str', ''), ('VersionMinor', 'str', ''),
    ('GyroCW', 'u16', ''), ('GyroCCW', 'u16', ''),
    ('KinematicsDelay', 'byte', 'ms'),
]


def decode_config(frame: bytes):
    r = Reader(frame[4:-2])          # après FA FB count type, avant checksum
    values = {}
    for name, kind, unit in FIELDS:
        try:
            v = getattr(r, kind)() if kind != 'str' else r.string()
        except (IndexError, ValueError):
            print(f'  ... fin du paquet atteinte avant « {name} »')
            break
        values[name] = v
        marker = '   <==' if name == 'HasGyro' else ''
        print(f'  {name:18} {v!s:>14}  {unit}{marker}')
    rest = len(frame) - 6 - r.i
    if rest > 0:
        print(f'  ({rest} octet(s) supplémentaires non décodés)')
    return values


def main():
    ap = argparse.ArgumentParser(description='Lecture du CONFIGpac SeekurJR (moteurs non activés)')
    ap.add_argument('--port', default='/dev/seekur')
    ap.add_argument('--baud', type=int, default=9600)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    ser.setDTR(False)
    ser.setRTS(False)
    ser.reset_input_buffer()

    try:
        for cmd, name in [(0, 'SYNC0'), (1, 'SYNC1'), (2, 'SYNC2')]:
            ser.write(build_cmd(cmd))
            time.sleep(0.5)
            rx = ser.read(4096)
            print(f'{name}: {"écho OK" if rx.startswith(build_cmd(cmd)) else "réponse " + rx[:12].hex(" ")}')

        ser.write(build_cmd(1))        # OPEN
        time.sleep(0.5)
        ser.reset_input_buffer()       # jette les SIP accumulés
        ser.write(build_cmd(0))        # PULSE (watchdog)
        ser.write(build_cmd(18))       # CONFIG
        print('CONFIG envoyé, attente du paquet 0x20...')

        buf, deadline, cfg = b'', time.time() + 3.0, None
        while time.time() < deadline and cfg is None:
            buf += ser.read(512)
            for p in extract_packets(buf):
                if p[3] == 0x20:
                    cfg = p
                    break

        if cfg is None:
            print('Aucun CONFIGpac complet reçu en 3 s.', file=sys.stderr)
            print(f'Derniers octets : {buf[-80:].hex(" ")}', file=sys.stderr)
            return 1

        print(f'\nCONFIGpac reçu : {len(cfg)} octets, checksum OK')
        print(f'Brut : {cfg.hex(" ")}\n')
        decode_config(cfg)
        return 0
    finally:
        ser.write(build_cmd(2))        # CLOSE
        time.sleep(0.1)
        ser.close()


if __name__ == '__main__':
    sys.exit(main())
