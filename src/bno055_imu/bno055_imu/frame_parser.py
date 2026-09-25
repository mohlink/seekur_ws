"""
frame_parser.py — Parseur des trames du sketch bno055_imu_bridge.

Module pur (aucune dépendance ROS) pour pouvoir être testé seul avec pytest.

Trame : I,seq,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal*HH
  q  : 1 LSB = 1/16384
  g  : 1 LSB = 1/16 deg/s
  a  : 1 LSB = 0.01 m/s² (gravité incluse)
  cal: bits 7-6 sys, 5-4 gyro, 3-2 accel, 1-0 mag
  HH : XOR hexadécimal de tous les caractères avant '*'
"""

import math
from dataclasses import dataclass
from typing import Tuple

QUAT_SCALE = 1.0 / 16384.0
GYRO_SCALE = math.pi / (16.0 * 180.0)   # LSB -> rad/s
ACCEL_SCALE = 1.0 / 100.0               # LSB -> m/s²
N_FIELDS = 13                           # 'I' + 12 valeurs


class FrameError(Exception):
    """Trame rejetée. reason vaut 'format' ou 'checksum'."""

    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


@dataclass
class ImuFrame:
    seq: int
    quat_wxyz: Tuple[float, float, float, float]   # non normalisé
    quat_norm: float
    gyro_rads: Tuple[float, float, float]
    accel_ms2: Tuple[float, float, float]
    cal_sys: int
    cal_gyr: int
    cal_acc: int
    cal_mag: int


def xor_checksum(payload: str) -> int:
    cs = 0
    for ch in payload:
        cs ^= ord(ch)
    return cs


def parse_line(line: str) -> ImuFrame:
    """Parse une ligne 'I,...*HH'. Lève FrameError si invalide."""
    star = line.rfind('*')
    if not line.startswith('I,') or star < 0 or len(line) < star + 3:
        raise FrameError('format', repr(line[:40]))

    payload, cs_txt = line[:star], line[star + 1:star + 3]
    try:
        cs_rx = int(cs_txt, 16)
    except ValueError:
        raise FrameError('format', f'checksum illisible {cs_txt!r}')
    if xor_checksum(payload) != cs_rx:
        raise FrameError('checksum')

    fields = payload.split(',')
    if len(fields) != N_FIELDS:
        raise FrameError('format', f'{len(fields)} champs au lieu de {N_FIELDS}')
    try:
        v = [int(f) for f in fields[1:]]
    except ValueError:
        raise FrameError('format', 'valeur non entière')

    seq, qw, qx, qy, qz, gx, gy, gz, ax, ay, az, cal = v
    q = (qw * QUAT_SCALE, qx * QUAT_SCALE, qy * QUAT_SCALE, qz * QUAT_SCALE)
    norm = math.sqrt(sum(c * c for c in q))

    return ImuFrame(
        seq=seq,
        quat_wxyz=q,
        quat_norm=norm,
        gyro_rads=(gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE),
        accel_ms2=(ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE),
        cal_sys=(cal >> 6) & 0x03,
        cal_gyr=(cal >> 4) & 0x03,
        cal_acc=(cal >> 2) & 0x03,
        cal_mag=cal & 0x03,
    )
