#!/usr/bin/env python3
"""seekur_gyro_test.py - Test du gyroscope interne (SAG) du SeekurJR.

Compare, a chaque SIP (10 Hz), trois estimations de la rotation :
  TH / cumul : cap THPOS (corrige par le SAG selon le manuel) et rotation cumulee
  dTH        : derivee de THPOS (deg/s)
  ROTVEL     : champ ROTVEL du SIP (deg/s)
  roues      : (rvel - lvel) / voie (deg/s)
Reference independante : la colonne "cap" de imu_pitch_probe.py (BNO055),
lance dans un autre terminal, BNO055 fixe sur le chassis.

Clavier (touche puis Entree) :
  +  rotation anti-horaire (CCW) a --rvel deg/s
  -  rotation horaire (CW)
  s  arret (VEL 0 / RVEL 0)
  z  remise a zero du cumul
  q  quitter : arret, moteurs desactives, CLOSE
  (seule la premiere lettre compte ; Ctrl+C arrete aussi proprement)

Usage : python3 seekur_gyro_test.py --port /dev/seekur --rvel 15 --track 0.663

SECURITE : robot au sol, 1 m degage, arret d'urgence a portee de main.
Le driver ROS ne doit pas tourner (port exclusif). Ce script n'a pas l'arret
de securite /cmd_vel du driver : une rotation s'arrete seule apres --max-time,
et si le script est tue, le watchdog firmware (8 s) arrete le robot.
Quitter avec 'q' de preference.
"""
import argparse
import math
import sys
import threading
import time

import serial

HDR = b'\xfa\xfb'
ARG_POS, ARG_NEG = 0x3B, 0x1B        # direction encodee par le type (convention du driver)
TH_UNIT_DEG = 360.0 / 4096.0         # THPOS : 2*pi/4096 rad par unite
SIP_PERIOD = 0.1                     # SipCycle du CONFIGpac (100 ms)


def aria_checksum(body: bytes) -> int:
    c, i, n = 0, 0, len(body)
    while n > 1:
        c = (c + ((body[i] << 8) | body[i + 1])) & 0xFFFF
        i += 2
        n -= 2
    if n > 0:
        c ^= body[i]
    return c & 0xFFFF


def build_cmd(cmd: int, arg_type=None, val=None) -> bytes:
    body = bytearray([cmd & 0xFF])
    if arg_type is not None:
        body += bytes([arg_type, val & 0xFF, (val >> 8) & 0xFF])
    chk = aria_checksum(body)
    return bytes([0xFA, 0xFB, len(body) + 2]) + bytes(body) + bytes([chk >> 8, chk & 0xFF])


def s16(f: bytes, o: int) -> int:
    return int.from_bytes(f[o:o + 2], 'little', signed=True)


class GyroTest:
    def __init__(self, a):
        self.a = a
        self.ser = serial.Serial(a.port, 9600, bytesize=8, parity='N', stopbits=1,
                                 timeout=0.2, xonxoff=False, rtscts=False)
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        self.lock = threading.Lock()
        self.running = True
        self.prev_th = None
        self.prev_t = None
        self.cumul = 0.0
        self.rot_start = None
        self.t0 = time.monotonic()

    # ------------------------------------------------------------ Liaison
    def send(self, frame: bytes):
        with self.lock:
            self.ser.write(frame)
            self.ser.flush()

    def connect(self) -> bool:
        self.ser.reset_input_buffer()
        for i in range(3):                      # SYNC0, SYNC1, SYNC2
            self.send(build_cmd(i))
            time.sleep(0.5)
            rx = self.ser.read(256)
            print(f'SYNC{i} : {len(rx)} octets recus')
            if not rx:
                print('Pas de reponse : robot eteint, mauvais port, ou port deja ouvert ?')
                return False
        time.sleep(0.5)
        self.send(build_cmd(1))                 # OPEN
        time.sleep(0.5)
        self.send(build_cmd(4, ARG_POS, 1))     # ENABLE 1
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        print('Connecte, moteurs actives (bouton MOTORS du robot requis).')
        return True

    # ------------------------------------------------------------ Reception
    def reader(self):
        buf = bytearray()
        while self.running:
            try:
                buf += self.ser.read(256)
            except serial.SerialException as e:
                print(f'Erreur serie : {e}')
                self.running = False
                break
            while True:
                i = buf.find(HDR)
                if i < 0:
                    del buf[:-1]                # garde un eventuel 0xFA en fin de tampon
                    break
                del buf[:i]
                if len(buf) < 3:
                    break
                n = 3 + buf[2]
                if len(buf) < n:
                    break
                f = bytes(buf[:n])
                del buf[:n]
                if aria_checksum(f[3:-2]) != ((f[-2] << 8) | f[-1]):
                    continue
                if f[3] in (0x32, 0x33) and len(f) >= 33:
                    self.on_sip(f)

    def on_sip(self, f: bytes):
        now = time.monotonic()
        th, lvel, rvel = s16(f, 8), s16(f, 10), s16(f, 12)
        rotvel = s16(f, 31) / 10.0
        wheels = math.degrees((rvel - lvel) / 1000.0 / self.a.track)
        dth = float('nan')
        if self.prev_th is not None:
            d = ((th - self.prev_th + 2048) % 4096) - 2048      # gere le passage +-180
            # Periode NOMINALE entre deux SIP : ils arrivent souvent par
            # paires dans la meme lecture serie, l'ecart d'arrivee (~0) ferait
            # exploser la derivee (valeurs a +13 000 observees le 2026-09-28).
            dth = d * TH_UNIT_DEG / SIP_PERIOD
            self.cumul += d * TH_UNIT_DEG
        self.prev_th, self.prev_t = th, now
        th_deg = (((th + 2048) % 4096) - 2048) * TH_UNIT_DEG   # affiche dans +-180
        print(f'{now - self.t0:6.1f}s  TH={th_deg:+7.1f}  cumul={self.cumul:+7.1f}  '
              f'dTH={dth:+6.1f}  ROTVEL={rotvel:+6.1f}  roues={wheels:+6.1f}  (deg, deg/s)',
              flush=True)

    # ------------------------------------------------------------ Commandes
    def rotate(self, sign: int):
        self.send(build_cmd(21, ARG_POS if sign > 0 else ARG_NEG, self.a.rvel))
        self.rot_start = time.monotonic()
        print(f'>>> Rotation {"CCW (+)" if sign > 0 else "CW (-)"} a {self.a.rvel} deg/s '
              f'(arret auto apres {self.a.max_time:.0f} s)')

    def stop(self):
        self.send(build_cmd(21, ARG_POS, 0))    # RVEL 0
        self.send(build_cmd(11, ARG_POS, 0))    # VEL 0 (STOP non fiable selon firmware)
        self.rot_start = None
        print('>>> Arret')

    def keyboard(self):
        while self.running:
            try:
                line = sys.stdin.readline()
            except (EOFError, KeyboardInterrupt):
                line = ''
            if line == '':                      # stdin ferme
                self.running = False
                break
            # Premiere lettre seulement : "qq" ou "q " valent 'q'
            # (le 2026-09-28, "qq" etait ignore et le robot continuait).
            k = line.strip().lower()[:1]
            if k == '+':
                self.rotate(+1)
            elif k == '-':
                self.rotate(-1)
            elif k == 's':
                self.stop()
            elif k == 'z':
                self.cumul = 0.0
                print('>>> Cumul remis a zero')
            elif k == 'q':
                self.running = False

    # ------------------------------------------------------------ Boucle
    def run(self):
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.keyboard, daemon=True).start()
        try:
            while self.running:
                self.send(build_cmd(0))         # PULSE
                if self.rot_start and time.monotonic() - self.rot_start > self.a.max_time:
                    print('>>> Duree max atteinte')
                    self.stop()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.running = False
        finally:
            self.shutdown()

    def shutdown(self):
        try:
            self.stop()
            self.send(build_cmd(4, ARG_POS, 0))  # ENABLE 0
            time.sleep(0.2)
            self.send(build_cmd(2))              # CLOSE
            time.sleep(0.2)
        except serial.SerialException as e:
            print(f'Erreur a la fermeture : {e}')
        self.ser.close()
        print(f'Ferme. Cumul final : {self.cumul:+.1f} deg')


def main():
    p = argparse.ArgumentParser(description='Test du gyro interne (SAG) du SeekurJR')
    p.add_argument('--port', default='/dev/seekur')
    p.add_argument('--rvel', type=int, default=15, help='deg/s, 30 max')
    p.add_argument('--track', type=float, default=0.663, help='voie en m (mesuree : 0.663)')
    p.add_argument('--max-time', type=float, default=40.0,
                   help='arret automatique d\'une rotation apres N s (defaut 40)')
    a = p.parse_args()
    if not 1 <= a.rvel <= 30:
        p.error('--rvel doit etre entre 1 et 30 deg/s')
    t = GyroTest(a)
    if not t.connect():
        t.ser.close()
        sys.exit(1)
    t.run()


if __name__ == '__main__':
    main()
