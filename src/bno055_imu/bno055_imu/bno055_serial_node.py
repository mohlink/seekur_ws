#!/usr/bin/env python3
"""
bno055_serial_node.py — Pont série BNO055 (Arduino) -> ROS2.

Lit les trames du sketch bno055_imu_bridge et publie :
  imu/data     sensor_msgs/Imu   (repère frame_id, unités SI, REP-103)
  diagnostics  diagnostic_msgs/DiagnosticArray

Trames rejetées : checksum faux, format invalide, quaternion de norme
hors tolérance (élimine les trames vides du démarrage du BNO055).
Reconnexion automatique si le port disparaît.

Arrêt (2026-09-29) : sous Jazzy, le Ctrl+C ferme le contexte ROS avant que
shutdown() arrête le thread de lecture. Le thread vérifie donc rclpy.ok()
et intercepte l'échec de publication sur contexte fermé (plus de trace
d'erreur « publisher's context is invalid » à l'arrêt).
"""

import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from bno055_imu.frame_parser import FrameError, parse_line

SEQ_RESYNC_THRESHOLD = 1000   # saut de séquence au-delà = reset de l'Arduino, pas des pertes


def diag3(values):
    """Liste de 3 variances -> matrice de covariance 3x3 aplatie (9 éléments)."""
    return [float(values[0]), 0.0, 0.0,
            0.0, float(values[1]), 0.0,
            0.0, 0.0, float(values[2])]


class Bno055SerialNode(Node):

    def __init__(self):
        super().__init__('bno055_imu')

        self.declare_parameter('serial_port', '/dev/imu')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('expected_rate_hz', 100.0)
        self.declare_parameter('quat_norm_tolerance', 0.05)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('data_timeout_s', 1.0)
        self.declare_parameter('diagnostics_rate_hz', 1.0)
        self.declare_parameter('orientation_covariance_diag', [3.0e-4, 3.0e-4, 1.2e-3])
        self.declare_parameter('angular_velocity_covariance_diag', [3.0e-5, 3.0e-5, 3.0e-5])
        self.declare_parameter('linear_acceleration_covariance_diag', [1.0e-2, 1.0e-2, 1.0e-2])
        self.declare_parameter('accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('accel_scale', [1.0, 1.0, 1.0])

        p = self.get_parameter
        self.port = p('serial_port').value
        self.baud = int(p('baud').value)
        self.frame_id = p('frame_id').value
        self.expected_rate = float(p('expected_rate_hz').value)
        self.quat_tol = float(p('quat_norm_tolerance').value)
        self.reconnect_period = float(p('reconnect_period_s').value)
        self.data_timeout = float(p('data_timeout_s').value)
        self.cov_orient = diag3(p('orientation_covariance_diag').value)
        self.cov_gyro = diag3(p('angular_velocity_covariance_diag').value)
        self.cov_accel = diag3(p('linear_acceleration_covariance_diag').value)
        self.accel_bias = [float(v) for v in p('accel_bias').value]
        self.accel_scale = [float(v) for v in p('accel_scale').value]

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # Statistiques (partagées entre le thread de lecture et le timer)
        self._lock = threading.Lock()
        self._stats = dict(published=0, rej_checksum=0, rej_format=0,
                           rej_quat=0, seq_gaps=0, resyncs=0, reconnects=0)
        self._window_count = 0
        self._window_start = time.monotonic()
        self._last_valid = 0.0
        self._last_seq = None
        self._cal = (0, 0, 0, 0)
        self._connected = False

        self._ser = None
        self._running = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        diag_rate = float(p('diagnostics_rate_hz').value)
        self.create_timer(1.0 / diag_rate, self._publish_diagnostics)

        self.get_logger().info(
            f'bno055_imu : port={self.port} baud={self.baud} frame_id={self.frame_id}')

    # ------------------------------------------------------------- Série
    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self._ser.reset_input_buffer()
            with self._lock:
                self._connected = True
                self._last_seq = None
            self.get_logger().info(f'Port {self.port} ouvert (la Nano redémarre, ~2 s)')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(
                f'Ouverture de {self.port} impossible : {e} — nouvel essai dans '
                f'{self.reconnect_period:.0f} s', throttle_duration_sec=10.0)
            self._ser = None
            time.sleep(self.reconnect_period)

    def _close_serial(self):
        with self._lock:
            self._connected = False
        if self._ser is not None:
            try:
                self._ser.close()
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f'Fermeture du port : {e}')
        self._ser = None

    def _reader_loop(self):
        while self._running and rclpy.ok():
            if self._ser is None:
                self._open_serial()
                continue
            try:
                raw = self._ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Lecture série échouée : {e} — reconnexion')
                self._close_serial()
                with self._lock:
                    self._stats['reconnects'] += 1
                time.sleep(self.reconnect_period)
                continue
            if raw:
                self._handle_line(raw.decode('ascii', errors='replace').strip())

    # ------------------------------------------------------------- Trames
    def _handle_line(self, line: str):
        if not line:
            return

        # Octets parasites possibles en début de ligne (reset de la Nano)
        i_msg, i_frame = line.find('#'), line.find('I,')
        if i_msg >= 0 and (i_frame < 0 or i_msg < i_frame):
            self._handle_info(line[i_msg:])
            return
        if i_frame > 0:
            line = line[i_frame:]

        try:
            frame = parse_line(line)
        except FrameError as e:
            with self._lock:
                key = 'rej_checksum' if e.reason == 'checksum' else 'rej_format'
                self._stats[key] += 1
            self.get_logger().debug(f'Trame rejetée ({e})')
            return

        if abs(frame.quat_norm - 1.0) > self.quat_tol:
            with self._lock:
                self._stats['rej_quat'] += 1
            return

        self._track_sequence(frame.seq)
        self._publish_imu(frame)

    def _handle_info(self, line: str):
        if line.startswith('# READY'):
            with self._lock:
                self._last_seq = None
            self.get_logger().info(f'Arduino : {line[2:]}')
        elif line.startswith(('# ERR', '# WARN')):
            self.get_logger().warn(f'Arduino : {line[2:]}')
        else:
            self.get_logger().info(f'Arduino : {line[2:]}')

    def _track_sequence(self, seq: int):
        with self._lock:
            if self._last_seq is not None:
                diff = (seq - self._last_seq) & 0xFFFF
                if diff == 0 or diff >= SEQ_RESYNC_THRESHOLD:
                    self._stats['resyncs'] += 1
                elif diff > 1:
                    self._stats['seq_gaps'] += diff - 1
            self._last_seq = seq

    def _publish_imu(self, f):
        w, x, y, z = (c / f.quat_norm for c in f.quat_wxyz)
        ax, ay, az = ((f.accel_ms2[i] - self.accel_bias[i]) * self.accel_scale[i]
                      for i in range(3))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.w, msg.orientation.x = w, x
        msg.orientation.y, msg.orientation.z = y, z
        msg.orientation_covariance = self.cov_orient
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = f.gyro_rads
        msg.angular_velocity_covariance = self.cov_gyro
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = self.cov_accel
        try:
            self.imu_pub.publish(msg)
        except RCLError:
            # Contexte ROS ferme par le Ctrl+C pendant la lecture : on s'arrete.
            self._running = False
            return

        with self._lock:
            self._stats['published'] += 1
            self._window_count += 1
            self._last_valid = time.monotonic()
            self._cal = (f.cal_sys, f.cal_gyr, f.cal_acc, f.cal_mag)

    # ------------------------------------------------------------- Diagnostics
    def _publish_diagnostics(self):
        now = time.monotonic()
        with self._lock:
            elapsed = now - self._window_start
            rate = self._window_count / elapsed if elapsed > 0 else 0.0
            self._window_count, self._window_start = 0, now
            stale = (now - self._last_valid) > self.data_timeout
            stats, cal, connected = dict(self._stats), self._cal, self._connected

        st = DiagnosticStatus()
        st.name = 'bno055_imu'
        st.hardware_id = f'BNO055@{self.port}'
        if not connected:
            st.level, st.message = DiagnosticStatus.ERROR, 'Port série non connecté'
        elif stale:
            st.level, st.message = DiagnosticStatus.ERROR, 'Aucune trame valide récente'
        elif rate < 0.8 * self.expected_rate:
            st.level, st.message = DiagnosticStatus.WARN, f'Fréquence basse ({rate:.0f} Hz)'
        else:
            st.level, st.message = DiagnosticStatus.OK, 'OK'

        st.values = [
            KeyValue(key='rate_hz', value=f'{rate:.1f}'),
            KeyValue(key='calib_sys_gyr_acc_mag', value='/'.join(map(str, cal))),
        ] + [KeyValue(key=k, value=str(v)) for k, v in stats.items()]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------- Arrêt
    def shutdown(self):
        self._running = False
        self._reader.join(timeout=2.0)
        self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = Bno055SerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
