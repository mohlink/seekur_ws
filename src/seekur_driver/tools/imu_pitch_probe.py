#!/usr/bin/env python3
"""imu_pitch_probe.py - affiche roulis / tangage / cap depuis /imu/data (5 Hz).

Marche en simulation (IMU Gazebo) comme sur le vrai robot (BNO055).
Sert a mesurer le basculement du chassis (cf. MEMO.md, Modele de simulation).
Pas de dependance a tf_transformations (absent du ROG).

Usage : python3 imu_pitch_probe.py [topic]   (defaut /imu/data)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/imu/data'
    rclpy.init()
    node = Node('imu_pitch_probe')
    count = [0]

    def cb(m):
        count[0] += 1
        if count[0] % 20:
            return
        x, y, z, w = m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        print(f"roulis={math.degrees(roll):6.2f}  tangage={math.degrees(pitch):6.2f}"
              f"  cap={math.degrees(yaw):7.2f}", flush=True)

    node.create_subscription(Imu, topic, cb, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
