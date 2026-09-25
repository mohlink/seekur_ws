# bno055_imu

Pont série entre un BNO055 lu par une Arduino Nano (sketch `firmware/bno055_imu_bridge`)
et ROS2 Jazzy. Publie `sensor_msgs/Imu` sur `imu/data` (repère `imu_link`, REP-103)
et `diagnostic_msgs/DiagnosticArray` sur `diagnostics`.

## Matériel validé (2026-09)
- Arduino Nano ATmega328P (clone, « Old Bootloader »), BNO055 en I2C sur A4/A5
- Sketch : mode IMUPLUS (sans magnétomètre), 115200 baud, 100 Hz, calibration en EEPROM
- Rotation anti-horaire à plat -> gz > 0, yaw croissant (REP-103, aucune inversion)
- Accéléro : biais résiduel ~ -0.29 m/s² sur Z, échelle ~0.99 (accel non fusionnée)

## Build (depuis ~/seekur_ws)
    colcon build --packages-select bno055_imu
    source install/setup.bash

Pas de `--symlink-install` (même raison que seekur_driver).

## Lancement
    ros2 launch bno055_imu bno055.launch.py

Le port série se règle dans `config/bno055_params.yaml` (rebuild ensuite).

## Tests du parseur
    cd src/bno055_imu && python3 -m pytest test/
