"""Tests du parseur avec des lignes réelles du banc (2026-09)."""

import math

import pytest

from bno055_imu.frame_parser import FrameError, parse_line

REST = 'I,5783,16371,652,-6,-3,0,0,-1,0,74,940,48*58'
BOOT_EMPTY = 'I,0,0,0,0,0,0,0,0,0,0,0,0*49'
ROTATING = 'I,300,-7318,59,294,14656,5,101,1738,21,-69,943,0*7E'


def test_rest_frame_values():
    f = parse_line(REST)
    assert f.seq == 5783
    assert f.quat_norm == pytest.approx(1.0, abs=1e-3)
    assert f.accel_ms2 == pytest.approx((0.0, 0.74, 9.40))
    assert f.gyro_rads[2] == pytest.approx(-math.radians(1 / 16))
    assert (f.cal_sys, f.cal_gyr, f.cal_acc, f.cal_mag) == (0, 3, 0, 0)


def test_rotation_ccw_gives_positive_gz():
    f = parse_line(ROTATING)
    assert f.gyro_rads[2] == pytest.approx(math.radians(1738 / 16))
    assert f.gyro_rads[2] > 0


def test_boot_frame_parses_but_null_quaternion():
    f = parse_line(BOOT_EMPTY)
    assert f.quat_norm == 0.0   # rejetée ensuite par le nœud (tolérance de norme)


def test_bad_checksum():
    with pytest.raises(FrameError) as e:
        parse_line(REST.replace('*58', '*59'))
    assert e.value.reason == 'checksum'


def test_truncated_line():
    with pytest.raises(FrameError) as e:
        parse_line('I,78,16378,442,130,-3,0,2,5,-18,52,94')
    assert e.value.reason == 'format'


def test_garbage_and_info_lines():
    for line in ['# READY rate_hz=100 offsets_eeprom=oui', '', 'xx*zz']:
        with pytest.raises(FrameError):
            parse_line(line)
