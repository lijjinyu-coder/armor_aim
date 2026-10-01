# -*- coding: utf-8 -*-
"""AimResult / 角度换算的单元测试（纯逻辑，无需 ROS 图）。

运行: python3 -m pytest src/prm_control/aim_output/test/test_aim_result.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from aim_output.aim_result import (  # noqa: E402
    aim_result_from_armor_xyz, gimbal_angles_from_world_direction,
    quaternion_to_matrix, rotate_vector, limit_rate_step, shortest_angle_delta,
)


def test_zero_solution_is_invalid():
    r = aim_result_from_armor_xyz(0.0, 0.0, 0.0)
    assert not r.target_valid
    assert r.distance_m == 0.0


def test_straight_ahead_3m():
    r = aim_result_from_armor_xyz(0.0, 0.0, 3000.0)
    assert r.target_valid
    assert abs(r.yaw_rel_deg) < 1e-9
    assert abs(r.pitch_rel_deg) < 1e-9
    assert abs(r.distance_m - 3.0) < 1e-9


def test_target_right_and_above():
    # 相机系: x 右 1000mm, y 上 -> -1000mm, z 前 1000mm
    r = aim_result_from_armor_xyz(1000.0, -1000.0, 1000.0)
    assert abs(r.yaw_rel_deg - 45.0) < 1e-6          # 右侧为正
    assert abs(r.pitch_rel_deg - 35.264) < 1e-2      # 向上为正
    assert abs(r.distance_m - math.sqrt(3.0)) < 1e-9


def test_world_direction_level_is_pitch_zero():
    # 命令 pitch = 目标仰角：水平方向 -> 0
    yaw, pitch = gimbal_angles_from_world_direction((1.0, 0.0, 0.0))
    assert abs(yaw) < 1e-9
    assert abs(pitch) < 1e-9


def test_world_direction_left_is_positive_yaw():
    yaw, _ = gimbal_angles_from_world_direction((0.0, 1.0, 0.0))
    assert abs(yaw - 90.0) < 1e-9


def test_world_direction_up_is_positive_elevation():
    # 45 度仰角 -> 命令 pitch = +45（实测标定：pitch_cmd 等于仰角）
    _, pitch = gimbal_angles_from_world_direction((1.0, 0.0, 1.0))
    assert abs(pitch - 45.0) < 1e-6


def test_world_direction_down_is_negative_elevation():
    _, pitch = gimbal_angles_from_world_direction((1.0, 0.0, -1.0))
    assert abs(pitch + 45.0) < 1e-6


def test_identity_rotation_keeps_vector():
    m = quaternion_to_matrix(0.0, 0.0, 0.0, 1.0)
    assert rotate_vector(m, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_yaw_90_rotation_maps_x_to_y():
    # 绕 Z 轴 90 度: x 前 -> y 左
    s = math.sin(math.radians(45.0))
    m = quaternion_to_matrix(0.0, 0.0, s, s)
    x, y, z = rotate_vector(m, (1.0, 0.0, 0.0))
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(z) < 1e-9


# ---------------- 指令速率限制 ----------------

def test_shortest_angle_delta_wraps():
    assert abs(shortest_angle_delta(10.0, 350.0) - 20.0) < 1e-9
    assert abs(shortest_angle_delta(350.0, 10.0) + 20.0) < 1e-9


def test_rate_limit_clamps_yaw_step():
    # 60 deg/s, dt=0.05 -> 最多 3 度
    y, p = limit_rate_step(0.0, 0.0, 90.0, 90.0, 60.0, 45.0, 0.05)
    assert abs(y - 3.0) < 1e-9
    assert abs(p - 2.25) < 1e-9


def test_rate_limit_passes_small_step_through():
    y, p = limit_rate_step(0.0, 0.0, 1.0, -1.0, 60.0, 45.0, 0.05)
    assert abs(y - 1.0) < 1e-9
    assert abs(p + 1.0) < 1e-9


def test_rate_limit_disabled_when_zero():
    y, p = limit_rate_step(0.0, 0.0, 120.0, -80.0, 0.0, 0.0, 0.05)
    assert abs(y - 120.0) < 1e-9
    assert abs(p + 80.0) < 1e-9


def test_rate_limit_takes_short_way_around_for_yaw():
    # 350 -> 10 度：最近方向是 +20 度，限速 3 度/步后应为 353
    y, _ = limit_rate_step(350.0, 0.0, 10.0, 0.0, 60.0, 0.0, 0.05)
    assert abs(y - 353.0) < 1e-9
