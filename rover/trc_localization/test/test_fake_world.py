import math

import numpy as np
import pytest

from trc_localization import fake_world as fw


def test_world_to_sensor_point_ahead():
    # ローバーが北（yaw 90 度）を向いて原点にいる。LiDAR は 0.2 m 前・0.4 m 上
    # 北に 5 m・高さ 1 m の点は、LiDAR から見て前に 4.8 m、上に 0.6 m
    p = np.array([[0.0, 5.0, 1.0]])
    s = fw.world_to_sensor(p, 0.0, 0.0, math.pi / 2, lidar_x=0.2, lidar_z=0.4)
    assert s[0] == pytest.approx([4.8, 0.0, 0.6], abs=1e-9)


def test_world_to_sensor_point_on_left():
    # 東向き（yaw 0）のとき、北にある点は左（+y）に見える
    p = np.array([[0.0, 3.0, 0.4]])
    s = fw.world_to_sensor(p, 0.0, 0.0, 0.0, lidar_x=0.0, lidar_z=0.4)
    assert s[0] == pytest.approx([0.0, 3.0, 0.0], abs=1e-9)


def test_world_keeps_driving_area_clear():
    world = fw.make_world(seed=1)
    assert len(world) > 30000
    # 物体の中心は走行範囲の外。表面の点がはみ出すのは物体の大きさ分（壁で最大 7.5 m 程度）まで
    inside = ((world[:, 0] > -2.0) & (world[:, 0] < -1.0) & (world[:, 1] > 3.0) & (world[:, 1] < 4.0))
    assert inside.sum() < 0.01 * len(world)


def test_scan_respects_range_and_size():
    rng = np.random.default_rng(0)
    world = fw.make_world(seed=2)
    ground = fw.make_ground(seed=2, n=20000)
    cloud = fw.scan(world, ground, 0.0, 0.0, 0.0, 0.2, 0.4,
                    min_range=0.5, max_range=25.0, max_points=5000, noise=0.0, rng=rng,
                    ground_points=1000)
    assert cloud.dtype == np.float32
    assert len(cloud) <= 5000 + 1000
    d = np.linalg.norm(cloud, axis=1)
    assert d.min() >= 0.5 and d.max() <= 25.0


def test_ground_is_fixed_in_world_not_to_sensor():
    # ローバーが 1 m 進むと、地面の点群も変わる（変わらないと LiDAR オドメトリが止まって見える）
    ground = fw.make_ground(seed=3, n=5000)
    a = fw.world_to_sensor(ground, 0.0, 0.0, 0.0, 0.2, 0.4)
    b = fw.world_to_sensor(ground, 1.0, 0.0, 0.0, 0.2, 0.4)
    assert np.allclose(b[:, 0], a[:, 0] - 1.0)
    assert abs(ground[:, 2]).max() <= 0.08 * 1.5 + 1e-9
