import math

import numpy as np

from trc_localization import fake_world
from trc_perception import obstacle_math as om


def flat_ground(rng, n=20000, half=10.0, z=0.0):
    xy = rng.uniform(-half, half, (n, 2))
    return np.c_[xy, np.full(n, z)]


def test_box_on_flat_ground():
    rng = np.random.default_rng(0)
    ground = flat_ground(rng)
    box = np.c_[rng.uniform(3.0, 3.5, 500), rng.uniform(-0.3, 0.3, 500), rng.uniform(0.2, 0.8, 500)]
    obs, gnd = om.segment(np.vstack([ground, box]))
    assert obs[len(ground):].mean() > 0.99      # 箱はほぼ全部が障害物
    assert obs[:len(ground)].mean() < 0.001     # 地面はほとんど障害物にならない
    assert gnd[:len(ground)].mean() > 0.95


def test_ground_height_offset_does_not_matter():
    """地面の高さ（センサの取り付け高さの誤差など）がずれていても、地面は地面と判定する。"""
    rng = np.random.default_rng(1)
    obs, _ = om.segment(flat_ground(rng, z=-0.3))
    assert obs.mean() < 0.001


def test_self_points_and_high_points_ignored():
    pts = np.array([[0.1, 0.0, 0.5],    # ローバー自身
                    [3.0, 0.0, 5.0],    # 高すぎる
                    [3.0, 0.0, 0.0]])
    obs, gnd = om.segment(pts)
    assert not obs[0] and not gnd[0]
    assert not obs[1]


def test_front_distance():
    pts = np.array([[2.0, 0.2, 0.5], [1.5, 2.0, 0.5], [4.0, -0.4, 0.3], [-1.0, 0.0, 0.5]])
    assert math.isclose(om.front_distance(pts, corridor_width=1.0, max_distance=5.0), 2.0)
    assert math.isclose(om.front_distance(pts, 1.0, 5.0, front_offset=0.5), 1.5)
    assert om.front_distance(pts, corridor_width=1.0, max_distance=1.0) == float('inf')
    assert om.front_distance(np.empty((0, 3)), 1.0, 5.0) == float('inf')


def test_transform_points():
    rot = om.quaternion_to_matrix(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # 左に 90 度
    p = om.transform_points(np.array([[1.0, 0.0, 0.0]]), rot, (0.2, 0.0, 0.4))
    assert np.allclose(p, [[0.2, 1.0, 0.4]])


def test_fake_world_scans():
    """仮想フィールドの点群で、正解（物体か地面か）と比べる。"""
    objects = fake_world.make_world(seed=0)
    ground = fake_world.make_ground(seed=0)
    rng = np.random.default_rng(2)
    lidar_x, lidar_z = 0.2, 0.4
    recalls, false_rates = [], []
    for e, n, yaw in [(0.0, 0.0, math.pi / 2), (0.0, 6.0, math.pi / 2), (-3.3, 9.3, math.pi), (-6.6, 6.0, -math.pi / 2)]:
        o = fake_world._visible(objects, e, n, yaw, lidar_x, lidar_z, 0.5, 25.0, 8000, rng)
        g = fake_world._visible(ground, e, n, yaw, lidar_x, lidar_z, 0.5, 25.0, 3000, rng)
        cloud = np.vstack([o, g]) + rng.normal(0.0, 0.02, (len(o) + len(g), 3))
        base = cloud + np.array([lidar_x, 0.0, lidar_z])          # LiDAR 座標 → base_link
        params = om.SegmentParams()
        obs, _ = om.segment(base, params)
        # 物体の点のうち、検出する距離の内側で、地面から 0.25 m 以上の高さのもの
        # （低い所は地面と区別できなくてよい）
        near = np.hypot(base[:len(o), 0], base[:len(o), 1]) <= params.max_range
        tall = (o[:, 2] + lidar_z > 0.25) & near
        recalls.append(obs[:len(o)][tall].mean())
        false_rates.append(obs[len(o):].mean())
    assert min(recalls) > 0.95, recalls
    assert max(false_rates) < 0.01, false_rates


def test_safety_filter():
    from trc_perception.safety_stop_node import filter_command
    assert filter_command(0.5, 0.1, blocked=True) == (0.0, 0.1, True)     # 前進は止め、回転は通す
    assert filter_command(-0.3, 0.0, blocked=True) == (-0.3, 0.0, False)  # 後退は通す
    assert filter_command(0.5, 0.1, blocked=False) == (0.5, 0.1, False)
