import math

import numpy as np

from trc_localization import fake_terrain as ft
from trc_localization import fake_world as fw


def test_flat_terrain_is_old_ripple():
    t = ft.Terrain('flat', 0.3)
    e, n = np.array([1.0, -3.0]), np.array([2.0, 5.0])
    assert np.allclose(t.height(e, n), ft.ripple(e, n))
    assert t.flat


def test_pose_on_slope_pitches_nose_up_when_climbing():
    t = ft.Terrain('dunes', math.radians(90.0))   # map x = 北
    z, pitch, roll = t.pose(0.0, 3.5, math.radians(90.0))   # 砂丘（map (6, 0)）の手前で北を向く
    assert pitch < -math.radians(3.0)             # 機首上げ（ROS では負）
    assert z > 0.1


def test_rotation_and_gravity_reading():
    rot = ft.rotation(0.4, math.radians(10.0), math.radians(-5.0))
    acc = rot.T @ np.array([0.0, 0.0, 9.81])
    roll = math.atan2(acc[1], acc[2])
    pitch = math.atan2(-acc[0], math.hypot(acc[1], acc[2]))
    assert math.isclose(roll, math.radians(-5.0), abs_tol=1e-9)
    assert math.isclose(pitch, math.radians(10.0), abs_tol=1e-9)


def test_slip_grows_uphill_and_in_soft_sand():
    p = ft.SandParams(slip=0.1, soft_sand=[(7.0, 0.0, 1.5)])
    assert math.isclose(ft.slip_ratio(p, 0.5, 0.0, -1.0), 0.1)
    assert ft.slip_ratio(p, 0.5, -math.radians(16.0), -1.0) > 0.4
    assert ft.slip_ratio(p, 0.5, -math.radians(25.0), -1.0) == 1.0
    assert ft.slip_ratio(p, -0.5, -math.radians(25.0), -1.0) == 0.1      # 下り（後退）は空転しない
    assert ft.slip_ratio(p, 0.5, 0.0, 0.6) == 1.0                      # やわらかい砂の奥では進めない
    assert ft.slip_ratio(p, -0.3, 0.0, 0.6) == p.reverse_slip           # 後ろへは抜け出せる
    assert ft.soft_depth_in(p, 6.0, 0.0) > 0.0 > ft.soft_depth_in(p, 9.0, 0.0)


def test_pit_hides_its_floor_from_a_distance():
    t = ft.Terrain('pit', 0.0)                    # map = world
    ground = fw.make_ground_on(t)
    floor = ground[np.hypot(ground[:, 0] - 8.0, ground[:, 1]) < 0.8]
    sensor = np.array([4.0, 0.0, float(t.height(4.0, 0.0)) + 0.4])
    assert t.occluded(sensor, floor).mean() > 0.9
    assert t.in_pit(8.0, 0.0) and not t.in_pit(5.0, 0.0)


def test_dust_lives_one_or_two_scans():
    d = ft.Dust(rate=2.0, terrain=ft.Terrain('flat'), seed=1)
    counts = [len(d.step(0.0, 0.0)) for _ in range(50)]
    assert max(counts) > 0
    d.rate = 0.0
    d.step(0.0, 0.0)
    d.step(0.0, 0.0)
    assert len(d.step(0.0, 0.0)) == 0


def test_scan_3d_level_matches_old_scan_geometry():
    rng = np.random.default_rng(0)
    rock = fw.make_rock(5.0, 0.0)
    t = ft.Terrain('flat')
    cloud = fw.scan_3d(rock, np.empty((0, 3)), t, np.zeros(3), ft.rotation(0.0, 0.0, 0.0),
                       0.2, 0.4, 0.5, 25.0, 10000, 0.0, rng)
    assert np.isclose(cloud[:, 0].min(), 5.0 - 0.4 - 0.2, atol=0.02)
    assert np.isclose(cloud[:, 2].max(), 0.5 - 0.4, atol=0.02)
