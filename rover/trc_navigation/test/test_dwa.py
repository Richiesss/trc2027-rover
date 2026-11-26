import math

import numpy as np

from trc_navigation import dwa


def rock(cx, cy, size=0.8, step=0.05):
    xs = np.arange(cx - size / 2, cx + size / 2 + 1e-9, step)
    ys = np.arange(cy - size / 2, cy + size / 2 + 1e-9, step)
    return np.array([(x, y) for x in xs for y in ys])


def simulate(obstacles_map, goal_map, steps=400, dt=0.1, p=dwa.DwaParams()):
    """DWA の指令どおりに動かして、(最終位置, 最小の余裕, 前に進めなかった回数) を返す。"""
    x = y = yaw = 0.0
    min_clear = float('inf')
    stuck = 0
    for _ in range(steps):
        obs = dwa.to_base(x, y, yaw, obstacles_map)
        g = dwa.to_base(x, y, yaw, np.array([goal_map]))[0]
        if math.hypot(*g) < 0.5:
            break
        pl = dwa.plan(obs, g, p)
        if not pl.feasible:
            stuck += 1
            yaw += 0.3 * dt
            continue
        yaw += pl.w * dt
        x += pl.v * math.cos(yaw) * dt
        y += pl.v * math.sin(yaw) * dt
        d = np.hypot(obstacles_map[:, 0] - x, obstacles_map[:, 1] - y).min()
        min_clear = min(min_clear, d)
    return (x, y), min_clear, stuck


def test_rollout_straight_and_turn():
    t = dwa.rollout(0.5, 0.0, 2.0, 0.5)
    assert np.allclose(t[-1], [1.0, 0.0, 0.0])
    t = dwa.rollout(0.5, 0.5, 2.0 * math.pi, 0.1)  # 半径 1 m で半周（π / 0.5 秒）
    assert np.allclose(t[-1, :2], [0.0, 2.0], atol=0.05)


def test_no_obstacle_goes_straight():
    pl = dwa.plan(np.empty((0, 2)), (10.0, 0.0))
    assert pl.feasible and pl.v == dwa.DwaParams().max_speed and abs(pl.w) < 1e-9


def test_turns_toward_goal():
    pl = dwa.plan(np.empty((0, 2)), (0.0, 5.0))    # 左にある目標
    assert pl.w > 0.0


def test_rock_ahead_steers_away():
    pl = dwa.plan(rock(3.0, 0.0), (10.0, 0.0))
    assert pl.feasible and abs(pl.w) > 0.05        # まっすぐは行かない


def test_wall_ahead_does_not_drive_into_it():
    wall = np.array([(1.5, y) for y in np.arange(-5, 5, 0.05)])
    pl = dwa.plan(wall, (10.0, 0.0))
    # 進めるとしても、壁に robot_radius + safety_margin より近づかない
    p = dwa.DwaParams()
    assert not pl.feasible or pl.clearance >= p.robot_radius + p.safety_margin


def test_surrounded_not_feasible():
    th = np.linspace(0, 2 * math.pi, 200)
    ring = np.c_[0.6 * np.cos(th), 0.6 * np.sin(th)]
    assert not dwa.plan(ring, (10.0, 0.0)).feasible


def test_reaches_goal_around_rock():
    obstacles = rock(4.0, 0.0)
    (x, y), min_clear, _ = simulate(obstacles, (10.0, 0.0))
    p = dwa.DwaParams()
    assert math.hypot(x - 10.0, y) < 0.5
    assert min_clear > p.robot_radius              # ぶつかっていない


def test_reaches_goal_through_gap():
    """2 つの岩の間（すき間 2.4 m）を抜けて目標へ。"""
    obstacles = np.vstack([rock(5.0, 1.6), rock(5.0, -1.6), rock(8.0, 0.5)])
    (x, y), min_clear, _ = simulate(obstacles, (12.0, 0.0))
    assert math.hypot(x - 12.0, y) < 0.5
    assert min_clear > dwa.DwaParams().robot_radius


def test_thin():
    pts = np.array([[0.01, 0.01], [0.02, 0.02], [1.0, 1.0]])
    assert len(dwa.thin(pts, 0.15)) == 2


def test_parse_waypoints():
    from trc_navigation.local_planner_node import parse_waypoints
    assert parse_waypoints('15,0; 15, 8 ;') == [(15.0, 0.0), (15.0, 8.0)]
    assert parse_waypoints('') == []
