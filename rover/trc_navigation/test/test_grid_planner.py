import math

import numpy as np

from trc_navigation import grid_planner as gp


def wall(x, y0, y1, step=0.05):
    ys = np.arange(y0, y1, step)
    return np.c_[np.full(len(ys), x), ys]


def grid_with(points, times=3):
    g = gp.ObstacleGrid()
    for _ in range(times):
        g.add(points)
    return g


def path_clearance(path, obstacles):
    return np.hypot(path[:, None, 0] - obstacles[None, :, 0], path[:, None, 1] - obstacles[None, :, 1]).min()


def test_straight_when_empty():
    path = gp.plan_path(gp.ObstacleGrid(), (0.0, 0.0), (10.0, 0.0))
    assert path is not None
    assert np.allclose(path[-1], (10.0, 0.0)) and np.abs(path[:, 1]).max() < 0.3


def test_goes_around_wall():
    obs = wall(5.0, -4.0, 4.0)
    path = gp.plan_path(grid_with(obs), (0.0, 0.0), (10.0, 0.0))
    assert path is not None
    assert path_clearance(path, obs) >= gp.GridParams().inflate - 0.2
    assert np.abs(path[:, 1]).max() > 4.0            # 壁の端を回っている


def test_noise_needs_repeated_hits():
    g = gp.ObstacleGrid()
    g.add(wall(5.0, -4.0, 4.0))                      # 1 回だけ見えた点はまだ障害物にしない
    path = gp.plan_path(g, (0.0, 0.0), (10.0, 0.0))
    assert np.abs(path[:, 1]).max() < 0.3


def test_goal_inside_obstacle_is_none():
    obs = np.array([(10.0 + dx, dy) for dx in np.arange(-0.5, 0.5, 0.05) for dy in np.arange(-0.5, 0.5, 0.05)])
    assert gp.plan_path(grid_with(obs), (0.0, 0.0), (10.0, 0.0)) is None


def test_start_next_to_obstacle_can_leave():
    obs = wall(0.4, -2.0, 2.0)                       # すぐ前に壁（太らせた範囲の中にいる）
    path = gp.plan_path(grid_with(obs), (0.0, 0.0), (-5.0, 0.0))
    assert path is not None


def test_lookahead():
    path = np.c_[np.linspace(0, 10, 101), np.zeros(101)]
    p = gp.lookahead_point(path, (3.0, 0.2), 2.0)
    assert math.isclose(p[0], 5.0, abs_tol=0.11)


def test_ground_clears_wrong_obstacles():
    obs = wall(5.0, -4.0, 4.0)
    g = grid_with(obs, times=5)
    assert g.occupied().sum() > 0
    ground = np.array([(x, y) for x in np.arange(4.5, 5.5, 0.1) for y in np.arange(-4, 4, 0.1)])
    for _ in range(10):
        g.clear(ground)
    assert g.occupied().sum() == 0


def test_clear_keeps_cells_with_obstacles_in_same_scan():
    obs = wall(5.0, -4.0, 4.0)
    g = grid_with(obs, times=5)
    ground = np.array([(x, y) for x in np.arange(4.5, 5.5, 0.1) for y in np.arange(-4, 4, 0.1)])
    for _ in range(10):
        g.add(obs)
        g.clear(ground, obs)
    assert g.occupied().sum() > 0


def test_robust_uses_nearby_free_goal():
    obs = np.array([(10.0 + dx, dy) for dx in np.arange(-0.5, 0.5, 0.05) for dy in np.arange(-0.5, 0.5, 0.05)])
    path, goal = gp.plan_path_robust(grid_with(obs), (0.0, 0.0), (10.0, 0.0), min_inflate=0.45)
    assert path is not None
    assert 0.3 < math.hypot(goal[0] - 10.0, goal[1]) <= 2.0


def test_hazard_when_watched_but_never_seen():
    g = gp.ObstacleGrid(gp.GridParams())
    pts = np.array([[3.0, 0.0]])
    for _ in range(g.p.watch_needed):
        g.watch(pts)
    assert g.occupied()[g.to_cell(3.0, 0.0)]
    g.clear(pts)                                          # 地面が見えたら穴ではない
    assert not g.occupied()[g.to_cell(3.0, 0.0)]


def test_blocked_area_is_avoided():
    g = gp.ObstacleGrid(gp.GridParams())
    g.block(5.0, 0.0, 1.0)
    path = gp.plan_path(g, (0.0, 0.0), (10.0, 0.0))
    assert path is not None
    d = np.hypot(path[:, 0] - 5.0, path[:, 1])
    assert d.min() > 1.0 + g.p.inflate - 0.3
    assert len(g.special_points(5.0, 0.0, 2.0)) > 10
