"""障害物をよけながら目標へ向かう速度指令を選ぶ（DWA: Dynamic Window Approach、ROS に依存しない）。

やること（指令を出すたびに）:
  1. 前進の速さ v と回転の速さ w の組み合わせを何通りも用意する
  2. それぞれ「その指令のまま horizon 秒走ったらどこを通るか」を予測する
  3. 次の候補は捨てる
       - 通り道が障害物に robot_radius + safety_margin より近づく
       - 行き着いた先で、前方の帯に stop_distance 以内の障害物がある（安全停止に引っかかる）
  4. 残りに点数を付けて一番よいものを選ぶ
       進み具合（目標に近づいた距離）・向き（目標の方を向いているか）・余裕（障害物との距離）・速さ

座標はすべて base_link（前 x・左 y、原点はいまのローバーの中心）。
障害物は (M, 2) の点、目標は base_link での (x, y)。
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class DwaParams:
    max_speed: float = 0.5        # 前進の最高速度 [m/s]
    max_yaw_rate: float = 0.6     # 回転の最高速度 [rad/s]
    speed_samples: int = 6        # v を 0〜max_speed で何通り試すか
    yaw_samples: int = 21         # w を -max〜+max で何通り試すか
    horizon: float = 4.0          # 何秒先まで予測するか [s]
    dt: float = 0.2               # 予測の刻み [s]
    robot_radius: float = 0.45    # ローバーを円とみなしたときの半径 [m]
    safety_margin: float = 0.25   # さらに空ける距離 [m]
    stop_distance: float = 1.5    # 安全停止の距離（行き着いた先でこれより近い障害物が前にあれば捨てる）[m]
    stop_margin: float = 0.2      # 安全停止に引っかからないための余裕 [m]
    corridor_width: float = 1.0   # 安全停止が見る前方の帯の幅 [m]
    front_offset: float = 0.4     # ローバー前端の x [m]
    clearance_cap: float = 2.0    # これ以上離れていれば余裕は満点 [m]
    w_progress: float = 1.0       # 点数の重み：進み具合
    w_heading: float = 0.5        # 　　　　　　向き
    w_clearance: float = 0.4      # 　　　　　　余裕
    w_speed: float = 0.2          # 　　　　　　速さ


@dataclass
class Plan:
    v: float
    w: float
    trajectory: np.ndarray        # (N, 3) x, y, yaw
    feasible: bool                # 前に進める候補があったか
    clearance: float


def rollout(v: float, w: float, horizon: float, dt: float) -> np.ndarray:
    """一定の (v, w) で走ったときの通り道 (N, 3)。最初の点は含めない。"""
    n = max(1, int(round(horizon / dt)))
    t = dt * np.arange(1, n + 1)
    yaw = w * t
    if abs(w) < 1e-6:
        x, y = v * t, np.zeros(n)
    else:
        x = v / w * np.sin(yaw)
        y = v / w * (1.0 - np.cos(yaw))
    return np.c_[x, y, yaw]


def front_blocked(obstacles: np.ndarray, pose, p: DwaParams, margin: float = None) -> bool:
    """pose (x, y, yaw) に立ったとき、前方の帯に stop_distance + margin 以内の障害物があるか。

    margin を省くと p.stop_margin。
    """
    if len(obstacles) == 0:
        return False
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = obstacles[:, 0] - x, obstacles[:, 1] - y
    fx = c * dx + s * dy
    fy = -s * dx + c * dy
    reach = p.front_offset + p.stop_distance + (p.stop_margin if margin is None else margin)
    return bool(np.any((fx >= p.front_offset) & (fx <= reach) & (np.abs(fy) <= p.corridor_width / 2.0)))


def plan(obstacles: np.ndarray, goal, p: DwaParams = DwaParams()) -> Plan:
    """いま出すべき (v, w) を選ぶ。前に進める候補がなければ feasible=False（v=0）。"""
    goal = np.asarray(goal, dtype=float)
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    d0 = float(np.hypot(*goal))
    reach = p.max_speed * p.horizon + p.robot_radius + p.clearance_cap
    if len(obstacles):
        obstacles = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) <= reach]

    best = None
    for v in np.linspace(p.max_speed / p.speed_samples, p.max_speed, p.speed_samples):
        for w in np.linspace(-p.max_yaw_rate, p.max_yaw_rate, p.yaw_samples):
            traj = rollout(v, w, p.horizon, p.dt)
            if len(obstacles):
                d = np.hypot(traj[:, None, 0] - obstacles[None, :, 0], traj[:, None, 1] - obstacles[None, :, 1])
                clearance = float(d.min())
                if clearance < p.robot_radius + p.safety_margin:
                    continue
                if front_blocked(obstacles, traj[-1], p):
                    continue
            else:
                clearance = p.clearance_cap + p.robot_radius
            end = traj[-1]
            # 目標を行き過ぎる候補は、目標に一番近づいた所で評価する
            dist = np.hypot(goal[0] - traj[:, 0], goal[1] - traj[:, 1])
            k = int(dist.argmin())
            progress = (d0 - dist[k]) / (p.max_speed * p.horizon)
            bearing = math.atan2(goal[1] - end[1], goal[0] - end[0]) - end[2]
            heading = 1.0 - abs(math.atan2(math.sin(bearing), math.cos(bearing))) / math.pi
            free = min(clearance - p.robot_radius, p.clearance_cap) / p.clearance_cap
            score = (p.w_progress * progress + p.w_heading * heading
                     + p.w_clearance * free + p.w_speed * v / p.max_speed)
            if best is None or score > best[0]:
                best = (score, v, w, traj, clearance)

    if best is None:
        return Plan(0.0, 0.0, np.zeros((0, 3)), False, 0.0)
    _, v, w, traj, clearance = best
    return Plan(float(v), float(w), traj, True, clearance)


def to_base(x: float, y: float, yaw: float, points: np.ndarray) -> np.ndarray:
    """map 座標の点を、(x, y, yaw) にいるローバーの base_link 座標に直す。"""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.c_[c * dx + s * dy, -s * dx + c * dy]


def thin(points: np.ndarray, cell: float = 0.15) -> np.ndarray:
    """点を cell 四方に 1 点へ間引く（計算を軽くする）。"""
    if len(points) == 0:
        return points.reshape(0, 2)
    keys = np.floor(points[:, :2] / cell).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx, :2]
