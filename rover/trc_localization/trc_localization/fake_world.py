"""にせの LiDAR のための仮想フィールドと、点群の計算（ROS に依存しない）。

フィールドには、マーカーの支柱（細い円柱）、岩（箱）、砂山や段差（長い壁）を置く。
各物体の表面に点をばらまいておき、ローバーの位置から見た座標に直して、
LiDAR の届く範囲の点だけを取り出す（遮蔽は考えない簡略版）。

座標:
  world  : ENU（東 x・北 y・上 z）。地面は z = 0。原点はローバーのスタート地点
  sensor : LiDAR の座標（前 x・左 y・上 z）。ローバーの base_link から (lidar_x, 0, lidar_z) の位置
"""

import math

import numpy as np


def _cylinder(rng, cx, cy, radius, height, n):
    th = rng.uniform(0.0, 2.0 * math.pi, n)
    z = rng.uniform(0.0, height, n)
    return np.c_[cx + radius * np.cos(th), cy + radius * np.sin(th), z]


def _box(rng, cx, cy, sx, sy, sz, yaw, n):
    """箱の表面（上面と4つの側面）に点をばらまく。"""
    faces = []
    areas = np.array([sx * sy, sx * sz, sx * sz, sy * sz, sy * sz])
    counts = np.maximum(1, (n * areas / areas.sum()).astype(int))
    u = lambda k: rng.uniform(-0.5, 0.5, k)  # noqa: E731
    faces.append(np.c_[u(counts[0]) * sx, u(counts[0]) * sy, np.full(counts[0], sz)])          # 上面
    faces.append(np.c_[u(counts[1]) * sx, np.full(counts[1], 0.5 * sy), (u(counts[1]) + 0.5) * sz])
    faces.append(np.c_[u(counts[2]) * sx, np.full(counts[2], -0.5 * sy), (u(counts[2]) + 0.5) * sz])
    faces.append(np.c_[np.full(counts[3], 0.5 * sx), u(counts[3]) * sy, (u(counts[3]) + 0.5) * sz])
    faces.append(np.c_[np.full(counts[4], -0.5 * sx), u(counts[4]) * sy, (u(counts[4]) + 0.5) * sz])
    p = np.vstack(faces)
    c, s = math.cos(yaw), math.sin(yaw)
    x = c * p[:, 0] - s * p[:, 1] + cx
    y = s * p[:, 0] + c * p[:, 1] + cy
    return np.c_[x, y, p[:, 2]]


def make_world(seed: int = 0, size: float = 60.0, keep_clear=((-7.5, 1.5), (-1.5, 10.5))) -> np.ndarray:
    """仮想フィールドの点（N x 3、world 座標）を作る。

    keep_clear: ((xmin, xmax), (ymin, ymax)) の範囲には物を置かない（ローバーの走る場所）。
    """
    rng = np.random.default_rng(seed)
    (x0, x1), (y0, y1) = keep_clear
    half = size / 2.0

    def free_spot():
        while True:
            x, y = rng.uniform(-half, half, 2)
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                return x, y

    parts = []
    for _ in range(40):                       # マーカーの支柱
        x, y = free_spot()
        parts.append(_cylinder(rng, x, y, rng.uniform(0.05, 0.15), rng.uniform(1.0, 2.0), 300))
    for _ in range(60):                       # 岩
        x, y = free_spot()
        parts.append(_box(rng, x, y, *rng.uniform(0.3, 1.5, 2), rng.uniform(0.2, 1.0),
                          rng.uniform(0.0, math.pi), 400))
    for _ in range(8):                        # 砂山・段差（長い壁）
        x, y = free_spot()
        parts.append(_box(rng, x, y, rng.uniform(5.0, 15.0), 0.5, rng.uniform(0.4, 1.5),
                          rng.uniform(0.0, math.pi), 2500))
    return np.vstack(parts).astype(np.float64)


def make_ground(seed: int = 0, size: float = 60.0, n: int = 150000,
                amplitude: float = 0.08, wavelength: float = 4.0) -> np.ndarray:
    """地面の点（world 座標）。砂地を想定して、ゆるい起伏（振幅 amplitude）を付ける。

    注意：地面の点を「LiDAR に対して決まった位置」に作ると、ローバーが動いても点群が変わらず、
    LiDAR オドメトリが「止まっている」と誤認する（試験で確認済み）。必ず world に固定した点にする。
    """
    rng = np.random.default_rng(seed + 1000)
    half = size / 2.0
    x = rng.uniform(-half, half, n)
    y = rng.uniform(-half, half, n)
    k = 2.0 * math.pi / wavelength
    z = amplitude * (np.sin(k * x) * np.cos(0.7 * k * y) + 0.5 * np.sin(1.9 * k * y + 0.3))
    return np.c_[x, y, z]


def world_to_sensor(points: np.ndarray, base_e: float, base_n: float, base_yaw: float,
                    lidar_x: float, lidar_z: float) -> np.ndarray:
    """world 座標の点を、ローバーに載った LiDAR の座標に直す。"""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    sx = base_e + lidar_x * c
    sy = base_n + lidar_x * s
    dx = points[:, 0] - sx
    dy = points[:, 1] - sy
    return np.c_[c * dx + s * dy, -s * dx + c * dy, points[:, 2] - lidar_z]


def _visible(points: np.ndarray, base_e, base_n, base_yaw, lidar_x, lidar_z,
             min_range, max_range, max_points, rng) -> np.ndarray:
    local = world_to_sensor(points, base_e, base_n, base_yaw, lidar_x, lidar_z)
    dist = np.linalg.norm(local, axis=1)
    local = local[(dist >= min_range) & (dist <= max_range)]
    if len(local) > max_points:
        local = local[rng.choice(len(local), max_points, replace=False)]
    return local


def scan(objects: np.ndarray, ground: np.ndarray, base_e: float, base_n: float, base_yaw: float,
         lidar_x: float, lidar_z: float, min_range: float, max_range: float,
         max_points: int, noise: float, rng: np.random.Generator,
         ground_points: int = 3000) -> np.ndarray:
    """1回分の点群（sensor 座標、float32）を作る。物体と地面は別々に点数を決める。"""
    parts = [_visible(objects, base_e, base_n, base_yaw, lidar_x, lidar_z,
                      min_range, max_range, max_points, rng)]
    if len(ground):
        parts.append(_visible(ground, base_e, base_n, base_yaw, lidar_x, lidar_z,
                              min_range, max_range, ground_points, rng))
    cloud = np.vstack(parts)
    if noise > 0.0:
        cloud = cloud + rng.normal(0.0, noise, cloud.shape)
    return cloud.astype(np.float32)
