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


def make_rock(x: float, y: float, size: float = 0.8, height: float = 0.5, seed: int = 0) -> np.ndarray:
    """(x, y) に岩（箱）を 1 つ置く（障害物検出の試験用）。"""
    rng = np.random.default_rng(seed + 2000)
    return _box(rng, x, y, size, size, height, 0.0, 600)


# 試験用の配置。座標はスタート地点が原点、x がスタート時の前方、y が左（map 座標と同じ）。
# 置く前に、この範囲にある元の仮想フィールドの物体は取り除く（配置どうしが混ざらないように）。
LAYOUT_ZONE = ((1.0, 17.0), (-8.0, 8.0))
LAYOUTS = ('default', 'gap', 'narrow', 'dead_end', 'clutter', 'checkpoints', 'open')


def _wall(rng, x0, y0, x1, y1, height=0.6, thickness=0.3, density=300):
    """(x0, y0) から (x1, y1) までの壁。"""
    length = math.hypot(x1 - x0, y1 - y0)
    return _box(rng, (x0 + x1) / 2, (y0 + y1) / 2, length, thickness, height,
                math.atan2(y1 - y0, x1 - x0), max(50, int(density * length)))


# 自律ミッションの試験用のマーカー（map 座標）。チェックポイントは 2 本のマーカーを結んだ線分
CHECKPOINT_MARKERS = {2: (7.0, -1.5), 5: (7.0, 1.5),      # CP1：まっすぐ横切る（間 3.0 m）
                      3: (13.0, -5.5), 7: (15.5, -4.0)}   # CP2：斜めの線分（間 2.9 m）
CHECKPOINT_ROUTE = [{'checkpoint': [2, 5], 'name': 'CP1'},
                    {'checkpoint': [3, 7], 'name': 'CP2'},
                    {'latlon_map': (4.0, -7.0), 'name': '最後の地点'}]


def map_to_latlon(x: float, y: float, lat0: float, lon0: float, heading0: float, radius: float = 6378137.0):
    """シミュレーターの map 座標（スタート地点が原点、x がスタート時の前方）を緯度・経度にする。

    fake_sensors_node の GNSS と同じ近似（スタート地点のまわりの平面）を使う。
    """
    c, s = math.cos(heading0), math.sin(heading0)
    e, n = c * x - s * y, s * x + c * y
    lat = lat0 + math.degrees(n / radius)
    lon = lon0 + math.degrees(e / (radius * math.cos(math.radians(lat0))))
    return lat, lon


def make_layout(name: str, seed: int = 0) -> np.ndarray:
    """試験用の障害物の配置（map 座標）。目標は (14, 0) を想定。

    gap      : x = 7 に横長の壁。真ん中に幅 2.0 m のすき間（通れる）
    narrow   : 同じ壁で、すき間は幅 1.0 m（ローバーには狭すぎる。壁の端を回る必要がある）
    dead_end : スタート側に口を開けた U 字（奥行き 5 m・幅 4 m）。まっすぐ行くと袋小路
    checkpoints : 自律ミッションのチェックポイント用のマーカーの柱（CHECKPOINT_MARKERS）
    clutter  : x = 4〜9.6 に、千鳥の格子に沿って岩を 11 個ばらまく（岩どうしの間は 1.4 m 以上）
    open     : 何も置かない（範囲の中の物体を取り除くだけ。砂丘やくぼみなど、地形の試験用）
    """
    rng = np.random.default_rng(seed + 3000)
    if name in ('default', 'open'):
        return np.empty((0, 3))
    if name in ('gap', 'narrow'):
        half = 1.0 if name == 'gap' else 0.5
        return np.vstack([_wall(rng, 7.0, -6.0, 7.0, -half), _wall(rng, 7.0, half, 7.0, 6.0)])
    if name == 'dead_end':
        return np.vstack([_wall(rng, 4.0, 2.0, 9.0, 2.0), _wall(rng, 4.0, -2.0, 9.0, -2.0),
                          _wall(rng, 9.0, -2.0, 9.0, 2.0)])
    if name == 'checkpoints':
        # ArUco マーカーの柱（20 cm 角・高さ 1.2 m）。LiDAR から障害物として見える
        return np.vstack([_box(rng, x, y, 0.2, 0.2, 1.2, 0.0, 300) for x, y in CHECKPOINT_MARKERS.values()])
    if name == 'clutter':
        # 千鳥に並べた格子（間隔 2.8 m）に少しずらして岩を置く。岩どうしの間は 1.4 m 以上空く
        rocks = []
        for i, x in enumerate((4.0, 6.8, 9.6)):
            for y in (-4.2, -1.4, 1.4, 4.2):
                yy = y + (1.4 if i % 2 else 0.0)
                if abs(yy) > 5.0:
                    continue
                rocks.append((x + rng.uniform(-0.25, 0.25), yy + rng.uniform(-0.25, 0.25)))
        return np.vstack([_box(rng, x, y, rng.uniform(0.4, 0.8), rng.uniform(0.4, 0.8),
                               rng.uniform(0.3, 0.7), rng.uniform(0, math.pi), 250) for x, y in rocks])
    raise ValueError(f'layout は {LAYOUTS} のどれか: {name}')


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


def make_ground_on(terrain, seed: int = 0, size: float = 60.0, n: int = 150000) -> np.ndarray:
    """地形 terrain（fake_terrain.Terrain）に沿った地面の点（world 座標）。

    点の水平な位置は make_ground と同じ。くぼみの壁のような急な所は点がまばらになる（実際の LiDAR と同じ）。
    """
    rng = np.random.default_rng(seed + 1000)
    half = size / 2.0
    x = rng.uniform(-half, half, n)
    y = rng.uniform(-half, half, n)
    return np.c_[x, y, terrain.height(x, y)]


def make_ground_grid(terrain, seed: int = 0, size: float = 60.0, spacing: float = 0.125) -> np.ndarray:
    """地形 terrain に沿った地面の点（world 座標）を、少しずらした格子（spacing 間隔）に置く。

    どのマス（0.25 m 四方）にも必ず地面の点があるので、「地面が見えないマス = 穴」の判定を試せる。
    点は world に固定（スキャンごとに同じ点が見える）。
    """
    rng = np.random.default_rng(seed + 1000)
    half = size / 2.0
    g = np.arange(-half, half, spacing) + spacing / 2.0
    x, y = np.meshgrid(g, g)
    x = x.ravel() + rng.uniform(-0.4, 0.4, x.size) * spacing
    y = y.ravel() + rng.uniform(-0.4, 0.4, y.size) * spacing
    return np.c_[x, y, terrain.height(x, y)]


def place_on(terrain, points: np.ndarray) -> np.ndarray:
    """物体の点（地面 z = 0 が基準）を、地形の上に載せる。物体ごとではなく点ごとに地面の高さを足す簡略版。"""
    if terrain.flat or len(points) == 0:
        return points
    out = points.copy()
    out[:, 2] += terrain.height(points[:, 0], points[:, 1])
    return out


def scan_3d(objects: np.ndarray, ground: np.ndarray, terrain, base: np.ndarray, rot: np.ndarray,
            lidar_x: float, lidar_z: float, min_range: float, max_range: float,
            max_points: int, noise: float, rng: np.random.Generator,
            ground_points: int = 3000, extra: np.ndarray = None) -> np.ndarray:
    """傾いた車体から見た 1 回分の点群（sensor 座標、float32）。地形の陰になる点は消す。

    base: 車体（base_link の原点 = 車体中心の地面）の world 座標。rot: 車体 → world の回転行列。
    ground: 地面の点（world に固定。近いほど多く選ぶ）。None なら、スキャンごとに地形 terrain の上から
            選び直す（水平距離が対数一様）。選び直すと、地面の点どうしは毎回ちがう所になるので、
            LiDAR オドメトリの手がかりにならない（模様のない平らな砂地）。
    extra: そのスキャンだけに見える点（舞い上がった砂など、world 座標）。
    """
    sensor = base + rot @ np.array([lidar_x, 0.0, lidar_z])
    if ground is None:
        lo, hi = max(min_range, 0.3), max_range
        r = lo * (hi / lo) ** rng.random(ground_points)
        a = rng.uniform(-math.pi, math.pi, ground_points)
        gx, gy = sensor[0] + r * np.cos(a), sensor[1] + r * np.sin(a)
        ground = np.c_[gx, gy, terrain.height(gx, gy)]

    def visible(points, limit, falloff=False):
        if len(points) == 0:
            return np.empty((0, 3))
        d = points - sensor
        r = np.linalg.norm(d, axis=1)
        ok = (r >= min_range) & (r <= max_range)
        pts, r = points[ok], r[ok]
        if len(pts) > limit:
            # falloff：実際の LiDAR のように、近いほど点が多い（1/r^2）。一様に選ぶと足もとの地面がまばらになる
            if falloff:     # 重み付きで重複なしに選ぶ（鍵 = log(u) / 重み の大きい順）
                keys = np.log(rng.random(len(pts))) * np.maximum(r, 1.0) ** 2
                pts = pts[np.argpartition(-keys, limit)[:limit]]
            else:
                pts = pts[rng.choice(len(pts), limit, replace=False)]
        return pts[~terrain.occluded(sensor, pts)]

    parts = [visible(objects, max_points), visible(ground, ground_points, falloff=True)]
    if extra is not None and len(extra):
        parts.append(visible(extra, len(extra)))
    world = np.vstack(parts)
    cloud = (world - sensor) @ rot                 # = rot^T (p - sensor)
    if noise > 0.0:
        cloud = cloud + rng.normal(0.0, noise, cloud.shape)
    return cloud.astype(np.float32)
