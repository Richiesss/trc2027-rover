"""点群から障害物を見つける計算（ROS に依存しない）。

考え方（グリッド最低高さ法）:
  1. 点群を base_link 座標（前 x・左 y・上 z、原点はローバー中心の地面）に直す
  2. 水平面を cell_size 四方のマスに区切り、マスごとに一番低い点の高さを求める
  3. 周り (2*neighbor+1)^2 マスの最低高さを「そこの地面の高さ」とみなす
     （岩の上だけのマスでも、周りのマスには地面の点があるので地面の高さが分かる）。
     離れたマスの最低高さには、勾配 max_slope の分だけ高さを足してから比べる（坂を障害物にしない）
  4. その高さから height_threshold 未満の点を地面の候補とし、マスごとに候補の平均の高さを
     地面の高さとして求め直す（最低高さはノイズで低く出やすいため）
  5. 地面の高さより height_threshold 以上高い点を障害物とする

砂地のゆるい起伏（数 cm〜十数 cm、波長数 m）は地面として扱える。
max_slope より急な斜面は障害物と判定されるので、
実機では cell_size / height_threshold をフィールドに合わせて調整すること。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class SegmentParams:
    cell_size: float = 0.4          # マスの大きさ [m]
    neighbor: int = 2               # 地面の高さを探す範囲（周り何マスまで）
    height_threshold: float = 0.15  # 地面からこれ以上高い点を障害物にする [m]
    max_height: float = 2.0         # これより高い点は無視（木の枝・天井など）[m]
    self_radius: float = 0.5        # ローバー自身の点を消す半径 [m]
    max_range: float = 20.0         # これより遠い点は使わない [m]
    max_slope: float = 0.3          # 地面として許す勾配（0.3 = 30 %、約 17°）


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """回転のクォータニオンを 3x3 回転行列にする。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_points(points: np.ndarray, rotation: np.ndarray, translation) -> np.ndarray:
    """点（N x 3）を回転・平行移動する：p' = R p + t"""
    return points @ np.asarray(rotation).T + np.asarray(translation)


def segment(points: np.ndarray, p: SegmentParams = SegmentParams()):
    """点（base_link 座標、N x 3）を障害物と地面に分ける。

    戻り値: (obstacle_mask, ground_mask)。どちらにも入らない点（遠すぎ・高すぎ・自分自身）もある。
    """
    n = len(points)
    obstacle = np.zeros(n, dtype=bool)
    ground = np.zeros(n, dtype=bool)
    if n == 0:
        return obstacle, ground

    finite = np.all(np.isfinite(points), axis=1)
    r = np.hypot(points[:, 0], points[:, 1])
    use = finite & (r >= p.self_radius) & (r <= p.max_range)
    if not np.any(use):
        return obstacle, ground

    pts = points[use]
    # マスの番号（0 始まり、周りを探せるように neighbor マス分の余白を付ける）
    ix = np.floor(pts[:, 0] / p.cell_size).astype(np.int64)
    iy = np.floor(pts[:, 1] / p.cell_size).astype(np.int64)
    pad = p.neighbor
    ix0, iy0 = ix.min() - pad, iy.min() - pad
    ix -= ix0
    iy -= iy0
    w = ix.max() + pad + 1
    h = iy.max() + pad + 1

    cell_min = np.full((w, h), np.inf)
    np.minimum.at(cell_min, (ix, iy), pts[:, 2])

    # 周りのマスの最低高さ（= 地面の高さの推定）
    ground_z = np.full((w, h), np.inf)
    for dx in range(-pad, pad + 1):
        for dy in range(-pad, pad + 1):
            shifted = np.full((w, h), np.inf)
            xs = slice(max(dx, 0), w + min(dx, 0))
            xd = slice(max(-dx, 0), w + min(-dx, 0))
            ys = slice(max(dy, 0), h + min(dy, 0))
            yd = slice(max(-dy, 0), h + min(-dy, 0))
            # 離れたマスほど、坂の分だけ高くてもよいとする
            shifted[xd, yd] = cell_min[xs, ys] + p.max_slope * p.cell_size * np.hypot(dx, dy)
            np.minimum(ground_z, shifted, out=ground_z)

    # 2 回目：いったん地面とみなした点の平均の高さを、そのマスの地面の高さにする。
    # 最低高さだけだと、ノイズや起伏で低く出すぎて、地面の点まで障害物になりやすいため。
    # 地面とみなした点がないマス（岩の上だけ等）は、1 回目の推定をそのまま使う。
    cand = (pts[:, 2] - ground_z[ix, iy]) < p.height_threshold
    z_sum = np.zeros((w, h))
    z_cnt = np.zeros((w, h))
    np.add.at(z_sum, (ix[cand], iy[cand]), pts[cand, 2])
    np.add.at(z_cnt, (ix[cand], iy[cand]), 1.0)
    surface = np.where(z_cnt > 0, z_sum / np.maximum(z_cnt, 1.0), ground_z)

    height = pts[:, 2] - surface[ix, iy]
    is_obs = (height >= p.height_threshold) & (height <= p.max_height)
    is_ground = height < p.height_threshold

    idx = np.flatnonzero(use)
    obstacle[idx[is_obs]] = True
    ground[idx[is_ground]] = True
    return obstacle, ground


def front_distance(obstacles: np.ndarray, corridor_width: float, max_distance: float,
                   front_offset: float = 0.0) -> float:
    """進行方向（+x）の幅 corridor_width の帯に入った障害物までの最短距離 [m]。

    front_offset: ローバー前端の x（ここから先だけを見る）。何もなければ inf。
    """
    if len(obstacles) == 0:
        return float('inf')
    x, y = obstacles[:, 0], obstacles[:, 1]
    inside = (x >= front_offset) & (x <= front_offset + max_distance) & (np.abs(y) <= corridor_width / 2.0)
    if not np.any(inside):
        return float('inf')
    return float(x[inside].min() - front_offset)
