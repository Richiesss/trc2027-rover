"""障害物の地図（map 座標の格子）と、地図の上で目標までの道筋を探す A*（ROS に依存しない）。

DWA は「いま見えている範囲で、数秒先」しか考えないので、長い壁や袋小路で行き詰まる。
見つけた障害物を地図に書きためておき、その地図の上で遠回りも含めた道筋を探して、
DWA にはその道筋の少し先の点を目指させる。

地図:
  - resolution 四方のマス。障害物の点が hits_to_occupy 回以上入ったマスを「ふさがっている」とする
  - 地面の点だけが見えたマスは数を 1 減らす（位置のずれで間違って書いた障害物や、動いた物を消すため）
  - 見えていない所は「通れる」とみなす（楽観的。近づいて見えたら地図が更新され、道筋を引き直す）
道筋:
  - ふさがったマスを robot_radius + safety_margin だけ太らせたうえで、8 近傍の A* で探す
  - 障害物の近くは通りにくく（コストを足す）して、なるべく真ん中を通る
"""

import heapq
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class GridParams:
    resolution: float = 0.25      # マスの大きさ [m]
    size: float = 120.0           # 地図の一辺 [m]（原点 = スタート地点が真ん中）
    hits_to_occupy: int = 3       # 何回見えたら「ふさがっている」とするか（ノイズよけ）
    max_hits: int = 10            # 数える上限（地面として見えたら 1 ずつ減らして、消えるようにするため）
    inflate: float = 0.7          # ふさがったマスを太らせる半径 [m]（robot_radius + safety_margin）
    soft: float = 0.4             # さらにこの幅は通りにくくする [m]
    # （広すぎると、チェックポイントのマーカーの間（3 m）の真ん中まで「通りにくい」になり、
    #   マーカーの外を遠回りする道を選んでしまった。0.4 なら 3 m のすき間の真ん中 0.8 m は通常のコスト）
    soft_cost: float = 3.0        # 通りにくさ（1 マス進むコストに足す最大値）
    search_margin: float = 15.0   # 探す範囲：スタートと目標を囲む四角＋この幅 [m]


class ObstacleGrid:

    def __init__(self, p: GridParams = GridParams()):
        self.p = p
        self.n = int(round(p.size / p.resolution))
        self.origin = -p.size / 2.0
        self.hits = np.zeros((self.n, self.n), dtype=np.uint16)

    def to_cell(self, x, y):
        return (np.floor((np.asarray(x) - self.origin) / self.p.resolution).astype(np.int64),
                np.floor((np.asarray(y) - self.origin) / self.p.resolution).astype(np.int64))

    def to_xy(self, i, j):
        r = self.p.resolution
        return self.origin + (i + 0.5) * r, self.origin + (j + 0.5) * r

    def add(self, points_map: np.ndarray):
        """map 座標の障害物の点（1 回のスキャン分）を書き込む。同じマスは 1 回のスキャンで 1 回だけ数える。"""
        if len(points_map) == 0:
            return
        i, j = self.to_cell(points_map[:, 0], points_map[:, 1])
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        cells = np.unique(np.c_[i[ok], j[ok]], axis=0)
        if len(cells):
            h = self.hits[cells[:, 0], cells[:, 1]]
            self.hits[cells[:, 0], cells[:, 1]] = np.minimum(h + 1, self.p.max_hits)

    def clear(self, ground_map: np.ndarray, obstacles_map: np.ndarray = None):
        """地面として見えたマス（同じスキャンで障害物が見えていないマス）の数を 1 減らす。"""
        if len(ground_map) == 0:
            return
        i, j = self.to_cell(ground_map[:, 0], ground_map[:, 1])
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        cells = np.unique(np.c_[i[ok], j[ok]], axis=0)
        if obstacles_map is not None and len(obstacles_map):
            oi, oj = self.to_cell(obstacles_map[:, 0], obstacles_map[:, 1])
            busy = set(zip(oi.tolist(), oj.tolist()))
            cells = np.array([c for c in cells.tolist() if (c[0], c[1]) not in busy]).reshape(-1, 2)
        if len(cells):
            h = self.hits[cells[:, 0], cells[:, 1]].astype(np.int32)
            self.hits[cells[:, 0], cells[:, 1]] = np.maximum(h - 1, 0).astype(np.uint16)

    def occupied(self) -> np.ndarray:
        return self.hits >= self.p.hits_to_occupy


def _dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """mask を半径 radius_cells の円で太らせる。"""
    out = mask.copy()
    w, h = mask.shape
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if (dx == 0 and dy == 0) or dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            xs = slice(max(dx, 0), w + min(dx, 0))
            xd = slice(max(-dx, 0), w + min(-dx, 0))
            ys = slice(max(dy, 0), h + min(dy, 0))
            yd = slice(max(-dy, 0), h + min(-dy, 0))
            out[xd, yd] |= mask[xs, ys]
    return out


def plan_path_robust(grid: ObstacleGrid, start, goal, min_inflate: float):
    """plan_path で見つからなければ、太らせる幅を min_inflate まで減らして探し直す。

    目標が障害物に近すぎる（太らせた範囲の中）ときは、目標のまわり 2 m 以内で空いている所を代わりにする。
    戻り値: (道筋 or None, 実際に向かう目標)
    """
    for inflate in (grid.p.inflate, min_inflate):
        path = plan_path(grid, start, goal, inflate)
        if path is not None:
            return path, tuple(goal)
    alt = nearest_free(grid, goal, min_inflate, radius=2.0)
    if alt is not None:
        path = plan_path(grid, start, alt, min_inflate)
        if path is not None:
            return path, alt
    return None, tuple(goal)


def nearest_free(grid: ObstacleGrid, goal, inflate: float, radius: float):
    """goal のまわり radius [m] 以内で、障害物から inflate 以上離れたいちばん近い点。"""
    r = grid.p.resolution
    gi, gj = grid.to_cell(*goal)
    k = int(math.ceil(radius / r))
    m = k + int(math.ceil(inflate / r)) + 1
    i0, j0 = max(gi - m, 0), max(gj - m, 0)
    occ = grid.occupied()[i0:gi + m + 1, j0:gj + m + 1]
    hard = _dilate(occ, int(math.ceil(inflate / r)))
    best = None
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            i, j = gi + di - i0, gj + dj - j0
            if 0 <= i < hard.shape[0] and 0 <= j < hard.shape[1] and not hard[i, j]:
                d = di * di + dj * dj
                if d <= k * k and (best is None or d < best[0]):
                    best = (d, gi + di, gj + dj)
    if best is None:
        return None
    return grid.to_xy(best[1], best[2])


def plan_path(grid: ObstacleGrid, start, goal, inflate: float = None):
    """start から goal までの道筋（map 座標の (N, 2)）。見つからなければ None。

    スタートが太らせた障害物の中にあるとき（障害物のすぐそば）は、そこから出る所までは通れるとみなす。
    """
    p = grid.p
    r = p.resolution
    inflate = p.inflate if inflate is None else inflate
    (si, sj), (gi, gj) = grid.to_cell(*start), grid.to_cell(*goal)
    m = int(math.ceil(p.search_margin / r))
    i0, i1 = max(min(si, gi) - m, 0), min(max(si, gi) + m + 1, grid.n)
    j0, j1 = max(min(sj, gj) - m, 0), min(max(sj, gj) + m + 1, grid.n)
    if not (i0 <= si < i1 and j0 <= sj < j1 and i0 <= gi < i1 and j0 <= gj < j1):
        return None

    occ = grid.occupied()[i0:i1, j0:j1]
    hard = _dilate(occ, int(math.ceil(inflate / r)))
    near = _dilate(occ, int(math.ceil((inflate + p.soft) / r)))
    extra = np.where(near & ~hard, p.soft_cost, 0.0)
    si, sj, gi, gj = si - i0, sj - j0, gi - i0, gj - j0
    if hard[gi, gj]:
        return None
    if hard[si, sj]:
        # スタートのまわりの「太らせた部分」だけは通れるようにする（本当にふさがったマスは通らない）
        blob = np.zeros_like(hard)
        blob[max(si - 4, 0):si + 5, max(sj - 4, 0):sj + 5] = True
        hard = hard & ~(blob & ~occ)

    W, H = hard.shape
    INF = float('inf')
    cost = np.full((W, H), INF)
    parent = np.full((W, H, 2), -1, dtype=np.int64)
    cost[si, sj] = 0.0
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    heap = [(math.hypot(gi - si, gj - sj), 0.0, si, sj)]
    while heap:
        _, c, i, j = heapq.heappop(heap)
        if c > cost[i, j]:
            continue
        if i == gi and j == gj:
            break
        for di, dj, step in steps:
            ni, nj = i + di, j + dj
            if ni < 0 or nj < 0 or ni >= W or nj >= H or hard[ni, nj]:
                continue
            nc = c + step * (1.0 + extra[ni, nj])
            if nc < cost[ni, nj]:
                cost[ni, nj] = nc
                parent[ni, nj] = (i, j)
                heapq.heappush(heap, (nc + math.hypot(gi - ni, gj - nj), nc, ni, nj))
    if cost[gi, gj] == INF:
        return None

    cells = [(gi, gj)]
    while cells[-1] != (si, sj):
        pi, pj = parent[cells[-1]]
        cells.append((int(pi), int(pj)))
    cells.reverse()
    xs, ys = grid.to_xy(np.array([c[0] for c in cells]) + i0, np.array([c[1] for c in cells]) + j0)
    path = np.c_[xs, ys]
    path[0] = start
    path[-1] = goal
    return path


def lookahead_point(path: np.ndarray, position, distance: float):
    """道筋の上で、いまの位置にいちばん近い点から distance 先の点。"""
    d = np.hypot(path[:, 0] - position[0], path[:, 1] - position[1])
    k = int(d.argmin())
    seg = np.hypot(np.diff(path[k:, 0]), np.diff(path[k:, 1]))
    acc = np.concatenate([[0.0], np.cumsum(seg)])
    idx = int(np.searchsorted(acc, distance))
    return path[min(k + idx, len(path) - 1)]
