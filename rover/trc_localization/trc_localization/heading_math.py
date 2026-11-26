"""向きの初期化に使う計算（ROS に依存しない。test/test_heading_math.py で確認できる）。

考え方:
  同じ移動を「GNSS（東西南北の座標）」と「LiDARオドメトリ（スタート時の向きが基準の座標）」の
  2通りで測る。2つの移動ベクトルの向きの差が、オドメトリ座標と東西南北（ENU）の回転のずれになる。
  曲がりながら走っても、始点と終点を結ぶベクトル同士を比べるので成り立つ。
"""

import math
from typing import Optional, Tuple

EARTH_RADIUS = 6378137.0  # WGS84 長半径 [m]


def latlon_to_local_en(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """(lat0, lon0) を原点とした東 [m]・北 [m]。数十 m 以内なら十分な精度の近似。"""
    east = math.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * EARTH_RADIUS
    return east, north


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def estimate_rotation(gnss_delta: Tuple[float, float], odom_delta: Tuple[float, float],
                      min_distance: float, max_length_mismatch: float) -> Optional[float]:
    """オドメトリ座標 → ENU の回転角 [rad] を返す。条件を満たさなければ None。

    gnss_delta: GNSS で測った移動（東, 北）[m]
    odom_delta: オドメトリで測った移動（x, y）[m]
    min_distance: これより短い移動では推定しない（短いと GNSS の誤差の影響が大きい）
    max_length_mismatch: 2つの移動距離の差の許容値 [m]（車輪の空転や GNSS の飛びを弾く）
    """
    g_len = math.hypot(*gnss_delta)
    o_len = math.hypot(*odom_delta)
    # 距離の判定は GNSS 側だけで行う（オドメトリは配信周期の違いで少し短く出ることがあるため、
    # 両方に min_distance を課すと、距離が合っているのに弾いてしまう）
    if g_len < min_distance:
        return None
    if abs(g_len - o_len) > max_length_mismatch:
        return None
    g_bearing = math.atan2(gnss_delta[1], gnss_delta[0])
    o_bearing = math.atan2(odom_delta[1], odom_delta[0])
    return normalize_angle(g_bearing - o_bearing)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
