"""自律ミッションの目標（緯度・経度）を読み、経路計画のウェイポイントを作る計算（ROS に依存しない）。

TRC2027 自律ミッションのルール（競技説明編 1.2）に合わせている:
  - チェックポイントは、2 本の ArUco マーカーを結んだ線分。機体全体が線分を跨ぎ切ると通過
  - マーカーの座標は対応表（csv: id, 緯度, 経度）で渡される（各チーム 1 組だけ。他はおおよその領域）
  - エントリー部門は、人が座標をもとに通過目標点を設定して順に回ってよい

ミッションファイル（YAML）の例:

    markers_csv: markers.csv        # 対応表（このファイルからの相対パスでもよい）。なくてもよい
    route:
      - checkpoint: [2, 5]          # 対応表の ID 2 と ID 5 を結ぶ線分を横切る
        name: CP1
      - checkpoint:                 # 対応表にないマーカーは、座標を直接書く（推定した座標など）
          a: [35.536932, 134.210631]
          b: [35.536900, 134.210700]
      - latlon: [35.536784, 134.210739]   # この地点へ向かう
        name: 待機地点

チェックポイントは「手前の点」と「奥の点」の 2 つのウェイポイントにする。
線分の中点を垂直に横切る向きで、手前はひとつ前の地点の側、奥は機体全体が跨ぎ切れる距離だけ先。
"""

import csv
import io
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import yaml

LatLon = Tuple[float, float]
XY = Tuple[float, float]

_NUM = re.compile(r'[-+]?\d+(?:\.\d+)?')


def load_markers(path: str) -> Dict[int, LatLon]:
    """対応表（csv）を読む。'id, 緯度, 経度' の並びなら、見出しや空白・括弧があっても読める。"""
    with open(path, encoding='utf-8-sig') as f:
        return parse_markers(f.read())


def parse_markers(text: str) -> Dict[int, LatLon]:
    out = {}
    for row in csv.reader(io.StringIO(text)):
        nums = _NUM.findall(','.join(row))
        if len(nums) < 3:
            continue                                  # 見出しや空行
        try:
            mid = int(float(nums[0]))
            lat, lon = float(nums[1]), float(nums[2])
        except ValueError:
            continue
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'緯度・経度の範囲外です（ID {mid}: {lat}, {lon}）')
        if mid in out and out[mid] != (lat, lon):
            raise ValueError(f'ID {mid} が対応表に 2 回あり、座標が違います')
        out[mid] = (lat, lon)
    return out


@dataclass
class Step:
    kind: str                       # 'latlon' または 'checkpoint'
    name: str
    point: Optional[LatLon] = None  # latlon のとき
    a: Optional[LatLon] = None      # checkpoint の 2 本のマーカー
    b: Optional[LatLon] = None
    ids: Tuple = field(default_factory=tuple)


def _latlon(v, where) -> LatLon:
    if not (isinstance(v, (list, tuple)) and len(v) == 2):
        raise ValueError(f'{where}: [緯度, 経度] で書いてください: {v!r}')
    lat, lon = float(v[0]), float(v[1])
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError(f'{where}: 緯度・経度の範囲外です: {v!r}')
    return lat, lon


def load_mission(path: str) -> List[Step]:
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    markers = {}
    if doc.get('markers_csv'):
        csv_path = doc['markers_csv']
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(os.path.dirname(os.path.abspath(path)), csv_path)
        markers = load_markers(csv_path)
    return parse_route(doc.get('route') or [], markers)


def parse_route(route, markers: Dict[int, LatLon]) -> List[Step]:
    steps = []
    for i, item in enumerate(route, 1):
        where = f'route の {i} 番目'
        if not isinstance(item, dict):
            raise ValueError(f'{where}: checkpoint か latlon を書いてください')
        if 'latlon' in item:
            steps.append(Step('latlon', str(item.get('name', f'地点{i}')), point=_latlon(item['latlon'], where)))
        elif 'checkpoint' in item:
            cp = item['checkpoint']
            if isinstance(cp, (list, tuple)) and len(cp) == 2 and all(isinstance(v, (int, float)) for v in cp) \
                    and all(float(v).is_integer() for v in cp):
                ids = tuple(int(v) for v in cp)
                missing = [m for m in ids if m not in markers]
                if missing:
                    raise ValueError(f'{where}: 対応表に ID {missing} がありません')
                a, b = markers[ids[0]], markers[ids[1]]
                name = str(item.get('name', f'CP（ID{ids[0]}–ID{ids[1]}）'))
            elif isinstance(cp, dict) and 'a' in cp and 'b' in cp:
                ids = ()
                a, b = _latlon(cp['a'], where + ' の a'), _latlon(cp['b'], where + ' の b')
                name = str(item.get('name', f'CP{i}'))
            else:
                raise ValueError(f'{where}: checkpoint は [ID, ID] か {{a: [緯度, 経度], b: [緯度, 経度]}}')
            steps.append(Step('checkpoint', name, a=a, b=b, ids=ids))
        else:
            raise ValueError(f'{where}: checkpoint か latlon を書いてください')
    return steps


def crossing_points(a: XY, b: XY, came_from: XY, before: float, after: float):
    """チェックポイント（a–b の線分）を中点で垂直に横切る (手前の点, 奥の点, マーカー間の距離)。

    手前の点は came_from（ひとつ前の地点）の側に before [m]、奥の点は反対側に after [m]。
    """
    ax, ay = a
    bx, by = b
    gap = math.hypot(bx - ax, by - ay)
    if gap < 1e-6:
        raise ValueError('チェックポイントの 2 本のマーカーが同じ位置です')
    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
    nx, ny = -(by - ay) / gap, (bx - ax) / gap             # 線分に垂直な向き
    if (came_from[0] - mx) * nx + (came_from[1] - my) * ny > 0.0:
        nx, ny = -nx, -ny                                  # 手前（来た側）を -n に
    pre = (mx - nx * before, my - ny * before)
    post = (mx + nx * after, my + ny * after)
    return pre, post, gap


def crossed_fully(path: List[XY], a: XY, b: XY, half_length: float) -> bool:
    """通り道（ローバーの中心）が線分 a–b を横切り、そのあと half_length 以上先まで抜けたか。

    ローバーの中心が線分から half_length 進めば、機体全体が線分を跨ぎ切ったとみなす（ルール 1.2.6 B）。
    """
    ax, ay = a
    bx, by = b
    L = math.hypot(bx - ax, by - ay)
    ux, uy = (bx - ax) / L, (by - ay) / L

    def side(p):                   # 線分の直線からの符号付き距離
        return (p[0] - ax) * (-uy) + (p[1] - ay) * ux

    def along(p):                  # 線分に沿った位置（0〜L が線分の範囲）
        return (p[0] - ax) * ux + (p[1] - ay) * uy

    for i in range(1, len(path)):
        s0, s1 = side(path[i - 1]), side(path[i])
        if s0 == s1 or (s0 > 0) == (s1 > 0):
            continue
        t = s0 / (s0 - s1)
        cross = (path[i - 1][0] + t * (path[i][0] - path[i - 1][0]),
                 path[i - 1][1] + t * (path[i][1] - path[i - 1][1]))
        if not (0.0 <= along(cross) <= L):
            continue                                       # 線分の外（マーカーの外側）を通った
        sign = 1.0 if s1 > 0 else -1.0
        for p in path[i:]:
            if sign * side(p) >= half_length:
                return True
            if sign * side(p) < 0:
                break                                      # 跨ぎ切る前に戻った
    return False
