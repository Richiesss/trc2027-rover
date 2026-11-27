"""砂地の地形と、砂地での車体の動き（ROS に依存しない）。シミュレーターの砂地の試験用。

地形の種類（terrain）:
  flat   : これまでと同じ、ゆるい起伏（±8 cm、波長 4 m）だけ
  dunes  : 通り道に、登れる砂丘が 2 つ（高さ 0.6 m・0.8 m。細かい起伏と合わせて、いちばん急な所で 13° ほど）
  steep  : 通り道を、登れない急な砂丘（高さ 1.5 m、40° ほど）がふさぐ。左右の端は回り込める
  pit    : 通り道に、深さ 1 m・半径 1.5 m のくぼみ（崖のふち）。落ちたら終わり

地形の特徴は map 座標（スタート地点が原点、x がスタート時の前方、y が左）で置く。
起伏の高さは、これまでの地面と同じく world（ENU）座標の式で足す。

車体の動き:
  - 車体は地面に沿って傾く（前後・左右の高さの差から、前後の傾き pitch と左右の傾き roll を出す）
  - 登り坂ほどタイヤが空転する（slip_start_deg で空転が増え始め、max_climb_deg で 1 = まったく進まない）
  - やわらかい砂（soft_sand）：見た目では分からない。中に入るほど空転が増え、奥では前に進めなくなる。
    後ろへは（自分のわだちを戻るので）抜け出せる
  - 傾きが tip_deg を超えたら転倒、くぼみの底へ落ちたら転落として、それ以降は動かない
"""

import math
from dataclasses import dataclass, field

import numpy as np

TERRAINS = ('flat', 'dunes', 'steep', 'pit')

# 地形の特徴（map 座標）。(中心 x, 中心 y, 高さ, 前後の広がり σx, 左右の広がり σy)。高さが負ならくぼみ
FEATURES = {
    'flat': [],
    'dunes': [('hill', 6.0, 0.0, 0.6, 3.0, 3.0), ('hill', 12.5, 1.0, 0.8, 3.5, 3.5)],
    'steep': [('hill', 8.0, 0.0, 1.5, 1.0, 2.5)],
    'pit': [('pit', 8.0, 0.0, -1.0, 1.5, 1.5)],
}


def ripple(e, n, amplitude: float = 0.08, wavelength: float = 4.0):
    """これまでの地面のゆるい起伏（world 座標の式。fake_world.make_ground と同じ）。"""
    k = 2.0 * math.pi / wavelength
    return amplitude * (np.sin(k * e) * np.cos(0.7 * k * n) + 0.5 * np.sin(1.9 * k * n + 0.3))


class Terrain:
    """地面の高さ h(e, n)（world 座標）。heading0 はスタートの向き（map 座標と world の換算に使う）。"""

    def __init__(self, kind: str = 'flat', heading0: float = 0.0):
        if kind not in TERRAINS:
            raise ValueError(f'terrain は {TERRAINS} のどれか: {kind}')
        self.kind = kind
        self.heading0 = heading0
        self.features = FEATURES[kind]

    @property
    def flat(self) -> bool:
        return not self.features

    def to_map(self, e, n):
        c, s = math.cos(-self.heading0), math.sin(-self.heading0)
        return c * e - s * n, s * e + c * n

    def height(self, e, n):
        e = np.asarray(e, dtype=float)
        n = np.asarray(n, dtype=float)
        z = ripple(e, n)
        if self.features:
            mx, my = self.to_map(e, n)
            for kind, cx, cy, h, sx, sy in self.features:
                d2 = ((mx - cx) / sx) ** 2 + ((my - cy) / sy) ** 2
                if kind == 'hill':
                    z = z + h * np.exp(-0.5 * d2)
                else:                                    # くぼみ：壁が急で、底が平ら
                    z = z + h / (1.0 + d2 ** 4)
        return z

    def pose(self, e: float, n: float, yaw: float, half_length: float = 0.5, half_width: float = 0.4):
        """(e, n) に向き yaw で置いた車体の (高さ, pitch, roll)。

        pitch は ROS と同じ向き（正 = 機首下げ）、roll は正 = 左が上。
        """
        c, s = math.cos(yaw), math.sin(yaw)
        hf = float(self.height(e + half_length * c, n + half_length * s))
        hb = float(self.height(e - half_length * c, n - half_length * s))
        hl = float(self.height(e - half_width * s, n + half_width * c))
        hr = float(self.height(e + half_width * s, n - half_width * c))
        z = float(self.height(e, n))
        pitch = math.atan2(hb - hf, 2.0 * half_length)
        roll = math.atan2(hl - hr, 2.0 * half_width)
        return z, pitch, roll

    def occluded(self, sensor: np.ndarray, points: np.ndarray, samples: int = 12,
                 clearance: float = 0.03) -> np.ndarray:
        """センサから見て、地形（砂丘の稜線・くぼみのふち）の陰になる点（world 座標）。

        センサと点を結ぶ線の途中で、地面がその線より高ければ陰とする。物体どうしの陰は考えない。
        """
        if self.flat or len(points) == 0:
            return np.zeros(len(points), dtype=bool)
        t = np.linspace(0.08, 0.95, samples)[None, :]
        px = sensor[0] + t * (points[:, 0:1] - sensor[0])
        py = sensor[1] + t * (points[:, 1:2] - sensor[1])
        pz = sensor[2] + t * (points[:, 2:3] - sensor[2])
        return np.any(self.height(px, py) > pz + clearance, axis=1)

    def in_pit(self, e: float, n: float, depth: float = 0.4) -> bool:
        """くぼみの中（まわりより depth 以上低い所）にいるか。"""
        if self.kind != 'pit':
            return False
        return float(self.height(e, n) - ripple(e, n)) < -depth


def rotation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """車体 → world の回転行列 Rz(yaw) Ry(pitch) Rx(roll)。"""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def parse_circles(text: str):
    """'7,0,1.5; 12,2,1' → [(7.0, 0.0, 1.5), (12.0, 2.0, 1.0)]（map 座標の x, y, 半径）"""
    out = []
    for part in (text or '').split(';'):
        part = part.strip()
        if part:
            x, y, r = (float(v) for v in part.split(','))
            out.append((x, y, r))
    return out


@dataclass
class SandParams:
    slip: float = 0.0               # 平らな所でも起きる空転の割合（砂地なら 0.1〜0.2）
    slip_start_deg: float = 10.0    # 登り坂がこの角度を超えると空転が増え始める
    max_climb_deg: float = 22.0     # この角度の登り坂では、まったく進まない
    soft_sand: list = field(default_factory=list)   # やわらかい砂 [(x, y, 半径)]（map 座標）
    soft_depth: float = 0.6         # やわらかい砂のふちから、この距離入ると前に進めなくなる [m]
    reverse_slip: float = 0.3       # やわらかい砂の中で、後ろへ戻るときの空転
    tip_deg: float = 30.0           # 傾きがこれを超えたら転倒


def slip_ratio(p: SandParams, v: float, pitch: float, soft_depth_in: float) -> float:
    """空転の割合（0 = 指令どおり進む、1 = まったく進まない）。

    v: 指令の前進の速さ（負なら後退）。pitch: 機首下げが正。soft_depth_in: やわらかい砂のふちから中へ入った距離（外なら負）。
    """
    climb = math.degrees(-pitch if v >= 0.0 else pitch)       # 進む向きの登りの角度
    s = p.slip
    if climb > p.slip_start_deg:
        s += (1.0 - p.slip) * min(1.0, (climb - p.slip_start_deg) / (p.max_climb_deg - p.slip_start_deg))
    if soft_depth_in > 0.0:
        s = max(s, p.reverse_slip if v < 0.0 else min(1.0, soft_depth_in / p.soft_depth))
    return min(max(s, 0.0), 1.0)


def soft_depth_in(p: SandParams, mx: float, my: float) -> float:
    """やわらかい砂のふちから中へ入った距離 [m]（どの砂の外でも負）。"""
    best = -1.0
    for cx, cy, r in p.soft_sand:
        best = max(best, r - math.hypot(mx - cx, my - cy))
    return best


class Dust:
    """風で舞い上がった砂（world 座標の点の塊）。

    スキャンごとに平均 rate 個の塊が、ローバーから 1.0〜6.0 m の所、地面から 0.1〜1.0 m の高さに現れる。
    1 つの塊は 10〜40 点で、1〜2 回のスキャンのあいだ見え、風（wind [m/s]、東向き）に流される。
    """

    def __init__(self, rate: float, terrain: Terrain, seed: int = 0, wind=(0.8, 0.3)):
        self.rate = rate
        self.terrain = terrain
        self.rng = np.random.default_rng(seed + 3000)
        self.wind = np.array([wind[0], wind[1], 0.0])
        self.clusters = []                       # [点 (M x 3), 残りのスキャン数]

    def step(self, e: float, n: float, dt: float = 0.1) -> np.ndarray:
        """1 回のスキャン分の砂の点（world 座標）。"""
        alive = []
        for pts, life in self.clusters:
            if life > 1:
                alive.append([pts + self.wind * dt, life - 1])
        k = self.rng.poisson(self.rate) if self.rate > 0 else 0
        for _ in range(k):
            r = self.rng.uniform(1.0, 6.0)
            a = self.rng.uniform(-math.pi, math.pi)
            cx, cy = e + r * math.cos(a), n + r * math.sin(a)
            cz = float(self.terrain.height(cx, cy)) + self.rng.uniform(0.1, 1.0)
            m = int(self.rng.integers(10, 41))
            pts = np.array([cx, cy, cz]) + self.rng.normal(0.0, 0.12, (m, 3))
            alive.append([pts, int(self.rng.integers(1, 3))])
        self.clusters = alive
        return np.vstack([p for p, _ in alive]) if alive else np.empty((0, 3))
