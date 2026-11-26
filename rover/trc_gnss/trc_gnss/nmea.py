"""NMEA GGA センテンスの解析と、RTKの状態に応じた誤差（共分散）の計算。

ROS に依存しない純粋な Python なので、単体テスト（test/test_nmea.py）で動作を確認できる。
"""

from dataclasses import dataclass
from typing import Optional

# GGA の「Fix quality」（受信機が報告する測位の状態）
QUALITY_INVALID = 0      # 測位できていない
QUALITY_SINGLE = 1       # 単独測位（数m級）
QUALITY_DGPS = 2         # DGPS / SBAS（1m前後）
QUALITY_RTK_FIXED = 4    # RTK Fix（cm級）
QUALITY_RTK_FLOAT = 5    # RTK Float（数十cm級）
QUALITY_DEAD_RECKONING = 6

QUALITY_NAMES = {
    QUALITY_INVALID: 'invalid',
    QUALITY_SINGLE: 'single',
    QUALITY_DGPS: 'dgps',
    QUALITY_RTK_FIXED: 'rtk_fixed',
    QUALITY_RTK_FLOAT: 'rtk_float',
    QUALITY_DEAD_RECKONING: 'dead_reckoning',
}


@dataclass
class GgaFix:
    utc: str                  # hhmmss.ss
    latitude: float           # 度（北が正）
    longitude: float          # 度（東が正）
    quality: int              # QUALITY_*
    num_satellites: int
    hdop: float
    altitude: float           # 平均海面からの高さ [m]
    geoid_separation: float   # ジオイド高 [m]（楕円体高 = altitude + geoid_separation）

    @property
    def ellipsoid_height(self) -> float:
        return self.altitude + self.geoid_separation


def checksum_ok(sentence: str) -> bool:
    """`$...*HH` 形式のチェックサムを確認する。"""
    sentence = sentence.strip()
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    body, _, given = sentence[1:].partition('*')
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def _to_degrees(value: str, hemisphere: str, deg_digits: int) -> float:
    """ddmm.mmmm（緯度）/ dddmm.mmmm（経度）を度に直す。"""
    degrees = float(value[:deg_digits])
    minutes = float(value[deg_digits:])
    result = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        result = -result
    return result


def gga_quality(sentence: str) -> Optional[int]:
    """GGA センテンスの Fix quality だけを返す（測位できていない 0 も返す）。GGA でなければ None。"""
    sentence = sentence.strip()
    if not checksum_ok(sentence):
        return None
    fields = sentence[1:].split('*')[0].split(',')
    if len(fields) < 7 or not fields[0].endswith('GGA'):
        return None
    try:
        return int(fields[6]) if fields[6] else QUALITY_INVALID
    except ValueError:
        return None


def parse_gga(sentence: str) -> Optional[GgaFix]:
    """GGA センテンスを解析する。GGA 以外・チェックサム不一致・測位なしは None を返す。

    $GNGGA / $GPGGA などトーカーIDは問わない。
    """
    sentence = sentence.strip()
    if not checksum_ok(sentence):
        return None
    fields = sentence[1:].split('*')[0].split(',')
    if len(fields) < 15 or not fields[0].endswith('GGA'):
        return None
    try:
        quality = int(fields[6]) if fields[6] else QUALITY_INVALID
        if quality == QUALITY_INVALID or not fields[2] or not fields[4]:
            return None
        return GgaFix(
            utc=fields[1],
            latitude=_to_degrees(fields[2], fields[3], 2),
            longitude=_to_degrees(fields[4], fields[5], 3),
            quality=quality,
            num_satellites=int(fields[7]) if fields[7] else 0,
            hdop=float(fields[8]) if fields[8] else 99.0,
            altitude=float(fields[9]) if fields[9] else 0.0,
            geoid_separation=float(fields[11]) if fields[11] else 0.0,
        )
    except ValueError:
        return None


# RTK の状態ごとの水平方向の標準偏差 [m]（初期値。試験走行の結果を見て調整する）
DEFAULT_SIGMA = {
    QUALITY_RTK_FIXED: 0.02,
    QUALITY_RTK_FLOAT: 0.30,
    QUALITY_DGPS: 0.80,
    QUALITY_SINGLE: 2.50,
    QUALITY_DEAD_RECKONING: 10.0,
}


def horizontal_sigma(fix: GgaFix, sigma_table: dict) -> float:
    """RTK の状態から水平方向の標準偏差 [m] を決める。

    RTK（Fix/Float）以外は HDOP が大きいほど誤差を大きく見積もる。
    """
    sigma = sigma_table.get(fix.quality, sigma_table.get(QUALITY_SINGLE, 2.5))
    if fix.quality not in (QUALITY_RTK_FIXED, QUALITY_RTK_FLOAT):
        sigma *= max(fix.hdop, 1.0)
    return sigma


def position_covariance(fix: GgaFix, sigma_table: dict, vertical_scale: float = 2.0) -> list:
    """NavSatFix.position_covariance（ENU、行優先の3x3）を返す。"""
    h = horizontal_sigma(fix, sigma_table)
    v = h * vertical_scale
    return [h * h, 0.0, 0.0,
            0.0, h * h, 0.0,
            0.0, 0.0, v * v]
