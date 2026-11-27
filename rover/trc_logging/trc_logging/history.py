"""制御履歴の1行を組み立てる（ROS に依存しない。test/test_history.py で確認できる）。

大会規定：時刻・位置・方位・制御指令を 1 Hz 以上で記録し、提出・説明できること。
"""

import math
from typing import Optional

COLUMNS = [
    'time',            # 記録した時刻（ISO 8601、タイムゾーン付き）
    'ros_time',        # ROS の時刻 [s]（rosbag 再生時は記録当時の時刻）
    'latitude',        # 推定位置（度）
    'longitude',
    'map_x',           # map 座標の位置 [m]（スタート付近が原点）
    'map_y',
    'heading_deg',     # 方位（北が 0 度、時計回り）。向きが決まる前は空欄
    'speed_mps',       # 速さ [m/s]
    'gnss_quality',    # 4 = RTK Fix、5 = RTK Float、1 = 単独、0 = 測位なし
    'cmd_linear',      # 制御指令：前後の速さ [m/s]
    'cmd_angular',     # 制御指令：旋回の速さ [rad/s]
    'cmd_age_sec',     # 最後に制御指令を受け取ってからの時間 [s]
    'control_mode',    # 運転の種類（auto / command / remote など。送られていなければ空欄）
    'mission_status',  # ミッションの状態（/mission/status。例「CP1 の奥（通過） に着きました（2/5）」）
]


def enu_yaw_to_compass_deg(yaw_enu: float) -> float:
    """ENU の向き（東が 0、反時計回り、rad）を、方位（北が 0、時計回り、度）に直す。"""
    return (90.0 - math.degrees(yaw_enu)) % 360.0


def _fmt(value, digits: int) -> str:
    if value is None:
        return ''
    if isinstance(value, float):
        if math.isnan(value):
            return ''
        return f'{value:.{digits}f}'
    return str(value)


def format_row(time_iso: str, ros_time: float,
               latitude: Optional[float], longitude: Optional[float],
               map_x: Optional[float], map_y: Optional[float],
               yaw_enu: Optional[float], speed: Optional[float],
               gnss_quality: Optional[int],
               cmd_linear: Optional[float], cmd_angular: Optional[float],
               cmd_age: Optional[float], control_mode: Optional[str],
               mission_status: Optional[str] = None) -> list:
    heading = None if yaw_enu is None else enu_yaw_to_compass_deg(yaw_enu)
    return [
        time_iso,
        _fmt(ros_time, 3),
        _fmt(latitude, 9),     # 1e-9 度 ≈ 0.1 mm
        _fmt(longitude, 9),
        _fmt(map_x, 3),
        _fmt(map_y, 3),
        _fmt(heading, 1),
        _fmt(speed, 3),
        _fmt(gnss_quality, 0),
        _fmt(cmd_linear, 3),
        _fmt(cmd_angular, 3),
        _fmt(cmd_age, 2),
        control_mode or '',
        mission_status or '',
    ]
