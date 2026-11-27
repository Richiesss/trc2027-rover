"""GNSS ドライバ（大塚さんの gnss_fix）の出力を、位置推定に渡せる形に整えるノード。

gnss_fix は測位の状態でトピックを分けて出す:
  /gnss        RTK FIXED に達していない測位（未測位・単独・RTK FLOAT など）
  /gnss/fix    RTK FIXED
  /gnss/clas   RTK FIXED で、同じ周期に CLAS の補正を使ったことを確認したもの
この 3 つを受け取り、次を出す:
  /gnss/checked  sensor_msgs/NavSatFix  有効な測位だけ。位置推定（navsat_transform・向きの初期化）はこれを使う
  /gnss/quality  std_msgs/UInt8         GGA の Fix quality と同じ番号（制御履歴の記録に使う）
                                        0 未測位 / 1 単独 / 2 DGPS・SBAS / 4 RTK Fix / 5 RTK Float

整えること:
  - 精度（共分散）が不明（COVARIANCE_TYPE_UNKNOWN や 0）のときは、測位の種類に応じた控えめな値で埋める。
    0 のまま EKF に入れると「完全に正確」と扱われ、単独測位でも RTK と同じように信じてしまう。
    向きの初期化も「精度 5 cm 以下 = RTK」で判定しているので、単独測位を RTK と取り違える
  - 精度が小さすぎる値（min_sigma 未満）は min_sigma にする（受信機が楽観的すぎるときの保険）
  - frame_id を URDF のアンテナ座標系（gnss_link）にそろえる
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8

QUALITY_NONE, QUALITY_SINGLE, QUALITY_DGPS, QUALITY_RTK_FIX, QUALITY_RTK_FLOAT = 0, 1, 2, 4, 5

# 精度が不明なときに使う水平の 1σ [m]（gnss_fix の GGA だけのとき、など）。上下はこの 2 倍
UNKNOWN_SIGMA = {QUALITY_RTK_FIX: 0.05, QUALITY_RTK_FLOAT: 0.5, QUALITY_DGPS: 1.5, QUALITY_SINGLE: 3.0}


def quality_of(route: str, status: int) -> int:
    """届いたトピック（'gnss' / 'fix' / 'clas'）と status から、GGA の Fix quality の番号を決める。"""
    if status < NavSatStatus.STATUS_FIX:
        return QUALITY_NONE
    if route in ('fix', 'clas'):
        return QUALITY_RTK_FIX
    if status == NavSatStatus.STATUS_GBAS_FIX:
        return QUALITY_RTK_FLOAT          # gnss_fix は RTK FLOAT を GBAS_FIX で /gnss に出す
    if status == NavSatStatus.STATUS_SBAS_FIX:
        return QUALITY_DGPS
    return QUALITY_SINGLE


def covariance_ok(msg: NavSatFix) -> bool:
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return False
    c = msg.position_covariance
    return all(math.isfinite(c[i]) and c[i] > 0.0 for i in (0, 4, 8))


def checked_covariance(msg: NavSatFix, quality: int, min_sigma: float):
    """位置推定に渡す (共分散 9 要素, 種類, 埋めたか) を返す。"""
    if not covariance_ok(msg):
        s = UNKNOWN_SIGMA.get(quality, UNKNOWN_SIGMA[QUALITY_SINGLE])
        cov = [s * s, 0.0, 0.0, 0.0, s * s, 0.0, 0.0, 0.0, 4.0 * s * s]
        return cov, NavSatFix.COVARIANCE_TYPE_APPROXIMATED, True
    cov = list(msg.position_covariance)
    floor = min_sigma * min_sigma
    for i, f in ((0, 1.0), (4, 1.0), (8, 4.0)):
        cov[i] = max(cov[i], f * floor)
    return cov, msg.position_covariance_type, False


class GnssAdapter(Node):

    def __init__(self):
        super().__init__('gnss_adapter')
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        self.min_sigma = float(self.declare_parameter('min_sigma', 0.01).value)
        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/checked', 10)
        self.quality_pub = self.create_publisher(UInt8, 'gnss/quality', 10)
        for topic, route in (('gnss', 'gnss'), ('gnss/fix', 'fix'), ('gnss/clas', 'clas')):
            self.create_subscription(NavSatFix, topic, lambda m, r=route: self.on_fix(m, r), 10)
        self.filled = 0
        self.last_quality = None

    def on_fix(self, msg: NavSatFix, route: str):
        quality = quality_of(route, msg.status.status)
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            quality = QUALITY_NONE
        self.quality_pub.publish(UInt8(data=quality))
        if quality != self.last_quality:
            names = {0: '未測位', 1: '単独測位', 2: 'DGPS/SBAS', 4: 'RTK Fix', 5: 'RTK Float'}
            via = '（CLAS）' if route == 'clas' else ''
            self.get_logger().info(f'GNSS: {names.get(quality, quality)}{via}')
            self.last_quality = quality
        if quality == QUALITY_NONE:
            return
        out = NavSatFix()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.status = msg.status
        out.latitude, out.longitude = msg.latitude, msg.longitude
        out.altitude = msg.altitude if math.isfinite(msg.altitude) else 0.0
        cov, kind, filled = checked_covariance(msg, quality, self.min_sigma)
        out.position_covariance = cov
        out.position_covariance_type = kind
        if filled:
            self.filled += 1
            self.get_logger().warn(
                f'GNSS の精度が不明なので {math.sqrt(cov[0]):.2f} m とみなしました（累計 {self.filled} 回）',
                throttle_duration_sec=10.0)
        self.fix_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GnssAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
