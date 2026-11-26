"""シミュレーション用：推定位置と本当の位置の水平誤差 [m] を配信する（グラフ表示用）。

購読: /gps/filtered（推定、NavSatFix）、/fake/truth（真値、NavSatFix）
配信: /fake/position_error（std_msgs/Float64）

真値は 5 Hz なので、推定の時刻に合わせて前後の真値を直線で補間してから比べる。
"""

import bisect
import math
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from trc_localization import heading_math as hm


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PositionError(Node):

    def __init__(self):
        super().__init__('position_error')
        self.truth = deque(maxlen=200)        # (t, lat, lon)
        self.pending = deque(maxlen=200)      # 真値が追いつくのを待つ推定値
        self.create_subscription(NavSatFix, '/fake/truth', self.on_truth, 50)
        self.create_subscription(NavSatFix, '/gps/filtered', self.on_est, 50)
        self.pub = self.create_publisher(Float64, '/fake/position_error', 10)

    def on_truth(self, msg):
        self.truth.append((stamp_sec(msg), msg.latitude, msg.longitude))
        self.flush()

    def on_est(self, msg):
        self.pending.append((stamp_sec(msg), msg.latitude, msg.longitude))
        self.flush()

    def flush(self):
        if len(self.truth) < 2:
            return
        ts = [t[0] for t in self.truth]
        while self.pending:
            t, lat, lon = self.pending[0]
            if t > ts[-1]:
                return                         # まだ後ろの真値が来ていない
            self.pending.popleft()
            i = bisect.bisect_left(ts, t)
            if i == 0:
                continue                       # 古すぎて補間できない
            (t0, a0, o0), (t1, a1, o1) = self.truth[i - 1], self.truth[i]
            w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            e, n = hm.latlon_to_local_en(a0 + w * (a1 - a0), o0 + w * (o1 - o0), lat, lon)
            self.pub.publish(Float64(data=math.hypot(e, n)))


def main(args=None):
    rclpy.init(args=args)
    node = PositionError()
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
