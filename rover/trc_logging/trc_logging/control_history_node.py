"""大会提出用の制御履歴を CSV に記録するノード。

大会規定：時刻・位置・方位・制御指令を 1 Hz 以上で記録し、提出・説明できること。
決まった間隔（既定 2 Hz）で、その時点の最新の値を1行ずつ書く。データが来ていない項目は空欄にする
（空欄があれば「その時間はセンサが止まっていた」と後から分かる）。

購読:
  /gps/filtered     sensor_msgs/NavSatFix  推定位置（緯度経度）
  /odometry/global  nav_msgs/Odometry      map 座標の位置と速さ
  /heading/imu      sensor_msgs/Imu        ENU 基準の向き（heading_init の出力）
  /gnss/quality     std_msgs/UInt8         RTK の状態
  /cmd_vel          geometry_msgs/Twist    制御指令
  /control_mode     std_msgs/String        運転の種類（任意）
  /mission/status   std_msgs/String        ミッションの状態（mission_node が出す。任意）
出力:
  ~/trc2027/logs/control_history_YYYYmmdd_HHMMSS.csv（1行ごとに書き出すので、電源が落ちても残る）
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, UInt8

from trc_logging import history


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ControlHistoryNode(Node):

    def __init__(self):
        super().__init__('control_history')
        # launch から 5 のような整数で渡されても受け付けられるようにする
        rate = float(self.declare_parameter(
            'rate', 2.0, ParameterDescriptor(dynamic_typing=True)).value)
        out_dir = os.path.expanduser(self.declare_parameter('out_dir', '~/trc2027/logs').value)
        cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value

        self.fix = None
        self.odom = None
        self.yaw_enu = None
        self.quality = None
        self.cmd = None
        self.cmd_time = None
        self.mode = None

        self.create_subscription(NavSatFix, '/gps/filtered', self.on_fix, 10)
        self.create_subscription(Odometry, '/odometry/global', self.on_odom, 10)
        self.create_subscription(Imu, '/heading/imu', self.on_heading, 10)
        self.create_subscription(UInt8, '/gnss/quality', self.on_quality, 10)
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(String, '/control_mode', self.on_mode, 10)
        self.mission = None
        self.create_subscription(String, '/mission/status', self.on_mission,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, datetime.now().strftime('control_history_%Y%m%d_%H%M%S.csv'))
        self.file = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(history.COLUMNS)
        self.file.flush()
        self.rows = 0

        self.create_timer(1.0 / rate, self.write_row)
        self.get_logger().info(f'制御履歴を {rate:.1f} Hz で記録します: {self.path}')

    def on_fix(self, msg):
        self.fix = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_heading(self, msg):
        self.yaw_enu = yaw_from_quaternion(msg.orientation)

    def on_quality(self, msg):
        self.quality = msg.data

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def on_mode(self, msg):
        self.mode = msg.data

    def on_mission(self, msg):
        self.mission = msg.data

    def write_row(self):
        now = self.get_clock().now()
        cmd_age = None
        if self.cmd_time is not None:
            cmd_age = (now - self.cmd_time).nanoseconds / 1e9
        speed = None
        if self.odom is not None:
            v = self.odom.twist.twist.linear
            speed = math.hypot(v.x, v.y)
        row = history.format_row(
            time_iso=datetime.now().astimezone().isoformat(timespec='milliseconds'),
            ros_time=now.nanoseconds / 1e9,
            latitude=self.fix.latitude if self.fix else None,
            longitude=self.fix.longitude if self.fix else None,
            map_x=self.odom.pose.pose.position.x if self.odom else None,
            map_y=self.odom.pose.pose.position.y if self.odom else None,
            yaw_enu=self.yaw_enu,
            speed=speed,
            gnss_quality=self.quality,
            cmd_linear=self.cmd.linear.x if self.cmd else None,
            cmd_angular=self.cmd.angular.z if self.cmd else None,
            cmd_age=cmd_age,
            control_mode=self.mode,
            mission_status=self.mission,
        )
        self.writer.writerow(row)
        self.file.flush()
        self.rows += 1

    def destroy_node(self):
        self.file.close()
        # Ctrl+C のあとは ROS のログが使えないので、画面に直接出す
        print(f'[control_history] {self.rows} 行を記録しました: {self.path}', flush=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlHistoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C で正常終了する
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
