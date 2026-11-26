"""スタート直後の走行から「ローバーが東西南北のどちらを向いているか」を求めるノード。

アンテナ1本の GNSS では止まっているときの向きが分からず、L2 の内蔵 IMU にも北の基準がない。
そこで、RTK Fix の状態で数 m 走ったときの「GNSS の移動」と「オドメトリの移動」を比べて、
オドメトリの座標と東西南北（ENU、東が 0 rad・反時計回りが正）のずれを求める。

求まったら、ENU 基準の向きを sensor_msgs/Imu の orientation として配信し続ける。
robot_localization の navsat_transform はこの向きを使って、GNSS の座標を map 座標に重ねる
（navsat_transform は3つの入力 ― GNSS・向き・オドメトリ ― がそろうまで待つので、
 向きが求まるまでは GNSS が地図に入らない）。

購読:
  gnss/fix       sensor_msgs/NavSatFix   trc_gnss の出力
  odometry       nav_msgs/Odometry        EKF の出力（既定は /odometry/global）
配信:
  heading/imu    sensor_msgs/Imu          ENU 基準の向き（orientation だけ有効）
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from trc_localization import heading_math as hm


class HeadingInitNode(Node):

    def __init__(self):
        super().__init__('heading_init')
        self.min_distance = self.declare_parameter('min_distance', 3.0).value
        self.max_length_mismatch = self.declare_parameter('max_length_mismatch', 0.5).value
        self.max_sigma = self.declare_parameter('max_horizontal_sigma', 0.05).value
        self.yaw_sigma = math.radians(self.declare_parameter('yaw_sigma_deg', 2.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        publish_rate = self.declare_parameter('publish_rate', 10.0).value

        self.create_subscription(NavSatFix, 'gnss/fix', self.on_fix, 10)
        self.create_subscription(Odometry, 'odometry', self.on_odom, 20)
        self.pub = self.create_publisher(Imu, 'heading/imu', 10)
        self.create_timer(1.0 / publish_rate, self.publish_heading)

        self.latest_odom = None      # (x, y, yaw)
        self.start = None            # (lat, lon, odom_x, odom_y)
        self.rotation = None         # オドメトリ座標 → ENU の回転 [rad]
        self.get_logger().info(
            f'RTK Fix で {self.min_distance:.1f} m 以上走ると向きを決めます')

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.latest_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                            hm.yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_fix(self, msg: NavSatFix):
        if self.rotation is not None or self.latest_odom is None:
            return
        sigma = math.sqrt(max(msg.position_covariance[0], 0.0))
        good = msg.status.status >= NavSatStatus.STATUS_FIX and sigma <= self.max_sigma
        if not good:
            self.start = None        # RTK Fix が切れたら測り直す
            return

        ox, oy, _ = self.latest_odom
        if self.start is None:
            self.start = (msg.latitude, msg.longitude, ox, oy)
            return

        lat0, lon0, ox0, oy0 = self.start
        gnss_delta = hm.latlon_to_local_en(lat0, lon0, msg.latitude, msg.longitude)
        odom_delta = (ox - ox0, oy - oy0)
        if math.hypot(*gnss_delta) < self.min_distance:
            return

        rotation = hm.estimate_rotation(gnss_delta, odom_delta,
                                        self.min_distance, self.max_length_mismatch)
        if rotation is None:
            self.get_logger().warn(
                'GNSS とオドメトリの移動距離が合わないので測り直します'
                f'（GNSS {math.hypot(*gnss_delta):.2f} m / オドメトリ {math.hypot(*odom_delta):.2f} m）')
            self.start = None
            return

        self.rotation = rotation
        yaw_enu = hm.normalize_angle(self.latest_odom[2] + rotation)
        self.get_logger().info(
            f'向きが決まりました: 東から反時計回りに {math.degrees(yaw_enu):.1f} 度'
            f'（オドメトリとのずれ {math.degrees(rotation):.1f} 度）')

    def publish_heading(self):
        if self.rotation is None or self.latest_odom is None:
            return
        yaw_enu = hm.normalize_angle(self.latest_odom[2] + self.rotation)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        qx, qy, qz, qw = hm.quaternion_from_yaw(yaw_enu)
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        msg.orientation_covariance = [1e-3, 0.0, 0.0,
                                      0.0, 1e-3, 0.0,
                                      0.0, 0.0, self.yaw_sigma ** 2]
        msg.angular_velocity_covariance[0] = -1.0     # 角速度・加速度は配信しない
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingInitNode()
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
