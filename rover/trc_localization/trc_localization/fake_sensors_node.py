"""実機なしで位置推定を試すための、にせのセンサ。

ローバーが「東から heading_deg 度の向き」でスタートし、まっすぐ進んでからゆるく曲がる、
という走行を作り、次の3つを配信する（実機と同じトピック名）:

  /lidar/odom    nav_msgs/Odometry   LiDARオドメトリの代わり（スタート地点・向きが原点）
  /unilidar/imu  sensor_msgs/Imu     回転の速さだけ
  /gnss/fix      sensor_msgs/NavSatFix  RTK Fix 相当（誤差 sigma_gnss）

あわせて /fake/truth（NavSatFix）に本当の位置を出すので、
/gps/filtered（推定位置を緯度経度に戻したもの）と比べれば、推定の良し悪しが分かる。
"""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from trc_localization import heading_math as hm


class FakeSensors(Node):

    def __init__(self):
        super().__init__('fake_sensors')
        self.heading0 = math.radians(self.declare_parameter('heading_deg', 90.0).value)
        self.speed = self.declare_parameter('speed', 0.5).value
        self.straight_time = self.declare_parameter('straight_time', 12.0).value
        self.turn_rate = self.declare_parameter('turn_rate', 0.15).value
        self.lat0 = self.declare_parameter('lat0', 35.5400).value
        self.lon0 = self.declare_parameter('lon0', 134.2300).value
        self.sigma_gnss = self.declare_parameter('sigma_gnss', 0.02).value

        self.odom_pub = self.create_publisher(Odometry, '/lidar/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/unilidar/imu', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.truth_pub = self.create_publisher(NavSatFix, '/fake/truth', 10)

        # 本当の状態（ENU: 東 x, 北 y）
        self.t = 0.0
        self.e, self.n, self.yaw = 0.0, 0.0, self.heading0
        self.dt = 0.02
        self.tick = 0
        self.create_timer(self.dt, self.step)

    def yaw_rate(self):
        return 0.0 if self.t < self.straight_time else self.turn_rate

    def step(self):
        w = self.yaw_rate()
        self.yaw += w * self.dt
        self.e += self.speed * math.cos(self.yaw) * self.dt
        self.n += self.speed * math.sin(self.yaw) * self.dt
        self.t += self.dt
        self.tick += 1
        now = self.get_clock().now().to_msg()

        imu = Imu()                                 # 50 Hz
        imu.header.stamp = now
        imu.header.frame_id = 'unilidar_imu'
        imu.angular_velocity.z = w
        imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.orientation_covariance[0] = -1.0         # 向きは出さない（北の基準がない）
        self.imu_pub.publish(imu)

        if self.tick % 5 == 0:                       # 10 Hz
            # オドメトリ座標 = スタート地点・スタート時の向きが基準
            c, s = math.cos(-self.heading0), math.sin(-self.heading0)
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = c * self.e - s * self.n
            odom.pose.pose.position.y = s * self.e + c * self.n
            qx, qy, qz, qw = hm.quaternion_from_yaw(self.yaw - self.heading0)
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
            cov = [0.0] * 36
            cov[0] = cov[7] = 1e-4
            cov[35] = 1e-4
            odom.pose.covariance = cov
            self.odom_pub.publish(odom)

        if self.tick % 10 == 0:                      # 5 Hz
            self.gnss_pub.publish(self.navsat(now, self.sigma_gnss, 'gnss_link'))
            self.truth_pub.publish(self.navsat(now, 0.0, 'base_link'))

    def navsat(self, stamp, sigma, frame):
        e = self.e + random.gauss(0.0, sigma) if sigma > 0 else self.e
        n = self.n + random.gauss(0.0, sigma) if sigma > 0 else self.n
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self.lat0 + math.degrees(n / hm.EARTH_RADIUS)
        msg.longitude = self.lon0 + math.degrees(
            e / (hm.EARTH_RADIUS * math.cos(math.radians(self.lat0))))
        msg.altitude = 30.0
        var = max(sigma, 0.02) ** 2
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4 * var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensors()
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
