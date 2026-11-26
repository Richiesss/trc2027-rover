"""実機なしで位置推定を試すための、にせのセンサ。

ローバーが「東から heading_deg 度の向き」でスタートし、まっすぐ進んでからゆるく曲がる、
という走行を作り、次を配信する（実機と同じトピック名）:

  /lidar/odom     nav_msgs/Odometry      LiDARオドメトリの代わり（publish_odom:=true のとき）
  /unilidar/cloud sensor_msgs/PointCloud2 にせの点群（publish_cloud:=true のとき）。
                                          支柱・岩・砂山を置いた仮想フィールドを、ローバーから見た点群。
                                          KISS-ICP などの LiDAR オドメトリに入力して試せる
  /unilidar/imu   sensor_msgs/Imu        回転の速さだけ
  /gnss/fix       sensor_msgs/NavSatFix  RTK Fix 相当（誤差 sigma_gnss）
  /gnss/quality   std_msgs/UInt8         4（RTK Fix）

あわせて /fake/truth（NavSatFix）に本当の位置を出すので、
/gps/filtered（推定位置を緯度経度に戻したもの）と比べれば、推定の良し悪しが分かる。
"""

import math
import random

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import UInt8

from trc_localization import fake_world
from trc_localization import heading_math as hm


class FakeSensors(Node):

    def __init__(self):
        super().__init__('fake_sensors')
        any_number = ParameterDescriptor(dynamic_typing=True)   # 200 のような整数でも受け付ける
        self.heading0 = math.radians(float(self.declare_parameter('heading_deg', 90.0, any_number).value))
        # GNSS が途切れる区間（スタートからの秒数）。LiDAR オドメトリだけで位置を保てるかを試す
        self.outage_start = float(self.declare_parameter('gnss_outage_start', -1.0, any_number).value)
        self.outage_duration = float(self.declare_parameter('gnss_outage_duration', 10.0, any_number).value)
        self.speed = self.declare_parameter('speed', 0.5).value
        self.straight_time = self.declare_parameter('straight_time', 12.0).value
        self.turn_rate = self.declare_parameter('turn_rate', 0.15).value
        self.lat0 = self.declare_parameter('lat0', 35.5400).value
        self.lon0 = self.declare_parameter('lon0', 134.2300).value
        self.sigma_gnss = self.declare_parameter('sigma_gnss', 0.02).value
        self.publish_odom = self.declare_parameter('publish_odom', True).value
        self.publish_cloud = self.declare_parameter('publish_cloud', False).value
        # LiDAR の取付位置（trc_description の値と合わせる）と性能
        self.lidar_x = self.declare_parameter('lidar_x', 0.20).value
        self.lidar_z = self.declare_parameter('lidar_z', 0.40).value
        self.cloud_min_range = self.declare_parameter('cloud_min_range', 0.5).value
        self.cloud_max_range = self.declare_parameter('cloud_max_range', 25.0).value
        self.cloud_points = self.declare_parameter('cloud_points', 8000).value
        self.cloud_noise = self.declare_parameter('cloud_noise', 0.02).value
        self.cloud_ground = self.declare_parameter('cloud_ground', True).value

        self.odom_pub = self.create_publisher(Odometry, '/lidar/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/unilidar/imu', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.quality_pub = self.create_publisher(UInt8, '/gnss/quality', 10)
        self.truth_pub = self.create_publisher(NavSatFix, '/fake/truth', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/unilidar/cloud', 5)

        if self.publish_cloud:
            self.rng = np.random.default_rng(0)
            self.world = fake_world.make_world(seed=0)
            self.ground = fake_world.make_ground(seed=0) if self.cloud_ground else np.empty((0, 3))
            self.get_logger().info(f'仮想フィールドを作りました（{len(self.world)} 点）')

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

        if self.tick % 5 == 0 and self.publish_cloud:   # 10 Hz
            self.cloud_pub.publish(self.make_cloud(now))

        if self.tick % 5 == 0 and self.publish_odom:    # 10 Hz
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
            in_outage = self.outage_start >= 0.0 and \
                self.outage_start <= self.t < self.outage_start + self.outage_duration
            if not in_outage:
                self.gnss_pub.publish(self.navsat(now, self.sigma_gnss, 'gnss_link'))
                self.quality_pub.publish(UInt8(data=4))
            else:
                self.quality_pub.publish(UInt8(data=0))
            self.truth_pub.publish(self.navsat(now, 0.0, 'base_link'))

    def make_cloud(self, stamp):
        pts = fake_world.scan(self.world, self.ground, self.e, self.n, self.yaw,
                              self.lidar_x, self.lidar_z, self.cloud_min_range,
                              self.cloud_max_range, self.cloud_points, self.cloud_noise, self.rng)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'unilidar_lidar'
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        return msg

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
