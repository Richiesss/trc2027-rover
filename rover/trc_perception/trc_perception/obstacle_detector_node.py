"""LiDAR の点群から地面を取り除いて障害物を見つけ、前方の障害物までの距離を出すノード。

入力:
  /unilidar/cloud   sensor_msgs/PointCloud2（LiDAR 座標。TF で base_link に直す）
  /unilidar/imu     sensor_msgs/Imu（加速度計で重力の向きを求め、車体の傾きを打ち消す。level_with_imu）
出力（すべて base_link 座標）:
  /obstacles/points          sensor_msgs/PointCloud2  障害物の点
  /obstacles/ground          sensor_msgs/PointCloud2  地面とみなした点（表示用、publish_ground）
  /obstacles/front_distance  std_msgs/Float32         前方の帯に入った障害物までの距離 [m]。なければ corridor_length
  /obstacles/stop            std_msgs/Bool            front_distance < stop_distance のとき true
                                                      （一度 true になったら stop_distance + release_margin まで空くと false）
  /obstacles/corridor        visualization_msgs/Marker 前方の帯（緑：空いている／赤：止まるべき）
  /terrain/tilt              geometry_msgs/Vector3Stamped 車体の傾き [度]（x = roll 左が上で正、y = pitch 機首下げで正）
  /lidar/degenerate          std_msgs/Bool            点群に手がかり（地面以外の物）がほとんどない
                                                      （LiDAR オドメトリが前後・左右の動きを読めない。odom_to_twist が使う）

砂地の対策:
  - 傾きの補正：坂で車体が傾くと、平らな地面まで斜めの壁に見える。IMU の重力の向きで点群を水平に直してから判定する
    （出力の点は「向きは base_link のまま、水平にした座標」。x, y は base_link とほぼ同じ）
  - 舞い上がる砂：persist_scans 回続けて同じ所（odom 座標で比べる）に見えた点だけを障害物にする

いまは「見つけて知らせる」だけで、ローバーの動きは変えない（止める・避けるは走行制御側で使う）。
"""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from trc_perception import obstacle_math as om


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        d = om.SegmentParams()
        self.params = om.SegmentParams(
            cell_size=float(self.declare_parameter('cell_size', d.cell_size).value),
            neighbor=int(self.declare_parameter('neighbor', d.neighbor).value),
            height_threshold=float(self.declare_parameter('height_threshold', d.height_threshold).value),
            max_height=float(self.declare_parameter('max_height', d.max_height).value),
            self_radius=float(self.declare_parameter('self_radius', d.self_radius).value),
            max_range=float(self.declare_parameter('max_range', d.max_range).value),
            max_slope=float(self.declare_parameter('max_slope', d.max_slope).value),
        )
        self.corridor_width = float(self.declare_parameter('corridor_width', 1.0).value)
        self.corridor_length = float(self.declare_parameter('corridor_length', 5.0).value)
        self.front_offset = float(self.declare_parameter('front_offset', 0.4).value)
        self.stop_distance = float(self.declare_parameter('stop_distance', 1.5).value)
        # 一度止まったら、stop_distance + release_margin より空くまで止めたままにする（ノイズでのばたつき防止）
        self.release_margin = float(self.declare_parameter('release_margin', 0.3).value)
        self.publish_ground = bool(self.declare_parameter('publish_ground', True).value)
        # 傾きの補正（IMU の加速度計）と、舞い上がる砂よけ（何回続けて見えたら障害物にするか。1 で使わない）
        self.level_with_imu = bool(self.declare_parameter('level_with_imu', True).value)
        self.imu_time_constant = float(self.declare_parameter('imu_time_constant', 0.3).value)
        self.persist = om.PersistenceFilter(int(self.declare_parameter('persist_scans', 3).value),
                                            float(self.declare_parameter('persist_cell', 0.3).value))
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        # 手がかりの少なさ：障害物の点（舞う砂を消したあと）が min_feature_points 未満のスキャンが
        # feature_scans 回続いたら「少ない」、その 2 倍以上が同じ回数続いたら「戻った」
        self.min_feature_points = int(self.declare_parameter('min_feature_points', 100).value)
        self.feature_scans = int(self.declare_parameter('feature_scans', 5).value)
        self.degenerate = False
        self.feature_run = 0
        self.gravity = None              # 車体座標の重力の向き（ならしたもの）
        self.imu_rotation = None
        self.last_imu_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obs_pub = self.create_publisher(PointCloud2, '/obstacles/points', 5)
        self.ground_pub = self.create_publisher(PointCloud2, '/obstacles/ground', 5)
        self.dist_pub = self.create_publisher(Float32, '/obstacles/front_distance', 10)
        self.stop_pub = self.create_publisher(Bool, '/obstacles/stop', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacles/corridor', 5)
        self.tilt_pub = self.create_publisher(Vector3Stamped, '/terrain/tilt', 10)
        self.degenerate_pub = self.create_publisher(Bool, '/lidar/degenerate', 10)
        self.create_subscription(PointCloud2, 'pointcloud', self.on_cloud, 5)
        if self.level_with_imu:
            self.create_subscription(Imu, 'imu', self.on_imu, 50)

        self.was_stop = False
        self.slow_warned = False
        self.get_logger().info(
            f'障害物検出: 地面から {self.params.height_threshold:.2f} m 以上を障害物、'
            f'前方 {self.corridor_width:.1f} m 幅で {self.stop_distance:.1f} m 以内なら停止信号')

    def lookup(self, frame_id: str):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time(),
                                                 timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f'{frame_id} → {self.base_frame} の TF がありません: {e}',
                                   throttle_duration_sec=5.0)
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        return om.quaternion_to_matrix(q.x, q.y, q.z, q.w), (t.x, t.y, t.z)

    def on_imu(self, msg: Imu):
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if not np.all(np.isfinite(a)) or not 5.0 < np.linalg.norm(a) < 15.0:
            return                       # 加速度が出ていない（0）・大きく揺れているときは使わない
        if self.imu_rotation is None:
            tf = self.lookup(msg.header.frame_id)
            if tf is None:
                return
            self.imu_rotation = tf[0]
        a = self.imu_rotation @ a
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.gravity is None or self.last_imu_time is None:
            self.gravity = a
        else:
            dt = min(max(t - self.last_imu_time, 0.0), 0.5)
            k = dt / (self.imu_time_constant + dt)
            self.gravity = (1.0 - k) * self.gravity + k * a
        self.last_imu_time = t

    def lookup_fixed(self, stamp):
        """点群を測った時刻の base_link → 動かない座標（odom）の (回転, 平行移動)。なければ None。"""
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_frame, Time.from_msg(stamp),
                                                 timeout=Duration(seconds=0.02))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_frame, Time())
            except TransformException:
                return None
        q, t = tf.transform.rotation, tf.transform.translation
        return om.quaternion_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        tf = self.lookup(msg.header.frame_id)
        if tf is None:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        pts = om.transform_points(pts, *tf)
        roll = pitch = 0.0
        if self.gravity is not None:
            roll, pitch = om.tilt_from_gravity(self.gravity)
            pts = pts @ om.level_matrix(roll, pitch).T
        obs_mask, ground_mask = om.segment(pts, self.params)
        obstacles = pts[obs_mask]
        if self.persist.scans > 1:
            fixed = self.lookup_fixed(msg.header.stamp)
            if fixed is not None:
                obstacles = obstacles[self.persist.apply(om.transform_points(obstacles, *fixed))]

        dist = om.front_distance(obstacles, self.corridor_width, self.corridor_length, self.front_offset)
        limit = self.stop_distance + (self.release_margin if self.was_stop else 0.0)
        stop = dist < limit
        header = msg.header
        header.frame_id = self.base_frame

        self.obs_pub.publish(pc2.create_cloud_xyz32(header, obstacles.astype(np.float32)))
        if self.publish_ground:
            self.ground_pub.publish(pc2.create_cloud_xyz32(header, pts[ground_mask].astype(np.float32)))
        self.dist_pub.publish(Float32(data=dist if math.isfinite(dist) else self.corridor_length))
        self.stop_pub.publish(Bool(data=bool(stop)))
        self.marker_pub.publish(self.corridor_marker(header, stop))
        self.update_degenerate(len(obstacles))
        tilt = Vector3Stamped()
        tilt.header = header
        tilt.vector.x, tilt.vector.y = math.degrees(roll), math.degrees(pitch)
        self.tilt_pub.publish(tilt)

        if stop != self.was_stop:
            if stop:
                self.get_logger().info(f'前方 {dist:.2f} m に障害物')
            else:
                self.get_logger().info('前方が空きました')
            self.was_stop = stop

        elapsed = time.perf_counter() - t0
        if elapsed > 0.1 and not self.slow_warned:
            self.get_logger().warn(f'1 回の処理に {elapsed * 1000:.0f} ms かかっています（点群は 10 Hz）')
            self.slow_warned = True

    def update_degenerate(self, n: int):
        want = n < self.min_feature_points if not self.degenerate else n < 2 * self.min_feature_points
        self.feature_run = self.feature_run + 1 if want != self.degenerate else 0
        if self.feature_run >= self.feature_scans:
            self.degenerate = want
            self.feature_run = 0
            if want:
                self.get_logger().warn(f'点群に地面以外の物がほとんどありません（{n} 点）。LiDAR オドメトリが不確かです')
            else:
                self.get_logger().info(f'点群に手がかりが戻りました（{n} 点）')
        self.degenerate_pub.publish(Bool(data=self.degenerate))

    def corridor_marker(self, header, stop: bool) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'corridor'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self.front_offset + self.corridor_length / 2.0
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = self.corridor_length, self.corridor_width, 0.1
        m.color.r, m.color.g, m.color.b = (1.0, 0.2, 0.2) if stop else (0.2, 0.9, 0.3)
        m.color.a = 0.35
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
