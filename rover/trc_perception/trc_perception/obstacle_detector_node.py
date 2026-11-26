"""LiDAR の点群から地面を取り除いて障害物を見つけ、前方の障害物までの距離を出すノード。

入力:
  /unilidar/cloud   sensor_msgs/PointCloud2（LiDAR 座標。TF で base_link に直す）
出力（すべて base_link 座標）:
  /obstacles/points          sensor_msgs/PointCloud2  障害物の点
  /obstacles/ground          sensor_msgs/PointCloud2  地面とみなした点（表示用、publish_ground）
  /obstacles/front_distance  std_msgs/Float32         前方の帯に入った障害物までの距離 [m]。なければ corridor_length
  /obstacles/stop            std_msgs/Bool            front_distance < stop_distance のとき true
  /obstacles/corridor        visualization_msgs/Marker 前方の帯（緑：空いている／赤：止まるべき）

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
        )
        self.corridor_width = float(self.declare_parameter('corridor_width', 1.0).value)
        self.corridor_length = float(self.declare_parameter('corridor_length', 5.0).value)
        self.front_offset = float(self.declare_parameter('front_offset', 0.4).value)
        self.stop_distance = float(self.declare_parameter('stop_distance', 1.5).value)
        self.publish_ground = bool(self.declare_parameter('publish_ground', True).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obs_pub = self.create_publisher(PointCloud2, '/obstacles/points', 5)
        self.ground_pub = self.create_publisher(PointCloud2, '/obstacles/ground', 5)
        self.dist_pub = self.create_publisher(Float32, '/obstacles/front_distance', 10)
        self.stop_pub = self.create_publisher(Bool, '/obstacles/stop', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacles/corridor', 5)
        self.create_subscription(PointCloud2, 'pointcloud', self.on_cloud, 5)

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

    def on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        tf = self.lookup(msg.header.frame_id)
        if tf is None:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        pts = om.transform_points(pts, *tf)
        obs_mask, ground_mask = om.segment(pts, self.params)
        obstacles = pts[obs_mask]

        dist = om.front_distance(obstacles, self.corridor_width, self.corridor_length, self.front_offset)
        stop = dist < self.stop_distance
        header = msg.header
        header.frame_id = self.base_frame

        self.obs_pub.publish(pc2.create_cloud_xyz32(header, obstacles.astype(np.float32)))
        if self.publish_ground:
            self.ground_pub.publish(pc2.create_cloud_xyz32(header, pts[ground_mask].astype(np.float32)))
        self.dist_pub.publish(Float32(data=dist if math.isfinite(dist) else self.corridor_length))
        self.stop_pub.publish(Bool(data=bool(stop)))
        self.marker_pub.publish(self.corridor_marker(header, stop))

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
