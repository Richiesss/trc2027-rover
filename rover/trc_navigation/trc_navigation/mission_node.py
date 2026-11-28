"""自律ミッション：緯度・経度で書いたチェックポイントや地点を、経路計画のウェイポイントにして順に回る。

  ros2 run trc_navigation mission_node --ros-args -p mission_file:=~/trc2027/missions/day1.yaml

ミッションファイルの書き方は trc_navigation/mission.py を参照（対応表の csv も読める）。

流れ:
  1. スタート直後は、まっすぐ init_distance [m] 進む（向きの初期化 heading_init が RTK で 3 m 走るのを待つ）
  2. 向きが決まったら、navsat_transform の fromLL サービスで、すべての緯度・経度を map 座標に直す
     （位置推定と同じ変換を使うので、GNSS と同じ座標系でずれない）
  3. チェックポイントは「手前の点」と「奥の点」に、地点はそのままウェイポイントにして、経路計画に渡す
  4. 進み具合を /mission/status に出す。最後のウェイポイントに着いたら「ミッション完了」

出力:
  /planner/set_waypoints  nav_msgs/Path              経路計画（local_planner_node）へのウェイポイント（map）
  /mission/status         std_msgs/String            いまの状態（制御履歴・画面用）
  /mission/markers        visualization_msgs/MarkerArray  チェックポイントの線分・マーカー・ウェイポイント（Foxglove 用）
"""

import math
import os

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from trc_navigation import mission as ms


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        path = os.path.expanduser(self.declare_parameter('mission_file', '').value)
        self.init_distance = float(self.declare_parameter('init_distance', 4.0).value)
        body_length = float(self.declare_parameter('body_length', 1.2).value)    # 規定の上限 1.2 m
        self.before = float(self.declare_parameter('before_distance', 2.0).value)
        # 奥の点：機体の半分 + 経路計画が「着いた」とみなす距離 + 位置の誤差の分だけ線分から先
        self.after = float(self.declare_parameter('after_distance', body_length / 2.0 + 1.0).value)
        self.min_gap = float(self.declare_parameter('min_gap', 1.6).value)      # マーカー間がこれより狭いと警告

        if not path:
            raise SystemExit('mission_file を指定してください')
        self.steps = ms.load_mission(path)
        if not self.steps:
            raise SystemExit(f'{path} に route がありません')
        self.get_logger().info(f'ミッション {os.path.basename(path)}: ' +
                               ' → '.join(s.name for s in self.steps))

        self.pose = None
        self.heading_ready = False
        self.state = 'wait_pose'
        self.labels = []          # ウェイポイントごとの説明
        self.sent_count = 0
        self.last_remaining = None
        self.converting = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.wp_pub = self.create_publisher(Path, '/planner/set_waypoints', latched)
        self.status_pub = self.create_publisher(String, '/mission/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/markers', latched)
        self.create_subscription(Odometry, '/odometry/global', self.on_odom, 10)
        self.create_subscription(Imu, '/heading/imu', self.on_heading, 10)
        self.create_subscription(Path, '/planner/waypoints', self.on_remaining, 10)
        self.from_ll = self.create_client(FromLL, 'fromLL')
        self.convert_timeout = float(self.declare_parameter('convert_timeout', 3.0).value)
        self.create_timer(0.5, self.tick)
        self.set_status('位置の推定を待っています')

    # --- 受信 ---
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_heading(self, _msg):
        if not self.heading_ready:
            self.heading_ready = True
            self.get_logger().info('向きが決まったので、緯度・経度を map 座標に直せます')

    def on_remaining(self, msg):
        """経路計画の残りのウェイポイント（最初の 1 点はいまの位置）から進み具合を知る。"""
        if self.state != 'running':
            return
        remaining = max(len(msg.poses) - 1, 0)
        if remaining == self.last_remaining:
            return
        self.last_remaining = remaining
        done = self.sent_count - remaining
        if 0 < done <= len(self.labels):
            self.set_status(f'{self.labels[done - 1]} に着きました（{done}/{self.sent_count}）')
        if remaining == 0 and done == self.sent_count:
            self.state = 'done'
            self.set_status('ミッション完了')

    # --- 状態の進め方（2 Hz）---
    def tick(self):
        if self.state == 'wait_pose' and self.pose is not None:
            x, y, yaw = self.pose
            if self.heading_ready:
                self.state = 'convert'
            else:
                gx, gy = x + self.init_distance * math.cos(yaw), y + self.init_distance * math.sin(yaw)
                self.send([(gx, gy)], ['向きを決めるための直進'])
                self.state = 'init_run'
                self.set_status(f'向きを決めるため {self.init_distance:.0f} m 直進します')
        if self.state == 'init_run' and self.heading_ready:
            self.state = 'convert'
        if self.state == 'convert' and self.converting is not None:
            # 返事が来ないことがある（ROS 2 のサービスの返事が落ちる）。しばらく待っても揃わなければ、やり直す
            waited = (self.get_clock().now() - self.converting['started']).nanoseconds * 1e-9
            if waited > self.convert_timeout:
                self.get_logger().warn(f'fromLL の返事が {waited:.0f} 秒来ないので、緯度・経度の変換をやり直します')
                self.converting = None
        if self.state == 'convert' and self.converting is None:
            if not self.from_ll.service_is_ready():
                self.set_status('navsat_transform の fromLL サービスを待っています', throttle=10.0)
                return
            self.start_conversion()

    def start_conversion(self):
        points = []
        for s in self.steps:
            points += [s.point] if s.kind == 'latlon' else [s.a, s.b]
        conv = {'points': points, 'xy': [None] * len(points), 'pending': len(points),
                'started': self.get_clock().now()}
        self.converting = conv
        self.get_logger().info(f'緯度・経度 {len(points)} 点を fromLL で map 座標に直します')
        for i, (lat, lon) in enumerate(points):
            req = FromLL.Request()
            req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            fut = self.from_ll.call_async(req)
            fut.add_done_callback(lambda f, i=i, conv=conv: self.on_converted(i, f, conv))

    def on_converted(self, i, fut, conv):
        if conv is not self.converting or conv['xy'][i] is not None:
            return                                   # やり直す前の古い返事
        try:
            p = fut.result().map_point
            conv['xy'][i] = (p.x, p.y)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f'緯度・経度の変換に失敗しました: {e}')
            self.converting = None                   # 次の tick でやり直す
            return
        conv['pending'] -= 1
        if conv['pending'] == 0:
            self.build_route(conv['xy'])

    def build_route(self, xy):
        """map 座標に直した点から、経路計画に渡すウェイポイントを作る。"""
        x, y, _ = self.pose
        came_from = (x, y)
        waypoints, labels, segments = [], [], []
        k = 0
        for s in self.steps:
            if s.kind == 'latlon':
                p = xy[k]
                k += 1
                waypoints.append(p)
                labels.append(s.name)
                came_from = p
            else:
                a, b = xy[k], xy[k + 1]
                k += 2
                pre, post, gap = ms.crossing_points(a, b, came_from, self.before, self.after)
                if gap < self.min_gap:
                    self.get_logger().warn(f'{s.name}: マーカーの間が {gap:.2f} m しかありません（通れないかも）')
                waypoints += [pre, post]
                labels += [f'{s.name} の手前', f'{s.name} の奥（通過）']
                segments.append((s.name, a, b))
                came_from = post
                self.get_logger().info(f'{s.name}: マーカー間 {gap:.2f} m、中点 ({(a[0]+b[0])/2:.1f}, '
                                       f'{(a[1]+b[1])/2:.1f}) を横切ります')
        self.state = 'running'
        self.last_remaining = None
        self.send(waypoints, labels)
        self.publish_markers(segments, waypoints, xy)
        self.set_status(f'{labels[0]} へ向かいます（ウェイポイント {len(waypoints)} 個）')

    # --- 出力 ---
    def send(self, points, labels):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for px, py in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.labels = labels
        self.sent_count = len(points)
        self.wp_pub.publish(path)

    def set_status(self, text, throttle=None):
        if throttle:
            self.get_logger().info(text, throttle_duration_sec=throttle)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def publish_markers(self, segments, waypoints, xy):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid = 0

        def base(kind, ns):
            nonlocal mid
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            mid += 1
            return m

        from geometry_msgs.msg import Point
        for name, a, b in segments:
            line = base(Marker.LINE_LIST, 'checkpoint')
            line.scale.x = 0.08
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.8, 0.0, 1.0
            line.points = [Point(x=a[0], y=a[1], z=0.05), Point(x=b[0], y=b[1], z=0.05)]
            arr.markers.append(line)
            for p in (a, b):
                pole = base(Marker.CYLINDER, 'marker')
                pole.pose.position.x, pole.pose.position.y, pole.pose.position.z = p[0], p[1], 0.6
                pole.scale.x = pole.scale.y = 0.2
                pole.scale.z = 1.2
                pole.color.r, pole.color.g, pole.color.b, pole.color.a = 1.0, 0.5, 0.0, 0.9
                arr.markers.append(pole)
            text = base(Marker.TEXT_VIEW_FACING, 'label')
            text.pose.position.x, text.pose.position.y, text.pose.position.z = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 1.5
            text.scale.z = 0.5
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = name
            arr.markers.append(text)
        for i, (px, py) in enumerate(waypoints):
            s = base(Marker.SPHERE, 'waypoint')
            s.pose.position.x, s.pose.position.y, s.pose.position.z = px, py, 0.2
            s.scale.x = s.scale.y = s.scale.z = 0.35
            s.color.r, s.color.g, s.color.b, s.color.a = 0.2, 0.6, 1.0, 0.9
            arr.markers.append(s)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionNode()
    except SystemExit as e:
        rclpy.logging.get_logger('mission').error(str(e))
        rclpy.shutdown()
        return
    except (ValueError, OSError) as e:
        rclpy.logging.get_logger('mission').error(f'ミッションファイルを読めません: {e}')
        rclpy.shutdown()
        return
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
