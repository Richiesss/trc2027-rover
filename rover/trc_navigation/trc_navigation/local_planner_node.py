"""目標地点（ウェイポイント）へ向かって、障害物をよけながら進む速度指令を出すノード。

入力:
  /odometry/global   nav_msgs/Odometry         いまの位置と向き（map 座標、位置推定の出力）
  /obstacles/points  sensor_msgs/PointCloud2   障害物の点（base_link 座標、obstacle_detector の出力）
  /goal_pose         geometry_msgs/PoseStamped 新しい目標（map 座標）。Foxglove の 3D パネルの「Publish pose」でも送れる
出力:
  cmd_vel              geometry_msgs/Twist   速度指令（ふつうは /cmd_vel_nav に出し、safety_stop を通す）
  /planner/trajectory  nav_msgs/Path         選んだ指令で進んだときの予測の通り道（base_link）
  /planner/waypoints   nav_msgs/Path         残りのウェイポイント（map）
  /planner/global_path nav_msgs/Path         地図の上で探した、目標までの道筋（map）
  /planner/map         nav_msgs/OccupancyGrid 見つけた障害物を書きためた地図（map、1 Hz）

2 段構え:
  1. 見つけた障害物を map 座標の格子地図に書きため、地図の上で目標までの道筋を A* で探す（1 秒ごと）
  2. 道筋の lookahead [m] 先の点を目指して、DWA で目の前の障害物をよけながら進む（10 Hz）

ウェイポイントは map 座標で、原点はスタート地点、x はスタート時の前方、y は左。
  waypoints:="15,0; 15,8"   のように「x,y」を ; で区切って渡す

前に進める候補がないとき（行き止まり）は、同じ向きにその場で回り続けて抜け道を探す。
目標の 2 m 以内まで来て、目標そのものが障害物に近すぎるとわかったら、そのウェイポイントを飛ばす。

注意：DWA は「いま見えている範囲で、数秒先までの動き」しか考えない。長い壁や袋小路では、
回り込めずに行ったり来たりすることがある（その場合は地図を使った経路計画が必要）。
"""

import bisect
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from trc_navigation import dwa
from trc_navigation import grid_planner as gp


def parse_waypoints(text: str):
    """'15,0; 15,8' → [(15.0, 0.0), (15.0, 8.0)]"""
    out = []
    for part in text.replace('\n', ';').split(';'):
        part = part.strip()
        if not part:
            continue
        x, y = (float(v) for v in part.split(','))
        out.append((x, y))
    return out


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')
        d = dwa.DwaParams()
        dp = lambda name, default: float(self.declare_parameter(name, default).value)  # noqa: E731
        self.p = dwa.DwaParams(
            max_speed=dp('max_speed', d.max_speed),
            max_yaw_rate=dp('max_yaw_rate', d.max_yaw_rate),
            horizon=dp('horizon', d.horizon),
            robot_radius=dp('robot_radius', d.robot_radius),
            safety_margin=dp('safety_margin', d.safety_margin),
            stop_distance=dp('stop_distance', d.stop_distance),
            corridor_width=dp('corridor_width', d.corridor_width),
            front_offset=dp('front_offset', d.front_offset),
        )
        self.goal_tolerance = dp('goal_tolerance', 0.5)
        self.stuck_yaw_rate = dp('stuck_yaw_rate', 0.4)
        self.data_timeout = dp('data_timeout', 0.5)
        self.lookahead = dp('lookahead', 2.0)
        self.map_range = dp('map_range', 12.0)   # 地図に書くのはこの距離までの点（遠いほど向きのずれが効く）
        self.pose_buf = deque(maxlen=300)         # (時刻, x, y, yaw)：点群を測った時刻の位置を求めるため
        self.waypoints = parse_waypoints(self.declare_parameter('waypoints', '').value)
        self.grid = gp.ObstacleGrid(gp.GridParams(inflate=self.p.robot_radius + self.p.safety_margin))
        self.path = None
        self.path_goal = None
        self.path_failed = False

        self.pose = None             # (x, y, yaw, 受信時刻)
        self.obstacles = None        # ((M, 2), 受信時刻)
        self.stuck_dir = 0.0
        self.done_logged = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.traj_pub = self.create_publisher(Path, '/planner/trajectory', 5)
        self.wp_pub = self.create_publisher(Path, '/planner/waypoints', 5)
        self.path_pub = self.create_publisher(Path, '/planner/global_path', 5)
        self.map_pub = self.create_publisher(OccupancyGrid, '/planner/map', 1)
        self.create_subscription(Odometry, '/odometry/global', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/obstacles/points', self.on_obstacles, 5)
        self.create_subscription(PointCloud2, '/obstacles/ground', self.on_ground, 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 5)
        self.create_timer(0.1, self.step)
        self.create_timer(1.0, self.replan)
        if self.waypoints:
            pts = ', '.join(f'({x:.1f}, {y:.1f})' for x, y in self.waypoints)
            self.get_logger().info(f'ウェイポイント {len(self.waypoints)} 個: {pts}')
        else:
            self.get_logger().info('目標がありません。/goal_pose で目標を送ってください')

    # --- 受信 ---
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation), self.get_clock().now())
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.pose_buf or t > self.pose_buf[-1][0]:
            self.pose_buf.append((t, *self.pose[:3]))

    def pose_at(self, stamp):
        """点群を測った時刻の (x, y, yaw)。古すぎる・新しすぎるときは None。

        点群は処理の分だけ遅れて届くので、受け取ったときの位置を使うと、回っている最中は
        障害物がずれて地図に書かれる（10 m 先で 1 m 以上）。
        """
        t = stamp.sec + stamp.nanosec * 1e-9
        buf = self.pose_buf
        if len(buf) < 2 or t < buf[0][0]:
            return None
        if t >= buf[-1][0]:
            return buf[-1][1:] if t - buf[-1][0] < 0.2 else None
        ts = [b[0] for b in buf]
        i = bisect.bisect_left(ts, t)
        (t0, x0, y0, a0), (t1, x1, y1, a1) = buf[i - 1], buf[i]
        w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * da

    def on_obstacles(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
        base = dwa.thin(np.asarray(pts, dtype=float))
        self.obstacles = (base, self.get_clock().now())
        self.last_obstacles_map = None
        pose = self.pose_at(msg.header.stamp)
        near = base[np.hypot(base[:, 0], base[:, 1]) <= self.map_range] if len(base) else base
        if pose is not None and len(near):                # 地図に書きこむ（測った時刻の位置で map 座標に）
            self.last_obstacles_map = self.base_to_map(near, pose)
            self.grid.add(self.last_obstacles_map)

    def on_ground(self, msg: PointCloud2):
        """地面として見えたマスは、地図の障害物の数を減らす（間違って書いた障害物を消す）。"""
        pose = self.pose_at(msg.header.stamp)
        if pose is None:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
        base = dwa.thin(np.asarray(pts, dtype=float), self.grid.p.resolution)
        base = base[np.hypot(base[:, 0], base[:, 1]) <= self.map_range] if len(base) else base
        if len(base):
            self.grid.clear(self.base_to_map(base, pose), getattr(self, 'last_obstacles_map', None))

    @staticmethod
    def base_to_map(base, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.c_[x + c * base[:, 0] - s * base[:, 1], y + s * base[:, 0] + c * base[:, 1]]

    def on_goal(self, msg: PoseStamped):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn(f'目標の座標系 {msg.header.frame_id} は使えません（map で送ってください）')
            return
        self.waypoints = [(msg.pose.position.x, msg.pose.position.y)]
        self.done_logged = False
        self.replan()
        self.get_logger().info(f'新しい目標: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})')

    # --- 毎回の計算（10 Hz）---
    def age(self, stamp) -> float:
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def step(self):
        self.publish_waypoints()
        if not self.waypoints:
            if not self.done_logged and self.pose is not None:
                self.get_logger().info('すべてのウェイポイントに着きました')
                self.done_logged = True
            self.cmd_pub.publish(Twist())
            return
        if self.pose is None or self.age(self.pose[3]) > self.data_timeout:
            self.get_logger().warn('位置（/odometry/global）が来ていないので止まっています',
                                   throttle_duration_sec=5.0)
            self.cmd_pub.publish(Twist())
            return
        if self.obstacles is None or self.age(self.obstacles[1]) > self.data_timeout:
            self.get_logger().warn('障害物（/obstacles/points）が来ていないので止まっています',
                                   throttle_duration_sec=5.0)
            self.cmd_pub.publish(Twist())
            return

        x, y, yaw, _ = self.pose
        gx, gy = self.waypoints[0]
        if self.path is not None and self.path_goal is not None:
            gx, gy = self.path_goal                       # 目標が障害物に近いときは代わりの点
        goal = dwa.to_base(x, y, yaw, np.array([[gx, gy]]))[0]
        if math.hypot(*goal) < self.goal_tolerance:
            self.get_logger().info(f'ウェイポイント ({gx:.1f}, {gy:.1f}) に着きました')
            self.waypoints.pop(0)
            self.replan()
            return

        obs = self.obstacles[0]
        if math.hypot(*goal) < 2.0 and len(obs) and \
                np.hypot(obs[:, 0] - goal[0], obs[:, 1] - goal[1]).min() < self.p.robot_radius + self.p.safety_margin:
            self.get_logger().warn(f'ウェイポイント ({gx:.1f}, {gy:.1f}) は障害物に近すぎて行けないので、次へ進みます')
            self.waypoints.pop(0)
            self.replan()
            return

        # 目指す点：地図の道筋の lookahead 先（道筋がなければ目標そのもの）
        target = goal
        if self.path is not None and math.hypot(*goal) > self.lookahead:
            tx, ty = gp.lookahead_point(self.path, (x, y), self.lookahead)
            target = dwa.to_base(x, y, yaw, np.array([[tx, ty]]))[0]
        plan = dwa.plan(obs, target, self.p)
        # いまの位置で前がふさがっていれば、安全停止が前進を止めてしまうので、その場で回る
        blocked_now = dwa.front_blocked(obs, (0.0, 0.0, 0.0), self.p, margin=0.0)
        cmd = Twist()
        if plan.feasible and not blocked_now:
            if self.stuck_dir != 0.0:
                self.get_logger().info('抜け道が見つかりました')
            self.stuck_dir = 0.0
            cmd.linear.x, cmd.angular.z = plan.v, plan.w
        else:
            # 行き止まり：最初は目標のある側に回り、その後は同じ向きに回り続ける（左右に迷わない）
            if self.stuck_dir == 0.0:
                self.stuck_dir = 1.0 if target[1] >= 0.0 else -1.0
                self.get_logger().info('前に進めないので、その場で回って抜け道を探します')
            cmd.angular.z = self.stuck_dir * self.stuck_yaw_rate
        self.cmd_pub.publish(cmd)
        self.publish_trajectory(plan.trajectory)

    # --- 地図の上で道筋を探す（1 Hz）---
    def replan(self):
        if self.pose is None or not self.waypoints:
            self.path = None
            self.publish_map()
            return
        x, y = self.pose[0], self.pose[1]
        goal = self.waypoints[0]
        self.path, self.path_goal = gp.plan_path_robust(self.grid, (x, y), goal, min_inflate=self.p.robot_radius)
        if self.path is not None and math.hypot(self.path_goal[0] - goal[0], self.path_goal[1] - goal[1]) > 0.01:
            self.get_logger().info(f'目標 ({goal[0]:.1f}, {goal[1]:.1f}) は障害物に近いので、'
                                   f'({self.path_goal[0]:.1f}, {self.path_goal[1]:.1f}) を目指します',
                                   throttle_duration_sec=10.0)
        if self.path is None:
            if not self.path_failed:
                self.get_logger().warn('地図の上で目標までの道筋が見つかりません。目標へまっすぐ向かいます')
            self.path_failed = True
        else:
            if self.path_failed:
                self.get_logger().info('道筋が見つかりました')
            self.path_failed = False
            self.publish_global_path()
        self.publish_map()

    # --- 表示 ---
    def publish_global_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for px, py in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_map(self):
        g = self.grid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = g.p.resolution
        msg.info.width = msg.info.height = g.n
        msg.info.origin.position.x = msg.info.origin.position.y = g.origin
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid は行（y）ごとに並べる。ふさがり 100、見えたが少ない 30、それ以外 0
        hits = g.hits.T
        data = np.where(hits >= g.p.hits_to_occupy, 100, np.where(hits > 0, 30, 0)).astype(np.int8)
        msg.data = data.ravel().tolist()
        self.map_pub.publish(msg)

    def publish_trajectory(self, traj):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'base_link'
        for px, py, pyaw in traj:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(pyaw / 2), math.cos(pyaw / 2)
            path.poses.append(ps)
        self.traj_pub.publish(path)

    def publish_waypoints(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        pts = ([(self.pose[0], self.pose[1])] if self.pose else []) + list(self.waypoints)
        for px, py in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(px), float(py)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.wp_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
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
