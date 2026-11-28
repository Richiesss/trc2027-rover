"""目標地点（ウェイポイント）へ向かって、障害物をよけながら進む速度指令を出すノード。

入力:
  /odometry/global   nav_msgs/Odometry         いまの位置と向き（map 座標、位置推定の出力）
  /obstacles/points  sensor_msgs/PointCloud2   障害物の点（base_link 座標、obstacle_detector の出力）
  /goal_pose         geometry_msgs/PoseStamped 新しい目標（map 座標）。Foxglove の 3D パネルの「Publish pose」でも送れる
  /planner/set_waypoints nav_msgs/Path         ウェイポイントの並びをまとめて入れ替える（map 座標、mission_node が使う）
  /terrain/tilt      geometry_msgs/Vector3Stamped 車体の傾き [度]（obstacle_detector が IMU から求める）
  /odometry/local    nav_msgs/Odometry         odom 座標の位置（はまったかどうかの判断に使う。GNSS の揺れが入らない）
出力:
  cmd_vel              geometry_msgs/Twist   速度指令（ふつうは /cmd_vel_nav に出し、safety_stop を通す）
  /planner/trajectory  nav_msgs/Path         選んだ指令で進んだときの予測の通り道（base_link）
  /planner/waypoints   nav_msgs/Path         残りのウェイポイント（map）
  /planner/global_path nav_msgs/Path         地図の上で探した、目標までの道筋（map）
  /planner/map         nav_msgs/OccupancyGrid 見つけた障害物を書きためた地図（map、1 Hz。100 障害物・90 通らない・70 穴）

2 段構え:
  1. 見つけた障害物を map 座標の格子地図に書きため、地図の上で目標までの道筋を A* で探す（1 秒ごと）
  2. 道筋の lookahead [m] 先の点を目指して、DWA で目の前の障害物をよけながら進む（10 Hz）

ウェイポイントは map 座標で、原点はスタート地点、x はスタート時の前方、y は左。
  waypoints:="15,0; 15,8"   のように「x,y」を ; で区切って渡す

前に進める候補がないとき（行き止まり）は、同じ向きにその場で回り続けて抜け道を探す。

砂地の対策:
  - 砂にはまる：前進の指令を出しているのに stuck_time 秒でほとんど進まなければ「はまった」とみなし、
    backup_time 秒下がって、はまった所（前方 block_ahead m、半径 block_radius m）を地図で「通らない」にして道を引き直す
  - 傾きすぎ：車体の傾きが tilt_limit 度を超えたら、同じように下がって、その先を「通らない」にする
  - 穴・崖のふち：すぐ目の前（watch_near〜watch_far m）なのに地面が見えないマスは、地図で「穴」として通らない
目標の 2 m 以内まで来て、目標そのものが障害物に近すぎるとわかったら、そのウェイポイントを飛ばす。

注意：DWA は「いま見えている範囲で、数秒先までの動き」しか考えない。長い壁や袋小路では、
回り込めずに行ったり来たりすることがある（その場合は地図を使った経路計画が必要）。
"""

import bisect
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool

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
        # その場で回り始めたら、前が stop_distance + release_margin まで空き、それが resume_cycles 回続くまで回る
        # （safety_stop / obstacle_detector の解除の余裕と合わせる。合わないと「進む／止まる」を繰り返す）
        self.release_margin = dp('release_margin', 0.3)
        self.resume_cycles = int(self.declare_parameter('resume_cycles', 3).value)
        self.lookahead = dp('lookahead', 2.0)
        self.map_range = dp('map_range', 12.0)   # 地図に書くのはこの距離までの点（遠いほど向きのずれが効く）
        self.pose_buf = deque(maxlen=300)         # (時刻, x, y, yaw)：点群を測った時刻の位置を求めるため
        self.waypoints = parse_waypoints(self.declare_parameter('waypoints', '').value)
        # --- 砂地の対策 ---
        self.stuck_time = dp('stuck_time', 3.0)          # この秒数で
        self.stuck_expected = dp('stuck_expected', 0.6)  # 指令どおりなら これ以上 [m] 進むはずなのに
        self.stuck_ratio = dp('stuck_ratio', 0.25)       # その割合しか進んでいなければ「はまった」
        self.backup_speed = dp('backup_speed', 0.3)
        self.backup_time = dp('backup_time', 2.5)
        self.block_ahead = dp('block_ahead', 0.8)
        self.block_radius = dp('block_radius', 0.6)
        self.tilt_limit = dp('tilt_limit', 18.0)
        self.max_recoveries = int(self.declare_parameter('max_recoveries', 6).value)
        self.watch_near = dp('watch_near', 0.8)
        self.watch_far = dp('watch_far', 1.6)
        self.watch_fov = math.radians(dp('watch_fov_deg', 45.0))
        r = np.arange(self.watch_near, self.watch_far + 1e-6, 0.125)
        a = np.linspace(-self.watch_fov, self.watch_fov, 9)
        rr, aa = np.meshgrid(r, a)
        self.watch_base = np.c_[(rr * np.cos(aa)).ravel(), (rr * np.sin(aa)).ravel()]
        self.progress = deque(maxlen=200)                # (時刻, x, y, 前進の指令 [m/s])
        self.recovery = None                             # 下がっている間：{'until': 時刻}
        self.recoveries = 0                              # いまのウェイポイントで下がった回数
        self.tilt = (0.0, 0.0)
        self.local_pose = None                           # (x, y, 受信時刻)：odom 座標
        self.skip_time = dp('skip_time', 2.0)            # 目標のそばの障害物がこの秒数続いたら次へ
        self.blocked_goal_steps = 0
        self.last_v = 0.0
        self.grid = gp.ObstacleGrid(gp.GridParams(inflate=self.p.robot_radius + self.p.safety_margin))
        self.path = None
        self.path_goal = None
        self.path_failed = False

        self.pose = None             # (x, y, yaw, 受信時刻)
        self.obstacles = None        # ((M, 2), 受信時刻)
        self.stuck_dir = 0.0
        self.clear_count = 0
        self.safety_stop = False
        self.stuck_log_time = None
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
        self.create_subscription(Bool, '/obstacles/stop', self.on_stop, 10)
        self.create_subscription(Vector3Stamped, '/terrain/tilt', self.on_tilt, 10)
        self.create_subscription(Odometry, '/odometry/local', self.on_local, 10)
        self.create_subscription(Path, '/planner/set_waypoints', self.on_set_waypoints,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
        self.grid.watch(self.base_to_map(self.watch_base, pose))   # 目の前は地面が見えるはず

    def on_local(self, msg: Odometry):
        self.local_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, self.get_clock().now())

    def on_tilt(self, msg: Vector3Stamped):
        self.tilt = (msg.vector.x, msg.vector.y)

    @staticmethod
    def base_to_map(base, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.c_[x + c * base[:, 0] - s * base[:, 1], y + s * base[:, 0] + c * base[:, 1]]

    def on_set_waypoints(self, msg: Path):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn(f'ウェイポイントの座標系 {msg.header.frame_id} は使えません（map で送ってください）')
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.done_logged = False
        self.recoveries = 0
        pts = ', '.join(f'({x:.1f}, {y:.1f})' for x, y in self.waypoints)
        self.get_logger().info(f'ウェイポイントを受け取りました（{len(self.waypoints)} 個）: {pts}')
        self.replan()

    def on_stop(self, msg: Bool):
        self.safety_stop = msg.data

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
            self.recoveries = 0
            self.replan()
            return
        if self.recovery is not None:
            self.run_recovery()
            return
        if self.recoveries > self.max_recoveries:
            self.get_logger().warn(f'ウェイポイント ({gx:.1f}, {gy:.1f}) の手前で {self.recoveries} 回はまったので、'
                                   f'あきらめて次へ進みます')
            self.waypoints.pop(0)
            self.recoveries = 0
            self.replan()
            return

        obs = self.obstacles[0]
        special = self.grid.special_points(x, y, 5.0)        # 「通らない」所と「穴」も DWA に教える
        obs_all = np.vstack([obs, dwa.to_base(x, y, yaw, special)]) if len(special) else obs
        # 目標のすぐそばに障害物があるときは次へ進む。ただし一瞬の誤検出（舞う砂・坂の見え方）で
        # チェックポイントを飛ばさないよう、skip_time 秒続いたときだけ
        near = math.hypot(*goal) < 2.0 and len(obs) and \
            np.hypot(obs[:, 0] - goal[0], obs[:, 1] - goal[1]).min() < self.p.robot_radius + self.p.safety_margin
        self.blocked_goal_steps = self.blocked_goal_steps + 1 if near else 0
        if self.blocked_goal_steps >= int(self.skip_time * 10):
            self.blocked_goal_steps = 0
            self.get_logger().warn(f'ウェイポイント ({gx:.1f}, {gy:.1f}) は障害物に近すぎて行けないので、次へ進みます')
            self.waypoints.pop(0)
            self.replan()
            return

        # 目指す点：地図の道筋の lookahead 先（道筋がなければ目標そのもの）
        target = goal
        if self.path is not None and math.hypot(*goal) > self.lookahead:
            tx, ty = gp.lookahead_point(self.path, (x, y), self.lookahead)
            target = dwa.to_base(x, y, yaw, np.array([[tx, ty]]))[0]
        plan = dwa.plan(obs_all, target, self.p)
        cmd = Twist()
        if self.stuck_dir == 0.0:
            # ふだん：安全停止がかかっている、または前に進める候補がなければ、その場で回り始める
            if plan.feasible and not self.safety_stop:
                cmd.linear.x, cmd.angular.z = plan.v, plan.w
            else:
                self.stuck_dir = 1.0 if target[1] >= 0.0 else -1.0   # 目指す点のある側へ回る
                self.clear_count = 0
                now = self.get_clock().now()
                if self.stuck_log_time is None or (now - self.stuck_log_time).nanoseconds > 5e9:
                    self.get_logger().info('前に進めないので、その場で回って抜け道を探します')
                    self.stuck_log_time = now
        if self.stuck_dir != 0.0:
            # 回っている間：同じ向きに回り続け、前が余裕をもって空いた状態が続いたら進み始める
            clear = (plan.feasible and not self.safety_stop
                     and not dwa.front_blocked(obs_all, (0.0, 0.0, 0.0), self.p, margin=self.release_margin))
            self.clear_count = self.clear_count + 1 if clear else 0
            if self.clear_count >= self.resume_cycles:
                self.stuck_dir = 0.0
                self.get_logger().info('抜け道が見つかりました', throttle_duration_sec=5.0)
                cmd.linear.x, cmd.angular.z = plan.v, plan.w
            else:
                cmd.angular.z = self.stuck_dir * self.stuck_yaw_rate
        self.cmd_pub.publish(cmd)
        self.publish_trajectory(plan.trajectory)
        self.check_progress(cmd.linear.x, obs_all)

    # --- 砂地の対策：はまった・傾きすぎたら下がる ---
    def check_progress(self, v_cmd: float, obs):
        now = self.get_clock().now().nanoseconds * 1e-9
        # 進んだ距離は odom 座標（LiDAR オドメトリ＋IMU）で測る。map 座標は GNSS の揺れ・跳びで
        # 引き戻されることがあり、単独測位のときに「はまった」と誤った
        if self.local_pose is not None and self.age(self.local_pose[2]) < self.data_timeout:
            x, y = self.local_pose[0], self.local_pose[1]
        else:
            x, y = self.pose[0], self.pose[1]
        v_eff = 0.0 if self.safety_stop else max(v_cmd, 0.0)   # 安全停止中の前進は実際には出ていない
        self.progress.append((now, x, y, v_eff))
        self.last_v = v_eff
        tilt = max(abs(self.tilt[0]), abs(self.tilt[1]))
        if tilt > self.tilt_limit and v_eff > 0.05:
            self.start_recovery(f'車体が {tilt:.0f}° 傾いた', obs)
            return
        window = [p for p in self.progress if p[0] >= now - self.stuck_time]
        if not window or window[-1][0] - window[0][0] < self.stuck_time - 0.2:
            return
        expected = sum(p[3] for p in window) * 0.1           # 10 Hz
        moved = math.hypot(window[-1][1] - window[0][1], window[-1][2] - window[0][2])
        if expected >= self.stuck_expected and moved < self.stuck_ratio * expected:
            self.start_recovery(f'{self.stuck_time:.0f} 秒で {expected:.1f} m 進むはずが {moved:.2f} m', obs)

    def start_recovery(self, why: str, obs):
        x, y, yaw, _ = self.pose
        bx, by = x + self.block_ahead * math.cos(yaw), y + self.block_ahead * math.sin(yaw)
        self.grid.block(bx, by, self.block_radius)
        behind = obs[(obs[:, 0] < -0.2) & (obs[:, 0] > -1.6) & (np.abs(obs[:, 1]) < 0.6)] if len(obs) else obs
        now = self.get_clock().now().nanoseconds * 1e-9
        backup = len(behind) == 0
        self.recovery = {'until': now + (self.backup_time if backup else 0.0)}
        self.recoveries += 1
        self.get_logger().warn(
            f'砂にはまったようです（{why}）。' + ('下がって、' if backup else '後ろがふさがっているので下がらずに、')
            + f'({bx:.1f}, {by:.1f}) のまわりを通らないようにして道を引き直します（{self.recoveries} 回目）')
        self.progress.clear()
        self.stuck_dir = 0.0

    def run_recovery(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.recovery['until']:
            cmd = Twist()
            cmd.linear.x = -self.backup_speed
            self.cmd_pub.publish(cmd)
            return
        self.recovery = None
        self.progress.clear()
        self.replan()

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
        # OccupancyGrid は行（y）ごとに並べる。ふさがり 100、通らない（はまった所）90、穴 70、見えたが少ない 30、それ以外 0
        hits = g.hits.T
        data = np.where(hits >= g.p.hits_to_occupy, 100, np.where(hits > 0, 30, 0)).astype(np.int8)
        data = np.where(g.hazard().T, 70, data).astype(np.int8)
        data = np.where(g.blocked.T, 90, data).astype(np.int8)
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
