"""実機なしで位置推定を試すための、にせのセンサ。

ローバーが「東から heading_deg 度の向き」でスタートし、まっすぐ進んでからゆるく曲がる、
という走行を作り、次を配信する（実機と同じトピック名）:

  /lidar/odom     nav_msgs/Odometry      LiDARオドメトリの代わり（publish_odom:=true のとき）
  /unilidar/cloud sensor_msgs/PointCloud2 にせの点群（publish_cloud:=true のとき）。
                                          支柱・岩・砂山を置いた仮想フィールドを、ローバーから見た点群。
                                          KISS-ICP などの LiDAR オドメトリに入力して試せる
  /unilidar/imu   sensor_msgs/Imu        回転の速さだけ
  /gnss/fix       sensor_msgs/NavSatFix  RTK Fix のとき（誤差 sigma_gnss）。大塚さんの gnss_fix と同じ分け方
  /gnss           sensor_msgs/NavSatFix  RTK Fix 以外（Float・単独測位・未測位）
  /gnss/best      sensor_msgs/NavSatFix  有効な測位すべて
  （/gnss/quality は gnss_adapter_node が作る）
  /fake/world     sensor_msgs/PointCloud2 仮想フィールド全体（map 座標、表示用、10 秒ごと）
  /fake/truth_path nav_msgs/Path          本当に通った道筋（map 座標、表示用）
  /fake/obstacles  sensor_msgs/PointCloud2 物体だけの点（map 座標、地面を除く。シナリオ試験の最接近の計算用）
  /fake/robot      visualization_msgs/MarkerArray 本当の姿勢の車体（高さ・傾きつき、map 座標。3D 表示用）
  /fake/cloud_map  sensor_msgs/PointCloud2 LiDAR の点群を本当の姿勢で map 座標に置いたもの（3D 表示用、5 Hz）
                   位置推定は平面なので、ふつうの /unilidar/cloud は坂で傾いて・埋まって見える

砂地の試験（terrain・slip・soft_sand・dust・world。fake_terrain.py）:
  terrain:=dunes / steep / pit で砂丘やくぼみを置く。車体は地面に沿って傾き、
  LiDAR の点群も傾く。IMU は重力の向き（linear_acceleration）も出す。
  登り坂・やわらかい砂ではタイヤが空転して、指令どおりに進まない。
  転倒・転落したら「転倒しました」「転落しました」とログに出して止まる。

drive_mode:=cmd_vel のときは /cmd_vel（geometry_msgs/Twist）の速度指令どおりに動く。

あわせて /fake/truth（NavSatFix）に本当の位置を出すので、
/gps/filtered（推定位置を緯度経度に戻したもの）と比べれば、推定の良し悪しが分かる。
"""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField

from trc_localization import fake_terrain as ft
from trc_localization import fake_world
from trc_localization import heading_math as hm


def parse_events(text: str, default_kind: str = None):
    """'10:30:float; 50:5:outage' → [(10.0, 30.0, 'float'), (50.0, 5.0, 'outage')]"""
    out = []
    for part in (text or '').split(';'):
        part = part.strip()
        if not part:
            continue
        f = [v.strip() for v in part.split(':')]
        kind = f[2] if len(f) > 2 else default_kind
        if kind not in ('outage', 'float', 'single', 'nocov', 'drop'):
            raise ValueError(f'トラブルの種類が分かりません: {part}')
        out.append((float(f[0]), float(f[1]), kind))
    return out


def parse_jump(text: str):
    """'20:5' → (20.0, 5.0)（20 秒後に 5 m 跳ぶ）。空なら None。"""
    if not (text or '').strip():
        return None
    t, d = (float(v) for v in text.split(':'))
    return t, d


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
        # 動かし方：script = 決まった道筋（直進 straight_time 秒 → 一定の速さで左に曲がる）
        #           cmd_vel = /cmd_vel の速度指令どおりに動く（cmd_timeout 秒来なければ止まる）
        self.drive_mode = self.declare_parameter('drive_mode', 'script').value
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
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
        # 障害物検出の試験用：スタートの向きにこの距離 [m] 進んだ所に岩を置く（負なら置かない）
        self.obstacle_ahead = float(self.declare_parameter('obstacle_ahead', -1.0, any_number).value)

        # --- 試験用のトラブル（シナリオ試験で使う。すべて既定では起きない）---
        # 障害物の配置：default / gap / narrow / dead_end / clutter（fake_world.make_layout）
        self.layout = self.declare_parameter('layout', 'default').value
        # GNSS の劣化 "開始秒:続く秒:種類; ..."。種類は outage（途切れ）/ float（RTK Float）/ single（単独測位）/ nocov（精度不明の単独測位）
        self.gnss_events = parse_events(self.declare_parameter('gnss_events', '').value)
        # GNSS が RTK Fix のふりをして 1 回だけ跳ぶ（マルチパス）。"秒:距離[m]"
        self.gnss_jump = parse_jump(self.declare_parameter('gnss_jump', '').value)
        # LiDAR の点群が止まる "開始秒:続く秒; ..."
        self.lidar_drops = parse_events(self.declare_parameter('lidar_drop', '').value, default_kind='drop')
        # IMU の回転の速さのずれ [rad/s]（0.01 で 1 分に 34°）
        self.imu_bias = float(self.declare_parameter('imu_bias', 0.0, any_number).value)

        # --- 砂地の試験（fake_terrain.py）---
        # 地形：flat（これまでどおり、車体は水平）/ dunes / steep / pit
        self.terrain = ft.Terrain(self.declare_parameter('terrain', 'flat').value, self.heading0)
        sd = ft.SandParams()
        self.sand = ft.SandParams(
            slip=float(self.declare_parameter('slip', sd.slip, any_number).value),
            max_climb_deg=float(self.declare_parameter('max_climb_deg', sd.max_climb_deg, any_number).value),
            tip_deg=float(self.declare_parameter('tip_deg', sd.tip_deg, any_number).value),
            # やわらかい砂 "x,y,半径; ..."（map 座標）。見た目では分からず、はまると前に進めない
            soft_sand=ft.parse_circles(self.declare_parameter('soft_sand', '').value),
        )
        # 舞い上がる砂：1 回のスキャンあたりの砂の塊の数の平均
        self.dust_rate = float(self.declare_parameter('dust', 0.0, any_number).value)
        # 周りの物体：default（支柱・岩・壁を散らばらせる）/ bare（何もない砂地。LiDAR の手がかりが少ない）
        self.world_kind = self.declare_parameter('world', 'default').value
        # true：地形の陰・近いほど多い地面の点・車体の傾きを入れた点群（scan_3d）。false：これまでの簡略版
        self.realistic_cloud = bool(self.declare_parameter('realistic_cloud', True).value)
        # 地面の点：fixed（world に固定した点。LiDAR オドメトリの手がかりになる）/
        #           resample（スキャンごとに選び直す。模様のない平らな砂地で、手がかりにならない）
        self.ground_mode = self.declare_parameter('ground_mode', 'fixed').value
        if self.ground_mode not in ('fixed', 'resample'):
            raise ValueError(f'ground_mode は fixed か resample: {self.ground_mode}')
        self.sandy = (not self.terrain.flat) or self.sand.slip > 0.0 or bool(self.sand.soft_sand) \
            or self.dust_rate > 0.0

        self.odom_pub = self.create_publisher(Odometry, '/lidar/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/unilidar/imu', 10)
        # gnss_fix と同じ分け方で出す（/gnss/fix = RTK Fix、/gnss = それ以外、/gnss/best = 有効なものすべて）
        self.gnss_fixed_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.gnss_other_pub = self.create_publisher(NavSatFix, '/gnss', 10)
        self.gnss_best_pub = self.create_publisher(NavSatFix, '/gnss/best', 10)
        self.truth_pub = self.create_publisher(NavSatFix, '/fake/truth', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/unilidar/cloud', 5)
        # 可視化用（map 座標）：仮想フィールド全体と、本当に通った道筋
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(PointCloud2, '/fake/world', latched)
        self.truth_path_pub = self.create_publisher(Path, '/fake/truth_path', 10)
        self.objects_pub = self.create_publisher(PointCloud2, '/fake/obstacles', latched)
        self.event_pub = self.create_publisher(String, '/fake/event', 10)
        # 3D 表示用（本当の姿勢の車体と点群）。重いときは view_3d:=false
        self.view_3d = bool(self.declare_parameter('view_3d', True).value)
        self.robot_pub = self.create_publisher(MarkerArray, '/fake/robot', 10)
        self.cloud_map_pub = self.create_publisher(PointCloud2, '/fake/cloud_map', 5)   # 転倒・転落の間、毎回出す（残らない）
        self.truth_path = Path()
        self.truth_path.header.frame_id = 'map'

        if self.publish_cloud:
            self.rng = np.random.default_rng(0)
            if self.world_kind == 'bare':
                self.world = np.empty((0, 3))
            elif self.world_kind == 'default':
                self.world = fake_world.make_world(seed=0)
            else:
                raise ValueError(f'world は default か bare: {self.world_kind}')
            if self.layout != 'default':
                self.world = self.apply_layout(self.world, self.layout)
            if self.obstacle_ahead > 0.0:
                d = self.obstacle_ahead
                rock = fake_world.make_rock(d * math.cos(self.heading0), d * math.sin(self.heading0))
                self.world = np.vstack([self.world, rock])
                self.get_logger().info(f'通り道の {d:.1f} m 先に岩を置きました（ローバーは通り抜けます）')
            self.world = fake_world.place_on(self.terrain, self.world)
            if not self.cloud_ground:
                self.ground = np.empty((0, 3))
            elif self.sandy or self.realistic_cloud:
                self.ground = fake_world.make_ground_grid(self.terrain, seed=0)
            else:
                self.ground = fake_world.make_ground(seed=0)
            self.dust = ft.Dust(self.dust_rate, self.terrain)
            if self.sandy:
                self.get_logger().info(
                    f'砂地: 地形 {self.terrain.kind}、空転 {self.sand.slip:.2f}、'
                    f'やわらかい砂 {len(self.sand.soft_sand)} か所、舞う砂 {self.dust_rate:.1f} 塊/スキャン、'
                    f'周り {self.world_kind}')
            self.get_logger().info(f'仮想フィールドを作りました（{len(self.world)} 点）')
            self.publish_world()
            self.create_timer(10.0, self.publish_world)   # あとから接続した表示ツールにも届くように

        # 本当の状態（ENU: 東 x, 北 y）
        self.t = 0.0
        self.e, self.n, self.yaw = 0.0, 0.0, self.heading0
        self.z, self.pitch, self.roll = 0.0, 0.0, 0.0
        if not self.terrain.flat:
            self.z, self.pitch, self.roll = self.terrain.pose(self.e, self.n, self.yaw)
        self.dead = None                      # 転倒・転落したら、その理由
        self.slip_now = 0.0
        self.dt = 0.02
        self.tick = 0
        self.last_time = None
        self.cmd = (0.0, 0.0)
        self.cmd_time = None
        self.jump_done = False
        if self.drive_mode == 'cmd_vel':
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
            self.get_logger().info('/cmd_vel の速度指令で動きます')
        elif self.drive_mode != 'script':
            raise ValueError(f'drive_mode は script か cmd_vel: {self.drive_mode}')
        self.create_timer(self.dt, self.step)

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def velocity(self, now):
        """いまの (前進の速さ [m/s], 回転の速さ [rad/s])。"""
        if self.drive_mode == 'script':
            return self.speed, (0.0 if self.t < self.straight_time else self.turn_rate)
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            return 0.0, 0.0                          # 指令が途切れたら止まる
        return self.cmd

    def step(self):
        # 実際に経った時間で進める（処理が遅い機体でも、動きと時刻が食い違わないように）
        clock_now = self.get_clock().now()
        dt = self.dt if self.last_time is None else (clock_now - self.last_time).nanoseconds * 1e-9
        dt = min(max(dt, 0.0), 0.1)
        self.last_time = clock_now
        v, w = self.velocity(clock_now)
        if self.dead:
            v, w = 0.0, 0.0
            self.event_pub.publish(String(data=self.dead))
        if self.sandy:
            mx, my = self.enu_to_map(self.e, self.n)
            s = ft.slip_ratio(self.sand, v, self.pitch, ft.soft_depth_in(self.sand, mx, my))
            self.slip_now = s
            v, w = v * (1.0 - s) * math.cos(self.pitch), w * (1.0 - 0.5 * s)   # 坂では水平に進む分が減る
        self.yaw += w * dt
        self.e += v * math.cos(self.yaw) * dt
        self.n += v * math.sin(self.yaw) * dt
        if not self.terrain.flat:
            self.z, self.pitch, self.roll = self.terrain.pose(self.e, self.n, self.yaw)
            self.check_alive()
        self.t += dt
        self.tick += 1
        now = clock_now.to_msg()

        imu = Imu()                                 # 50 Hz
        imu.header.stamp = now
        imu.header.frame_id = 'unilidar_imu'
        # 車体に固定した IMU：回転の速さ（傾いていれば z 軸の分だけ）と、重力の向き（静止時の加速度計の読み）
        rot = self.body_rotation()
        gyro = rot.T @ np.array([0.0, 0.0, w])
        acc = rot.T @ np.array([0.0, 0.0, 9.81])
        imu.angular_velocity.x, imu.angular_velocity.y = float(gyro[0]), float(gyro[1])
        imu.angular_velocity.z = float(gyro[2]) + self.imu_bias
        imu.linear_acceleration.x = float(acc[0]) + random.gauss(0.0, 0.05)
        imu.linear_acceleration.y = float(acc[1]) + random.gauss(0.0, 0.05)
        imu.linear_acceleration.z = float(acc[2]) + random.gauss(0.0, 0.05)
        imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.orientation_covariance[0] = -1.0         # 向きは出さない（北の基準がない）
        self.imu_pub.publish(imu)

        if self.tick % 5 == 0 and self.publish_cloud and not self.active(self.lidar_drops):   # 10 Hz
            pts = self.make_cloud_points()
            self.cloud_pub.publish(self.to_cloud(pts, now, 'unilidar_lidar'))
            if self.view_3d and self.tick % 10 == 0:    # 5 Hz
                self.publish_view(pts, now)

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
            kind = self.active(self.gnss_events)
            if in_outage or kind == 'outage':
                lost = self.navsat(now, 0.0, 'gnss_link', NavSatStatus.STATUS_NO_FIX)
                lost.latitude = lost.longitude = lost.altitude = float('nan')
                lost.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                lost.position_covariance = [0.0] * 9
                self.gnss_other_pub.publish(lost)          # gnss_fix は未測位も /gnss に出す
            elif kind == 'float':
                self.publish_gnss(self.navsat(now, 0.30, 'gnss_link', NavSatStatus.STATUS_GBAS_FIX), fixed=False)
            elif kind == 'single':
                self.publish_gnss(self.navsat(now, 2.5, 'gnss_link', NavSatStatus.STATUS_FIX), fixed=False)
            elif kind == 'nocov':
                # GGA だけのときの gnss_fix：単独測位で、精度が不明（共分散 0・UNKNOWN）
                msg = self.navsat(now, 2.5, 'gnss_link', NavSatStatus.STATUS_FIX)
                msg.position_covariance = [0.0] * 9
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.publish_gnss(msg, fixed=False)
            else:
                msg = self.navsat(now, self.sigma_gnss, 'gnss_link')
                if self.gnss_jump and not self.jump_done and self.t >= self.gnss_jump[0]:
                    self.jump_done = True              # RTK Fix・精度 2 cm と言いながら、横に跳んだ位置
                    msg = self.navsat(now, self.sigma_gnss, 'gnss_link', offset=self.gnss_jump[1])
                    self.get_logger().info(f'GNSS を {self.gnss_jump[1]:.1f} m 跳ばしました（RTK Fix のふり）')
                self.publish_gnss(msg, fixed=True)
            self.truth_pub.publish(self.navsat(now, 0.0, 'base_link'))

        if self.tick % 25 == 0:                      # 2 Hz：本当の道筋（map 座標）
            x, y = self.enu_to_map(self.e, self.n)
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            qx, qy, qz, qw = hm.quaternion_from_yaw(self.yaw - self.heading0)
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            self.truth_path.poses.append(pose)
            if len(self.truth_path.poses) > 5000:
                del self.truth_path.poses[0]
            self.truth_path.header.stamp = now
            self.truth_path_pub.publish(self.truth_path)

    def body_rotation(self):
        return ft.rotation(self.yaw, self.pitch, self.roll)

    def check_alive(self):
        if self.dead:
            return
        tilt = math.degrees(max(abs(self.pitch), abs(self.roll)))
        x, y = self.enu_to_map(self.e, self.n)
        if self.terrain.in_pit(self.e, self.n):
            self.dead = '転落'
        elif tilt > self.sand.tip_deg:
            self.dead = '転倒'
        if self.dead:
            self.get_logger().error(f'{self.dead}しました（map ({x:.1f}, {y:.1f})、傾き {tilt:.0f}°）。もう動けません')
            self.event_pub.publish(String(data=self.dead))

    def publish_gnss(self, msg, fixed: bool):
        (self.gnss_fixed_pub if fixed else self.gnss_other_pub).publish(msg)
        self.gnss_best_pub.publish(msg)

    def active(self, events):
        """いまの時刻に起きているトラブルの種類（なければ None）。"""
        for start, duration, kind in events:
            if start <= self.t < start + duration:
                return kind
        return None

    def apply_layout(self, world, name):
        """試験用の配置に入れ替える（その範囲にあった元の物体は取り除く）。"""
        (x0, x1), (y0, y1) = fake_world.LAYOUT_ZONE
        c, s = math.cos(-self.heading0), math.sin(-self.heading0)
        mx = c * world[:, 0] - s * world[:, 1]
        my = s * world[:, 0] + c * world[:, 1]
        keep = ~((mx >= x0) & (mx <= x1) & (my >= y0) & (my <= y1))
        lay = fake_world.make_layout(name)
        c, s = math.cos(self.heading0), math.sin(self.heading0)   # map → ENU
        lay_enu = np.c_[c * lay[:, 0] - s * lay[:, 1], s * lay[:, 0] + c * lay[:, 1], lay[:, 2]]
        self.get_logger().info(f'障害物の配置: {name}')
        return np.vstack([world[keep], lay_enu])

    def enu_to_map(self, e, n):
        """ENU（東・北）を map 座標に直す。

        map 座標は EKF が決める座標で、原点はスタート地点、x 軸はスタート時のローバーの前方。
        """
        c, s = math.cos(-self.heading0), math.sin(-self.heading0)
        return c * e - s * n, s * e + c * n

    def publish_world(self):
        """仮想フィールド全体を map 座標の点群として配信する（表示用に間引く）。"""
        rng = np.random.default_rng(1)
        objects = self.world[rng.choice(len(self.world), min(30000, len(self.world)), replace=False)]
        parts = [objects]
        if len(self.ground):
            parts.append(self.ground[rng.choice(len(self.ground), min(15000, len(self.ground)), replace=False)])
        pts = np.vstack(parts)
        c, s = math.cos(-self.heading0), math.sin(-self.heading0)
        mx = c * pts[:, 0] - s * pts[:, 1]
        my = s * pts[:, 0] + c * pts[:, 1]
        stamp = self.get_clock().now().to_msg()
        self.world_pub.publish(self.to_cloud(np.c_[mx, my, pts[:, 2]].astype(np.float32), stamp, 'map'))
        ox = c * objects[:, 0] - s * objects[:, 1]
        oy = s * objects[:, 0] + c * objects[:, 1]
        self.objects_pub.publish(self.to_cloud(np.c_[ox, oy, objects[:, 2]].astype(np.float32), stamp, 'map'))

    def truth_map_pose(self):
        """本当の姿勢（map 座標）：位置 (x, y, z) と、車体 → map の回転行列。"""
        x, y = self.enu_to_map(self.e, self.n)
        return np.array([x, y, self.z]), ft.rotation(self.yaw - self.heading0, self.pitch, self.roll)

    def publish_view(self, pts, stamp):
        pos, rot = self.truth_map_pose()
        sensor = pos + rot @ np.array([self.lidar_x, 0.0, self.lidar_z])
        cloud = (pts.astype(float) @ rot.T + sensor).astype(np.float32)
        self.cloud_map_pub.publish(self.to_cloud(cloud, stamp, 'map'))
        q = rotation_to_quaternion(rot)
        body = Marker()
        body.header.stamp, body.header.frame_id = stamp, 'map'
        body.ns, body.id, body.type, body.action = 'truth', 0, Marker.CUBE, Marker.ADD
        center = pos + rot @ np.array([0.0, 0.0, 0.25])     # 車体の箱の中心（地面から 0.25 m）
        body.pose.position.x, body.pose.position.y, body.pose.position.z = (float(v) for v in center)
        body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w = q
        body.scale.x, body.scale.y, body.scale.z = 1.0, 0.8, 0.4
        body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.55, 0.1, 0.9
        if self.dead:
            body.color.r, body.color.g, body.color.b = 0.9, 0.1, 0.1
        arrow = Marker()
        arrow.header = body.header
        arrow.ns, arrow.id, arrow.type, arrow.action = 'truth', 1, Marker.ARROW, Marker.ADD
        top = pos + rot @ np.array([0.0, 0.0, 0.5])
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = (float(v) for v in top)
        arrow.pose.orientation = body.pose.orientation
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.8, 0.12, 0.12
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0
        text = Marker()
        text.header = body.header
        text.ns, text.id, text.type, text.action = 'truth', 2, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y = float(pos[0]), float(pos[1])
        text.pose.position.z = float(pos[2]) + 1.2
        text.scale.z = 0.35
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        text.text = (f'高さ {self.z:+.2f} m  前後 {math.degrees(-self.pitch):+.0f}°  左右 {math.degrees(self.roll):+.0f}°'
                     f'  空転 {self.slip_now * 100:.0f}%' + (f'  {self.dead}' if self.dead else ''))
        self.robot_pub.publish(MarkerArray(markers=[body, arrow, text]))

    def make_cloud_points(self):
        if self.sandy or self.realistic_cloud:
            ground = None if (self.cloud_ground and self.ground_mode == 'resample') else self.ground
            pts = fake_world.scan_3d(self.world, ground, self.terrain,
                                     np.array([self.e, self.n, self.z]), self.body_rotation(),
                                     self.lidar_x, self.lidar_z, self.cloud_min_range,
                                     self.cloud_max_range, self.cloud_points, self.cloud_noise, self.rng,
                                     extra=self.dust.step(self.e, self.n))
            return pts
        return fake_world.scan(self.world, self.ground, self.e, self.n, self.yaw,
                               self.lidar_x, self.lidar_z, self.cloud_min_range,
                               self.cloud_max_range, self.cloud_points, self.cloud_noise, self.rng)

    @staticmethod
    def to_cloud(pts, stamp, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
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

    def navsat(self, stamp, sigma, frame, status=NavSatStatus.STATUS_GBAS_FIX, offset=0.0):
        e = self.e + random.gauss(0.0, sigma) if sigma > 0 else self.e
        n = self.n + random.gauss(0.0, sigma) if sigma > 0 else self.n
        if offset:                                    # 進行方向の左へずらす
            e -= offset * math.sin(self.yaw)
            n += offset * math.cos(self.yaw)
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.status.status = status
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self.lat0 + math.degrees(n / hm.EARTH_RADIUS)
        msg.longitude = self.lon0 + math.degrees(
            e / (hm.EARTH_RADIUS * math.cos(math.radians(self.lat0))))
        msg.altitude = 30.0
        var = max(sigma, 0.02) ** 2
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4 * var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        return msg


def rotation_to_quaternion(r):
    """回転行列 → (x, y, z, w)"""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return x, y, z, w


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
