"""LiDAR オドメトリ（位置だけ）から、ローバー基準の速度（twist）を作るノード。

KISS-ICP は位置（pose）しか出さない。これを EKF に「位置の差分」（differential）として入れると、
map 側の EKF（ekf_map）は差分を自分の出す map→odom で回してから使うので、一度向きがずれると
ずれが自分自身を増やしてしまう（シミュレーションで向きが 100° 以上ずれた）。
robot_localization の推奨どおり、速度（ローバー基準の前後・左右・回転）にして入れる。

あわせて、ありえない速さ（max_speed / max_yaw_rate 超え）の値は捨てる（点群の合わせ間違いで位置が跳んだとき）。

砂地の対策（周りに何もない砂地）:
  点群に手がかり（地面以外の物）がほとんどないと、LiDAR オドメトリは前後・左右の動きを読めず、
  止まっている・跳んだと誤る（シミュレーションで GNSS が 30 秒途切れたとき 9 m ずれた）。
  obstacle_detector が「手がかりが少ない」（lidar_degenerate）と知らせている間は LiDAR オドメトリを使わず、
  速度指令（cmd_vel）×（1 − 空転の割合）を速度として出す。空転の割合は、LiDAR オドメトリが使えるあいだに
  速度指令と実際の速さを比べて覚えておく。

入力:  odometry          nav_msgs/Odometry（pose だけ使う）
       cmd_vel           geometry_msgs/Twist（モータへの速度指令）
       lidar_degenerate  std_msgs/Bool（true の間は速度指令から速度を作る）
出力:  twist        nav_msgs/Odometry（twist だけ有効、child_frame_id は base_frame）
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_velocity(prev, cur):
    """(t, x, y, yaw) の 2 つから、前の姿勢から見た (vx, vy, vyaw) と dt。"""
    t0, x0, y0, a0 = prev
    t1, x1, y1, a1 = cur
    dt = t1 - t0
    if dt <= 0.0:
        return None
    dx, dy = x1 - x0, y1 - y0
    c, s = math.cos(a0), math.sin(a0)
    da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, da / dt, dt


class OdomToTwist(Node):

    def __init__(self):
        super().__init__('odom_to_twist')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.max_speed = float(self.declare_parameter('max_speed', 2.0).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 2.0).value)
        self.max_dt = float(self.declare_parameter('max_dt', 1.0).value)
        self.speed_sigma = float(self.declare_parameter('speed_sigma', 0.1).value)        # [m/s]
        self.yaw_rate_sigma = float(self.declare_parameter('yaw_rate_sigma', 0.03).value)  # [rad/s]
        self.prev = None
        self.rejected = 0
        # 手がかりが少ないときの代わり（速度指令から作る速度）
        self.cmd_sigma = float(self.declare_parameter('cmd_speed_sigma', 0.15).value)   # [m/s]
        self.slip = float(self.declare_parameter('slip_initial', 0.15).value)          # 空転の割合（覚えた値）
        self.slip_max = float(self.declare_parameter('slip_max', 0.6).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
        self.cmd = None                                 # (v, w, 受信時刻)
        self.degenerate = False
        self.frame_id = 'odom'
        self.pub = self.create_publisher(Odometry, 'twist', 10)
        self.create_subscription(Odometry, 'odometry', self.on_odom, 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, 'lidar_degenerate', self.on_degenerate, 10)
        self.create_timer(0.1, self.publish_from_cmd)

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z, self.get_clock().now())

    def on_degenerate(self, msg: Bool):
        if msg.data != self.degenerate:
            if msg.data:
                self.get_logger().warn(f'点群に手がかりが少ないので、LiDAR オドメトリの代わりに速度指令から速さを出します'
                                       f'（空転 {self.slip * 100:.0f} % として）')
            else:
                self.get_logger().info('点群の手がかりが戻ったので、LiDAR オドメトリに戻します')
        self.degenerate = msg.data

    def fresh_cmd(self):
        if self.cmd is None or (self.get_clock().now() - self.cmd[2]).nanoseconds * 1e-9 > self.cmd_timeout:
            return 0.0, 0.0
        return self.cmd[0], self.cmd[1]

    def learn_slip(self, vx: float):
        """LiDAR オドメトリが使えるあいだに、速度指令と実際の速さから空転の割合を覚える。"""
        v, w = self.fresh_cmd()
        if v < 0.2 or abs(w) > 0.2:
            return
        slip = 1.0 - vx / v
        if 0.0 <= slip <= self.slip_max:                 # はまっている最中などは覚えない
            self.slip = 0.98 * self.slip + 0.02 * slip

    def publish_from_cmd(self):
        if not self.degenerate:
            return
        v, _ = self.fresh_cmd()
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.base_frame
        out.twist.twist.linear.x = v * (1.0 - self.slip)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.cmd_sigma ** 2
        cov[14] = cov[21] = cov[28] = 1e-3
        cov[35] = 1.0
        out.twist.covariance = cov
        self.pub.publish(out)

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        cur = (t, p.position.x, p.position.y, yaw_of(p.orientation))
        prev, self.prev = self.prev, cur
        self.frame_id = msg.header.frame_id
        if prev is None or self.degenerate:
            return
        v = body_velocity(prev, cur)
        if v is None:
            return
        vx, vy, w, dt = v
        if dt > self.max_dt:
            return                                     # 間があきすぎた（止まっていた・落ちていた）
        if math.hypot(vx, vy) > self.max_speed or abs(w) > self.max_yaw_rate:
            self.rejected += 1
            self.get_logger().warn(
                f'LiDAR オドメトリが跳んだので捨てました（{math.hypot(vx, vy):.1f} m/s, '
                f'{math.degrees(w):.0f}°/s、累計 {self.rejected} 回）', throttle_duration_sec=5.0)
            return
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = self.base_frame
        out.twist.twist.linear.x, out.twist.twist.linear.y = vx, vy
        out.twist.twist.angular.z = w
        cov = [0.0] * 36
        cov[0] = cov[7] = self.speed_sigma ** 2
        cov[14] = cov[21] = cov[28] = 1e-3
        cov[35] = self.yaw_rate_sigma ** 2
        out.twist.covariance = cov
        self.pub.publish(out)
        self.learn_slip(vx)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTwist()
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
