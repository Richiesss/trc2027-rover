"""LiDAR オドメトリ（位置だけ）から、ローバー基準の速度（twist）を作るノード。

KISS-ICP は位置（pose）しか出さない。これを EKF に「位置の差分」（differential）として入れると、
map 側の EKF（ekf_map）は差分を自分の出す map→odom で回してから使うので、一度向きがずれると
ずれが自分自身を増やしてしまう（シミュレーションで向きが 100° 以上ずれた）。
robot_localization の推奨どおり、速度（ローバー基準の前後・左右・回転）にして入れる。

あわせて、ありえない速さ（max_speed / max_yaw_rate 超え）の値は捨てる（点群の合わせ間違いで位置が跳んだとき）。

入力:  odometry     nav_msgs/Odometry（pose だけ使う）
出力:  twist        nav_msgs/Odometry（twist だけ有効、child_frame_id は base_frame）
"""

import math

import rclpy
from nav_msgs.msg import Odometry
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
        self.pub = self.create_publisher(Odometry, 'twist', 10)
        self.create_subscription(Odometry, 'odometry', self.on_odom, 20)

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        cur = (t, p.position.x, p.position.y, yaw_of(p.orientation))
        prev, self.prev = self.prev, cur
        if prev is None:
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
