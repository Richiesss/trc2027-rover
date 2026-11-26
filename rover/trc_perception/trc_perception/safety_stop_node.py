"""前方に障害物があるときに前進を止める安全装置。

走行制御（自律走行・手動操作）の速度指令と、モータ（シミュレーションではにせのローバー）の間にはさむ。

  cmd_vel_in （走行制御から） ──▶ safety_stop ──▶ cmd_vel_out （モータへ）
                                       ▲
                          /obstacles/stop（obstacle_detector から）

ルール:
  - /obstacles/stop が true の間は、前進（linear.x > 0）を 0 にする。
    その場での回転と後退はそのまま通す（障害物から向きを変えて抜け出せるように）
  - /obstacles/stop が stop_timeout 秒以上来ていなければ、障害物検出が止まったとみなして前進を止める（安全側）
  - cmd_vel_in が来たときだけ出力する（指令が途切れたときの停止はモータ側の指令切れの扱いに任せる）

出力:
  cmd_vel_out     geometry_msgs/Twist
  /safety/active  std_msgs/Bool       前進を止めているとき true
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


def filter_command(linear_x: float, angular_z: float, blocked: bool):
    """止めるべきなら前進だけを 0 にした (linear_x, angular_z, 止めたか) を返す。"""
    if blocked and linear_x > 0.0:
        return 0.0, angular_z, True
    return linear_x, angular_z, False


class SafetyStop(Node):

    def __init__(self):
        super().__init__('safety_stop')
        self.stop_timeout = float(self.declare_parameter('stop_timeout', 0.5).value)
        self.stop = False
        self.stop_time = None
        self.active = None        # いま止めている理由（止めていなければ None）
        self.out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.active_pub = self.create_publisher(Bool, '/safety/active', 10)
        self.create_subscription(Bool, '/obstacles/stop', self.on_stop, 10)
        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd, 10)
        self.get_logger().info('安全停止: 前方に障害物があるときは前進を止めます（回転・後退は通す）')

    def on_stop(self, msg: Bool):
        self.stop = msg.data
        self.stop_time = self.get_clock().now()

    def blocked(self):
        """(止めるか, 理由) を返す。"""
        if self.stop_time is None:
            return True, '障害物検出の起動待ち'
        age = (self.get_clock().now() - self.stop_time).nanoseconds * 1e-9
        if age > self.stop_timeout:
            return True, '障害物の情報が来ていない'
        return self.stop, '障害物'

    def on_cmd(self, msg: Twist):
        blocked, reason = self.blocked()
        lx, az, active = filter_command(msg.linear.x, msg.angular.z, blocked)
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = lx, msg.linear.y, msg.linear.z
        out.angular.x, out.angular.y, out.angular.z = msg.angular.x, msg.angular.y, az
        self.out_pub.publish(out)
        self.active_pub.publish(Bool(data=active))
        state = reason if active else None
        if state != self.active:
            if state == '障害物':
                self.get_logger().info('障害物のため前進を止めました')
            elif state is not None:
                self.get_logger().warn(f'{state}のため前進を止めています')
            elif self.active is not None:
                self.get_logger().info('前進を再開します')
            self.active = state


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
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
