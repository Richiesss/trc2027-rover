"""決まった道筋の速度指令を出すノード（シミュレーション用の「運転手」）。

  直進 straight_time 秒（speed [m/s]）→ そのあとは speed で走りながら turn_rate [rad/s] で左に曲がる

fake_sensors_node（drive_mode:=cmd_vel）と組み合わせ、間に safety_stop_node をはさむと
「障害物の手前で止まる」を試せる。

出力:
  cmd_vel   geometry_msgs/Twist（20 Hz）
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ScriptedDriver(Node):

    def __init__(self):
        super().__init__('scripted_driver')
        self.speed = float(self.declare_parameter('speed', 0.5).value)
        self.straight_time = float(self.declare_parameter('straight_time', 12.0).value)
        self.turn_rate = float(self.declare_parameter('turn_rate', 0.15).value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start = self.get_clock().now()
        self.create_timer(0.05, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = 0.0 if t < self.straight_time else self.turn_rate
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedDriver()
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
