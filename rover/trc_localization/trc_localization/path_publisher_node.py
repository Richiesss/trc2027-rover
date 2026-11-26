"""nav_msgs/Odometry を受け取り、通った道筋を nav_msgs/Path にして配信する（可視化用）。

  ros2 run trc_localization path_publisher_node --ros-args \\
      -r odometry:=/odometry/global -r path:=/localization/path

min_distance [m] 以上動いたときだけ点を足し、max_points を超えたら古い点から捨てる。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.min_distance = self.declare_parameter('min_distance', 0.05).value
        self.max_points = self.declare_parameter('max_points', 5000).value
        self.path = Path()
        self.create_subscription(Odometry, 'odometry', self.on_odom, 20)
        self.pub = self.create_publisher(Path, 'path', 10)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.hypot(p.x - last.x, p.y - last.y) < self.min_distance:
                return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[0]
        self.path.header = msg.header
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
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
