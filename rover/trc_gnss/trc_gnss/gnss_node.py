"""simpleRTK2B（ZED-F9P）の NMEA を読み、位置を配信するノード。

配信するトピック:
  gnss/fix      sensor_msgs/NavSatFix  位置。RTK の状態に応じて共分散を変える
  gnss/quality  std_msgs/UInt8         GGA の Fix quality（4 = RTK Fix, 5 = RTK Float, 1 = 単独）

robot_localization の navsat_transform は共分散を見て GNSS をどれだけ信用するか決めるので、
RTK Fix のときは小さく、単独測位のときは大きくしておくことが精度に直結する。
"""

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8

from trc_gnss import nmea

STATUS_FROM_QUALITY = {
    nmea.QUALITY_SINGLE: NavSatStatus.STATUS_FIX,
    nmea.QUALITY_DGPS: NavSatStatus.STATUS_SBAS_FIX,
    nmea.QUALITY_RTK_FIXED: NavSatStatus.STATUS_GBAS_FIX,
    nmea.QUALITY_RTK_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
    nmea.QUALITY_DEAD_RECKONING: NavSatStatus.STATUS_FIX,
}


class GnssNode(Node):

    def __init__(self):
        super().__init__('gnss_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baudrate = self.declare_parameter('baudrate', 38400).value
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        self.vertical_scale = self.declare_parameter('vertical_scale', 2.0).value
        self.sigma_table = {
            nmea.QUALITY_RTK_FIXED: self.declare_parameter('sigma_rtk_fixed', 0.02).value,
            nmea.QUALITY_RTK_FLOAT: self.declare_parameter('sigma_rtk_float', 0.30).value,
            nmea.QUALITY_DGPS: self.declare_parameter('sigma_dgps', 0.80).value,
            nmea.QUALITY_SINGLE: self.declare_parameter('sigma_single', 2.50).value,
            nmea.QUALITY_DEAD_RECKONING: self.declare_parameter('sigma_dead_reckoning', 10.0).value,
        }

        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.quality_pub = self.create_publisher(UInt8, 'gnss/quality', 10)

        self.serial = None
        self.buffer = b''
        self.last_quality = None
        self.create_timer(0.01, self.poll)          # 100 Hz で受信バッファを読む
        self.create_timer(2.0, self.ensure_open)    # 抜けたら2秒ごとに開き直す
        self.ensure_open()

    def ensure_open(self):
        if self.serial is not None:
            return
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0)
            self.get_logger().info(f'{self.port}（{self.baudrate} bps）を開きました')
        except serial.SerialException as e:
            self.get_logger().warn(f'{self.port} を開けません: {e}', throttle_duration_sec=10.0)

    def poll(self):
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if waiting:
                self.buffer += self.serial.read(waiting)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'受信エラー（開き直します）: {e}')
            self.serial.close()
            self.serial = None
            self.buffer = b''
            return

        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            self.handle_line(line.decode('ascii', errors='ignore'))
        if len(self.buffer) > 4096:                  # UBX のバイナリなどで改行が来ない場合
            self.buffer = b''

    def handle_line(self, line: str):
        if 'GGA' not in line:
            return
        fix = nmea.parse_gga(line)
        if fix is None:
            # 受信機は動いているが、まだ測位できていない（屋内でアンテナが空を見られない など）
            if nmea.gga_quality(line) == nmea.QUALITY_INVALID:
                if self.last_quality != nmea.QUALITY_INVALID:
                    self.get_logger().info('測位待ち（アンテナを空の見える場所に出してください）')
                    self.last_quality = nmea.QUALITY_INVALID
                self.quality_pub.publish(UInt8(data=nmea.QUALITY_INVALID))
            return

        if fix.quality != self.last_quality:
            name = nmea.QUALITY_NAMES.get(fix.quality, str(fix.quality))
            self.get_logger().info(f'測位の状態: {name}（衛星 {fix.num_satellites} 機）')
            self.last_quality = fix.quality

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = STATUS_FROM_QUALITY.get(fix.quality, NavSatStatus.STATUS_FIX)
        msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)
        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.ellipsoid_height        # NavSatFix の高さは楕円体高
        msg.position_covariance = nmea.position_covariance(
            fix, self.sigma_table, self.vertical_scale)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(msg)
        self.quality_pub.publish(UInt8(data=fix.quality))


def main(args=None):
    rclpy.init(args=args)
    node = GnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
