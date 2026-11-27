"""センサ側を起動する: 取付位置（TF）と GNSS ドライバ（大塚さんの gnss_fix）。

  ros2 launch trc_bringup sensors.launch.py
  ros2 launch trc_bringup sensors.launch.py gnss_port:=/dev/ttyACM0

gnss_fix は ~/trc2027/deps_ws/src に clone してビルドしておく（docs/setup-native.md 参照）。
出力（/gnss, /gnss/fix, /gnss/clas, /gnss/best など）は、位置推定の gnss_adapter_node が受け取る。
frame_id は URDF のアンテナ座標系 gnss_link にそろえる。

LiDAR のドライバ（unilidar_sdk2）は別のワークスペースでビルドして別に起動する。
L2 は USB（シリアル）でつなぐ（Jetson の Ethernet は通信系統用に空けておく）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gnss_port', default_value='',
                              description='受信機のシリアルポート。空なら gnss_fix の既定（u-blox の /dev/serial/by-id/ のパス）'),
        DeclareLaunchArgument('gnss_config', default_value='',
                              description='gnss_fix の設定ファイル（例 simplertk2b_usb.yaml）'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_description'), 'launch', 'description.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('gnss_fix'), 'launch', 'gnss_fix.launch.py']),
            launch_arguments={
                'port': LaunchConfiguration('gnss_port'),
                'config': LaunchConfiguration('gnss_config'),
                'frame_id': 'gnss_link',
            }.items(),
        ),
    ])
