"""実機なしで「点群 → LiDAR オドメトリ → GNSS との融合」まで一通り試す。

  ros2 launch trc_bringup sim_test.launch.py
  ros2 launch trc_bringup sim_test.launch.py heading_deg:=200.0 log_history:=true
  ros2 launch trc_bringup sim_test.launch.py gnss_outage_start:=25.0   # 25 秒後から 10 秒間 GNSS を切る

起動するもの:
  - fake_sensors_node : 仮想フィールドの点群（/unilidar/cloud）・IMU・GNSS を出す
  - KISS-ICP          : 点群から LiDAR オドメトリ（/kiss/odometry）を作る
  - 位置推定一式       : 取付位置の TF + 2段の EKF + 向きの初期化（+ 制御履歴）

結果の見方:
  /gps/filtered（推定位置）と /fake/truth（本当の位置）を比べる
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('heading_deg', default_value='90.0',
                              description='にせのローバーのスタートの向き（東から反時計回り）'),
        DeclareLaunchArgument('log_history', default_value='false'),
        DeclareLaunchArgument('gnss_outage_start', default_value='-1.0',
                              description='GNSS を途切れさせる時刻 [s]（-1 で途切れさせない）'),
        DeclareLaunchArgument('gnss_outage_duration', default_value='10.0'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_bringup'), 'launch', 'localization.launch.py']),
            launch_arguments={
                'lidar_odom_topic': '/kiss/odometry',
                'log_history': LaunchConfiguration('log_history'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_localization'), 'launch', 'lidar_odometry.launch.py']),
        ),
        # TF（取付位置）と KISS-ICP がそろってから、にせのセンサを動かし始める
        TimerAction(period=3.0, actions=[
            Node(
                package='trc_localization', executable='fake_sensors_node', name='fake_sensors',
                parameters=[{
                    'publish_cloud': True,
                    'publish_odom': False,
                    'heading_deg': LaunchConfiguration('heading_deg'),
                    'gnss_outage_start': LaunchConfiguration('gnss_outage_start'),
                    'gnss_outage_duration': LaunchConfiguration('gnss_outage_duration'),
                }],
            ),
        ]),
    ])
