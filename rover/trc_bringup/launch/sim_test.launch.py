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

画面で見る（foxglove:=true、既定）:
  foxglove_bridge（WebSocket、ポート 8765）を起動する。Mac のブラウザで Lichtblick を開いてつなぐ
  （手順は docs/setup-native.md の「シミュレーションを画面で見る」）
  表示用に次のトピックも出す:
    /fake/world          仮想フィールド全体（map 座標）
    /fake/truth_path     本当の道筋
    /localization/path   推定した道筋（/odometry/global から）
    /gnss/path           GNSS の測定点（/odometry/gps から）
    /fake/position_error 推定位置の誤差 [m]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='true で foxglove_bridge を起動し、ブラウザで表示できるようにする'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

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
        # 表示用：道筋と誤差
        Node(
            package='trc_localization', executable='path_publisher_node', name='localization_path',
            remappings=[('odometry', '/odometry/global'), ('path', '/localization/path')],
        ),
        Node(
            package='trc_localization', executable='path_publisher_node', name='gnss_path',
            parameters=[{'min_distance': 0.0}],
            remappings=[('odometry', '/odometry/gps'), ('path', '/gnss/path')],
        ),
        Node(package='trc_localization', executable='position_error_node', name='position_error'),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
            parameters=[{'port': LaunchConfiguration('foxglove_port'), 'address': '0.0.0.0'}],
            condition=IfCondition(LaunchConfiguration('foxglove')),
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
