"""位置推定一式（取付位置の TF + 2段の EKF + 向きの初期化）を起動する。

実機:        ros2 launch trc_bringup localization.launch.py
bag の再生:  ros2 launch trc_bringup localization.launch.py use_sim_time:=true
             （別ターミナルで ros2 bag play <bag> --clock）

LiDAR オドメトリ（KISS-ICP など）は別に起動し、出力トピックを lidar_odom_topic に渡す。
大会本番や試験走行では log_history:=true で制御履歴（CSV）も記録する。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_odom_topic', default_value='/lidar/odom'),
        DeclareLaunchArgument('log_history', default_value='false',
                              description='true で制御履歴を ~/trc2027/logs に記録する'),
        DeclareLaunchArgument('history_rate', default_value='2.0',
                              description='制御履歴の記録頻度 [Hz]（規定は 1 Hz 以上）'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_description'), 'launch', 'description.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_localization'), 'launch', 'localization.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
            }.items(),
        ),
        Node(
            package='trc_logging', executable='control_history_node', name='control_history',
            parameters=[{
                'rate': LaunchConfiguration('history_rate'),
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(LaunchConfiguration('log_history')),
        ),
    ])
