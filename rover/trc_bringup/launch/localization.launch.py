"""位置推定一式（取付位置の TF + 2段の EKF + 向きの初期化）を起動する。

実機:        ros2 launch trc_bringup localization.launch.py
bag の再生:  ros2 launch trc_bringup localization.launch.py use_sim_time:=true
             （別ターミナルで ros2 bag play <bag> --clock）

LiDAR オドメトリ（KISS-ICP など）は別に起動し、出力トピックを lidar_odom_topic に渡す。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_odom_topic', default_value='/lidar/odom'),

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
    ])
