"""LiDAR オドメトリ（KISS-ICP）を、位置推定に合う設定で起動する。

KISS-ICP は別のワークスペース（~/trc2027/deps_ws）でビルドしておく（docs/setup-native.md 参照）。

  ros2 launch trc_localization lidar_odometry.launch.py                       # 実機の L2
  ros2 launch trc_localization lidar_odometry.launch.py use_sim_time:=true    # rosbag 再生

出力: /kiss/odometry（nav_msgs/Odometry、座標系 odom → base_link）
  - TF（odom→base_link）は配信しない。EKF（ekf_odom）が配信するため
  - base_link からの LiDAR の取付位置は、trc_description の TF を使う
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('trc_localization'), 'config', 'kiss_icp.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/unilidar/cloud'),

        Node(
            package='kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
            output='screen',
            remappings=[('pointcloud_topic', LaunchConfiguration('cloud_topic'))],
            parameters=[
                {
                    'base_frame': 'base_link',       # オドメトリを base_link の動きとして出す
                    'lidar_odom_frame': 'odom',      # EKF の odom 座標と同じ名前にする
                    'publish_odom_tf': False,        # TF は EKF が出す
                    'invert_odom_tf': False,
                    'publish_debug_clouds': False,
                    'position_covariance': 0.1,
                    'orientation_covariance': 0.1,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
                config,
            ],
        ),
    ])
