"""障害物検出（LiDAR の点群 → 地面を取り除く → 障害物・前方距離・停止信号）。

  ros2 launch trc_bringup perception.launch.py
  ros2 launch trc_bringup perception.launch.py stop_distance:=1.0

出力は trc_perception/obstacle_detector_node.py の説明を参照。
base_link と点群の座標系の TF（取付位置）が必要なので、localization.launch.py などと一緒に起動する。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('stop_distance', default_value='1.5',
                              description='前方の障害物がこれより近いと /obstacles/stop を true にする [m]'),
        DeclareLaunchArgument('corridor_width', default_value='1.0',
                              description='前方を見る帯の幅 [m]（ローバーの幅＋余裕）'),
        Node(
            package='trc_perception', executable='obstacle_detector_node', name='obstacle_detector',
            parameters=[{
                'stop_distance': LaunchConfiguration('stop_distance'),
                'corridor_width': LaunchConfiguration('corridor_width'),
            }],
            remappings=[('pointcloud', LaunchConfiguration('cloud_topic'))],
        ),
    ])
