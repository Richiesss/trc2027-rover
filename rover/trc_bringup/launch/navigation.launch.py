"""実機で走る：障害物検出 + 安全停止 + 経路計画（+ 自律ミッション）。

  ros2 launch trc_bringup navigation.launch.py mission:=~/trc2027/missions/day1.yaml   # 緯度・経度のミッション
  ros2 launch trc_bringup navigation.launch.py waypoints:="10,0; 10,5"                 # map 座標のウェイポイント

先に sensors.launch.py（GNSS・取付位置）、LiDAR のドライバ、lidar_odometry.launch.py、
localization.launch.py を起動しておく。速度指令は /cmd_vel（モータへ）に出る。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission = LaunchConfiguration('mission')
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='',
                              description='ミッションファイル（緯度・経度のチェックポイント）。指定すると waypoints は使わない'),
        DeclareLaunchArgument('waypoints', default_value='',
                              description='map 座標のウェイポイント "x,y; x,y"（mission を使わないとき）'),
        DeclareLaunchArgument('max_speed', default_value='0.5', description='最高速度 [m/s]'),
        DeclareLaunchArgument('cloud_topic', default_value='/unilidar/cloud'),

        # 障害物検出と安全停止（経路計画の指令 /cmd_vel_nav → 安全停止 → /cmd_vel）
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_bringup'), 'launch', 'perception.launch.py']),
            launch_arguments={'safety': 'true', 'cloud_topic': LaunchConfiguration('cloud_topic')}.items(),
        ),
        Node(
            package='trc_navigation', executable='local_planner_node', name='local_planner',
            parameters=[{
                'waypoints': ParameterValue(PythonExpression(
                    ["'' if '", mission, "' else '", LaunchConfiguration('waypoints'), "'"]), value_type=str),
                'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
            }],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='trc_navigation', executable='mission_node', name='mission',
            parameters=[{'mission_file': mission}],
            condition=IfCondition(PythonExpression(["'", mission, "' != ''"])),
        ),
    ])
