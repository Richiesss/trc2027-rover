"""センサ側を起動する: 取付位置（TF）と GNSS ノード。

LiDAR のドライバ（unilidar_sdk2）は別のワークスペースでビルドして別に起動する
（docs/setup-native.md 参照）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gnss_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('gnss_baudrate', default_value='38400'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('trc_description'), 'launch', 'description.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='trc_gnss', executable='gnss_node', name='gnss_node',
            parameters=[{
                'port': LaunchConfiguration('gnss_port'),
                'baudrate': LaunchConfiguration('gnss_baudrate'),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
