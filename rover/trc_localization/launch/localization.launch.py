"""位置推定（2段のEKF + navsat_transform + 向きの初期化）を起動する。

入力（引数で差し替え可）:
  lidar_odom_topic  LiDARオドメトリ（nav_msgs/Odometry）。既定 /lidar/odom
                    odom_to_twist_node で速度（/lidar/twist）にしてから EKF に入れる
  imu_topic         L2 内蔵 IMU（sensor_msgs/Imu）。既定 /unilidar/imu
  gnss_ns           GNSS ドライバ（gnss_fix）のトピックの頭。既定 ''（/gnss, /gnss/fix, /gnss/clas）
                    gnss_adapter_node で /gnss/checked（精度を確かめた測位）と /gnss/quality にまとめて使う
出力:
  /odometry/local   odom 座標の位置（滑らか、ずれは残る）
  /odometry/global  map 座標の位置（GNSS で補正済み）← 走行・制御履歴にはこちらを使う
  /odometry/gps     GNSS を map 座標に直したもの
  /gps/filtered     推定位置を緯度経度に戻したもの
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('trc_localization'), 'config', 'ekf.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_odom = LaunchConfiguration('lidar_odom_topic')
    imu = LaunchConfiguration('imu_topic')
    gnss = '/gnss/checked'

    common = {'use_sim_time': use_sim_time}
    topics = {'odom0': '/lidar/twist', 'imu0': imu}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='rosbag 再生時は true（ros2 bag play --clock と組み合わせる）'),
        DeclareLaunchArgument('lidar_odom_topic', default_value='/lidar/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/unilidar/imu'),
        DeclareLaunchArgument('gnss_ns', default_value='',
                              description='gnss_fix を名前空間付きで起動したときの名前空間（例 front）'),

        # GNSS ドライバ（gnss_fix）の出力を整える：精度が不明なら控えめな値で埋める・frame_id をそろえる
        Node(
            package='trc_localization', executable='gnss_adapter_node', name='gnss_adapter',
            namespace=LaunchConfiguration('gnss_ns'),
            parameters=[common],
            remappings=[('gnss/checked', '/gnss/checked'), ('gnss/quality', '/gnss/quality')],
        ),
        # LiDAR オドメトリ（位置）→ ローバー基準の速度。EKF には速度として入れる
        Node(
            package='trc_localization', executable='odom_to_twist_node', name='lidar_twist',
            parameters=[common],
            remappings=[('odometry', lidar_odom), ('twist', '/lidar/twist')],
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_odom',
            parameters=[config, common, topics],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_map',
            parameters=[config, common, topics],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
        Node(
            package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
            parameters=[config, common],
            remappings=[
                ('imu', '/heading/imu'),
                ('gps/fix', gnss),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        ),
        Node(
            package='trc_localization', executable='heading_init_node', name='heading_init',
            parameters=[config, common],
            remappings=[
                ('gnss/fix', gnss),
                ('odometry', '/odometry/global'),
                ('heading/imu', '/heading/imu'),
            ],
        ),
    ])
