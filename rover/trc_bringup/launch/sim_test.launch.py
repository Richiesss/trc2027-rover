"""実機なしで「点群 → LiDAR オドメトリ → GNSS との融合」まで一通り試す。

  ros2 launch trc_bringup sim_test.launch.py
  ros2 launch trc_bringup sim_test.launch.py heading_deg:=200.0 log_history:=true
  ros2 launch trc_bringup sim_test.launch.py gnss_outage_start:=25.0   # 25 秒後から 10 秒間 GNSS を切る
  ros2 launch trc_bringup sim_test.launch.py obstacle_ahead:=6.0       # 6 m 先に岩を置く（よけて 18 m 先へ。途中の長い壁も回り込む）
  ros2 launch trc_bringup sim_test.launch.py waypoints:="15,0; 15,10; 0,10; 0,0"   # 四角く回る
  ros2 launch trc_bringup sim_test.launch.py layout:=checkpoints mission:=<ミッションファイル>  # 緯度・経度のチェックポイント
    （ミッションファイルの例は scripts/make_sim_mission.py で作れる）
  ros2 launch trc_bringup sim_test.launch.py terrain:=dunes slip:=0.15 dust:=1.0   # 砂地（砂丘・空転・舞う砂）
  ros2 launch trc_bringup sim_test.launch.py driver:=script obstacle_ahead:=6.0      # 決まった道筋（手前で止まるだけ）

起動するもの:
  - local_planner     : 目標（waypoints）へ向かって、障害物をよけながら進む速度指令を出す（driver:=planner、既定）
    scripted_driver   : 決まった道筋の速度指令を出す（直進 12 秒 → 左に曲がり続ける）（driver:=script）
  - safety_stop       : 前方に障害物があれば前進を止める（safety:=false で通り抜ける）
  - fake_sensors_node : 速度指令どおりに動き、仮想フィールドの点群（/unilidar/cloud）・IMU・GNSS を出す
  - KISS-ICP          : 点群から LiDAR オドメトリ（/kiss/odometry）を作る
  - 位置推定一式       : 取付位置の TF + 2段の EKF + 向きの初期化（+ 制御履歴）
  - 障害物検出         : 点群から地面を除いて障害物を見つける（obstacles:=false で止める）

結果の見方:
  /gps/filtered（推定位置）と /fake/truth（本当の位置）を比べる

画面で見る（foxglove:=true、既定）:
  foxglove_bridge（WebSocket、ポート 8765）を起動する。Mac のブラウザで Foxglove を開いてつなぐ
  （手順は docs/setup-native.md の「シミュレーションを画面で見る」）
  表示用に次のトピックも出す:
    /fake/world          仮想フィールド全体（map 座標）
    /fake/truth_path     本当の道筋
    /localization/path   推定した道筋（/odometry/global から）
    /gnss/path           GNSS の測定点（/odometry/gps から）
    /fake/position_error 推定位置の誤差 [m]
    /obstacles/points    障害物とみなした点（base_link 座標）
    /obstacles/ground    地面とみなした点
    /obstacles/corridor  前方の帯（緑：空いている／赤：止まるべき）
    /obstacles/front_distance  前方の障害物までの距離 [m]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 安全停止を使う = 障害物検出も安全停止も true
    use_safety = PythonExpression(["'", LaunchConfiguration('obstacles'), "'.lower() == 'true' and '",
                                   LaunchConfiguration('safety'), "'.lower() == 'true'"])
    cmd_topic = PythonExpression(["'/cmd_vel_nav' if (", use_safety, ") else '/cmd_vel'"])
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
        DeclareLaunchArgument('obstacles', default_value='true',
                              description='true で障害物検出を起動する'),
        DeclareLaunchArgument('obstacle_ahead', default_value='-1.0',
                              description='通り道のこの距離 [m] 先に岩を置く（障害物検出の試験用、-1 で置かない）'),
        DeclareLaunchArgument('driver', default_value='planner',
                              description='planner：目標へ向かって障害物をよける／script：決まった道筋（直進→左に曲がり続ける）'),
        DeclareLaunchArgument('waypoints', default_value='18,0',
                              description='driver:=planner の目標（map 座標、スタート地点が原点・x が前）。例 "15,0; 15,8"'),
        # --- 試験用のトラブル（scripts/run_scenarios.py が使う）---
        DeclareLaunchArgument('layout', default_value='default',
                              description='障害物の配置: default / gap / narrow / dead_end / clutter（目標は 14,0 を想定）'),
        DeclareLaunchArgument('gnss_events', default_value='',
                              description='GNSS の劣化 "開始秒:続く秒:種類; ..."（種類 outage / float / single / nocov＝精度不明の単独測位）'),
        DeclareLaunchArgument('gnss_jump', default_value='',
                              description='GNSS が RTK Fix のふりをして 1 回だけ跳ぶ "秒:距離m"'),
        DeclareLaunchArgument('lidar_drop', default_value='',
                              description='LiDAR の点群が止まる "開始秒:続く秒; ..."'),
        DeclareLaunchArgument('imu_bias', default_value='0.0',
                              description='IMU の回転の速さのずれ [rad/s]'),
        # --- 砂地の試験（fake_terrain.py）---
        DeclareLaunchArgument('terrain', default_value='flat',
                              description='地形: flat（ゆるい起伏・車体は水平）/ dunes（登れる砂丘）/ hill（コースの途中に大きな砂丘）/ steep（登れない砂丘）/ pit（くぼみ）'),
        DeclareLaunchArgument('slip', default_value='0.0',
                              description='平らな所でも起きるタイヤの空転の割合（砂地なら 0.1〜0.2）'),
        DeclareLaunchArgument('soft_sand', default_value='',
                              description='やわらかい砂（はまると前に進めない）"x,y,半径; ..."（map 座標）'),
        DeclareLaunchArgument('dust', default_value='0.0',
                              description='舞い上がる砂：1 回のスキャンあたりの砂の塊の数の平均'),
        DeclareLaunchArgument('ground_mode', default_value='fixed',
                              description='地面の点: fixed（world に固定）/ resample（毎回選び直す＝模様のない砂地で LiDAR オドメトリの手がかりにならない）'),
        DeclareLaunchArgument('world', default_value='default',
                              description='周りの物体: default（支柱・岩・壁）/ bare（何もない砂地）'),
        DeclareLaunchArgument('mission', default_value='',
                              description='自律ミッションのファイル（緯度・経度のチェックポイント）。指定すると waypoints は使わない'),
        DeclareLaunchArgument('safety', default_value='true',
                              description='true で、前方に障害物があるときに前進を止める（obstacles:=true のときだけ）'),

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
        # scoped：読み込み先の引数（safety など）が、こちらの同じ名前の設定を上書きしないように
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration('obstacles')), actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('trc_bringup'), 'launch', 'perception.launch.py']),
                launch_arguments={'safety': use_safety}.items(),
            ),
        ]),
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
            # 運転手：planner（目標へ向かって障害物をよける）か script（決まった道筋）。
            # 安全停止を使うときは /cmd_vel_nav に出し、safety_stop を通して /cmd_vel へ
            Node(
                package='trc_navigation', executable='local_planner_node', name='local_planner',
                # mission を指定したときは、ウェイポイントは mission_node から受け取る
                parameters=[{'waypoints': ParameterValue(PythonExpression(
                    ["'' if '", LaunchConfiguration('mission'), "' else '", LaunchConfiguration('waypoints'), "'"]),
                    value_type=str)}],
                remappings=[('cmd_vel', cmd_topic)],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == 'planner'"])),
            ),
            Node(
                package='trc_navigation', executable='mission_node', name='mission',
                parameters=[{'mission_file': LaunchConfiguration('mission')}],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mission'), "' != ''"])),
            ),
            Node(
                package='trc_localization', executable='scripted_driver_node', name='scripted_driver',
                remappings=[('cmd_vel', cmd_topic)],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == 'script'"])),
            ),
            Node(
                package='trc_localization', executable='fake_sensors_node', name='fake_sensors',
                parameters=[{
                    'publish_cloud': True,
                    'drive_mode': 'cmd_vel',
                    'publish_odom': False,
                    'heading_deg': LaunchConfiguration('heading_deg'),
                    'gnss_outage_start': LaunchConfiguration('gnss_outage_start'),
                    'gnss_outage_duration': LaunchConfiguration('gnss_outage_duration'),
                    'obstacle_ahead': LaunchConfiguration('obstacle_ahead'),
                    # 文字列のまま渡す（"20:5" が YAML の 60 進数として数字に化けないように）
                    'layout': ParameterValue(LaunchConfiguration('layout'), value_type=str),
                    'gnss_events': ParameterValue(LaunchConfiguration('gnss_events'), value_type=str),
                    'gnss_jump': ParameterValue(LaunchConfiguration('gnss_jump'), value_type=str),
                    'lidar_drop': ParameterValue(LaunchConfiguration('lidar_drop'), value_type=str),
                    'imu_bias': ParameterValue(LaunchConfiguration('imu_bias'), value_type=float),
                    'terrain': ParameterValue(LaunchConfiguration('terrain'), value_type=str),
                    'slip': ParameterValue(LaunchConfiguration('slip'), value_type=float),
                    'soft_sand': ParameterValue(LaunchConfiguration('soft_sand'), value_type=str),
                    'dust': ParameterValue(LaunchConfiguration('dust'), value_type=float),
                    'world': ParameterValue(LaunchConfiguration('world'), value_type=str),
                    'ground_mode': ParameterValue(LaunchConfiguration('ground_mode'), value_type=str),
                }],
            ),
        ]),
    ])
