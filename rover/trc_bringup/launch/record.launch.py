"""センサのデータを rosbag に記録する（計測治具を押して歩くときなど）。

  ros2 launch trc_bringup record.launch.py                 # ~/trc2027/bags/日時 に保存
  ros2 launch trc_bringup record.launch.py name:=sand_01   # 名前を付ける

記録するのは生のセンサデータだけ（位置推定の結果は、再生して何度でも作り直せる）。
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

TOPICS = [
    '/unilidar/cloud',
    '/unilidar/imu',
    '/gnss',                  # gnss_fix：RTK Fix 以外
    '/gnss/fix',              # gnss_fix：RTK Fix
    '/gnss/clas',             # gnss_fix：CLAS で RTK Fix
    '/gnss/best',             # gnss_fix：有効な測位すべて
    '/gnss/velocity',
    '/gnss/time_reference',
    '/diagnostics',           # gnss_fix の受信機の状態
    '/tf_static',
]


def make_record(context):
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context))
    name = LaunchConfiguration('name').perform(context) or datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    return [ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '-o', path] + TOPICS,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('out_dir', default_value='~/trc2027/bags'),
        DeclareLaunchArgument('name', default_value=''),
        OpaqueFunction(function=make_record),
    ])
