#!/usr/bin/env python3
"""シミュレーターで色々な状況（シナリオ）を順に走らせ、結果を表にまとめる。

  ./scripts/run_scenarios.sh                    # 全部（20〜30 分）
  ./scripts/run_scenarios.sh --only gap,narrow  # 一部だけ
  ./scripts/run_scenarios.sh --list             # シナリオの一覧

シナリオごとに sim_test.launch.py を起動して（foxglove なし）、次を測る:
  到着     : すべてのウェイポイントに着いたか、かかった秒数
  最接近   : 本当の通り道と障害物のいちばん近い距離（ローバーの半径 0.45 m より大きければぶつかっていない）
  位置誤差 : 推定位置と本当の位置の差（平均・最大）
  向き誤差 : 推定の向き（/odometry/global）と本当の向きの差の最大
  安全停止 : 安全停止がかかった回数
結果は ~/trc2027/logs/scenarios_<日時>.md と .csv に保存する。
"""

import argparse
import bisect
import csv
import datetime
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROBOT_RADIUS = 0.45


@dataclass
class Scenario:
    name: str
    desc: str
    args: dict = field(default_factory=dict)
    waypoints: str = '18,0'
    timeout: float = 150.0
    max_pos_err: float = 0.5        # これを超えたら不合格 [m]
    expect_reach: bool = True


SCENARIOS = [
    Scenario('baseline', '基準：RTK Fix のまま 18 m 先へ（途中に長い壁）'),
    Scenario('heading_200', 'スタートの向きが南西（東から 200°）', {'heading_deg': '200.0'}),
    Scenario('gnss_outage_30', 'GNSS が 15 秒後から 30 秒途切れる', {'gnss_events': '15:30:outage'},
             max_pos_err=1.0),
    Scenario('gnss_float', 'GNSS が 10 秒後から 40 秒 RTK Float（誤差 30 cm）', {'gnss_events': '10:40:float'},
             max_pos_err=1.0),
    Scenario('gnss_single', 'GNSS が 10 秒後から 30 秒 単独測位（誤差 2.5 m）', {'gnss_events': '10:30:single'},
             max_pos_err=3.0),
    Scenario('gnss_jump', 'GNSS が 25 秒後に 1 回だけ 5 m 跳ぶ（RTK Fix のふり）', {'gnss_jump': '25:5'},
             max_pos_err=1.0),
    Scenario('lidar_drop', 'LiDAR が 20 秒後から 3 秒止まる', {'lidar_drop': '20:3'}),
    Scenario('imu_bias', 'IMU の回転の速さが 0.01 rad/s ずれている', {'imu_bias': '0.01'}),
    Scenario('gap', '幅 2.0 m のすき間を抜けて 14 m 先へ', {'layout': 'gap'}, waypoints='14,0'),
    Scenario('narrow', 'すき間が 1.0 m（通れない）。壁の端を回って 14 m 先へ', {'layout': 'narrow'},
             waypoints='14,0', timeout=200.0),
    Scenario('dead_end', 'まっすぐ行くと袋小路（U 字）。回り込んで 14 m 先へ', {'layout': 'dead_end'},
             waypoints='14,0', timeout=200.0),
    Scenario('clutter', '岩が 11 個ばらまかれた中を抜けて 14 m 先へ', {'layout': 'clutter'}, waypoints='14,0'),
    Scenario('square', '4 地点を回る（約 60 m）', waypoints='18,-10;18,0;0,-10;0,0', timeout=320.0),
]

NODE_PATTERN = ('ekf_node|navsat_transform_node|kiss_icp_node|robot_state_publisher|foxglove_bridge|'
                'trc_(localization|logging|gnss|perception|navigation)/lib|ros2 launch trc_bringup')


def stop_leftovers():
    subprocess.run(['pkill', '-INT', '-f', NODE_PATTERN], check=False)
    time.sleep(2.0)
    subprocess.run(['pkill', '-KILL', '-f', NODE_PATTERN], check=False)
    time.sleep(0.5)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_of(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Monitor:
    """シナリオの間、トピックを見て数字を集める。"""

    def __init__(self, node):
        from nav_msgs.msg import Odometry, Path as PathMsg
        from rcl_interfaces.msg import Log
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool, Float64
        self.node = node
        self.pos_err = []
        self.yaw_err = []
        self.truth = []                 # (x, y)
        self.truth_seen = 0
        self.truth_pending = []         # まだ推定の向きが届いていない本当の姿勢 (時刻, 向き)
        self.world = None
        self.est = []                   # (t, yaw)
        self.safety_on = False
        self.safety_count = 0
        self.done_time = None
        self.skipped = 0
        self.stuck = 0
        self.subs = [
            node.create_subscription(Float64, '/fake/position_error', lambda m: self.pos_err.append(m.data), 50),
            node.create_subscription(PathMsg, '/fake/truth_path', self.on_truth, 5),
            node.create_subscription(Odometry, '/odometry/global', self.on_est, 50),
            node.create_subscription(Bool, '/safety/active', self.on_safety, 20),
            node.create_subscription(Log, '/rosout', self.on_log, 100),
            node.create_subscription(PointCloud2, '/fake/world', self.on_world,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        self.start = time.monotonic()

    def destroy(self):
        for s in self.subs:
            self.node.destroy_subscription(s)

    def on_world(self, msg):
        from sensor_msgs_py import point_cloud2 as pc2
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        self.world = pts[pts[:, 2] > 0.15][:, :2].astype(float)

    def on_est(self, msg):
        t = stamp_of(msg.header)
        if not self.est or t > self.est[-1][0]:
            self.est.append((t, yaw_of(msg.pose.pose.orientation)))
            if len(self.est) > 3000:
                del self.est[:1000]
            if self.truth_pending:
                self.compare_yaw()

    def est_yaw_at(self, t):
        ts = [e[0] for e in self.est]
        i = bisect.bisect_left(ts, t)
        if i == 0 or i >= len(ts):
            return None
        (t0, a0), (t1, a1) = self.est[i - 1], self.est[i]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return a0 + w * math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))

    def on_truth(self, msg):
        poses = msg.poses
        self.truth = [(p.pose.position.x, p.pose.position.y) for p in poses]
        new = poses[self.truth_seen:] if len(poses) > self.truth_seen else poses[-1:]
        self.truth_seen = len(poses)
        self.truth_pending += [(stamp_of(p.header), yaw_of(p.pose.orientation)) for p in new]
        self.compare_yaw()

    def compare_yaw(self):
        """推定の向きが届いた分から、本当の向きと比べる（推定は処理の分だけ遅れて届く）。"""
        if not self.est:
            return
        latest = self.est[-1][0]
        rest = []
        for t, truth in self.truth_pending:
            if t > latest:
                rest.append((t, truth))                 # まだ推定が追いついていない
                continue
            est = self.est_yaw_at(t)
            if est is not None:
                d = est - truth
                self.yaw_err.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
        self.truth_pending = rest[-50:]

    def on_safety(self, msg):
        if msg.data and not self.safety_on:
            self.safety_count += 1
        self.safety_on = msg.data

    def on_log(self, msg):
        if msg.name != 'local_planner':
            return
        if 'すべてのウェイポイントに着きました' in msg.msg and self.done_time is None:
            self.done_time = time.monotonic() - self.start
        elif '次へ進みます' in msg.msg:
            self.skipped += 1
        elif 'その場で回って' in msg.msg:
            self.stuck += 1

    def clearance(self):
        if self.world is None or not self.truth:
            return float('nan')
        t = np.array(self.truth)
        w = self.world
        best = float('inf')
        for k in range(0, len(t), 200):                 # メモリを使いすぎないよう分けて計算
            seg = t[k:k + 200]
            d = np.hypot(seg[:, None, 0] - w[None, :, 0], seg[:, None, 1] - w[None, :, 1])
            best = min(best, float(d.min()))
        return best


def run_one(node, sc: Scenario, log_dir: Path, timeout_scale: float):
    import rclpy
    stop_leftovers()
    args = ['ros2', 'launch', 'trc_bringup', 'sim_test.launch.py', 'foxglove:=false',
            f'waypoints:={sc.waypoints}'] + [f'{k}:={v}' for k, v in sc.args.items()]
    log_path = log_dir / f'scenario_{sc.name}.log'
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        mon = Monitor(node)
        limit = sc.timeout * timeout_scale
        try:
            while time.monotonic() - mon.start < limit and mon.done_time is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if proc.poll() is not None:
                    break
            # 終わったあと少しだけ待って、最後の道筋も受け取る
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            mon.destroy()

    clear = mon.clearance()
    pe = np.array(mon.pos_err) if mon.pos_err else np.array([float('nan')])
    ye = max(mon.yaw_err) if mon.yaw_err else float('nan')
    reached = mon.done_time is not None and mon.skipped == 0
    problems = []
    if sc.expect_reach and not reached:
        problems.append('着かない' if mon.done_time is None else f'ウェイポイントを {mon.skipped} 個飛ばした')
    if not math.isnan(clear) and clear < ROBOT_RADIUS:
        problems.append(f'接触（{clear:.2f} m）')
    if np.nanmax(pe) > sc.max_pos_err:
        problems.append(f'位置誤差 {np.nanmax(pe):.2f} m > {sc.max_pos_err} m')
    return {
        'name': sc.name, 'desc': sc.desc,
        'result': '合格' if not problems else '不合格',
        'problems': '・'.join(problems),
        'time_s': round(mon.done_time, 1) if mon.done_time else '',
        'clearance_m': round(clear, 2),
        'pos_err_mean_m': round(float(np.nanmean(pe)), 3),
        'pos_err_max_m': round(float(np.nanmax(pe)), 3),
        'yaw_err_max_deg': round(ye, 1),
        'safety_stops': mon.safety_count,
        'stuck': mon.stuck,
        'log': str(log_path),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--only', default='', help='カンマ区切りのシナリオ名')
    ap.add_argument('--list', action='store_true', help='シナリオの一覧を出して終わる')
    ap.add_argument('--timeout-scale', type=float, default=1.0, help='時間切れまでの秒数の倍率（遅い機体用）')
    ap.add_argument('--log-dir', default=os.path.expanduser('~/trc2027/logs'))
    a = ap.parse_args()

    if a.list:
        for sc in SCENARIOS:
            print(f'{sc.name:16s} {sc.desc}')
        return 0
    chosen = SCENARIOS
    if a.only:
        names = [n.strip() for n in a.only.split(',') if n.strip()]
        unknown = set(names) - {s.name for s in SCENARIOS}
        if unknown:
            print(f'知らないシナリオ: {", ".join(sorted(unknown))}（--list で一覧）', file=sys.stderr)
            return 2
        chosen = [s for s in SCENARIOS if s.name in names]

    import rclpy
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = Path(a.log_dir) / f'scenarios_{stamp}'
    log_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('scenario_runner')
    rows = []
    try:
        for i, sc in enumerate(chosen, 1):
            print(f'[{i}/{len(chosen)}] {sc.name}: {sc.desc} ...', flush=True)
            r = run_one(node, sc, log_dir, a.timeout_scale)
            rows.append(r)
            print(f'    → {r["result"]}  到着 {r["time_s"] or "-"} s  最接近 {r["clearance_m"]} m  '
                  f'位置誤差 平均 {r["pos_err_mean_m"]} / 最大 {r["pos_err_max_m"]} m  '
                  f'向き誤差 最大 {r["yaw_err_max_deg"]}°  安全停止 {r["safety_stops"]} 回'
                  + (f'  （{r["problems"]}）' if r['problems'] else ''), flush=True)
    except KeyboardInterrupt:
        print('中断しました', file=sys.stderr)
    finally:
        stop_leftovers()
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        return 1
    csv_path = log_dir / 'results.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    md = ['| シナリオ | 内容 | 結果 | 到着 [s] | 最接近 [m] | 位置誤差 平均/最大 [m] | 向き誤差 最大 [°] | 安全停止 | 問題 |',
          '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in rows:
        md.append(f'| {r["name"]} | {r["desc"]} | {r["result"]} | {r["time_s"] or "-"} | {r["clearance_m"]} | '
                  f'{r["pos_err_mean_m"]} / {r["pos_err_max_m"]} | {r["yaw_err_max_deg"]} | {r["safety_stops"]} | '
                  f'{r["problems"]} |')
    md_path = log_dir / 'results.md'
    md_path.write_text('\n'.join(md) + '\n')
    passed = sum(r['result'] == '合格' for r in rows)
    print(f'\n合格 {passed} / {len(rows)}　結果: {md_path}')
    print('\n'.join(md))
    return 0 if passed == len(rows) else 1


if __name__ == '__main__':
    sys.exit(main())
