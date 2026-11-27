#!/usr/bin/env python3
"""シミュレーター用の対応表（markers.csv）とミッションファイル（mission.yaml）を作る。

  python3 scripts/make_sim_mission.py ~/trc2027/missions/sim         # 既定（東から 90° の向きでスタート）
  python3 scripts/make_sim_mission.py /tmp/m --heading-deg 200
  ./scripts/sim_view.sh layout:=checkpoints mission:=$HOME/trc2027/missions/sim/mission.yaml

マーカーの位置は仮想フィールドの配置 checkpoints（fake_world.CHECKPOINT_MARKERS）と同じ。
緯度・経度は、にせの GNSS と同じ基準点（lat0, lon0）とスタートの向きから計算する。
本番では、運営から配られる対応表（csv）を markers_csv に指定し、route を書き換えて使う。
"""

import argparse
import math
import os
import sys

DEFAULT_LAT0, DEFAULT_LON0 = 35.5400, 134.2300      # fake_sensors_node の既定


def write_sim_mission(out_dir: str, heading_deg: float = 90.0, lat0: float = DEFAULT_LAT0,
                      lon0: float = DEFAULT_LON0) -> str:
    from trc_localization import fake_world as fw
    os.makedirs(out_dir, exist_ok=True)
    h0 = math.radians(heading_deg)
    csv_path = os.path.join(out_dir, 'markers.csv')
    with open(csv_path, 'w', encoding='utf-8') as f:
        f.write('id, 緯度, 経度\n')
        for mid, (x, y) in sorted(fw.CHECKPOINT_MARKERS.items()):
            lat, lon = fw.map_to_latlon(x, y, lat0, lon0, h0)
            f.write(f'{mid}, {lat:.8f}, {lon:.8f}\n')
    lines = ['# シミュレーター用（scripts/make_sim_mission.py で作成）',
             f'# スタートの向き：東から {heading_deg:.0f}°、基準点 {lat0}, {lon0}',
             'markers_csv: markers.csv', 'route:']
    for step in fw.CHECKPOINT_ROUTE:
        if 'checkpoint' in step:
            a, b = step['checkpoint']
            lines.append(f'  - checkpoint: [{a}, {b}]')
        else:
            lat, lon = fw.map_to_latlon(*step['latlon_map'], lat0, lon0, h0)
            lines.append(f'  - latlon: [{lat:.8f}, {lon:.8f}]')
        lines.append(f'    name: {step["name"]}')
    yaml_path = os.path.join(out_dir, 'mission.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return yaml_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--heading-deg', type=float, default=90.0, help='sim_test の heading_deg と同じ値にする')
    ap.add_argument('--lat0', type=float, default=DEFAULT_LAT0)
    ap.add_argument('--lon0', type=float, default=DEFAULT_LON0)
    a = ap.parse_args()
    path = write_sim_mission(os.path.expanduser(a.out_dir), a.heading_deg, a.lat0, a.lon0)
    print(f'作りました: {path}')
    print(open(path, encoding='utf-8').read())
    return 0


if __name__ == '__main__':
    sys.exit(main())
