#!/usr/bin/env bash
# シミュレーターのシナリオ試験を順に走らせる（trc-dev で実行）。引数は run_scenarios.py にそのまま渡す。
#
#   ./scripts/run_scenarios.sh                    # 全部（20〜30 分）
#   ./scripts/run_scenarios.sh --only gap,narrow
#   ./scripts/run_scenarios.sh --list

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="${DEPS_DIR:-$HOME/trc2027/deps_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP" || { echo "ROS 2 が見つかりません（$ROS_SETUP）" >&2; exit 1; }
fi
# shellcheck disable=SC1091
source "$DEPS_DIR/install/setup.bash" || { echo "KISS-ICP がビルドされていません（$DEPS_DIR）" >&2; exit 1; }
# shellcheck disable=SC1091
source "$REPO_DIR/install/setup.bash" || { echo "先に colcon build してください" >&2; exit 1; }

exec python3 "$REPO_DIR/scripts/run_scenarios.py" "$@"
