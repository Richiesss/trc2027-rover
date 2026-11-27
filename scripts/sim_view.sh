#!/usr/bin/env bash
# シミュレーション（にせのセンサ + 位置推定 + foxglove_bridge）を起動する。trc-dev で実行する。
#
#   ./scripts/sim_view.sh                          # 普通に起動
#   ./scripts/sim_view.sh gnss_outage_start:=25.0  # launch の引数はそのまま渡せる
#   ./scripts/sim_view.sh -y                       # 前回の残りを確認なしで止める
#   ./scripts/sim_view.sh --stop                   # 残っているノードを止めるだけ
#
# 起動前に、前回のノードが残っていないかを確かめる（残っていると古い TF が混ざって推定が壊れる）。
# ログは ~/trc2027/logs/sim_<日時>.log に保存する。

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_DIR="${DEPS_DIR:-$HOME/trc2027/deps_ws}"
LOG_DIR="${LOG_DIR:-$HOME/trc2027/logs}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
PORT=8765

# このリポジトリで起動するノード（前回の残りを探すのに使う）
NODE_PATTERN='ekf_node|navsat_transform_node|kiss_icp_node|robot_state_publisher|foxglove_bridge|trc_(localization|logging|gnss|perception|navigation)/lib|ros2 launch trc_bringup'

YES=0
STOP_ONLY=0
LAUNCH_ARGS=()
for a in "$@"; do
  case "$a" in
    -y|--yes) YES=1 ;;
    --stop)   STOP_ONLY=1 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) LAUNCH_ARGS+=("$a") ;;
  esac
done

info() { printf '\033[1;32m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[注意]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[エラー]\033[0m %s\n' "$*" >&2; exit 1; }

# 該当するプロセスの PID（シェルや tee など、コマンド行に文字列を含むだけのものは除く）
find_leftovers() {
  local pid comm
  for pid in $(pgrep -f -- "$NODE_PATTERN"); do
    [ "$pid" = "$$" ] && continue
    comm=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    case "$(ps -o stat= -p "$pid" 2>/dev/null)" in Z*) continue ;; esac   # 終了済み（ゾンビ）
    case "$comm" in bash|sh|zsh|dash|tee|grep|less|tail|vi|vim|nano|code*) continue ;; esac
    echo "$pid"
  done
}

stop_leftovers() {
  local pids
  pids=$(find_leftovers)
  if [ -z "$pids" ]; then
    info "残っているノードはありません"
    return 0
  fi
  warn "前回のノードが残っています:"
  # shellcheck disable=SC2086
  ps -o pid=,lstart=,args= -p $(echo $pids | tr ' ' ',') | cut -c1-150
  if [ "$YES" -ne 1 ]; then
    read -r -p "止めますか？ [Y/n] " ans
    case "$ans" in n|N) die "残ったままでは推定が壊れるので中止します" ;; esac
  fi
  # shellcheck disable=SC2086
  kill -INT $pids 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    sleep 1
    [ -z "$(find_leftovers)" ] && break
  done
  pids=$(find_leftovers)
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1 || true
  info "止めました"
}

# --- ROS の環境 ---
if ! command -v ros2 >/dev/null 2>&1; then
  [ -f "$ROS_SETUP" ] || die "ROS 2 が見つかりません（$ROS_SETUP）"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

stop_leftovers
[ "$STOP_ONLY" -eq 1 ] && exit 0

if [ -f "$DEPS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$DEPS_DIR/install/setup.bash"
else
  die "KISS-ICP がビルドされていません（$DEPS_DIR）。docs/setup-native.md の「4. 実機のセンサを動かす」を参照"
fi

if [ ! -f "$REPO_DIR/install/setup.bash" ]; then
  info "初回のビルドをします"
  (cd "$REPO_DIR" && colcon build --symlink-install) || die "ビルドに失敗しました"
fi
# shellcheck disable=SC1091
source "$REPO_DIR/install/setup.bash"

ros2 pkg prefix kiss_icp >/dev/null 2>&1 || die "kiss_icp パッケージが見つかりません"
ros2 pkg prefix trc_bringup >/dev/null 2>&1 \
  || die "trc_bringup が見つかりません。cd $REPO_DIR && colcon build --symlink-install"

USE_FOXGLOVE=1
for a in "${LAUNCH_ARGS[@]}"; do [ "$a" = "foxglove:=false" ] && USE_FOXGLOVE=0; done
if [ "$USE_FOXGLOVE" -eq 1 ] && ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  die "foxglove_bridge がありません: sudo apt install ros-jazzy-foxglove-bridge"
fi

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/sim_$(date +%Y%m%d_%H%M%S).log"

info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ログ: $LOG_FILE"
if [ "$USE_FOXGLOVE" -eq 1 ]; then
  cat <<EOF

  画面で見るには、Mac で:
    ./scripts/foxglove_tunnel.sh        （または ssh -N -L $PORT:localhost:$PORT trc-dev）
  → https://app.foxglove.dev で ws://localhost:$PORT に接続
  止めるときは Ctrl+C

EOF
fi

ros2 launch trc_bringup sim_test.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee "$LOG_FILE"
