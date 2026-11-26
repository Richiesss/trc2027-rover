#!/usr/bin/env bash
# Mac で実行する：trc-dev の foxglove_bridge（ポート 8765）を手元に転送し、Foxglove を開く。
#
#   ./scripts/foxglove_tunnel.sh            # 接続先は trc-dev（~/.ssh/config の Host 名）
#   ./scripts/foxglove_tunnel.sh other-host
#
# 先に trc-dev で ./scripts/sim_view.sh を起動しておく。止めるときは Ctrl+C。

HOST="${1:-trc-dev}"
PORT="${PORT:-8765}"
URL="https://app.foxglove.dev"

if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "[注意] ポート $PORT はすでに使われています（前のトンネルが残っているかも）:"
  lsof -nP -iTCP:"$PORT" -sTCP:LISTEN
  echo "止めるには: kill \$(lsof -t -iTCP:$PORT -sTCP:LISTEN)"
  exit 1
fi

echo "==> $HOST:$PORT を localhost:$PORT に転送します（Ctrl+C で終了）"
echo "    Foxglove → Open connection → Foxglove WebSocket → ws://localhost:$PORT"
(sleep 2 && command -v open >/dev/null && open "$URL") &
exec ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L "$PORT:localhost:$PORT" "$HOST"
