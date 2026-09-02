#!/usr/bin/env bash
# Starts the robot simulator and the AI cockpit, and says plainly what went wrong
# if either refuses. One command, because typing on a phone is no fun.
#
#   bash scripts/start.sh
#
# Ctrl-C stops both.

set -u
cd "$(dirname "$0")/.." || exit 1

SIM_LOG=/tmp/sim.log
NT_PORT=5810
WEB_PORT="${WEB_PORT:-8000}"
WAIT_SECONDS=150

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }

nt_is_up() {
  python3 - "$NT_PORT" <<'PY'
import socket, sys
sys.exit(0 if socket.socket().connect_ex(("127.0.0.1", int(sys.argv[1]))) == 0 else 1)
PY
}

cleanup() {
  say "stopping the simulator"
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
  wait "${SIM_PID:-}" 2>/dev/null
}
trap cleanup EXIT INT TERM

if nt_is_up; then
  say "a simulator is already running - reusing it"
else
  say "installing the agent's dependencies"
  pip install --quiet --disable-pip-version-check -r scripts/gemini_agent/requirements.txt \
    || { echo "pip install failed - see above"; exit 1; }

  say "building and starting the robot simulator (a couple of minutes the first time)"
  chmod +x gradlew
  ./gradlew simulateJava > "$SIM_LOG" 2>&1 &
  SIM_PID=$!

  for _ in $(seq 1 "$WAIT_SECONDS"); do
    nt_is_up && break
    kill -0 "$SIM_PID" 2>/dev/null || break
    sleep 1
  done

  if ! nt_is_up; then
    say "the robot did not come up. The end of $SIM_LOG:"
    tail -30 "$SIM_LOG"
    if grep -qE "UnsatisfiedLink|NoClassDefFound|GLIBC|TimeSyncServer" "$SIM_LOG"; then
      say "That looks like a native library that will not load on this container."
      echo "In a Codespace:  git pull && gh codespace rebuild"
      echo "The devcontainer pins Ubuntu 24.04, which is what photonlib's natives need."
    fi
    exit 1
  fi
  say "the robot is up on NetworkTables port $NT_PORT"
fi

if [ -z "${GEMINI_API_KEY:-}" ] && [ -z "${GOOGLE_API_KEY:-}" ]; then
  say "GEMINI_API_KEY is not set - the cockpit will show the robot but cannot drive it"
  echo "Add it as a Codespaces secret, or: export GEMINI_API_KEY=..."
fi

say "starting the cockpit on port $WEB_PORT - open it from the Ports tab"
exec python3 scripts/gemini_agent/serve.py --web-port "$WEB_PORT"
