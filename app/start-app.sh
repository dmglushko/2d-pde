#!/usr/bin/env bash
# Starts the app and opens it in the browser. "Start pde2d app.cmd" runs
# this on a double-click in Windows; it also runs directly:
#
#   app/start-app.sh
#
# Builds pde2d_app first (only what changed), starts it unless it already
# runs, waits until it answers and opens the page. The app stops when this
# script stops: Ctrl+C, or closing its window.
# Environment: PDE2D_APP_PORT (default 8765), PDE2D_BUILD_DIR (default
# build), PDE2D_BUILD_JOBS (default 4, at most 4).
set -uo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root" || exit 1
build_dir="${PDE2D_BUILD_DIR:-build}"
jobs="${PDE2D_BUILD_JOBS:-4}"
if ((jobs > 4)); then
  jobs=4
fi
port="${PDE2D_APP_PORT:-8765}"
url="http://127.0.0.1:$port/"

fail() {
  echo
  echo "pde2d app: $*" >&2
  exit 1
}

# True when a pde2d app answers on the port.
answers() {
  curl -fsS -o /dev/null --max-time 1 "${url}api/catalog" 2> /dev/null
}

open_browser() {
  if [[ -n "${WSL_DISTRO_NAME:-}" ]]; then
    # The Windows default browser; cmd.exe needs a Windows directory.
    (cd /mnt/c && cmd.exe /c start "" "$1") > /dev/null 2>&1 && return
  fi
  if command -v xdg-open > /dev/null; then
    xdg-open "$1" > /dev/null 2>&1 &
    return
  fi
  echo "Open $1 in a browser."
}

if answers; then
  echo "The pde2d app already runs at $url; opening it."
  open_browser "$url"
  exit 0
fi

if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
  echo "First start: configuring the build in $build_dir."
  generator=()
  if command -v ninja > /dev/null; then
    generator=(-G Ninja)
  fi
  cmake -S . -B "$build_dir" "${generator[@]}" ||
    fail "configuring failed. Configure once by hand, for example
  tools/test.sh -DSymEngine_DIR=\$HOME/.local/opt/symengine-0.14/lib/cmake/symengine
and start again."
fi
echo "Building the app (only what changed)..."
cmake --build "$build_dir" --target pde2d_app -j "$jobs" ||
  fail "the build failed; see the messages above."

"$build_dir/app/pde2d_app" --port "$port" &
server=$!
stop() {
  kill "$server" 2> /dev/null
  wait "$server" 2> /dev/null
}
trap stop EXIT
trap 'exit 0' INT TERM HUP

for _ in $(seq 100); do
  answers && break
  kill -0 "$server" 2> /dev/null ||
    fail "the app did not start. If port $port is taken by another program,
set PDE2D_APP_PORT to a free port."
  sleep 0.2
done
answers || fail "the app did not answer on $url."

open_browser "$url"
echo
echo "The pde2d app runs at $url"
echo "Close this window (or press Ctrl+C) to stop it."
wait "$server"
