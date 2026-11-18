#!/usr/bin/env bash
# The page in a real browser: starts pde2d_app on a free port with the page
# and this harness, runs each scenario in headless Chrome and fails when a
# check fails, the page throws, or a scenario does not finish.
#
#   app/tests/browser/run.sh [--app PATH] [--shots DIR] [SCENARIO...]
#
# Chrome: $PDE2D_CHROME, else google-chrome or chromium on PATH, else the
# Windows Chrome (from WSL) with a profile in the Windows temp directory.
# --shots DIR also saves a screenshot of each scenario's end state.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../../.." && pwd)"
app="$root/build/app/pde2d_app"
shots=""
scenarios=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --app) app="$2"; shift 2 ;;
    --shots) shots="$2"; shift 2 ;;
    *) scenarios+=("$1"); shift ;;
  esac
done

# name width height [tab]: the scenarios and the window each runs in.
all=(
  "run_all 1440 900 solve"
  "invalid_option 1280 800 mesh"
  "switch_case 1280 800 geometry"
  "preset 1440 900 solve"
  "phone 390 844 solve"
  "time_bar 1280 800 solve"
  "steps 1280 800 assemble"
  "invalid_entry 1280 800 mesh"
  "race_example 1280 800 problem"
  "undo_redo 1280 800 problem"
  "two_tabs 1440 900 solve"
  "lock_range 1440 900 solve"
  "reference 1440 900 solve"
  "display_state 1440 900 solve"
  "domain_file 1280 800 geometry"
  "failures 1440 900 solve"
  "reload 1440 900 solve"
  "unsaved 1280 800 geometry"
  "region_click 1280 800 problem"
  "study 1440 900 solve"
  "case_menu 1280 800 geometry"
  "phone_menu 390 844 solve"
  "geometry_editor 1280 800 geometry"
  "drawing_incomplete 1280 800 geometry"
  "adaptive 1440 900 solve"
  "equation_line 1280 800 problem"
)

chrome="${PDE2D_CHROME:-}"
if [[ -z "$chrome" ]]; then
  for name in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$name" >/dev/null; then chrome="$(command -v "$name")"; break; fi
  done
fi
windows_chrome="/mnt/c/Program Files/Google/Chrome/Application/chrome.exe"
if [[ -z "$chrome" && -x "$windows_chrome" ]]; then chrome="$windows_chrome"; fi
if [[ -z "$chrome" ]]; then
  echo "no Chrome found: set PDE2D_CHROME" >&2
  exit 1
fi
[[ -x "$app" ]] || { echo "no app at $app (build it first)" >&2; exit 1; }

work="$(mktemp -d)"
server_pid=""
windows=false
[[ "$chrome" == *.exe ]] && windows=true
if $windows; then
  win_temp="$(cmd.exe /c 'echo %TEMP%' 2>/dev/null | tr -d '\r')"
  profile_base="$win_temp\\pde2d-browser-test-$$"
  profile_unix="$(wslpath -u "$profile_base")"
else
  profile_base="$work/profile"
  profile_unix="$profile_base"
fi
cleanup() {
  [[ -n "$server_pid" ]] && kill "$server_pid" 2>/dev/null
  wait "$server_pid" 2>/dev/null
  rm -rf "$work"
  [[ -n "${profile_unix:-}" ]] && rm -rf "$profile_unix"*
}
trap cleanup EXIT

# The page with the harness next to it.
cp -r "$root/app/web" "$work/web"
cp "$here/harness.html" "$work/web/"
cp -r "$here/scenarios" "$work/web/scenarios"

"$app" --port 0 --web "$work/web" >"$work/server.log" 2>&1 &
server_pid=$!
port=""
for _ in $(seq 100); do
  port="$(sed -n 's|.*http://127.0.0.1:\([0-9]*\)/.*|\1|p' "$work/server.log")"
  [[ -n "$port" ]] && break
  sleep 0.1
done
[[ -n "$port" ]] || { echo "the app did not start:"; cat "$work/server.log"; exit 1; }

run_chrome() { # profile-suffix width height url extra-args...
  local suffix="$1" width="$2" height="$3" url="$4"
  shift 4
  local profile="$profile_base-$suffix"
  timeout 180 "$chrome" --headless=new --disable-gpu --no-first-run --no-default-browser-check \
    --user-data-dir="$profile" --window-size="$width,$height" --virtual-time-budget=60000 "$@" "$url" 2>/dev/null
}

failed=0
count=0
for entry in "${all[@]}"; do
  read -r name width height tab <<<"$entry"
  if [[ ${#scenarios[@]} -gt 0 && ! " ${scenarios[*]} " =~ " $name " ]]; then continue; fi
  count=$((count + 1))
  url="http://127.0.0.1:$port/harness.html?s=$name&w=$width&tab=$tab"
  # Windows Chrome reaches a WSL port only once the localhost relay has
  # it, a moment after the app listens: a page that did not load is tried
  # again.
  for attempt in 1 2 3 4 5; do
    dom="$(run_chrome "$name-$attempt" "$width" "$height" "$url" --dump-dom)"
    [[ "$dom" == *'id="log"'* ]] && break
    sleep 1
  done
  logtext="$(python3 -I -c '
import html, re, sys
m = re.search(r"<pre id=\"log\" hidden=\"\">(.*?)</pre>", sys.stdin.read(), re.S)
print(html.unescape(m.group(1)) if m else "FAIL no log (the page did not run)")' <<<"$dom")"
  echo "--- $name (${width}×${height})"
  echo "$logtext"
  if grep -q '^FAIL' <<<"$logtext" || ! grep -q '^DONE' <<<"$logtext"; then
    failed=$((failed + 1))
  fi
  if [[ -n "$shots" ]]; then
    mkdir -p "$shots"
    if $windows; then
      target="$profile_base-shot.png"
      run_chrome "$name-shot" "$width" "$height" "$url" --screenshot="$target" >/dev/null
      cp "$(wslpath -u "$target")" "$shots/$name.png" 2>/dev/null
    else
      run_chrome "$name-shot" "$width" "$height" "$url" --screenshot="$shots/$name.png" >/dev/null
    fi
  fi
done
echo "$((count - failed))/$count scenarios passed"
[[ $failed -eq 0 && $count -gt 0 ]]
