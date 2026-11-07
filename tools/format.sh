#!/usr/bin/env bash
# Format every tracked C++ source with the repository .clang-format.
# Usage: tools/format.sh          rewrite files in place
#        tools/format.sh --check  fail if any file needs formatting
set -euo pipefail
cd "$(dirname "$0")/.."
mapfile -t files < <(git ls-files '*.hpp' '*.cpp')
if [[ "${1:-}" == "--check" ]]; then
  clang-format --dry-run --Werror "${files[@]}"
else
  clang-format -i "${files[@]}"
fi
