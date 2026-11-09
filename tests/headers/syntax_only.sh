#!/bin/sh
# Compiler launcher of the header_standalone check (CMakeLists.txt here):
# runs the compile command with -fsyntax-only, so each header is parsed and
# type-checked without generating code, then creates the empty object file
# the build tool expects. The dependency file (-MD) is still written, so a
# header is rechecked only when it or one of its includes changes.
out=
prev=
for arg in "$@"; do
  if [ "$prev" = "-o" ]; then
    out=$arg
  fi
  prev=$arg
done
"$@" -fsyntax-only || exit
if [ -n "$out" ]; then
  : >"$out"
fi
