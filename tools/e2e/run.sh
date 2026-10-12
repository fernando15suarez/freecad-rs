#!/bin/sh
# Drive the real app with scripted mouse/keyboard input (egui raw input) and check results.
# Usage: tools/e2e/run.sh [script.txt ...]   (default: all scripts here). Screenshots go to ./out/e2e.
set -e
cd "$(dirname "$0")/../.."
cargo build -q -p fc-app
mkdir -p out/e2e
fail=0
for s in ${@:-tools/e2e/*.txt}; do
  name=$(basename "$s" .txt)
  sed "s#OUT#$PWD/out/e2e#g" "$s" > "out/e2e/$name.script"
  log=$(FCRS_SIZE=1280x800 FCRS_INPUT="out/e2e/$name.script" "${CARGO_TARGET_DIR:-target}/debug/freecad-rs" 2>&1 | grep '^\[input\]')
  echo "$log" | grep -q 'done, 0 failure' && echo "PASS $name" || { echo "FAIL $name"; echo "$log" | grep FAIL; fail=1; }
done
exit $fail
