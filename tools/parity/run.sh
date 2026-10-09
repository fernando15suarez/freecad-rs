#!/bin/sh
# Regenerate the FreeCAD parity reference models (tools/parity/models/*.FCStd + *.json).
#   tools/parity/run.sh                 # all models
#   PARITY_ONLY=pad_rect tools/parity/run.sh
#   tools/parity/run.sh measure FILE.FCStd   # print FreeCAD's measurements of any file
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
FC="${FREECADCMD:-/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd}"
export PARITY_DIR="$HERE"
if [ "$1" = "measure" ]; then
  PARITY_FILE="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")" "$FC" "$HERE/measure.py" 2>&1 | grep -a '^MEASURE ' | sed 's/^MEASURE //'
else
  "$FC" "$HERE/make_models.py" 2>&1 | grep -a '^PARITY \|Error\|Traceback\|Exception' || true
fi
