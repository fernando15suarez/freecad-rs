#!/bin/sh
# Build OndselSolver from FreeCAD's sources plus drv.cpp, run it on asmt/*_start.asmt and
# write ondsel_master.json (current OndselSolver weights) and ondsel_r3.json (rotation
# weights with aJi/aJiMax = 1/3, which reproduces FreeCAD 1.1.3 exactly).
#   FREECAD_SRC=~/Projects/rust-gaming/freecad-src sh tools/parity/assembly/ondsel/run.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=${FREECAD_SRC:-$HOME/Projects/rust-gaming/freecad-src}/src/3rdParty/OndselSolver/OndselSolver
BUILD=${BUILD:-${TMPDIR:-/tmp}/ondsel-parity-build}
mkdir -p "$BUILD/obj"
i=0
for f in "$SRC"/*.cpp; do
  o="$BUILD/obj/$(basename "$f" .cpp).o"
  [ -f "$o" ] || c++ -std=c++20 -O1 -w -I"$SRC" -c "$f" -o "$o" &
  i=$((i+1)); [ $((i % 12)) -eq 0 ] && wait
done
wait
# Part.cpp with the inertia ratio as an environment variable (AJ_RATIO, default 1).
python3 -c 'import sys; s = open(sys.argv[1]).read(); open(sys.argv[2], "w").write(s.replace("wqE->at(i) = (maxw * aJi / aJiMax) + minw;", "wqE->at(i) = (maxw * (getenv(\"AJ_RATIO\") ? atof(getenv(\"AJ_RATIO\")) : aJi / aJiMax)) + minw;"))' "$SRC/Part.cpp" "$BUILD/PartRatio.cpp"
grep -q AJ_RATIO "$BUILD/PartRatio.cpp"
c++ -std=c++20 -O1 -w -include cstdlib -I"$SRC" -c "$BUILD/PartRatio.cpp" -o "$BUILD/PartRatio.o"
c++ -std=c++20 -O1 -w -I"$SRC" "$HERE/drv.cpp" $(ls "$BUILD"/obj/*.o | grep -v '/Part.o$') "$BUILD/PartRatio.o" -o "$BUILD/drv"
python3 "$HERE/collect.py" "$BUILD/drv" "$HERE"
