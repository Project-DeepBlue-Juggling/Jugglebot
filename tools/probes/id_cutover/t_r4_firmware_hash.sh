#!/usr/bin/env bash
# T-R4 firmware image unchanged: the cutover edits firmware comments only, so a fresh BUILD
# (never an upload) must reproduce the pre-sweep firmware.hex of both Teensy images.
# Plan row: plans/active/item-id-per-kind.md section 4 P4, proof T-R4. Baseline: the .hex
# lines of BASELINE (default temp/id-migration/p4-fw-before.txt in the main checkout; the .elf
# is nondeterministic across clean builds and is excluded). Also asserts no __DATE__/__TIME__/
# __FILE__/__LINE__ in either firmware source tree (both are flat: no src/ dir). Evidence: temp/probes/id_cutover/fw-after.txt.
# Exit 0 = both .hex hashes equal the baseline and the grep is 0.
set -u
ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT" || exit 2
BASELINE="${BASELINE:-/home/jetson/Desktop/Jugglebot/temp/id-migration/p4-fw-before.txt}"
OUT="$ROOT/temp/probes/id_cutover"; mkdir -p "$OUT"
AFTER="$OUT/fw-after.txt"; : > "$AFTER"
CB=ros_ws/src/jugglebot/Teensy_code_canbridge; PL=ros_ws/src/jugglebot/Teensy_code_platform
fail=0
pio run -d "$CB" -e teensy41 > "$OUT/pio-canbridge.log" 2>&1 || { echo "pio canbridge build failed (see $OUT/pio-canbridge.log)"; fail=1; }
pio run -d "$PL" -e teensy40 > "$OUT/pio-platform.log" 2>&1 || { echo "pio platform build failed (see $OUT/pio-platform.log)"; fail=1; }
( cd "$CB" && sha256sum .pio/build/teensy41/firmware.hex ) >> "$AFTER"
sha256sum "$PL/.pio/build/teensy40/firmware.hex" >> "$AFTER"
# baseline paths: canbridge relative to its dir, platform relative to repo root
grep '\.hex$' "$BASELINE" | sort > "$OUT/fw-before-hex.txt"
grep '\.hex$' "$AFTER" | sort > "$OUT/fw-after-hex.txt"
if cmp -s "$OUT/fw-before-hex.txt" "$OUT/fw-after-hex.txt"; then hexok=1; else hexok=0; fail=1; diff "$OUT/fw-before-hex.txt" "$OUT/fw-after-hex.txt"; fi
# both firmware trees are flat (no src/ dir): scan every source file outside the build dir
n=$(grep -rn '__DATE__\|__TIME__\|__FILE__\|__LINE__' "$CB" "$PL" --include='*.cpp' --include='*.h' --include='*.ino' --include='*.c' --include='*.py' --exclude-dir=.pio --exclude-dir=.git 2>/dev/null | wc -l)
[ "$n" -eq 0 ] || fail=1
if [ "$fail" -eq 0 ]; then v=PASS; else v=FAIL; fi
line="T-R4 $v: hex equal=$hexok (2 images) date/time/file/line macros=$n"
echo "$line" | tee -a "$AFTER"
exit "$fail"
