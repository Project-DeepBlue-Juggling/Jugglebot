#!/usr/bin/env bash
# Runs the per-kind Item ID cutover proofs T-R2..T-R6 in order (T-R4, the firmware build, last),
# continues past a failure, prints a summary table and exits non-zero if any proof failed.
# Plan row: plans/active/item-id-per-kind.md section 4 P4. Run from anywhere inside the repo:
#   bash tools/probes/id_cutover/run_all.sh > temp/logs/id-p4-proofs.log 2>&1
# Needs the venv on PATH (python, pio). BUILD ONLY: never uploads firmware.
set -u
ROOT="$(git rev-parse --show-toplevel)"; cd "$ROOT" || exit 2
D=tools/probes/id_cutover
echo "id cutover proofs, $(date -Iseconds), HEAD $(git rev-parse --short HEAD)"
rc_all=0; table=""
run() {  # name, command...
  local name="$1"; shift
  local out; out="$("$@" 2>&1)"; local rc=$?
  echo "$out"
  table="$table$(printf '%-6s rc=%d  %s' "$name" "$rc" "$(echo "$out" | tail -1 | cut -c1-200)")"$'\n'
  [ "$rc" -eq 0 ] || rc_all=1
}
run T-R2 python "$D/t_r2_round_trip.py"
run T-R3 python "$D/t_r3_python_ast.py"
run T-R5 python "$D/t_r5_generated_constants.py"
run T-R6 python "$D/t_r6_zero_count.py"
run T-R4 bash "$D/t_r4_firmware_hash.sh"
echo; echo "== summary =="; printf '%s' "$table"
[ "$rc_all" -eq 0 ] && echo "ALL PASS" || echo "SOME FAILED"
exit "$rc_all"
