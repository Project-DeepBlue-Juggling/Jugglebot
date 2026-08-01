#!/usr/bin/env bash
# Run the Jugglebot test suite with the correct interpreter.
#
# The system python lacks hypothesis/mcap/scipy and the ROS2 message stubs, so
# tests must run under the PDJ venv. This wrapper exists so the right interpreter
# is discoverable in-repo rather than tribal knowledge.
#
# Usage:
#   ./run_tests.sh                                       # full suite (the pre-commit gate)
#   ./run_tests.sh --hypothesis-profile=ci-deep          # same, at nightly depth
#   ./run_tests.sh tests/ros                             # a subset
#   ./run_tests.sh tests/ros/test_throw_ballistics.py -v # any pytest args, passed straight through
#   JB_TEST_WORKERS=5 ./run_tests.sh                     # override the worker count
#
# NAMING A PATH runs pytest directly, serial, exactly as this script always has:
# one startup, ordered output, pdb works. That is the right thing for iterating
# on a file. With no path named, you get the full two-phase gate below.
#
# `-m` and `-n`/`--dist` are REFUSED either way: this script uses `-m` to select
# the phase, and pytest's `-m` replaces rather than appends. Use JB_TEST_WORKERS
# for the worker count, or call pytest directly for a bespoke marker expression.
#
# tests/hardware/ needs a real robot; it is excluded unless you name it explicitly.
#
# ── THE FULL-SUITE GATE ───────────────────────────────────────────────────────
#   Phase 1  parallel  -m "not serial"  xdist, --dist loadfile   ~4290 tests
#   Phase 2  serial    -m serial        single process           ~7 tests, ~30 s
#
# Measured 2026-07-31 on the Jetson: 27:40 fully serial -> ~8 min here.
# Both phases always run, so one invocation reports every failure in the suite.
#
# WHY `--dist loadfile` AND NOT THE DEFAULT `--dist load`
# `loadfile` keeps every test in a file on a single worker. That is what makes
# the parallel phase safe without auditing 4300 tests individually:
#   * module-scoped fixtures (shared MuJoCoPlant, built solvers — 49 of them) are
#     constructed once per file, exactly as they are serially;
#   * the two files that bind FIXED localhost ports (motor_guard's :15555/:15556,
#     teensy_bridge_node_setpoint's :5599) cannot collide, because a file's tests
#     never straddle workers and no two files share a port. Tests that bind
#     EPHEMERAL ports (teensy_link, the ZMQ harnesses) were already safe.
# Switching to `--dist load` breaks both properties. Don't.
#
# WHY A FIXED WORKER COUNT AND NOT `-n auto`
# `-n auto` picks 6 here. RAM is the binding constraint, not cores. Measured
# 2026-07-31 (full 2x2 in logbook/2026-07-31-test-suite-parallelisation.md):
#
#                      wall     available-memory floor
#   -n 4 (default)    444.7 s          417 MB
#   -n 5              403.6 s          337 MB
#
# 5 workers is a real 9% faster and is defensible on a quiet box
# (`JB_TEST_WORKERS=5 ./run_tests.sh`). It is not the default because this Jetson
# also runs the robot: the GUI daemon, sometimes ROS2 nodes, sometimes a second
# agent session. Swap here is zram (compressed RAM, already ~50% used), not disk,
# so over-commit degrades hard instead of gracefully, and an OOM-killed worker
# surfaces as a confusing "node down" rather than an honest test failure.
set -uo pipefail

VENV_PY="${PDJ_VENV_PY:-$HOME/Desktop/PDJ_venv/venv/bin/python}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKERS="${JB_TEST_WORKERS:-4}"

if [[ ! -x "$VENV_PY" ]]; then
  echo "error: test interpreter not found at $VENV_PY" >&2
  echo "       set PDJ_VENV_PY to the venv python, or fix the path." >&2
  exit 1
fi

cd "$REPO_ROOT"

# Pin the BLAS/OpenMP pools to one thread each. numpy and CasADi otherwise size
# their pools from the CORE count *inside every xdist worker*, so 4 workers ask
# for ~24 threads on 6 cores. Measured 2026-07-31: pinning costs nothing in
# wall-clock (444.7 s vs 444.4 s at 4 workers) while dropping peak load
# 8.13 -> 6.41 and raising the memory floor ~100 MB. Both pinned runs reproduced
# the serial baseline result exactly (4294 passed, 3 xfailed), so it does not
# perturb numerical results. Exported before the passthrough branch so a subset
# run behaves like the gate.
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

# ── Reject the flags this script owns ────────────────────────────────────────
# This MUST come before the passthrough check below. pytest's `-m` is
# store-not-append, so a forwarded `-m` would silently REPLACE the phase filter:
# phase 1 would run the serial tests in parallel and phase 2 would re-run most of
# the suite. Checking first also avoids a subtler trap — a flag's VALUE (`-m "not
# slow"` -> the argument `not slow`) is not a flag, so a passthrough check that
# ran first would mistake it for a path and silently take the serial branch.
for arg in "$@"; do
  case "$arg" in
    -m|-m=*|-m?*|--markers)
      echo "error: -m selects the phase here and cannot be overridden." >&2
      echo "       run a marker expression directly:" >&2
      echo "       $VENV_PY -m pytest -m '<expr>'" >&2
      exit 4 ;;
    -n|-n=*|-n?*|--numprocesses*|--dist*)
      echo "error: use JB_TEST_WORKERS=<n> instead of $arg." >&2
      exit 4 ;;
  esac
done

# ── Passthrough: a path or node-id was named ─────────────────────────────────
# Preserves this script's long-standing contract: one plain serial pytest run,
# ordered output, pdb works.
#
# "Named a path" means "supplied an argument that is not a flag". Flags that take
# a SEPARATE value (`-k EXPR`) would otherwise look like a path, so their values
# are skipped. That list cannot be exhaustive — pytest has many such flags — but
# the failure mode of missing one is benign: an unrecognised separate value reads
# as a path and you get a plain serial pytest run, which is this script's
# historical behaviour and cannot corrupt the phase split. The dangerous flags
# (`-m`, `-n`) are refused above, before this runs.
skip_value=0
for arg in "$@"; do
  if [[ $skip_value -eq 1 ]]; then skip_value=0; continue; fi
  case "$arg" in
    -k|-p|-W|-o|-c|-r|--maxfail|--deselect|--ignore|--rootdir|--last-failed-no-failures)
        skip_value=1 ;;
    -*) ;;
    *)  exec "$VENV_PY" -m pytest "$@" ;;
  esac
done

# ── Full-suite two-phase gate ────────────────────────────────────────────────

if ! "$VENV_PY" -c 'import xdist' 2>/dev/null; then
  echo "error: pytest-xdist is not installed for $VENV_PY" >&2
  echo "       $VENV_PY -m pip install pytest-xdist==3.6.1 execnet==2.1.1" >&2
  exit 1
fi

# ── Suite mutex ───────────────────────────────────────────────────────────────
# Two concurrent gates corrupt the serial-phase timing baselines and can OOM
# this box (8 xdist workers on 6 cores, zram swap). Parallel Claude sessions
# are a real workflow, so overlapping gate invocations QUEUE here instead of
# colliding (observed twice on 2026-08-01). Scoped passthrough runs above do
# not take the lock — only the full gate does. Waits up to 30 min, then aborts
# loudly rather than overlap.
exec 9>"/tmp/jugglebot-run_tests.lock"
if ! flock -n 9; then
  echo "run_tests.sh: another gate holds /tmp/jugglebot-run_tests.lock — queuing (max 30 min)..."
  flock -w 1800 9 || { echo "error: timed out waiting for the suite lock" >&2; exit 1; }
fi

echo "═══ Phase 1/2: parallel (${WORKERS} workers, --dist loadfile, -m 'not serial')"
start=$SECONDS
rc_parallel=0
"$VENV_PY" -m pytest tests/ -q -n "$WORKERS" --dist loadfile -m "not serial" "$@" || rc_parallel=$?
t_parallel=$((SECONDS - start))

# pytest returns 2 for a user interrupt (it traps SIGINT itself), so without this
# a Ctrl-C during phase 1 would fall through and start phase 2.
if [[ $rc_parallel -eq 2 ]]; then
  echo "interrupted — skipping the serial phase" >&2
  exit 2
fi

echo
echo "═══ Phase 2/2: serial (-m serial)"
start=$SECONDS
rc_serial=0
# -p no:xdist makes this phase structurally serial: a stray -n becomes a usage
# error rather than a silently-parallelised run of the very tests that must not be.
"$VENV_PY" -m pytest tests/ -q -p no:xdist -m serial "$@" || rc_serial=$?
t_serial=$((SECONDS - start))

# pytest exit code 5 == "no tests collected".
if [[ $rc_parallel -eq 5 && $rc_serial -eq 5 ]]; then
  echo "error: no tests matched this selection at all." >&2
  exit 5
fi
if [[ $rc_parallel -eq 5 ]]; then
  # Legitimate when a flag-only selection (e.g. -k) matches only serial tests.
  echo "(no parallel-eligible tests in this selection)"
  rc_parallel=0
fi
if [[ $rc_serial -eq 5 ]]; then
  if [[ $# -eq 0 ]]; then
    # A bare full-suite run MUST have serial tests. Zero means the marker was
    # renamed or dropped and the serial phase silently became a no-op — the
    # allocation contract would never run while the gate still printed PASS.
    echo "error: full-suite run collected ZERO serial-marked tests." >&2
    echo "       the serial phase is a no-op — was the 'serial' marker renamed or dropped?" >&2
    exit 5
  fi
  echo "(no serial-marked tests in this selection)"
  rc_serial=0
fi

echo
echo "═══ Summary: parallel ${t_parallel}s (rc=${rc_parallel}) | serial ${t_serial}s (rc=${rc_serial}) | total $((t_parallel + t_serial))s"

if [[ $rc_parallel -ne 0 || $rc_serial -ne 0 ]]; then
  echo "═══ RESULT: FAIL"
  # Prefer the parallel phase's code so callers see a meaningful pytest exit code.
  [[ $rc_parallel -ne 0 ]] && exit "$rc_parallel"
  exit "$rc_serial"
fi

echo "═══ RESULT: PASS"
