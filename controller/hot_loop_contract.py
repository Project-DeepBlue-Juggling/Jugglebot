"""Shared constants for the MPC 40 Hz hot-loop zero-allocation contract.

This module is intentionally tiny: it exists so that
``controller/HOT_LOOP_CONTRACT.md``, the enforcement test
(``tests/sim/test_hot_loop_allocation_contract.py``), and future
ratchets all reference a single source of truth.

See ``controller/HOT_LOOP_CONTRACT.md`` for the normative spec.
"""

from __future__ import annotations


# Per-tick allocation budget enforced by
# ``tests/sim/test_hot_loop_allocation_contract.py``.
#
# Measured over 100 steady-state ticks (ticks 51..150 after a 50-tick
# warmup) of ``run_mpc_loop`` driving a ``MuJoCoPlant`` with a
# ``StaticTargetSource``.  Counts only Python-visible allocations that
# ``tracemalloc`` attributes to the hot-loop body; CasADi-internal C
# buffers and msgpack-internal C buffers are excluded by design (they do
# not drive Python's generational GC, which is the failure mode this
# contract defends against).
#
# Historical ratchet:
#   - Initial ship (W4e, 2026-04-23):              1024 B/tick.
#     Relaxed starting value that let the W4 sub-phases land
#     incrementally while the test stayed green.
#   - W7 ratchet (2026-04-23, post-W4+W5):          256 B/tick.
#     Tighter floor once the full W1 inventory landed.  Post-W4+W5
#     measurement was 137 B/tick (dominated by tracemalloc's own regex
#     filter machinery + fnmatchcase — zero application code in the
#     top-10).  256 B is ~2x current, enough headroom for minor
#     interpreter-internal variation but strict enough that any new
#     per-tick dict, ndarray, or similar-size retained object trips CI.
#     Below ~200 B/tick trips on interpreter-internal allocations
#     (f-string cache misses, exception objects on cold branches).
THRESHOLD_BYTES: int = 256


# Number of ticks the enforcement test runs past the warmup window.
# The threshold is evaluated over exactly this many ticks, so the
# allocation measurement is ``(snapshot_end - snapshot_start) /
# HOT_LOOP_CONTRACT_WINDOW_TICKS``.
HOT_LOOP_CONTRACT_WINDOW_TICKS: int = 100


# Ticks skipped before the measurement window opens.  Covers the
# MPC prime solve, first-target quintic build, CasADi JIT warmup, and
# the CSV header write.  Set empirically — at 40 Hz this is 1.25 s of
# sim time, more than enough for steady state to settle.
HOT_LOOP_CONTRACT_WARMUP_TICKS: int = 50


# Number of traceback frames captured by ``tracemalloc.start()`` in the
# enforcement test.  25 is enough to attribute allocations through
# several layers of helper functions (e.g. ``run_mpc_loop`` →
# ``log_mpc_step`` → ``record_from_arrays`` → numpy), which is what
# makes the top-10 failure diagnostic useful.
HOT_LOOP_CONTRACT_TRACEBACK_FRAMES: int = 25
