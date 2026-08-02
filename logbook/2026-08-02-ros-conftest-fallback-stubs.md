---
title: tests/ros/conftest.py claimed it ran without ROS2 and did not — two fallback stubs make the claim true
type: bugfix
date: 2026-08-02
status: resolved
phase: "Developer workflow — test-suite hygiene"
files_changed:
  - tests/ros/conftest.py
  - CLAUDE.md
subsystem:
  - ros
tags:
  - testing
  - docs
---

# tests/ros/conftest.py claimed it ran without ROS2 and did not

## Summary

`tests/ros/conftest.py`'s docstring says it injects fake ROS2 modules "so that
`import jugglebot.*` resolves on Windows without a ROS2 installation". It mocked
`rclpy`, `jugglebot_interfaces`, `geometry_msgs`, `std_msgs`, `std_srvs` and
`sensor_msgs` — but **not** `diagnostic_msgs` or `ament_index_python`, which five
modules import (`teensy_bridge_node`, `trajectory_node`, `orchestrator_node`,
`motion_bridge_node`, `ball_butler_node`). The claim had been false for those.

Both are now stubbed as a **fallback**, registered only when the real package is
absent. Also corrects a stale ci-deep figure in CLAUDE.md.

## Fix

`diagnostic_msgs.msg` gains `DiagnosticStatus` + `KeyValue`;
`ament_index_python.packages` gains `get_package_share_directory`,
`get_package_share_path` and `PackageNotFoundError`.

CLAUDE.md's "`--full` is 1259 s at ci-deep" (measured 2026-08-01) disagreed with
the 2026-08-02 nightly's 25m 52s. Rather than swap one number for another that
rots the same way, it now cites ~26 min with its date **and points at
`temp/reports/nightly/latest.md`** as the living source, since the nightly
re-measures it daily.

## Verification

- Stub path, `/tmp/probe_ros_stubs.py` (run 2026-08-02): blocks both packages via
  a `sys.meta_path` finder, proves the block took, imports the conftest, and
  asserts the stubs — **ALL STUB CHECKS PASSED**. Needed because on this box the
  real packages win, so the `except ImportError` branches never execute and a
  broken stub would be invisible.
- Jetson path unchanged (run 2026-08-02): `diagnostic_msgs.__file__` and
  `ament_index_python.packages.__file__` both still resolve under
  `/opt/ros/foxy/`, i.e. the real packages still win here.
- `pytest tests/ros/ -q` (run 2026-08-02): **1613 passed in 93.19 s**.
- Gate (`./run_tests.sh`, run 2026-08-02): **parallel 205 s (3983 passed) |
  serial 9 s | total 214 s** — `RESULT: PASS`.

## Discussion

### Fallback, not unconditional — the codebase had already decided this

The obvious implementation is to inject both alongside `rclpy` and be done. It
was rejected on evidence, not taste: `tests/ros/test_teensy_bridge_node_read.py`'s
docstring states outright that "`diagnostic_msgs` is the real package (installed
on the Jetson; used by the production `motion_bridge_node` too)". That is a
deliberate, documented choice. Shadowing it unconditionally would silently lower
fidelity on the box that is authoritative for ROS behaviour, to buy portability
nothing on that box needs — and would repeat, in a different file, the mistake of
overriding a landed decision without amending the text that records it.

The same held for `ament_index_python`, from the other direction: its absence is
*already* handled where it matters. `motion/friction_ff_params.py` wraps the
import in `try/except` and falls back to the source tree, and
`tests/motion/test_friction_ff_params.py` pins that fallback by simulating the
`ImportError`. Only `ball_butler_node.py`'s **top-level** import was unguarded —
its call site at `:317` is already inside `try/except` — so the stub exists purely
so that one module can be imported, not to change any behaviour.

### The bytes-vs-int trap, which a plausible stub gets wrong

Foxy declares `DiagnosticStatus.level` as `byte`, so the constants are
`b'\x00'`/`b'\x01'`/`b'\x02'`/`b'\x03'` — **not** ints — and the real class
asserts the field is `bytes` of length 1 (`DiagnosticStatus(level=1)` raises).
The natural stub writes `OK = 0`, which is self-consistent: production does
`status.level = DiagnosticStatus.OK` and tests compare against the same constant,
so a stub-only world passes. The divergence appears only when a value crosses
between the stub and the real package — precisely the cross-platform case the
stub exists to serve.

Caught by reading the installed package rather than reasoning about what the
constants "should" be, which is the same discipline that caught `pytest.skip`
being a `BaseException` in the previous commit. Both were invisible on this box.

### Why the ament stub raises instead of returning a path

`get_package_share_directory` could return a plausible-looking path. It raises
`PackageNotFoundError` instead, because without an ament index there genuinely is
no share directory, and every call site in this repo already catches and falls
back to the source tree. Returning a fabricated path would convert a correctly
handled absence into a silent mislookup — a worse failure than the one being
fixed. The stub's exception subclasses `KeyError`, matching the real class's
`PackageNotFoundError -> KeyError -> LookupError` chain, so `except KeyError` and
bare `except Exception` call sites behave identically either way.
