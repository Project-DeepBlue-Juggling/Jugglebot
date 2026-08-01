---
title: "teensy_link relocated from controller/ to the repo root (+ sys.modules compat shim)"
type: refactor
date: 2026-08-01
status: fix-landed-pending-hardware-confirm
phase: "refactor-2026-07 Phase 4"
related_plan: refactor-2026-07.md
files_changed:
  - teensy_link/
  - controller/teensy_link.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - tests/teensy_link/test_compat_shim.py
  - CLAUDE.md
subsystem:
  - canbridge
  - ros
tags:
  - refactor
  - teensy-link
  - packaging
---

# teensy_link → repo root

`git mv controller/teensy_link teensy_link`. The repo's hottest production code
(`protocol.py`, 16 commits since May) was living inside its most dormant
subsystem, and both launch files already inject the repo root on `PYTHONPATH`,
so the new location resolves with no launch change. Repo root over
install-into-the-ROS-package (owner decision): the injection makes the bridge
run the **live tree**, so a wire-format edit is live at the next relaunch;
installing it would put every `protocol.py` edit behind a `colcon build` whose
omission is silent — last week's frame layouts against this week's firmware, no
error. `teensy_bridge_launch.py`'s old "long-term fix: install into the ROS
package" note now records that decision instead of proposing the opposite.

**327 references across 134 files.** 89 files rewritten (code, tests, probes,
bench harnesses, active docs, codegen comments + regenerated artifacts).
`controller.teensy_link` now survives only in: the shim, its test, historical
logbook/plan narrative, and four `Teensy_code_canbridge/` firmware comments
(firmware source is out of scope for this branch).

Two things the mechanical rewrite would have missed:

- **`protocol.py` derived the repo root as `../..`** relative to its own file,
  to reach `config/generated/`. At the repo root that is one level too high;
  now `..`. Nothing would have failed loudly — the module also has a `sys.path`
  fallback — so this is the kind of break that surfaces later as a mystery.
- **The bridge no longer imports CasADi.** `from controller.teensy_link import …`
  executed `controller/__init__.py`, which imports `mpc` → CasADi. The ROS
  package now has **zero** `controller.*` imports (grep-verified), so the
  production bridge process stopped paying for the parked MPC runtime at import.

The compat shim `controller/teensy_link.py` is **`sys.modules` aliasing, not a
re-export**, and it aliases every submodule explicitly (enumerated live via
`pkgutil.iter_modules`). A re-export would give the two paths two class objects
for `RpcError`/`RpcTimeout`, so a `raise` on one identity sails through an
`except` written against the other — and nothing in an ordinary suite notices.
Aliasing only the package is equally unsafe: the finder would then build a
fresh `controller.teensy_link.rpc` off the aliased parent's `__path__`.
`tests/teensy_link/test_compat_shim.py` pins `is`-identity in fresh
interpreters in **both** import orders. TEMPORARY — delete after 2026-09.
The alias loop's cost, audited and accepted: `import controller.teensy_link` now
eagerly imports all 14 submodules where the old `__init__` pulled four — every
submodule is constants/loggers/type-aliases at module scope (no sockets, no file
reads), so it is ~10 extra imports on a deprecated path, not a hazard.

Review caught one thing the move introduced that no test covered: the top-level
name `teensy_link` is **also** claimed by `tests/teensy_link/`, a regular
package, and `tests/conftest.py` inserts `tests/` at `sys.path[0]` last. At
`sys.path = [tests, repo]`, `import teensy_link` resolves to the test directory
and `import teensy_link.setpoint_pump` dies — reproduced live. It is green only
because pytest's prepend import mode re-inserts the rootdir before each test
module, i.e. by accident of pytest internals. Now pinned in
`test_compat_shim.py`, the same guard `test_sim_import_style.py` installs for
`sim` (whose test directory collides identically). Renaming `tests/teensy_link/`
would remove the collision outright; pinning was chosen instead, matching the
`sim` precedent one day earlier, because the rename touches 15 files to fix a
hazard that is already loud — verified: under `--import-mode=importlib`,
`tests/teensy_link/conftest.py` itself dies on `from teensy_link import
protocol`. So the pin's contribution is the *named diagnosis* ("tests/teensy_link
is shadowing it") in front of an error that otherwise points at a submodule.
`teensy_link` also joined `test_sim_import_style.py`'s `_SCAN_DIRS`, so
relocating a package out of `controller/` does not silently drop it out of the
bare-import scan.

## Verification

- **Full gate** — `./run_tests.sh --full`, run 2026-08-01 on the Jetson under
  `~/Desktop/PDJ_venv/venv`: **parallel 4406 passed, 3 xfailed in 448.43 s;
  serial 9 passed, 4409 deselected in 40.52 s; total 494 s; RESULT: PASS**
  (exit 0). `--full` because the change touches `controller/` and `sim/`.
  4398 → 4406 is exactly the 8 new tests in `test_compat_shim.py`.
- `pytest tests/teensy_link/ tests/firmware -q` (2026-08-01): **580 passed
  before the move, 588 after** (+8 = the new shim test file), 26.17 s.
- Wire bytes unmoved: `test_udp_protocol_xlang.py::test_wire_layout_frozen`
  digest `1383b3fc18dc085c51eba58979ef60a3898b2ff74dc99931926d04c0bc7ceccb`
  **identical before and after**, pin file untouched. `fault_golden.json`'s only
  diff is its `_note` path string, and
  `pytest tests/firmware/test_native_firmware.py -q` (2026-08-01): **17 passed
  in 173.44 s** — a freshly compiled emission still equals the committed golden.
- `pytest tests/ros/ -q` (2026-08-01): **1613 passed in 94.21 s**.
  `pytest tests/motion/ tests/sim/test_toss_gate.py tests/sim/test_reload_gate.py
  tests/sim/test_sim_import_style.py -q` (2026-08-01): **903 passed in 313.86 s**.
- All 95 rewritten/moved `.py` files byte-compile and every `teensy_link`/
  `controller` import target resolves via `importlib.util.find_spec` — the
  Phase 6 lesson that a green suite does not cover probes and bench harnesses.
- `colcon build --packages-select jugglebot` (2026-08-01): finished in 2.42 s.
  Installed-copy smoke, run from `/tmp` so cwd cannot mask the path:
  `import jugglebot.teensy_bridge_node` **fails** with
  `ModuleNotFoundError: No module named 'teensy_link'` without the injection and
  **succeeds** with `PYTHONPATH=/home/jetson/Desktop/Jugglebot` — the launch
  injection is load-bearing and correct.

## Operator steps before the next powered session

1. **`cd ros_ws && colcon build --packages-select jugglebot`, then relaunch.**
   `ros2 launch` runs the INSTALLED copy; the launch files carry the PYTHONPATH
   injection, so a stale install means a stale injection.
2. **Disarmed bench link-up smoke — required, not yet done.** Launch, confirm
   `teensy_bridge_node` comes up and `link_status` reaches `UP` with heartbeats
   flowing, and leave it DISARMED. This is the first run of the moved import on
   real hardware; everything above is off-robot evidence.
