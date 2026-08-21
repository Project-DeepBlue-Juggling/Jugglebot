---
title: Arc-closure cleanup — R7 re-keyed from bridge uptime onto bridge health, plus the stale-reference sweep
type: refactor
date: 2026-08-15
status: resolved
phase: "bridge-temporal-trustworthiness closure cleanup"
related_plan: toss-selftuning.md
subsystem:
  - can
  - tools
tags:
  - safety
  - testing
  - docs
---

# Arc-closure cleanup — R7 re-keyed from uptime onto health, plus the stale-reference sweep

**What/why.** The bridge-temporal arc closed today (`2026-08-15-fw14-validated-arc-closed.md`):
FW 14 fixed the FlexCAN_T4 `_available` RX-ring leak, validated at 5.8 h and 15.2 h uptime, and
reboot-before-every-session retired — bridge uptime is no longer a tracking-quality variable.

**R7 gates on health, not age.** `toss_cal_grid.py`'s R7 hard-refused any bridge older than
`UPTIME_ABORT_MS` (30 min) — obsolete advice that would have blocked legitimate toss-calibration
captures on a healthy warm bridge. Shared helper `temporal_health_verdict(link_kv, uptime_ms)` now
lives in `tilt_cal_grid.py` (with `bridge_fw_version_value`, `worse_latency_monitor`,
`TemporalVerdict`), imported by `toss_cal_grid.py` the way it already imports `wire_armed_verdict` —
one implementation, one place for a threshold to drift. **The branch ORDER is the design**: (a) a
`latency_monitor` alarm (`RING_LEAK`/`CACHE_AGE`/`CLAMP_DUTY`) outranks everything **on any
firmware** — direct evidence the plant is degraded *now*, which no uptime number can acquit (the old
gate would have waved a 1-minute-old *leaking* bridge straight through); (b) FW ≥ 14 with the monitor
OK passes, `uptime_ms` logged for the record but never gated; (c) everything else — pre-FW-14 board,
unreadable `bridge_fw_version`, ROS build too old to publish the monitor row — falls back to the
30-min ceiling verbatim, each being a state where health cannot be *seen* and the board may genuinely
be uptime-sensitive.

**The non-obvious part.** The uptime ceiling bounded the WHOLE 2–3 h capture, the drift being
monotone in uptime; a health *snapshot* at preflight does not. So the runner tracks the **worst
`latency_monitor` token seen across the session**, R7 folds it in (recovered-just-now ≠
never-degraded), and `latency_monitor_first/last/worst/samples/bad_samples` go to `_meta.json` beside
`uptime_ms_first/last`, with an end-of-run console verdict — without which "the real protection is
preserved" would have been false. **Deliberately NOT done**: no second `/ring_diag` subscription
re-thresholding `leak_*` — the monitor's first and highest-precedence input already IS the ring leak,
at 10 Hz inside the node that receives the frames; a second subscription would re-implement
`RING_LEAK_WARN_FRAMES` one hop further from the data. `tilt_cal_grid.py` stays a WARNING, not a
refusal (static inclinometer reads are uptime-insensitive); only the question it asks changed.

**Tests** (`tests/motion/test_toss_cal_grid.py`, `test_tilt_cal_grid.py`): warm-healthy-FW14 passes;
each of the three alarms refuses even on a 1-minute-old bridge; alarmed-earlier-in-session refuses;
pre-FW-14 warm/unknown still refuse; missing `/link_status` refuses; every `bridge_fw_version` render
shape and all six verdict branches — plus an **xref test** pinning the four token strings and their
rank order against `teensy_bridge_node.py`'s source **text** (read, not imported: these harnesses
must import without `rclpy`, and a rename there would silently turn every comparison into "unknown
token").

**Stale references swept.** Arc plan + superseded lead-clamp draft moved to
`plans/archived/2026-08-15 <name>.md`; references repointed in `config/generate_udp_protocol.py`,
`teensy_link/{client,tod_server}.py`, `tests/firmware/test_udp_protocol_xlang.py`,
`tests/teensy_link/test_rx_timestamping.py`, `canbridge_config.h`,
`plans/active/{refactor-2026-07,leg-bus-frame-drops}.md`. **Trap worth recording**: the generator's
copy was SPLIT ACROSS TWO PYTHON SOURCE LINES — neither the full path nor the bare filename matched a
grep — and it regenerates five artifacts, so repointing those five without fixing the generator would
have restored the stale path at the next codegen run. Firmware **byte-identical** after the
comment-only edits (`pio run -e teensy41`: text 232768 / data 35520 / bss 107872, `firmware.hex` md5
`ea705b4bb4026047318c0361750c87ab` before and after) — no reflash implied. Retired-rule sites the
first docs pass missed, now amended: `session_phase7_reload.md`'s fifth-sitting banner (whose "reboot
isolation experiment still outstanding" claim was also false — that was S1), a dangling `per § Shared
preconditions` cross-reference in `session_anomaly_fixes.md` (retargeted to the **Platform** Teensy
per that file's own standing rule 2 — it named the wrong board), `session_phase8_toss_hardware.md`
tense, three per-rung "fresh can-bridge boot" preconditions in `session_tilt_calibration.md`, and a
`mocap_timebase_bench.py` comment still asserting the parent entry was `status OPEN` / cause unknown.
Historical debriefs and Platform-Teensy/ODrive power-cycles (a different subject) were left alone.
**Archive assessment: nothing qualified** — all nine arc-added test files cover telemetry with an
ongoing job under the standing keep-it decision (the latency-monitor test is now load-bearing for
R7), and the arc created **no committed probe at all**: the conviction came from bag analysis and
firmware instrumentation.

**Verification.**

- 2026-08-15, `python -m pytest tests/motion/test_toss_cal_grid.py tests/motion/test_tilt_cal_grid.py
  tests/teensy_link/ -q`: **486 passed in 11.10 s**.
- 2026-08-15, `python -m pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_logbook_search.py tests/firmware/ -q`: **452 passed in 194.77 s**.
- Gate (`./run_tests.sh`, run 2026-08-15, with every change in this batch in place): **5197 passed in 228.15 s, RESULT: PASS.** (This entry itself was written after that run; its own test surface — the logbook front-matter, search-index and plans-index gates — was re-run against the committed state: 53 passed.)
