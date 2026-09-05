# Bench session — unified-7DOF hand-lane bring-up (FW 17 lockstep flash + T-H1..T-H4)

Phase 3 of `plans/active/unified-7dof-planner.md` — the lockstep flash sitting
and its bench ladder (§ 5 T-H1..T-H4). Modeled on
`session_unified7_bus_headroom.md` (the Phase 0 sitting).

## Purpose

FW 17 puts the hand on the can-bridge's 500 Hz interpolated burst as the 7th
lane (v6 Setpoint, `HAS_HAND`, `hand_source` interlock, hand-specific guards).
This sitting (a) performs the **lockstep flash** — FW 17 and the v6 host are
mutually dark against every older counterpart, by design — and (b) validates
the hand streaming safety envelope on real hardware **before any planner work
touches it** (Phase 4/5 build on this lane).

Established offline, **do not re-derive**:

- Bus headroom for the 7-frame burst was qualified 2026-08-30
  (`session_unified7_bus_headroom.md` § Results): 62.2 % utilisation, deferral
  0, drop rate does NOT follow TX rate. This sitting re-reads the same validity
  counters but is not re-litigating headroom.
- The Hermite reconstruction bars (Phase 0 probe 2): **float-exact only on
  knot-aligned cubics** (the future planner's output); on the LEGACY closed-form
  stroke the honest v1-carriage prediction is **≤ 3.25 mm** worst-case
  (≤ 11.84 mm without v1). T-H2's stroke tolerance is the 3.25 mm number —
  "exact" is the wrong bar and would fail for a non-reason.
- Hand guard constants are owner-signed (Phase 0 Decision 4): deviation 2.5 rev
  velocity-compensated (OBSERVE-FIRST), lead 2.0 rev age-extrapolated,
  vel-FF cap 300 rev/s, overspeed 345 rev/s. This sitting reads the observe
  residuals; it does NOT arm the deviation trip (see § Observe-then-arm).

## The lockstep flash — read before anything is powered

**FW 17 (UDP protocol v6) and the v6 host tree deploy together, in this one
sitting, or not at all.**

| Pairing | Result |
|---|---|
| FW ≤ 16 board + v6 host (this tree) | **TOTAL LINK DARKNESS** — `decode_frame` rejects on version, both directions, loudly. No telemetry, no RPC, no motion. |
| FW 17 board + pre-v6 host | Same darkness, other direction. |
| FW 17 + this tree | The only live pairing. |

- The v6 host commit is **`fb972c3`** (Phase 2, "committed not deployed").
  Phase 3's firmware + host plumbing build on it; the tree you flash from and
  the tree you launch from must both contain it (`git log --oneline | grep
  fb972c3` or later).
- **Rollback is a PAIR, never a half**: reflash FW 16 (build it from the FW 16
  tree — the last commit before this phase's firmware change) **and** check out
  the pre-v6 host (`2aaaae1`, the Phase 2 fixture HEAD). A half-rollback is
  loud, not silent — the link simply stays dark until the pair matches. That
  loudness is the designed failure mode; do not "fix" it by mixing versions.
- This flash also **discharges FW 16's pending first flash** (poller cadence +
  tri-state TX accounting never went aboard): FW 16's behavioural content is
  carried forward in 17, so its first-flash acceptance rides along — read the
  hand ball-sensor poller cadence (~50 Hz) and the `[cantx]` per-class census
  once the board is up.
- `EXPECTED_BRIDGE_FW_VERSION` is already 17 in this tree, so until the flash
  the launch prints the `≤16 (SKEW — expected v17)` advisory — correct, and
  moot here because a skewed board is also a dark link under v6.

## Safety (operator-owned)

- **Whole sitting is BENCH. NO BALL anywhere near the hand.** E-stop within
  reach and tested before arming. The hand is on the same 48 V as the legs.
- The streamed lane's first frame commands **within the stated park gate of the
  commanded start** (hold/triangle/step/gap/moving_gap capture the live
  encoder; the stroke stage refuses unless the hand is inside the ±0.10 rev
  settle band of 0). Any residual is walked by the firmware recovery slew at
  ≤ 1 rev/s — arming is a bounded, near-no-op step.
- **Leg lanes are pinned holds at the live leg encoders** in every stage; legs
  may stay IDLE (streamed holds are latched, not acted on) or CLOSED_LOOP
  (zero-deviation no-op). No platform motion is commanded or needed.
- The deviation guard ships **observe-first** and stays observe this sitting.
  `hand7 arm` is the SECOND sitting's step — see § Observe-then-arm.
- **The hand lead-duty read is a hard rule — but read it as a DELTA, not an
  absolute (amended 2026-09-04).** Non-zero lead duty during a stroke = the
  clamp is shaping the command = **hard-abort the sitting** (the FW 14 arc's
  clamp-duty-0 precedent — a clamped stream is the S1/S2 silent-stall
  mechanism, not a curiosity). What changed is *how you read it*: `[hand7]
  lead=` and `dev_over=` are gated on `s_hand_active && hand_source_streamed()`
  (`leg_interp.cpp:684`) and **NOT on `s_output_enabled`**, so after any
  aborted or ended stage they keep counting ticks on which the firmware
  transmitted nothing — the 2026-09-04 sitting found `lead = 18 725 209` on an
  idle lane, reconciling to the exact wall-clock gap since the previous abort
  (`logbook/2026-09-04-fw17-hand-ladder-sitting-two.md` § "The first console
  capture ever taken"). **A non-zero ABSOLUTE is therefore uninformative and is
  NOT grounds to abort**; difference the counter across the stage and abort on
  a non-zero **delta**. There is also no runtime reset — `interp_reset()` has
  no caller and the console takes only `hand7 arm|observe` — so **the flash
  reboot is the only zero**: take the first post-flash `[hand7]` line as the
  baseline. `lead_clamp_mask` bit 6 / 0x40 on `/link_status` (launch up) is a
  different quantity and unaffected: it is an instantaneous last-tick snapshot,
  and it must read 0. **This amendment is a stopgap** — the fix is to add
  `out_en` to the gate (or a second output-enabled-only counter pair), which
  needs a reflash and is FW 18 work.
- **ABORT** on: any guard E-STOP, any hand motion in a hold stage, any nonzero
  `can3_errors` wire-error field, `leak_*` ≠ 0, a lost link, or a lead-duty
  **delta** ≠ 0 across a stage (see the amended rule above — the absolute is
  uninformative).

## Preconditions (once, before the flash)

| # | Step | Done |
|---|---|---|
| 1 | `./run_tests.sh --full` **green** — standing rule before any hardware sitting (this phase touched firmware natives + `controller/`-adjacent surfaces; `--full` is mandatory). Record the (date, command, result) triple in Results. | |
| 2 | `git fetch && git status -sb` clean, `origin` not ahead; confirm the tree contains `fb972c3` (v6 host) and this phase's FW 17 commit. | |
| 3 | Hand mechanically clear: no ball, cords clear of the slider, hand parked at the bottom rest (post-homing ≈ −0.1..0 rev). | |
| 4 | **BUILD ONLY — this row does NOT touch the board.** Clean build (the `extra_script.py` incremental hazard from the 4→5 bump applies to 5→6 identically): `cd ros_ws/src/jugglebot/Teensy_code_canbridge && rm -rf .pio/build && pio run -e teensy41`. This compiles a hex on the Jetson; **nothing is uploaded — the Teensy keeps running whatever it was already running.** Verify the hex md5 against the build-evidence table in `logbook/2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md` (FW 17 post-audit-fix build: `2e90c43d5b6889e1059982d2e7aaf961`, 769 144 B hex — reproducible across two clean builds 2026-09-03; supersedes `7abe226b…` after the audit's echo-stamp age-correction). A different md5 means the tree moved: stop and diff. **A green build and a matching md5 are NOT a flash** — on 2026-09-03 this row's md5 verify was read as "flashed", the board stayed on FW ≤ 16 against a v6 host, and the resulting (designed, correct) total link darkness cost a whole debugging session. The flash is row 5. | |
| 5 | **THIS IS THE FLASH — the only row that changes the firmware aboard.** **Operator flashes**: close any `pio device monitor` first, then `pio run -e teensy41 -t upload`. Nothing before this row has altered the board; nothing after it works until this row is done (confirm it in row 6 — the boot banner is the receipt). | |
| 6 | Open the monitor **captured to a file — use row 11c's command here, not a bare `pio device monitor`**, so the boot banner and rows 9-10 land in the same capture as the ladder. Confirm the boot banner reads `jugglebot-canbridge v17` and the 1 Hz `[hand7]` line appears with `src=LEGACY guard=observe lane=idle` and every counter 0. **LEGACY at boot is the contract** — a board that boots STREAMED is wrong; stop. | |
| 7 | `colcon build --packages-select jugglebot` + `source install/setup.bash` (the bridge node's hand_source service + link_status row are new). | |
| 8 | Launch with recording: `ros2 launch jugglebot jugglebot_launch.py record:=true`. Confirm `/link_status`: `bridge_fw_version` **`17 (proto 6)`**, `BRIDGE_FW_CHECK: OK`, `hand_source: LEGACY_STROKE`, `latency_monitor` OK, `uptime_ms` fresh. The link being ALIVE at all is the v6 lockstep succeeding. | |
| 9 | FW 16 carry-forward acceptance: hand ball-sensor poller cadence (~50 Hz claim, console-side), `[cantx]` census present, all columns 0. | |
| 10 | Legacy regression smoke (T-R3 hardware half): one `smooth_move_hand` to 1.0 rev and back via the launch (`ros2 service call /smooth_move_hand ...`). The LEGACY path must work exactly as before — FW 17 changes nothing while the latch is LEGACY. | |
| 11 | **Shut the launch down** for the bench ladder — `hand_stream_bench.py` is the sole owner of the UDP link (single-owner rule). The console monitor stays open; `[hand7]` is a primary observable. | |
| 11b | **The hand must be ENERGISED for every streaming row below** — `axis_state == CLOSED_LOOP` (8) in POSITION/PASSTHROUGH. The streamed lane only moves an energised axis: an IDLE hand ignores every `set_input_pos` the lane puts on the wire and is INDISTINGUISHABLE from a perfect hold (2026-09-03: T-H1 "passed" de-energised; T-H2a/T-H2b then aborted on the driver's deviation belt with the encoder dead flat under a ramping command). **With the launch DOWN the only clean way there is the driver's `--close-loop`** — it is on every streaming command below, and the driver now refuses to arm unless the axis-6 DIAGNOSTIC reads CLOSED_LOOP (and fails closed if no diagnostic arrives). Two non-routes, so nobody re-derives them at the bench: (a) arming via the launch and *then* shutting it down does NOT work — shutdown's stow `DEACTIVATE` idles the hand explicitly on the way out (`teensy_bridge_node.py:6327` and `:6365`); (b) the legacy `hand_ops` bring-up preamble is structurally unavailable once the latch is STREAMED — `hand_ops.cpp:84-88` returns `ERR_HAND_SOURCE` before any CAN side-effect, by design (two masters on one CAN id is the class the latch exists to make impossible). | |
| 11c | **The console session MUST be captured to a file for the WHOLE sitting** — started at row 6, still running now, and left running to row 21. Every counter this ladder judges (`lead`, `dev_max`, `dev_over`, `dev_last`, `dev_cmd`, `dev_fb`, `unseen`, `stale`, `discard_legacy`, `sent`, plus `src=`/`lane=`/`guard=`, and `[cantx]`'s `defer jb` / `txq jb`) exists ONLY on the USB serial console — `leg_interp.cpp:1194-1210`, printed by the `.ino`'s 1 Hz `interp_hand7_diag_step()` (`Teensy_code_canbridge.ino:394`). **None of it reaches a CSV, a UDP uplink or a bag**, so an uncaptured console is an unrecoverable sitting: the 2026-09-04 run captured none of it, and the observe-first residual baseline that the SECOND sitting arms the 2.5 rev guard against (§ Observe-then-arm step 2) therefore does not exist. Verified on this box (PlatformIO Core 6.1.19, `script` from util-linux): `mkdir -p /home/jetson/Desktop/Jugglebot/temp/logs && script -f /home/jetson/Desktop/Jugglebot/temp/logs/hand7_console_$(date +%Y%m%d_%H%M%S).log -c "pio device monitor -d /home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge -e teensy41"`. `script -f` flushes on every write (the log survives an abnormal end) and the monitor stays fully INTERACTIVE, which row 18 needs for `hand7 arm` / `hand7 observe` and which a `pio device monitor ... > file` redirect would cost. PlatformIO's own `-f log2file` filter also exists in 6.1.19, but the command body runs under `fs.cd(project_dir)`, so its log lands at `Teensy_code_canbridge/logs/device-monitor-*.log` — inside the firmware source tree, against this repo's temp/-only runtime-artifact rule. **Between every ladder row below, paste the current `[hand7]` line into the capture** (press Enter to mark the boundary, or type the row number): counters are boot-cumulative, so a row's value is the DIFFERENCE across it and an un-bracketed row has no value at all. | |

## The ladder — T-H1..T-H4

Driver: `tests/hardware/hand_stream_bench.py` (this phase; modeled on
`teensy_setpoint_bench.py`). Every stage: frames through the REAL
`SetpointPump`, leg lanes pinned at live encoders, stream-then-arm, disarm on
any fault. Run stages in this order; between stages read and log the `[hand7]`
line (counters are boot-cumulative — difference across a stage).

| # | Test | Command | Pass criteria |
|---|---|---|---|
| 12 | Source switch + refusals (first half of **T-H4**) | **Establish LEGACY first — `--source-only legacy` — and confirm `[hand7] src=LEGACY`.** Only then, with the hand mid-stroke or moving: `--source-only streamed` must be REFUSED (settle gate). Then with the hand parked + settled: `--source-only streamed` must succeed and `[hand7]` flips to `src=STREAMED`. | The LEGACY-first step is not ceremony: `hand_source.cpp:56` returns `RpcStatus::OK` on `source == s_source` **before** the mpc_active, telemetry-freshness and settle gates run (idempotent re-assert, so a host retry after a lost response cannot wedge on gates it already passed). Asserting `streamed` while the latch ALREADY reads STREAMED therefore returns OK with no gate executed, and the refusal half of this row proves nothing — which is what happened on 2026-09-04: the 09:50 hold armed with the hand at 3.70080 rev, in neither the retract band nor catch-prime, so the settle gate demonstrably never ran. Refusal is loud (`ERR_REJECTED` + the driver's gate diagnosis — the firmware returns one opaque status by design; the driver disambiguates from its own caches and prints the likely gate(s)); success flips HeartbeatT2J bit 6. Both transitions land in row 11c's capture. |
| 13 | **T-H1** streamed hold, 10 min | `python tests/hardware/hand_stream_bench.py --stage hold --duration 600 --close-loop` | **FIRST, the positive evidence — the driver's `post-bring-up: axis_state=8 (CLOSED_LOOP=8) … the hand is ENERGISED and holding` line.** Without it this row proves NOTHING: a hold stage cannot distinguish "holding in closed loop" from "unpowered and undisturbed", and a de-energised hand passes every criterion below perfectly (exactly what happened 2026-09-03 — see row 11b). Then: ZERO hand motion (by eye + encoder flat in the CSV), zero guard trips, `[hand7]` **deltas** across the row `lead=0 dev_over=0 unseen=0 stale=0` (the absolutes are boot-cumulative and, for `lead`/`dev_over`, gated wrong — see Safety), `dev_max` small (log it — this is the observe-first residual baseline; on a post-flash board the first `[hand7]` line is the true zero), `defer jb` increment 0, `txq jb` unmoved, drop-episode rate at the 2026-08-30 baseline **read from the driver's CacheDiag CSV** (`*_cachediag.csv` beside the stage CSV — the driver subscribes 0x91 and logs 1 Hz windowed `enc_frames` deficits per the headroom runbook's row-21 recipe; an episode = a window deficit past −20, clean baseline −3.6 ± 10.3, and the driver's close-out line prints windows/episodes/worst). **The 600 s is not optional** — this row was run at 120 s on 2026-09-03 and again on 2026-09-04, and a 5x-short sample is not a short version of this test, it is a different and weaker one: `dev_max` is an EXTREME-VALUE statistic (the worst single 500 Hz tick since boot) and it is the very number the second sitting arms the 2.5 rev guard against, so a short window systematically under-estimates the tail the guard has to bound; the drop-episode criterion needs windows to have a rate at all (128 one-second windows vs ~600); and `defer jb` / `txq jb` are RARE-EVENT counters whose `0` is only as strong as the exposure that produced it. The `[hand7]` delta across this row comes from **row 11c's console capture** — that file, not the CSVs, is this row's evidence of record for every counter named above. |
| 14 | **T-H2a** slow triangle | `--stage triangle --tri-span 2.0 --tri-speed 0.5 --duration 60 --close-loop` | Smooth 0.5 rev/s motion over 2 rev; tracking error steady; `lead` delta across the row = 0 (Safety, amended 2026-09-04); `dev_max` logged — the `[hand7]` delta across this row is read from **row 11c's console capture**, this row's evidence of record. |
| 15 | **T-H2b** legacy-stroke replay, 3 m/s | `--stage stroke --event-vel 3.0 --close-loop` (hand parked within the ±0.10 rev settle band of 0 first — the driver refuses outside it) | Stroke completes; **time-aligned reconstruction ≤ 3.25 mm + wall-sync noise only** — the driver evaluates the analytic AT each echo's `t_bridge_us` (bridge wall, disciplined by the driver's own TimeOfDayServer; only TIME_SYNCED echoes scored), and since the 2026-09-03 audit fix the firmware AGE-CORRECTS that stamp to the interp tick that wrote the echoed bytes (the emit used to trail the sample by 0-2 ms — a one-sided up-to-~6 mm skew at the ~95 rev/s plateau, now corrected at source). The residual error budget is therefore wall-sync noise alone, ~3 mm per ms of sync error at the plateau (RTT runs 1-3 ms on this link ⇒ expect a few mm of noise on top of 3.25) — judge the driver's max + CSV `recon_mm` trend TOGETHER with `[hand7] dev_max`, never one tick; a NOW-comparison (pre-2026-09-02 driver) smeared 30-100 mm of echo latency over this bar and is not a valid read. `lead` **delta across the stage** = 0 (**non-zero delta = hard abort**; the absolute is uninformative until the FW 18 gate fix — Safety, amended 2026-09-04); no guard trip. **Before calling this row passed, compute the per-sample `recon_mm / analytic_velocity` ratio** — an equivalent TIME error per scored sample. Discriminator: if the ratio is roughly CONSTANT across the stroke's velocity range it is a clock / age-correction bias and is **not** passable as the zero-mean wall-sync noise this bar allows; if it SCALES with velocity it is the Hermite reconstruction term the 3.25 mm bound was derived for. The 2026-09-04 run failed that test — worst sample 6.29 mm against 3.25 mm, and the 8 scored MOVING samples span a 6.0x velocity range (15.78 → 94.85 rev/s) at a constant one-sided **2.17 ± 0.09 ms**, which is a fixed timestamp offset, not noise. Two candidate causes, and this row cannot separate them alone: a residual host↔bridge wall offset the driver's own `TimeOfDayServer` has not removed, or the firmware `interp_last_tick_us()` age correction (`telemetry.cpp:275-277`) UNDER-correcting. Separating them needs the console `[hand7] dev_max` **plus** a TimeOfDay offset/RTT sample captured in the SAME run (row 11c's capture). **Window to the moving samples**: `recon_mm` is populated on all 1200 rows but carries information on only ~8 of them — the stroke is ~205 ms of a 30 s stage — so a median or p90 of the whole column reads 0.000 and is meaningless. |
| 16 | **T-H3a** host step refusal | `--stage step --step-rev 6.0 --close-loop` | The PUMP refuses the oversized knot host-side (driver prints `pump refused=True`, reject counted); NOTHING reaches the wire for that knot; the hold continues unbroken; firmware counters unmoved — "unmoved" is a delta, read from **row 11c's console capture** either side of this row. |
| 17 | **T-H3c** gap re-entry — **run BEFORE any `hand7 arm`, ever** (Phase 2 review carry-in) | `--stage gap --gap-pre 3 --gap-s 1.0 --gap-delta 1.0 --close-loop` | During the gap the lane DECAYS (hand comes smoothly to rest — the normative falling edge; watch `[hand7] lane=active` persist while vel dies); re-entry is ONE bounded ~32 mm catch-up move; `dev_over` may increment transiently — log it, that is exactly the residual class the observe-first year exists to see; no E-STOP (guard is observe). **POSITIVE EVIDENCE REQUIRED — the echo must be observed to MOVE at the re-entry**, and `[hand7] sent=` must climb across the stage (row 11c's capture). The driver now proves this itself: it aborts if the hand echo has not moved within 0.25 s of the re-entry knot. That check exists because a 2026-09-04 driver bug dropped `hand_override` on the way to `frame()`, so this stage streamed a constant Hold while its CSV logged a 1.0 rev re-entry that never happened (echo bit-frozen at 0.00018 rev for all 1200 rows, 0.017 mm of encoder excursion) — and nothing else in the loop could see it, because the phantom's deviation sits under this stage's own belt. A run that reaches its close-out line has therefore PROVEN the re-entry reached the wire. |
| 17b | **T-H3d** moving gap — the falling-edge **DECAY** (**CARRIED IN to sitting three**; run BEFORE any `hand7 arm`, immediately after row 17) | `python tests/hardware/hand_stream_bench.py --stage moving_gap --duration 10 --close-loop` (hand parked low enough that `start + 2.0 rev` clears 10.6 — the same envelope check as row 14, and the driver refuses otherwise) | **WHY THIS ROW EXISTS.** FW 17 shipped a NORMATIVE falling-edge rule and row 17 could not test it: the `gap` stage rides a Hold, so on 2026-09-04 the hand was at REST when the gap opened (max \|vel\| 0.073 rev/s, 6.6 µm of encoder span) and the decay half measured nothing. This stage takes the gap **mid-ramp of the row-14 triangle, at the full 0.5 rev/s**, which is what the unified cycle does once per throw-catch. **What runs:** 2.0 s of hand-bearing triangle, then 9 consecutive frames with the hand channel withheld (a **250 ms** firmware gap against the **135 ms** wind-down = `SEG_T` 25 + `MAX_EXTRAP_DT_S` 50 + `EXTRAP_DECAY_DT_S` 60 ms, so the decay COMPLETES with ~115 ms of frozen hold left to watch), then the stream resumes on the triangle's own clock — no override, so this stage cannot reproduce row 17's dropped-`hand_override` class by construction. **What to watch, by eye:** the hand keeps moving for a beat past the falling edge, comes smoothly to a stop, sits still for ~0.1 s, then takes ONE small step (**2.3 mm**) and resumes the ramp. **The prediction under test, stated falsifiably** (`leg_interp.cpp:701-748` with `accel == jerk == 0`, which the pump guarantees): the firmware's own target coasts **+0.0525 rev = 1.66 mm** past the last knot and then FREEZES — against **+0.0125 rev** if it held the Mode-1 endpoint (the mode the rule forbids) and **+0.1125 rev** if it never wound down. **Pass = the driver's own five criteria, all PASS or a SKIP whose reason you accept** (it prints them: G1 the three-way coast discriminator, G2 the target frozen past the wind-down, G3 the encoder tracking the DECAYED target within 0.10 rev, G4 the re-entry step bounded at 0.25 rev, G5 `lead_clamp_mask` bit 6 never sampled set) **plus** `[hand7]` **deltas** across the row — `lead` and `dev_over` 0, `sent` climbing, `discard_legacy` unmoved — bracketed either side from row 11c's capture, never read as absolutes (Safety). **ABORT conditions:** any driver ABORT is the finding, not a nuisance — in particular a re-entry step past `--reentry-max-rev` is **not commanded** (the driver refuses before sending that knot) and means the lane did not wind down as the firmware says it must; **do not widen the bar, log it.** Note the guard is still OBSERVE at this point, so no E-STOP can fire: `MAX_DEVIATION_HAND_REV` (2.5) only counts an exceed tick and `MAX_LEAD_HAND_REV` (2.0) would silently absorb a bad step — the driver's 0.25 rev refusal is the ONLY layer that says no, which is why it is the tightest number in the chain. Artifact: the stage CSV gains a `pred_rev` column (the modelled firmware target) and reads `withheld` — not a number — in `cmd_rev` on every tick whose frame carried no hand channel. |
| 18 | **T-H3b** held-rotor deviation (operator) | Re-run `--stage triangle --tri-speed 0.2 --tri-span 0.5 --close-loop` while the operator gently restrains the slider (gloved, low speed). **Launch stays DOWN** — every step here is a driver/console step. | `[hand7] dev_last/dev_max` grows with the restraint and `dev_over` counts once past 2.5 rev — the tick verdict works; because the guard is OBSERVE the stream continues (expected!); release → tracking resumes. Log the observe-half residuals from `[hand7] dev_max`/`dev_cmd`/`dev_fb` — these are **boot-cumulative** (the worst tick since boot, NOT a trip trio; difference across the block like every `[hand7]` counter). The block boundaries — and the restrained-rotor curve itself — exist ONLY in **row 11c's console capture**; this row has no other record. Then, for the latch machinery itself: `hand7 arm` on the console, repeat the restraint briefly → MAX_DEVIATION E-STOP latches. **The trip trio reads from the driver's abort line** — on the fault the driver prints the HeartbeatT2J latch snapshot (`max_dev latch: leg/dev/u0/enc`; leg 6 = the hand) — or from `/link_status` later, with the launch up (`guard_fault_leg: 6`); no console surface carries it. Since the 2026-09-02 fix the latch freezes the trip's OWN exceed-tick trio, not the boot-cumulative `dev_max`; output gated. **Recover with the driver's `--clear-errors` verb** (`python tests/hardware/hand_stream_bench.py --clear-errors` — the launch is down; `/clear_errors` is the launch-up equivalent), then re-arm the driver (`--stage hold --duration 10 --close-loop` — a guard E-STOP leaves the hand de-energised, so the re-arm needs the bring-up too) and **observe the recovery slew on that re-arm**: the output-enable edge walks the hand back toward the streamed command at ≤ 1 rev/s on the hand's own convergence clock while the legs resume full-FF streaming immediately (2026-09-02 per-axis-group fix). Then **`hand7 observe` immediately** (see § Observe-then-arm: leaving it armed IS the second sitting's decision, not this one's). |
| 19 | **T-H4** interlock, both directions | **Launch still DOWN** (the driver is the sole UDP owner — running it while the launch is up violates row 11's single-owner rule and EADDRINUSEs on the driver's own bind): (b) `--source-only legacy`, then `--stage hold --duration 3 --no-source-switch --close-loop` — a v6 hand-bearing stream against the LEGACY latch (the hand must be ENERGISED for "does not move" to mean anything: an IDLE hand cannot move whatever the latch says, so the discard would prove nothing) → `[hand7] discard_legacy` increments, hand does NOT move, legs unaffected. Stop the driver. **Then bring the launch up** for the service-verb halves: (a) `ros2 service call /set_hand_source std_srvs/srv/SetBool "{data: true}"` (the launch-up PRIMARY verb — the driver's `--source-only` is its launch-down equivalent, never both at once), then `ros2 service call /set_hand_traj_cmd ...` → the ack must read **ERR_HAND_SOURCE** and `hand_traj_acks` count it; the hand must NOT move. (c) While ARMED (`/set_setpoint_output` true, trajectory_node streaming — mpc_active=1): `/set_hand_source` must be REFUSED. Close: disarm, `/set_hand_source {data: false}` → LEGACY. | All three refusals loud and attributed; zero hand motion in every refusal case; `discard_legacy` delta exactly the (b) run's frame count — that delta is read from **row 11c's console capture**, this row's evidence of record. |
| 20 | Close-out validity sweep | `/link_status` + console | `can3_errors` all-zero, `leak_* ≡ 0`, `interp_deadline_misses` 0, `interp_max_jitter_us` ≤ the 2026-08-30 envelope (~2-3 µs), `latency_monitor` OK, `tx_deferred` 0. |
| 21 | Close-out state | `--source-only legacy`; confirm `[hand7] src=LEGACY guard=observe`; hand parked at rest. The robot leaves the sitting on the LEGACY path — one `hand_source` switch away, exactly as Phase 5 expects. | |

## Observe-then-arm (Phase 0 Open Question 4 — the guard must not stay observe forever)

The `MAX_DEVIATION_HAND_REV` trip ships **observe-first** because a guard that
has never seen the real residual distribution nuisance-trips or under-trips.
The sequencing is **named and scheduled, not open-ended**:

1. **This sitting (first)**: everything runs `guard=observe`. The deliverable
   is the residual record — `[hand7] dev_max` after every stage, plus the
   T-H3b restrained-rotor curve. Row 18 exercises the LATCH machinery once,
   under operator control, then returns to observe.
2. **The second sitting — the arming step, by name**: at the start of the
   *next* hand sitting (the first UH-1/UH-2 rung of Phase 5, or a dedicated
   re-run of rows 13-15), the operator reviews this sitting's dev_max record
   against the 2.5 rev bound and runs **`hand7 arm` as a logged step**. From
   then on armed is the sitting default; boot returns to observe (safe), so
   `hand7 arm` joins the session-start checklist exactly like session limits.
   **An observe-only guard left in place indefinitely is a guard that does not
   exist** — if sitting two happens without the arming step, that omission is
   itself a finding to log.
3. Runtime `hand7 arm`/`observe` needs no reflash; the boot default stays
   observe until a later FW bump flips it — a decision for after the armed
   record exists.

**Status note, 2026-09-04 — the arming step is now SITTING THREE's, and this is
logged, not lost.** Sittings one (2026-09-03) and two (2026-09-04) both ran
`guard=observe` start to finish; the console capture confirms it directly
(`guard=observe` on all 264 `[hand7]` blocks, `hand7 arm` never issued). Step 2
above was **not** skipped silently: row 18's arming half stopped on motor
temperature under restraint, which was the correct call — the test is a
deliberate stall by construction — and § Results records it as CARRIED. Step 2
therefore reads *"the NEXT hand sitting"*, i.e. **sitting three**, and its
prerequisites now exist in a form step 2 did not anticipate:

- **The observe-first residual record is a CSV-derived substitute, not a
  `[hand7]` read** — `dev_max` **0.0013 rev = 0.041 mm** over the 600 s hold,
  flat with a statistically insignificant trend. The capture's own `dev_max`
  (10.9794) is poisoned boot-cumulative history from an overnight aborted
  stage and **must not be used for this decision** (Safety, and the sitting-two
  entry § "The first console capture ever taken").
- **Read § Results' "Before you type `hand7 arm`" box first.** The governing
  number is the 14:47 stroke's **1.9847 rev of worst `|cmd − enc|` — 79.4 % of
  the 2.5 rev bound**, not the residual baseline. Arm expecting a trip, and
  expect an aborted stroke to fire the guard by design.

## Artifacts

- Bag from row 8/19 (`record:=true`); driver CSVs under `temp/logs/hand_stream_*.csv`.
- **The row 11c console capture file** (`temp/logs/hand7_console_*.log`) — not
  scrollback, a file. `[hand7]` is console-only, so this is the sitting's
  primary deliverable: the 2026-08-30 sitting lost rows 19-20 by not capturing
  it, and the 2026-09-04 run lost the entire observe-first residual baseline
  the same way. Do not repeat that. **A capture taken AFTER the ladder is not a
  substitute** — `temp/logs/hand7_console_20260904_153911.log` exists and gave
  rows 12 and 21 their verdicts plus the counter-gating find, but every
  `[hand7]` counter in it is bit-identical across all 264 blocks, so rows 13-19
  got **no delta at all** from it.
- Results go in this file's § Results, verdicts to the phase logbook entry.
  **§ Results is filled in for the 2026-09-03 / 2026-09-04 sittings** — append
  sitting three's rows there rather than overwriting.

## Results

This ladder ran across **two sittings**, and the owner declared **Phase 3
COMPLETE on 2026-09-04** on the record below.

**Canonical records — read these before sitting three, not this table:**

- Sitting one (2026-09-03 flash + halted ladder):
  `logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md`
- **Sitting two (2026-09-04, the ladder proper):
  `logbook/2026-09-04-fw17-hand-ladder-sitting-two.md`** — the canonical
  verdict record, including the findings that arrived *after* the sitting —
  the `[hand7]` counter forensics, the arming-decision number, and the three
  rows (12, 19(c), 21) they closed.

**Preconditions.** Rows 1–11c all discharged. The lockstep flash landed
2026-09-03 (row 5; row 4's build-vs-flash trap is what cost sitting one — rows
4/5 are relabelled as a result). Row 11b (`--close-loop`) and row 11c (the
captured console) were both **written because of these two sittings**, so
sitting three is the first to run the ladder with them in place from row 6.

| Row | Test | Verdict | Note |
|---|---|---|---|
| 12 | Source switch + refusals | **PASS** | Three requests, each a genuine state CHANGE (so no idempotent short-circuit): STREAMED→LEGACY in band OK; LEGACY→STREAMED at +0.9999 rev **REFUSED** with the driver's gate diagnosis naming both bands; LEGACY→STREAMED in band OK. |
| 13 | **T-H1** streamed hold, 10 min | **PASS** | The real 600 s. `dev_max` **0.0013 rev = 0.041 mm**, flat over the full 10 min; `echo_rev ≡ cmd_rev` bit-identical on all 24 000 rows. |
| 14 | **T-H2a** slow triangle | **PASS** | Tracking error flat to 0.8 % across six 10 s blocks, worst 1.92 mm. Caveat recorded: a real ~6.3 Hz ±0.73 mm velocity ripple. |
| 15 | **T-H2b** legacy-stroke replay | **mechanism PASS; two open findings** | Every scored afternoon sample ≤ 1.85 mm. Open: a real tracking degradation (0.5387 → 1.2348 rev against a 1.5 rev belt) and a commanded 5.31 mm firmware-target overshoot. **Read the entry's finding (3) before arming** (see the box below). |
| 16 | **T-H3a** host step refusal | **PASS** | `pump refused=True reason='hand step 6.0000 rev > 5.0 limit'`, twice; the 5.0 rev boundary accepted (the gate is `>`). First live confirmation of 200 rev/s × 0.025 s = 5.0 rev. |
| 17 | **T-H3c** gap re-entry | **re-entry half PASS; decay half CARRIED** | 31.71 mm in one move, peak 19.86 rev/s, overshoot 0.26 %, echo moved within 25 ms. The **decay half is structurally untestable with the current stages** — the gap stage rides a Hold, so the hand is at rest when the gap opens; it needs a new stage that takes the gap **while the hand is moving**. |
| 17b | **T-H3d** moving gap (the decay half) | **NOT RUN — new for sitting three** | Written 2026-09-04 to close row 17's carried decay half: no stage before it could take a gap while the hand was moving, so FW 17's normative falling-edge rule has never been observed on hardware. Nothing to record until sitting three runs it. |
| 18 | **T-H3b** held-rotor deviation | **observe half PASS; arming half CARRIED** | Clean grow-then-recover curve, peak 0.1965 rev = 7.9 % of the 2.5 rev threshold, so the tick verdict was never exercised. The arming half was **not run: motor temperature rose very quickly under restraint and the operator stopped — correctly.** Holding the rotor to 2.5 rev of error at `pos_gain` 35 saturates current by construction; the test is a deliberate thermal event, not a 30-second step. It is also sitting three's step by this doc's own § Observe-then-arm. |
| 19(a) | `set_hand_traj_cmd` under STREAMED | **PASS** | `hand_traj_acks = calls=1 ok=0 fail_teensy=1` with all five wire-visible `hand_ops` exit counters at 0 — uniquely `ERR_HAND_SOURCE`'s fingerprint. Axis 6 stayed IDLE for 20 s after, proving the refusal preceded any CAN side-effect. |
| 19(b) | v6 hand stream against LEGACY | **CARRIED — not run** | Mis-invoked as one command; `--source-only` switches the latch and returns immediately, so the hold half never ran. `discard_legacy` is console-only, so this row has no bag surface even in principle — it needs row 11c's capture. |
| 19(c) | `/set_hand_source` while ARMED | **PASS** | Closed post-sitting on the 15:41 bag: over the 45.6 s armed window the hand held `pos_meas` inside **[−0.000219, +0.000271] rev** (0.27 % of the 0.3 rev band) and the firmware's own `hand_settled_at_rest()` predicate passes **0 failures / 4580 armed samples**, so the settle gate would have passed and only the arming gate can have refused — and `mpc_active` (`hand_source.cpp:60`) short-circuits above the settle gate (`:74`) regardless. `hand_source` STREAMED in 459/459 armed samples: the latch did not move. Caveat: rosbag2 records no services and nothing is logged, so the `ERR_REJECTED` byte is inferred from the latch not moving, not captured. |
| 20 | Close-out validity sweep | **PASS** | `leak_*`/`leak_hwm_*` 0 across 401 `/ring_diag`; `interp_deadline_misses` 0 and `interp_max_jitter_us` max 2 µs across 402 `/profile`; `latency_monitor` OK 4029/4029; `tx_deferred` jb delta 0; `bridge_fw_version` **17 (proto 6)**; `lead_clamp_mask` 0 throughout, including bit 6. Console corroborates: jb bus `err=0 rec=0 tec=0 defer=0 txq=0`, **zero leg and zero hand deferrals**, heap flat. |
| 21 | Close-out state | **PASS** | The console capture's final line reads `[hand7] src=LEGACY guard=observe lane=idle`, and `guard=observe` holds on all 264 blocks — `hand7 arm` was never issued. The robot left the sitting on the LEGACY path. |

### Carried to sitting three

1. **Row 18's arming half** (`hand7 arm` + restrain to the E-STOP). This is the
   step § Observe-then-arm names as the next sitting's, and the operator's
   thermal stop was the correct call, not a skip.
2. **Row 19(b)** — never run; needs the LEGACY latch established, *then* a
   separate hold command, with the hand energised.
3. **Row 17's decay half** — needed a NEW stage that takes the gap while the
   hand is moving; no stage in the sitting-two ladder could exercise the
   normative falling-edge rule at all. **That stage now exists**: `moving_gap`,
   the ladder's **row 17b**, added 2026-09-04. Run it immediately after row 17
   and before any `hand7 arm`; its verdict is the first hardware observation of
   the rule Phase 3 shipped.
4. **A bracketed row-11c capture.** A console file finally exists
   (`temp/logs/hand7_console_20260904_153911.log`) but it was opened at 15:39,
   after the last block closed at 15:09 — so it carries state and history and
   **no per-row delta**. Sitting three is the first that can produce the
   observe-first residual record from the counters themselves.

> ### Before you type `hand7 arm` — the number that governs the decision
>
> It is **not** the `dev_max=10.9794` on the console line. That is benign
> boot-cumulative history from a lane left latched across an overnight abort,
> fully explained in the sitting-two entry, and it **cannot trip an arm** —
> `fault_machine.cpp:409-411` samples the exceed-tick delta unconditionally,
> above the armed gate, precisely so a disarmed window cannot bank one.
>
> The number is the **2026-09-04 14:47 stroke's worst `|cmd − enc|`:
> 1.9847 rev = 99.2 % of `MAX_LEAD_HAND_REV` (2.0) and 79.4 % of
> `MAX_DEVIATION_HAND_REV` (2.5)** — against 1.2067 rev on the 09:56 stroke at
> the same `--event-vel`, with the encoder overshooting to 10.4693 rev against
> the 10.8 rev hard stop. (Caveat: the 40 Hz CSV's raw residual is coarser than
> the firmware's velocity-compensated one, and undersamples a 500 Hz quantity.)
>
> **Armed, expect the 2.5 rev band to be tight on strokes — budget for a
> trip**, and know that an **aborted** stroke will fire the guard by design
> (the lane decays; the residual passes 2.5 rev within ~30 ms of the last knot)
> and needs `CLEAR_ERRORS`. Row 18's recovery path is already written for that.
