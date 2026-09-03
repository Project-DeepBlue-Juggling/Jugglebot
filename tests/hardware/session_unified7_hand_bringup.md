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
  commanded start** (hold/triangle/step/gap capture the live encoder; the
  stroke stage refuses unless the hand is inside the ±0.10 rev settle band of
  0). Any residual is walked by the firmware recovery slew at ≤ 1 rev/s —
  arming is a bounded, near-no-op step.
- **Leg lanes are pinned holds at the live leg encoders** in every stage; legs
  may stay IDLE (streamed holds are latched, not acted on) or CLOSED_LOOP
  (zero-deviation no-op). No platform motion is commanded or needed.
- The deviation guard ships **observe-first** and stays observe this sitting.
  `hand7 arm` is the SECOND sitting's step — see § Observe-then-arm.
- **The hand lead-duty read is a hard rule**: `[hand7] lead=` (and
  `lead_clamp_mask` bit 6 / 0x40 on `/link_status` when the launch is up) must
  stay 0 through every stage. Non-zero lead duty during a stroke = the clamp is
  shaping the command = **hard-abort the sitting** (the FW 14 arc's
  clamp-duty-0 precedent — a clamped stream is the S1/S2 silent-stall
  mechanism, not a curiosity).
- **ABORT** on: any guard E-STOP, any hand motion in a hold stage, any nonzero
  `can3_errors` wire-error field, `leak_*` ≠ 0, a lost link, or lead duty ≠ 0.

## Preconditions (once, before the flash)

| # | Step | Done |
|---|---|---|
| 1 | `./run_tests.sh --full` **green** — standing rule before any hardware sitting (this phase touched firmware natives + `controller/`-adjacent surfaces; `--full` is mandatory). Record the (date, command, result) triple in Results. | |
| 2 | `git fetch && git status -sb` clean, `origin` not ahead; confirm the tree contains `fb972c3` (v6 host) and this phase's FW 17 commit. | |
| 3 | Hand mechanically clear: no ball, cords clear of the slider, hand parked at the bottom rest (post-homing ≈ −0.1..0 rev). | |
| 4 | **BUILD ONLY — this row does NOT touch the board.** Clean build (the `extra_script.py` incremental hazard from the 4→5 bump applies to 5→6 identically): `cd ros_ws/src/jugglebot/Teensy_code_canbridge && rm -rf .pio/build && pio run -e teensy41`. This compiles a hex on the Jetson; **nothing is uploaded — the Teensy keeps running whatever it was already running.** Verify the hex md5 against the build-evidence table in `logbook/2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md` (FW 17 post-audit-fix build: `2e90c43d5b6889e1059982d2e7aaf961`, 769 144 B hex — reproducible across two clean builds 2026-09-03; supersedes `7abe226b…` after the audit's echo-stamp age-correction). A different md5 means the tree moved: stop and diff. **A green build and a matching md5 are NOT a flash** — on 2026-09-03 this row's md5 verify was read as "flashed", the board stayed on FW ≤ 16 against a v6 host, and the resulting (designed, correct) total link darkness cost a whole debugging session. The flash is row 5. | |
| 5 | **THIS IS THE FLASH — the only row that changes the firmware aboard.** **Operator flashes**: close any `pio device monitor` first, then `pio run -e teensy41 -t upload`. Nothing before this row has altered the board; nothing after it works until this row is done (confirm it in row 6 — the boot banner is the receipt). | |
| 6 | Open the monitor (`pio device monitor`). Confirm the boot banner reads `jugglebot-canbridge v17` and the 1 Hz `[hand7]` line appears with `src=LEGACY guard=observe lane=idle` and every counter 0. **LEGACY at boot is the contract** — a board that boots STREAMED is wrong; stop. | |
| 7 | `colcon build --packages-select jugglebot` + `source install/setup.bash` (the bridge node's hand_source service + link_status row are new). | |
| 8 | Launch with recording: `ros2 launch jugglebot jugglebot_launch.py record:=true`. Confirm `/link_status`: `bridge_fw_version` **`17 (proto 6)`**, `BRIDGE_FW_CHECK: OK`, `hand_source: LEGACY_STROKE`, `latency_monitor` OK, `uptime_ms` fresh. The link being ALIVE at all is the v6 lockstep succeeding. | |
| 9 | FW 16 carry-forward acceptance: hand ball-sensor poller cadence (~50 Hz claim, console-side), `[cantx]` census present, all columns 0. | |
| 10 | Legacy regression smoke (T-R3 hardware half): one `smooth_move_hand` to 1.0 rev and back via the launch (`ros2 service call /smooth_move_hand ...`). The LEGACY path must work exactly as before — FW 17 changes nothing while the latch is LEGACY. | |
| 11 | **Shut the launch down** for the bench ladder — `hand_stream_bench.py` is the sole owner of the UDP link (single-owner rule). The console monitor stays open; `[hand7]` is a primary observable. | |
| 11b | **The hand must be ENERGISED for every streaming row below** — `axis_state == CLOSED_LOOP` (8) in POSITION/PASSTHROUGH. The streamed lane only moves an energised axis: an IDLE hand ignores every `set_input_pos` the lane puts on the wire and is INDISTINGUISHABLE from a perfect hold (2026-09-03: T-H1 "passed" de-energised; T-H2a/T-H2b then aborted on the driver's deviation belt with the encoder dead flat under a ramping command). **With the launch DOWN the only clean way there is the driver's `--close-loop`** — it is on every streaming command below, and the driver now refuses to arm unless the axis-6 DIAGNOSTIC reads CLOSED_LOOP (and fails closed if no diagnostic arrives). Two non-routes, so nobody re-derives them at the bench: (a) arming via the launch and *then* shutting it down does NOT work — shutdown's stow `DEACTIVATE` idles the hand explicitly on the way out (`teensy_bridge_node.py:6327` and `:6365`); (b) the legacy `hand_ops` bring-up preamble is structurally unavailable once the latch is STREAMED — `hand_ops.cpp:84-88` returns `ERR_HAND_SOURCE` before any CAN side-effect, by design (two masters on one CAN id is the class the latch exists to make impossible). | |

## The ladder — T-H1..T-H4

Driver: `tests/hardware/hand_stream_bench.py` (this phase; modeled on
`teensy_setpoint_bench.py`). Every stage: frames through the REAL
`SetpointPump`, leg lanes pinned at live encoders, stream-then-arm, disarm on
any fault. Run stages in this order; between stages read and log the `[hand7]`
line (counters are boot-cumulative — difference across a stage).

| # | Test | Command | Pass criteria |
|---|---|---|---|
| 12 | Source switch + refusals (first half of **T-H4**) | With the hand mid-stroke or moving: `--source-only streamed` must be REFUSED (settle gate). With the hand parked + settled: it must succeed and `[hand7]` flips to `src=STREAMED`. | Refusal is loud (`ERR_REJECTED` + the driver's gate diagnosis — the firmware returns one opaque status by design; the driver disambiguates from its own caches and prints the likely gate(s)); success flips HeartbeatT2J bit 6. |
| 13 | **T-H1** streamed hold, 10 min | `python tests/hardware/hand_stream_bench.py --stage hold --duration 600 --close-loop` | **FIRST, the positive evidence — the driver's `post-bring-up: axis_state=8 (CLOSED_LOOP=8) … the hand is ENERGISED and holding` line.** Without it this row proves NOTHING: a hold stage cannot distinguish "holding in closed loop" from "unpowered and undisturbed", and a de-energised hand passes every criterion below perfectly (exactly what happened 2026-09-03 — see row 11b). Then: ZERO hand motion (by eye + encoder flat in the CSV), zero guard trips, `[hand7] lead=0 dev_over=0 unseen=0 stale=0`, `dev_max` small (log it — this is the observe-first residual baseline), `defer jb` increment 0, `txq jb` unmoved, drop-episode rate at the 2026-08-30 baseline **read from the driver's CacheDiag CSV** (`*_cachediag.csv` beside the stage CSV — the driver subscribes 0x91 and logs 1 Hz windowed `enc_frames` deficits per the headroom runbook's row-21 recipe; an episode = a window deficit past −20, clean baseline −3.6 ± 10.3, and the driver's close-out line prints windows/episodes/worst). |
| 14 | **T-H2a** slow triangle | `--stage triangle --tri-span 2.0 --tri-speed 0.5 --duration 60 --close-loop` | Smooth 0.5 rev/s motion over 2 rev; tracking error steady; `lead=0`; `dev_max` logged. |
| 15 | **T-H2b** legacy-stroke replay, 3 m/s | `--stage stroke --event-vel 3.0 --close-loop` (hand parked within the ±0.10 rev settle band of 0 first — the driver refuses outside it) | Stroke completes; **time-aligned reconstruction ≤ 3.25 mm + wall-sync noise only** — the driver evaluates the analytic AT each echo's `t_bridge_us` (bridge wall, disciplined by the driver's own TimeOfDayServer; only TIME_SYNCED echoes scored), and since the 2026-09-03 audit fix the firmware AGE-CORRECTS that stamp to the interp tick that wrote the echoed bytes (the emit used to trail the sample by 0-2 ms — a one-sided up-to-~6 mm skew at the ~95 rev/s plateau, now corrected at source). The residual error budget is therefore wall-sync noise alone, ~3 mm per ms of sync error at the plateau (RTT runs 1-3 ms on this link ⇒ expect a few mm of noise on top of 3.25) — judge the driver's max + CSV `recon_mm` trend TOGETHER with `[hand7] dev_max`, never one tick; a NOW-comparison (pre-2026-09-02 driver) smeared 30-100 mm of echo latency over this bar and is not a valid read. `lead=0` (**non-zero = hard abort**); no guard trip. |
| 16 | **T-H3a** host step refusal | `--stage step --step-rev 6.0 --close-loop` | The PUMP refuses the oversized knot host-side (driver prints `pump refused=True`, reject counted); NOTHING reaches the wire for that knot; the hold continues unbroken; firmware counters unmoved. |
| 17 | **T-H3c** gap re-entry — **run BEFORE any `hand7 arm`, ever** (Phase 2 review carry-in) | `--stage gap --gap-pre 3 --gap-s 1.0 --gap-delta 1.0 --close-loop` | During the gap the lane DECAYS (hand comes smoothly to rest — the normative falling edge; watch `[hand7] lane=active` persist while vel dies); re-entry is ONE bounded ~32 mm catch-up move; `dev_over` may increment transiently — log it, that is exactly the residual class the observe-first year exists to see; no E-STOP (guard is observe). |
| 18 | **T-H3b** held-rotor deviation (operator) | Re-run `--stage triangle --tri-speed 0.2 --tri-span 0.5 --close-loop` while the operator gently restrains the slider (gloved, low speed). **Launch stays DOWN** — every step here is a driver/console step. | `[hand7] dev_last/dev_max` grows with the restraint and `dev_over` counts once past 2.5 rev — the tick verdict works; because the guard is OBSERVE the stream continues (expected!); release → tracking resumes. Log the observe-half residuals from `[hand7] dev_max`/`dev_cmd`/`dev_fb` — these are **boot-cumulative** (the worst tick since boot, NOT a trip trio; difference across the block like every `[hand7]` counter). Then, for the latch machinery itself: `hand7 arm` on the console, repeat the restraint briefly → MAX_DEVIATION E-STOP latches. **The trip trio reads from the driver's abort line** — on the fault the driver prints the HeartbeatT2J latch snapshot (`max_dev latch: leg/dev/u0/enc`; leg 6 = the hand) — or from `/link_status` later, with the launch up (`guard_fault_leg: 6`); no console surface carries it. Since the 2026-09-02 fix the latch freezes the trip's OWN exceed-tick trio, not the boot-cumulative `dev_max`; output gated. **Recover with the driver's `--clear-errors` verb** (`python tests/hardware/hand_stream_bench.py --clear-errors` — the launch is down; `/clear_errors` is the launch-up equivalent), then re-arm the driver (`--stage hold --duration 10 --close-loop` — a guard E-STOP leaves the hand de-energised, so the re-arm needs the bring-up too) and **observe the recovery slew on that re-arm**: the output-enable edge walks the hand back toward the streamed command at ≤ 1 rev/s on the hand's own convergence clock while the legs resume full-FF streaming immediately (2026-09-02 per-axis-group fix). Then **`hand7 observe` immediately** (see § Observe-then-arm: leaving it armed IS the second sitting's decision, not this one's). |
| 19 | **T-H4** interlock, both directions | **Launch still DOWN** (the driver is the sole UDP owner — running it while the launch is up violates row 11's single-owner rule and EADDRINUSEs on the driver's own bind): (b) `--source-only legacy`, then `--stage hold --duration 3 --no-source-switch --close-loop` — a v6 hand-bearing stream against the LEGACY latch (the hand must be ENERGISED for "does not move" to mean anything: an IDLE hand cannot move whatever the latch says, so the discard would prove nothing) → `[hand7] discard_legacy` increments, hand does NOT move, legs unaffected. Stop the driver. **Then bring the launch up** for the service-verb halves: (a) `ros2 service call /set_hand_source std_srvs/srv/SetBool "{data: true}"` (the launch-up PRIMARY verb — the driver's `--source-only` is its launch-down equivalent, never both at once), then `ros2 service call /set_hand_traj_cmd ...` → the ack must read **ERR_HAND_SOURCE** and `hand_traj_acks` count it; the hand must NOT move. (c) While ARMED (`/set_setpoint_output` true, trajectory_node streaming — mpc_active=1): `/set_hand_source` must be REFUSED. Close: disarm, `/set_hand_source {data: false}` → LEGACY. | All three refusals loud and attributed; zero hand motion in every refusal case; `discard_legacy` delta exactly the (b) run's frame count. |
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

## Artifacts

- Bag from row 8/19 (`record:=true`); driver CSVs under `temp/logs/hand_stream_*.csv`.
- **Console scrollback for the whole sitting** — `[hand7]` is console-only (the
  2026-08-30 sitting lost rows 19-20 by not capturing it; do not repeat that).
- Results go in this file's § Results, verdicts to the phase logbook entry.

## Results

*(filled by the operator at the sitting)*
