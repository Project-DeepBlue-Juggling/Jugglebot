# 2026-09-23 ~13:53 — hand driven into the bottom end stop during guard recovery

Read-only investigation. Bag `~/Desktop/rosbags/2026-09-23_13-46-55`, launch log
`temp/logs/launch_r2gate_20260923_1346.log`, worktree HEAD `d8814ee`.
Extraction scripts + pickle: this scratchpad (`endstop.py` -> `endstop.pkl`, `p1..p7.py`).

## Headline

The recovery hand park drives axis 6 to **0.0000 rev**, which sits **0.107 rev
(3.48 mm) above the measured bottom hard stop at −0.107 rev**, at **2.5 rev/s**
with a **30 rev/s² trap decel whose braking ramp alone is 0.104 rev (3.39 mm)** —
97 % of the entire clearance. The carriage ended at **−0.1101 rev**, i.e. on the
metal. Nothing in firmware or on the Jetson has a contact/stall test, so the
firmware op ran to its 10 s timeout and the Jetson monitor to its 20 s timeout.
For that whole 20.37 s **every teensy_bridge_node topic stopped publishing**, so
the operator's instruments were frozen — the E-stop was the only working guard.

`d8814ee` did not create the defect but is the change that **first let this park
actually execute** (before it, the firmware rejected every one).

---

## 1. Timeline (ROS epoch s)

### The latch
| t | event |
|---|---|
| 630.280–630.888 | axis-6 heartbeat dropout episode, 600 ms stale (`link_status` `hb_stale_axes='6'`, `hb_stale_episode_ms_max` axis6 = 599). Bridge log 630.886: "95 encoder frames in the window vs ~60 nominal (0 co-dropped); episode 3 for this axis since boot … real leg-bus frame loss" |
| 630.3577–630.3979 | `pos_meas` frozen at +0.3977 rev (40 ms), dev peaked +1.02 |
| 630.4597–630.4985 | `pos_meas` frozen at +6.2172 rev (39 ms), dev peaked +2.64 |
| 631.3667–631.4501 | **`pos_meas` frozen bit-identically at +0.5275 rev for 84 ms, 8 samples**, while `pos_cmd` ramped +0.7550 → +3.6537; `vel_meas` frozen with it at +14.828 |
| 631.4575 | `pos_meas` jumps +0.5275 → **+4.9318 rev in 7.4 ms** = 595 rev/s implied — impossible; stale-cache catch-up |
| **631.578/.580** | **Teensy guard MAX_DEVIATION latch**: `guard_fault_leg=6`, `max_dev_value=3.9967`, `max_dev_enc=1.7426`, `max_dev_u0=5.7393` |

### The recovery and the push
| t | event |
|---|---|
| 639.206 | operator `/clear_errors` #1. Wire ARMED (`mpc_active=1`) → `_svc_clear_errors` reroutes to `_svc_recover` |
| 639.296 | trajectory_node guard descent installed, converged; CLEAR_ERRORS fired |
| **639.269 / .277 / .279** | **LAST `/robot_state` / `/hand_telemetry` / `/link_status` message.** Node goes dark for 20.37 s |
| 639.277 | last hand sample: pos **+9.9640 rev**, vel −0.03 rev/s, **iq +0.882 A**, `pos_cmd` +8.8592 (frozen stream), ball_held=1/valid=1; axis6 `active_errors=0x0`, `disarm_reason=0x0`, state 8 (CLOSED_LOOP), bus 45.2 V, fet 34.0 °C, motor 31.2 °C |
| 639.307 / .309 | "recovery park: disarming the wire first"; `mpc_active` → 0 |
| **639.393** | **ACTIVATE(axis 6) TRAP_TRAJ +9.9640 → 0.0000 rev at 2.5 rev/s** (trap acc/dec 30/30 rev/s²) |
| ≈643.5 | nominal profile arrival (9.964/2.5 + 2.5/30 ≈ 4.07 s) |
| 643.622 | operator `/clear_errors` #2 — `mpc_active` already 0, so the UNARMED branch: a bare `teensy_clear_errors()`. **No second park.** (`teensy_bridge_node.py:7049-7051`, `:7102-7105`) |
| ≈649.393 | **firmware `A_TIMEOUT_US` = `JB_OP_GENTLE_MOVE_TIMEOUT_S` = 10.0 s expires** → `abort_all(ACTIVATE_FAILED)` → `encode_set_state(6, IDLE)` (`leg_activate.cpp:171`, `:104-112`) |
| 649.4–659.43 | Jetson `ActivateMonitor` still polling an IDLE axis; cannot succeed (needs CLOSED_LOOP *and* \|pos\| ≤ 0.05). Still dark |
| **659.428** | `hand park FAILED — axis 6: timed out after 20.0s … (pos −0.110, vel +0.025)`. **−0.1101 rev vs the measured bottom stop −0.107 rev → on the metal, 0.003 rev past it** |
| 659.429–.434 | armed `/clear_errors` falls back to disarm + direct clear, then fires a **second** ACTIVATE from −0.1101 |
| 659.637 | second park complete, −0.1101 → −0.0227 rev in 0.203 s (the mechanism was free) |
| **659.641/.642/.647** | **node resumes publishing.** First hand sample: pos −0.0227, vel +0.002, **iq +3.194 A** |
| 659.646 / 659.85 | guard cleared; wire re-armed |
| 660.375+ | hand settled at **+0.0003 rev holding with iq +4.622 A** (vs +0.882 A at 9.964 rev — 5.2× the mid-stroke hold) |
| 660.480–.586 | third recovery park → "hand park not needed — hand at +0.0003 rev" |
| 671.377 | **operator E-stop** → `DC_BUS_UNDER_VOLTAGE` on all 7 axes + bb_hand; guard latches ODRIVE_FATAL |

### Comms during the 20.37 s — the link was HEALTHY, the node was not
`link_status` straddling the gap (639.279 → 659.647):
* `rx_frames` 204,992 → **212,769** = 7,777 frames in 20.37 s = **382 frames/s** (360 fps immediately before) — UDP RX ran normally throughout
* `tx_frames` 19,394 → 19,602 = 10.2/s — the 10 Hz heartbeat thread ran normally
* `bridge_link` UP, `bridge_link_lost` 0, `seq_gaps` 0, `decode_errors` 0, `crc_errors` 0, `heartbeat_age_ms` 57 at resume
* `robot_state_stale_skips` **unchanged at 9422** across the gap → `_publish_robot_state` was never even *entered* (it did not run and skip)

Scope of the blackout: **every** teensy_bridge_node topic stopped —
`/robot_state`, `/hand_telemetry`, `/link_status`, `/leg_cmd_executed`,
`/bb/heartbeat`, `/bb/odrive_diag`. Other nodes were unaffected
(`/mocap_data` max gap 0.015 s, `/rigid_body_poses` 0.018 s,
`/trajectory/status` 0.204 s). Across the **whole 553 s bag** the only
`/robot_state` gaps > 0.3 s are **20.372 s, 0.371 s and 0.341 s — the three
recovery parks.** The publish gap equals the park duration, every time.

**No bag topic records what the hand did between 639.28 and 659.64.**

---

## 2. Root cause

**The park target sits inside the braking distance of the bottom hard stop.**

| quantity | value | source |
|---|---|---|
| bottom hard stop (operator-measured 2026-09-06, slider driven onto the metal) | **−0.107 rev** | `config/hardware_config.yaml:451` |
| park target `JB_OP_HAND_ACTIVATE_POSITION_REV` | **0.0 rev** | `config/generated/hardware_config.py:141` |
| clearance | **0.107 rev = 3.48 mm** | |
| approach speed `JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS` | 2.5 rev/s (≈ 81 mm/s) | `:161`, `leg_activate.cpp:206-213` |
| trap decel `ODRIVE_TRAP_DEC_LIMIT_RPS2` | 30 rev/s² | `:122`, `hardware_config.h:137` |
| braking ramp v²/2a | **0.104 rev = 3.39 mm — 97 % of the clearance** | |
| final measured position | **−0.1101 rev** | launch log 659.428 |

`config/hardware_config.yaml:455-470` already argues this case for the **top**
stop and is why `hand_clip_margin_rev: 0.2` exists there: *"no firmware guard can
see a stall in the gap between the clip and the metal, so a zero-margin clip is a
guard that cannot fire"*. The **bottom has no equivalent margin** — the firmware
clip floor is 0.0 (`odrive_protocol.h:157-160`, clips to `[0, max]`) and the park
target *is* the clip floor (`leg_activate.cpp:63-65`: *"0.0 rev = the clip floor,
one clean revolution-count above the homed hardstop at HAND_ABS_POS_REV = −0.10"*).

Secondary cause of the **duration**: once the carriage is on metal,
\|pos − 0.0\| = 0.11 rev can satisfy neither arrival test
(firmware `JB_OP_TARGET_REACHED_POS_TOL_REV` = 0.01; Jetson
`DEFAULT_POS_TOL_REV` = 0.05), so both monitors run to their timeouts. Neither
has any contact, stall, past-target or current test.

### Hypotheses the data REFUTE
* **Concurrent second `/clear_errors` / two recover flows** — REFUTED. At 643.622
  `mpc_active` was already 0, so `_svc_clear_errors` took the unarmed branch: one
  bare `teensy_clear_errors()`, no recover, no park. The firmware would also have
  rejected a concurrent ACTIVATE (`activate_request` rejects while `s_phase != IDLE`).
* **Stale/lost hand encoder during the park** — REFUTED for the park window: 7,777
  RX frames at the normal rate, 0 seq_gaps/decode/crc errors, heartbeat_age 57 ms
  at resume. (Staleness *was* real at the latch 8 s earlier — see §5.)
* **CAN or UDP link fault** — REFUTED by the same counters; `bridge_link` UP,
  `bridge_link_lost` 0 throughout.
* **An ODrive error on axis 6** — REFUTED. `active_errors = 0x0` and
  `disarm_reason = 0x0` in every sample either side of the gap; the first axis-6
  error is `DC_BUS_UNDER_VOLTAGE` at 671.39 (the E-stop). `ActivateMonitor` fails
  immediately on non-zero `active_errors` and it did not — it timed out.
* **Current / torque saturation as the cause of the overshoot** — NOT SUPPORTED.
  Hand current limit 50 A (0.2757 N·m at Kt = 0.0055133 N·m/A); torque soft limit
  ±0.7 N·m, explicitly 2.5× the current limit so it never binds
  (`hardware_config.yaml:1679-1689`). Observed hold current at the park is 4.62 A
  = 9 % of the limit.
* **The node was blind because the Teensy went quiet** — REFUTED (counters above);
  and there is no `_recovery_park_in_progress` early-return in any publisher.

---

## 3. Why nothing stopped it for 20 s

1. **Teensy hand MAX_DEVIATION guard** (`fault_machine.cpp:467-504`) — inert by
   design: the park runs at `mpc_active = 0`, and `_park_hand_disarmed`'s own
   docstring (`teensy_bridge_node.py:6683-6690`) states *"with mpc_active=0 the
   firmware's guard terms are inert"*. Structurally blind even when armed:
   `hardware_config.yaml:461-463` — *"once the slider is jammed against the stop at
   the clip value those two AGREE — residual ~0, guard silent, motor pushing into metal."*
2. **Lead clamp `MAX_LEAD_HAND_REV`** — same gating, and `:464-465`: it *"anchors the
   setpoint TO the encoder … so it re-issues the jam rather than refusing it."*
3. **ODrive current limit (50 A)** — never approached (4.6 A). At 32.567 mm/rev it
   corresponds to ~53 N of carriage force: a motor-protection limit, not a
   mechanism-protection limit.
4. **ODrive torque soft limit (±0.7 N·m)** — 2.5× what the current limit allows;
   cannot bind.
5. **Position soft limits** — none on the hand. The only position fence is the
   firmware `clip_position` on the COMMAND (`[0, HAND_MOTOR_MAX_POSITION]`). The
   park commands exactly 0.0, so the clip is satisfied while the mechanism sits
   0.11 rev below it. A clip on the command cannot see the mechanism.
6. **Firmware ACTIVATE MONITOR** (`leg_activate.cpp:245-257`) — no contact test;
   a position miss just holds `all_done = false`. Its only bound is the 10 s
   `A_TIMEOUT_US`, and its abort action is **`set_state(IDLE)`** (`:104-112`) —
   de-energise. From mid-stroke with a ball held and a 4.6 A standing load that
   means dropping the carriage up to 10 rev (325 mm) onto the stop. That abort is
   itself a hazard.
7. **Jetson `ActivateMonitor`** (`teensy_link/activate.py:143-176`) — the ONLY
   failure paths are `active_errors != 0` and the 20 s timeout. No abort on
   past-target, no abort on zero progress, no abort on current at limit, and an
   axis leaving CLOSED_LOOP merely prevents success rather than failing fast. Its
   0.05 rev tolerance is also looser than the firmware's 0.01, so it cannot detect
   that the firmware op has already given up. Its timeout (20 s) is 2× the
   firmware's (10 s), so it waits 10 s after the axis is already de-energised.
8. **Bridge axis-silence / heartbeat watchdog** — measures CAN frame arrival, not
   motion. Frames were flowing normally, so it correctly did not fire.
9. **The operator's instruments** — dark for the entire event (§1). This is the
   reported "communication seemed to halt". The E-stop was the only guard that worked.

---

## 4. Was `d8814ee` implicated?

**Yes — it is the enabling change, not the defect.**

Before it, `_park_hand` fired its ACTIVATE on an ARMED wire and the firmware's
MPC-stream interlock rejected it every time (`leg_activate.cpp:129`,
`if (fault_mpc_active()) return ERR_REJECTED`).
`logbook/2026-09-23-cup-contact-first-sitting.md` § "The second hand latch":
*"`grep "hand parked ("` over every launch log finds **zero** successes against
four rejections (2026-09-18 ×2, 2026-09-22 ×2)."*
`d8814ee` made `_park_hand_disarmed` disarm and confirm the disarm first, so
**2026-09-23 13:53:59 is the first time in any log that this ACTIVATE executed.**

The defects it exposed predate it: the 0.0 rev target 3.5 mm above metal, the
2.5 rev/s approach, and two monitors with no contact abort. It did introduce the
20 s blocking window into `/clear_errors` (the pre-`d8814ee` rejection returned
in milliseconds) and hence the observability blackout.

### Recommended fixes, ranked

**R1 — move the park target off the metal.** *(highest value, lowest risk)*
Park to the schedule's own rest `REST_HAND_REV = 0.3071 rev`
(`skill_node.py:927`, `:1337`) — 0.414 rev = 13.5 mm above the bottom stop —
instead of `JB_OP_HAND_ACTIVATE_POSITION_REV = 0.0`. That is where the opening
REST wants the hand anyway, so nothing downstream moves. One constant in
`config/hardware_config.yaml` + `python config/generate_config.py`.
*Must grep first:* `JB_OP_HAND_ACTIVATE_POSITION_REV` is also the ARMING_CONTRACT
A1 hand check and the `leg_activate.cpp` MONITOR PASSTHROUGH hand-off reference
(`:261-276` — motion-free at whatever the target is, so it should survive, but
verify), and the `_HAND_PARK_BAND_REV` = 0.10 band is centred on it.
If 0.0 must stay for homing-reference reasons, add a bottom
`hand_clip_floor_margin_rev` mirroring the top's `hand_clip_margin_rev: 0.2` and
park to that instead.

**R2 — slow the final approach.** Drop the hand's trap vel below 2.5 rev/s or
raise its decel. `leg_activate.cpp:206-213` notes the limit is "axis-agnostic on
purpose" and that *"there is no generated hand trap-traj limit; if one is ever
added this is its single consumer"* — so this is the sanctioned seam. Low risk,
but it only buys margin; it does not close the class.

**R3 — give both monitors a contact/stall abort.** *(closes the class)*
`ActivateMonitor.step`: terminal failure on (a) axis leaving CLOSED_LOOP after the
move began, (b) position past the target by > pos_tol in the direction of travel,
(c) \|vel\| < ε with \|pos − target\| > pos_tol for > N consecutive ticks.
`leg_activate.cpp` MONITOR: the same stall test, so the firmware — the only thing
actually in the loop — aborts on contact rather than at 10 s.
**Also re-examine the abort ACTION:** `abort_all` sends IDLE, i.e. drops the load.
For axis 6 a controlled hold-in-place is safer than de-energising.
Per the CLAUDE.md empirical-probe rule, prototype each trip condition in a probe
before writing the tests.

**R4 — stop the recovery path blanking the node.** Proven this session (publish
gap == park duration, three for three; no other gap > 0.3 s in 553 s). The design
comments at `teensy_bridge_node.py:2186-2191` and `:6611-6614` assert the
ReentrantCallbackGroup keeps the 100 Hz timers alive under the park; **the bag
refutes that.** Either move the park to its own thread and have `/clear_errors`
return "park in progress" immediately (the state machine is already held in FAULT
by `_recovery_park_in_progress`), or find what serialises the default-group timers
behind it. Until fixed, the operator is blind for every recovery.

**R5 — bound the push time.** Set `_park_hand`'s monitor timeout below
`JB_OP_GENTLE_MOVE_TIMEOUT_S` (10 s), or better, observe the firmware's
`activate_result` (ACTIVATE_FAILED) instead of inferring from position.

**R6 — serialise the recover calls.** *(lowest — the second call was harmless here)*
A `_recovery_park_in_progress` guard in `_svc_clear_errors`/`_svc_recover`
returning "recovery already in flight" stops a worried operator queueing 20 s
blocks. Note also that when `_svc_recover` fails **only** because the park failed,
the armed-`/clear_errors` fallback runs a **second** `_park_hand_disarmed`
(`:4658-4659`) — so one operator click can burn 2 × 20 s.

---

## 5. What tripped the MAX_DEVIATION latch (dev +3.997)

**A stale hand encoder, not a real tracking error.** The known load-gated
per-axis frame loss (`plans/active/leg-bus-frame-drops.md`), same mechanism as
the first 2026-09-22 latch.

* `pos_meas` frozen bit-identically at **+0.5275 rev for 84 ms** across 8
  consecutive `/hand_telemetry` samples (631.3667 → 631.4501) while `pos_cmd`
  ramped +0.7550 → +3.6537
* `vel_meas` frozen with it at +14.828 rev/s for the whole 84 ms — the S2
  "pos+vel+iq freeze as a unit" signature
* the unfreeze is **+0.5275 → +4.9318 rev in 7.4 ms** = 595 rev/s implied,
  ~5× the plant's own throw speed and physically impossible → stale-cache catch-up
* immediately preceded by a **600 ms axis-6 heartbeat dropout** (630.280 → 630.888),
  logged by the bridge as *"95 encoder frames in the window vs ~60 nominal
  (0 co-dropped); episode 3 for this axis since boot … real leg-bus frame loss"*
* the same stroke one second earlier had 40 ms freezes (dev peaked +2.64, under
  the threshold). The 84 ms freeze was simply the first long enough to cross.
* the guard's own snapshot confirms the Teensy's cache was stale too:
  `max_dev_enc = 1.7426` when the true position was ≈ 4.93 rev, against
  `max_dev_u0 = 5.7393`.

---

## 6. Open uncertainties

1. **What the hand did between 639.28 and 659.64 is not recorded anywhere.** The
   reconstruction (descent ≈ 4.1 s → contact → firmware 10 s timeout at ≈ 649.39 →
   IDLE → carriage on the stop until 659.43) is inferred from the firmware code
   plus the two endpoint samples. An alternative also fits the endpoint: it
   reached ≈ 0.0, missed the 0.01 rev firmware tolerance under the 4.6 A standing
   load, timed out, was IDLEd, and then **sagged** onto the stop. Both end on
   metal and both are fixed by R1/R3, but they imply very different push durations
   (≈ 5.8 s powered vs ≈ 0 s powered + a 3.5 mm drop).
   *Evidence that narrows it:* at 659.428 the axis showed \|vel\| = 0.025 rev/s at a
   0.11 rev error — no energised position loop behaves that way, so the axis was
   certainly de-energised by then, consistent with the firmware abort having fired.
   It does not discriminate what happened before 649.4.
   *Cheapest way to settle it:* re-run the park on the bench with R4 fixed (or a
   direct ODrive log) and watch `iq` through the last 0.5 s of the descent.
2. **The exact mechanism of the executor starvation is not identified.** Candidates:
   a default-group callback serialising behind the park, or ThreadPoolExecutor
   saturation (`MultiThreadedExecutor()` with `nproc` = 6). Refuted: a publisher
   early-return on `_recovery_park_in_progress` (no such gate exists) and an RX/link
   failure (counters prove both threads ran).
3. **Whether the home reference had drifted this session.**
   `HOMING_HAND_ABS_POS_REV = -0.1` vs the operator's 2026-09-06 measurement of
   −0.107 for the metal — a 0.007 rev (0.23 mm) discrepancy already baked in. A
   further drift would put 0.0 even closer to the stop. Not checkable from this bag.
4. **Why the hand needs 4.622 A to hold at +0.0003 rev but 0.882 A at 9.964 rev**
   (5.2×) is unexplained by `gravity_hold_current_a: 1.50` (ball-free; a ball was
   held here). That load is what makes the last 100 ms of the descent hard to brake,
   so it is worth understanding before tuning R2.
