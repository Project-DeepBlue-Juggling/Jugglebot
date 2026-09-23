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

---
---

# RE-EXAMINATION under the owner's correction (UPPER end-stop, ~60° of motor rotation)

Read-only, same bag and log. The owner's correction is treated as ground truth for
*which stop*; everything below states what the instrumentation can and cannot
support, and flags one unresolved conflict with a 30-second bench test that settles it.

**The headline changes.** The first report's "descent onto the bottom stop, firmware
IDLEd it, carriage sagged" reconstruction is withdrawn — not because of the stop
identity, but because the data refutes its final step and reveals a worse defect:
**the second park reported `hand park complete` on an axis that was stalled, and left
the drive pressing at 4.7 A for the 11 s up to the E-stop.** The park's success
criterion cannot tell "arrived" from "jammed". That finding is independent of which
stop was involved.

## 1. Reconciling `pos −0.1101 rev` with the carriage at the TOP

**Nothing in the bag reconciles them, and the bag positively REFUTES all three
candidate mechanisms.** The hand's position reference was continuous and correct
across the whole event.

**No re-reference ever happened.** `grep -i "homing|Command received: home|encoder
search|set_absolute|re-home"` over the entire 553 s launch log returns **one** line:
`cold-start state (boot): is_homed=1` at 1790135220.706 (the Platform Teensy's
persisted flag, read at launch). The complete operator command list is
`level, level, activate, trajectory, clear_errors, standby, deactivate, activate,
trajectory, standby, deactivate, activate, trajectory, clear_errors, clear_errors,
activate, trajectory` — **no home, ever**. The post-E-stop recovery at 1790135709.134
logs `activate: TRAP_TRAJ move to active pose on axes [0, 1, 2, 3, 4, 5]` —
**the hand is not in that list**, so axis 6 was never re-activated, never re-homed,
never re-referenced.

**The reference demonstrably survived.** At 1790135726.775, ~55 s after the E-stop,
`opening REST homes the hand: +0.0000 → +0.3071 rev over 1.50 s` — and the machine
then threw and caught **25+ balls** through 1790135769 with normal apexes
(0.588–0.614 m), normal stroke velocities (−17.7 to −23.3 rev/s) and `caught=True`
on every OUTCOME line. If the reference had been off by ~11 turns, the *first*
post-recovery throw (+9.65 rev of travel from 0.307) would have driven the carriage
from an already-jammed stop and latched within one stroke. It did not.

`+0.0000` at a schedule start is also **not unusual in this session**: the same
reading appears at 1790135441.539 (`+0.0001`), 1790135489.707 (`−0.0000`) and
1790135282.625 (`−0.0000`) — it is what the hand reads after a normal
DEACTIVATE/ACTIVATE park. And the full stroke was exercised normally at
1790135393.802 and 1790135416.516 (`+8.7955 → +0.3071 rev over 6.37 s`).

Mechanism by mechanism:

* **Turn-count loss / single-turn-absolute wrap** — REFUTED. It requires a
  re-derivation event, and the only one in the firmware is
  `encode_set_absolute_position` in `leg_homing.cpp:244`, which never ran. A wrap
  would also have to be *undone* by 726.775 with no re-home, which nothing can do.
* **Belt / coupling slip** — REFUTED quantitatively. `hand_mm_per_rev: 32.567`
  (`config/hardware_config.yaml:494`), no reduction stage on the Jugglebot hand
  (`hand_spool_radius_m` at `:1803` is Ball Butler's). The discrepancy to explain is
  **10.074 rev = 328 mm of belt** against a 352 mm total stroke
  (`hand_motor_hard_stop_revs: 10.701`, bottom stop −0.107, `:451`) — essentially the
  whole belt skipping the pulley. That would also destroy the reference, and the
  reference survived. By contrast **0.167 rev (60°) = 5.4 mm** *is* a plausible amount
  of belt/structural compliance at a stop, which is consistent with the owner's "60°"
  being over-travel into metal rather than lost motion.
* **ODrive position-estimate reset at the firmware abort's IDLE** — REFUTED for the
  same reason (no re-reference, and the estimate was still good 55 s later).
* **Motor-side vs carriage-side of a slipped stage** — no evidence of a second stage
  in the config; and any persistent slip would show as a reference error afterwards.

`robot_state` axis-6 adds: `active_errors = 0x0`, `disarm_reason = 0x0`,
`current_state = 8` (CLOSED_LOOP), `procedure_result = 0`, `trajectory_done = 0`
**unchanged from 1790135630.008 through 1790135671.308**. The first change is
`state8 → state1` at 1790135671.356 (the E-stop). The bag has no sample inside
639.269–659.641, so a transient IDLE at the firmware's 10 s abort (≈649.39) would be
invisible; outside that window nothing anomalous appears. `/bb/odrive_diag` is Ball
Butler and carries nothing about axis 6.

**So: in the machine's own encoder frame the hand ended at the RETRACT / homed end**
— the end `hand_speed_rps: -3.0 # (negative = retract)` (`:505`) drives into and
`hand_abs_pos_rev: -0.1` (`:508`) references, whose metal is at −0.107 rev.

### The unresolved conflict, and the test that settles it

The drive's current was **positive** (+4.7 A decaying to +2.5 A) — the same sign it
uses to lift against gravity — while its estimate sat at the 0-rev end. Reported
position and the owner's eyewitness therefore disagree about which physical stop the
0-rev end is. Two possibilities remain, and **the bag cannot distinguish them**:

* **(i) Frame labelling.** The 0-rev / "retract" end of the encoder frame may be the
  physical TOP of the machine. `hand_catch_prime_rev: 9.9594` is commented "top of
  stroke" (`:677`) and `hand_axis_bottom_offset_mm: -129.0` (`:370`) puts the axis
  bottom below the platform centroid — but if the cup/carriage linkage inverts, cup
  high can mean carriage at the lower end of the screw. If (i) holds, every number
  above is consistent with the owner and the reconstruction simply flips label.
* **(ii) Recollection.** The stop was identified under stress mid-incident.

**Settling test (30 s, launch down, hand de-energised):** push the carriage by hand
onto each mechanical stop and read `/robot_state` `motor_states[6].pos_estimate`. The
end that reads ≈ **−0.107 rev** is the homed/retract end; the operator then says
whether that is physically the top or the bottom. **No fix below depends on the
answer** — but the answer belongs in the logbook entry, because every doc in the repo
currently calls the −0.107 end "the bottom".

## 2. What in ACTIVATE(6) could command UPWARD from 9.964 rev

Reading `leg_activate.cpp` in execution order for axis 6:

| step | frame | line |
|---|---|---|
| error gate (`active_errors != 0` → abort) | — | `:194` |
| **seed** `set_input_pos(leg_sign(6, axes[6].pos_rev), 0, 0)` — the LIVE firmware encoder cache, NON-clipped | 9.9640 | `:199-201` |
| `set_traj_vel_limit(2.5)` | | `:213-214` |
| `set_traj_acc_limits(30, 30)` | | `:215-218` |
| `set_controller_mode(POSITION, TRAP_TRAJ)` | (from PASSTHROUGH) | `:219-221` |
| `set_state(CLOSED_LOOP)` (axis was already state 8) | | `:222-223` |
| 10 ms settle, then `encode_leg_setpoint(6, 0.0, 0, 0)` — clipped to `[0, max]`, `leg_sign` identity on axis 6 | 0.0 | `:230-240`, `odrive_protocol.h:157-168` |
| MONITOR to `|pos−0| ≤ 0.01` and `|vel| ≤ 0.1`, else the 10 s `A_TIMEOUT_US` → `abort_all` → `set_state(IDLE)` | | `:245-257`, `:171`, `:104-112` |

**The commanded direction is unambiguously −rev.** The target is a clipped `0.0` and
`leg_sign` is identity on axis 6, so no sign error can invert it (and the sign of zero
is zero regardless). The two ways upward motion can still occur:

1. **A stale seed.** The seed reads the firmware's own `axes[6].pos_rev` cache — the
   very cache that froze in 40–90 ms bursts on this axis all session (three heartbeat
   dropout episodes by 630.9, one 84 ms freeze that caused the latch). A seed on a
   stale value makes CLOSED_LOOP engage holding a position the carriage is not at.
   **Not the cause here** — the cache was steady at 9.9639–9.9641 for the 8 s before
   the op — but it is a live hazard on the same axis and the same op.
2. **The PASSTHROUGH → TRAP_TRAJ switch starting the planner from a stale
   `input_pos`.** The bag does not carry the ODrive's `input_pos`, so **this cannot be
   ruled out from the data.**

**Answering the specific sub-questions:**
* **The last streamed hand command was NOT above 9.96.** `/hand_telemetry pos_cmd` is
  frozen at **+8.8592** from 1790135631.49 onward — but that is the last *sniffed*
  `HAND_CMD_ECHO`, and it is demonstrably stale, not what the ODrive holds: from
  631.9 to 639.28 the ODrive sat at 9.9639–9.9641 rev with iq +0.57…+0.88 A and
  |vel| ≤ 0.05 rev/s. Had `input_pos` really been 8.8592, a `pos_gain` of 35
  (`:630`) on a 1.105 rev error would have commanded 38 rev/s and moved it instantly.
  So on a latch the firmware suppresses hand output and the ODrive keeps its last real
  `input_pos` ≈ 9.964. The `dev = −1.105 rev` visible in the echo is an artefact.
* **The disarm at 639.309 does NOT touch the ODrive.** `_stop_setpoint_output`
  (`teensy_bridge_node.py:4243-4268`) stops the Jetson's setpoint thread, stages
  `mpc_active = 0` on the wire and closes the :5557 source. It writes no ODrive mode,
  no target, no limits.
* **The trap limits could not have been left at the stream's values.** The streaming
  lane runs POSITION/PASSTHROUGH, which ignores the trap limits, and ACTIVATE writes
  `traj_vel_limit = 2.5` and `traj_acc_limits = 30/30` explicitly every time
  (`:213-218`). The session limits `hand_vel_limit_rps: 200` / `hand_acc_limit_rps2:
  3500` (`:1076`, `:1085`) are the *velocity* limit and the planner's envelope, not
  trap limits. The op was bounded at 2.5 rev/s.

## 3. Where the hand was between the latch and the park — Q3's premise is withdrawn

**The stroke did NOT continue 5 rev after the latch.** The +0.5275 rev reading at the
latch was the *stale* value; the true position was already climbing through it.
`/hand_telemetry`:

```
631.405  cmd +1.9147  meas +0.5275  vel +14.828  iq +16.551   <- STALE (frozen 84 ms)
631.458  cmd +6.1211  meas +4.9318  vel +63.173  iq  -9.069   <- cache catches up
631.517  cmd +8.8592  meas +9.0156  vel +39.999  iq  -2.667
631.571  cmd +8.8592  meas +9.8369  vel  +4.199  iq  +1.808
631.798  cmd +8.8592  meas +9.9762  vel  +0.069  iq  +2.249   <- arrived at catch prime
...
639.257  cmd +8.8592  meas +9.9641  vel  +0.066  iq  +0.939   <- 7.5 s motionless
```

The throw stroke completed normally into catch-prime (`hand_catch_prime_rev: 9.9594`)
by ~631.80, i.e. **0.22 s after the latch**, and the hand then sat motionless at
9.9626–9.9641 rev for 7.5 s at 0.28–0.88 A — a normal gravity hold. Total drift over
those 7.5 s: **+0.0015 rev (0.05 mm)**. Nothing drove it. The apparent −1.105 rev
deviation is the stale echo (above).

## 4. NEW — the second park reported SUCCESS on a stalled axis

This is the finding the owner's correction exposes, and it is the most dangerous one.

`hand park complete — -0.1101 rev -> -0.0227 rev` at 1790135659.637. It was not an
arrival:

* **Traverse time.** 0.11 rev took ~0.75 s (`-0.1101` at 659.43 → `-0.0227` at
  659.64 → `-0.0098` at 659.99 → `+0.0002` at 660.375). A free TRAP_TRAJ move of
  0.11 rev at `TRAP_ACC/DEC = 30 rev/s²` takes **~0.12 s**. Six times slow.
* **The integrator wound up.** Holding current after "arrival" was **+4.699 A**
  (659.99), **+4.622** (660.4–661.3), then monotonically **3.999 → 3.914 → 3.693 →
  2.944 → 2.980 → 2.856 → 2.718 → 2.506 A** by 671.4 — an unwinding velocity
  integrator, still falling when the E-stop landed. Compare the hand's normal
  mid-stroke gravity hold: **+0.88 A at 9.964 rev**, and
  `gravity_hold_current_a: 1.50` ball-free (`:1692`). It settled ~3–5× the static
  requirement.
* **Every arrival test passed anyway.** `ActivateMonitor` (`teensy_link/activate.py:157-170`)
  requires only `axis_state == CLOSED_LOOP`, `|pos − target| ≤ 0.05`, `|vel| ≤ 0.1`.
  A drive pressed hard against a hard stop satisfies all three exactly.
* **And the park then left the axis ENERGISED and holding**, in CLOSED_LOOP at ~0.0,
  for the whole 11 s to the E-stop.

The number matches the eyewitness: **0.107 rev of motor rotation from the stop
reference to the final resting estimate = 38.5°**, and the full excursion from
`-0.1101` to `+0.0006` is **0.111 rev = 40°** — the right order for the owner's
"about 60 degrees", and the right *kind* of quantity (belt/structural compliance,
5.4 mm at 32.567 mm/rev), not lost motion.

One disturbance is visible mid-push: at 661.76–662.18 `pos` moves +0.0004 → +0.0279 →
+0.0494 → +0.0022 → −0.0267 with iq swinging +2.2 → −0.63 → −3.14 → −1.39, then it
returns to ~0.0. That is an external disturbance on a drive that was holding, ~2 s
after the "successful" park. The bag cannot say what caused it.

## 5. The owner's proposal — IDLE the hand when it does not need to move

**Right instinct, and it is already the convention for DEACTIVATE — but it must be
park-then-IDLE, never IDLE-in-place.**

* **IDLE mid-stroke is a hazard, not a safety measure.** The hand does not hold itself:
  `gravity_hold_current_a: 1.50 A` (`:1692`). De-energising at catch-prime
  (9.964 rev = 324 mm up) drops the carriage the full stroke onto the far stop, and
  with `ball_held = 1` it also releases the ball. **That is exactly what the firmware
  already does today on an ACTIVATE abort** — `abort_all` sends
  `encode_set_state(i, IDLE)` (`leg_activate.cpp:104-112`), reached unconditionally
  at the 10 s `A_TIMEOUT_US` (`:171`) from whatever position the axis is in. That is a
  defect to remove, not a pattern to generalise.
* **IDLE at the park is already what the machine does.** `_run_deactivate` ends with
  `teensy_set_axis_state(_HAND_AXIS, IDLE)` (`teensy_bridge_node.py:7025-7031`), and
  the `+0.0001 / −0.0000 / +0.0000` readings at schedule starts
  (441.539, 489.707, 726.775) are the hand sitting de-energised at the park between
  schedules. So the safe idle state is **"parked at the 0-rev end, confirmed inside
  the band, then IDLE"** — and it is already reachable.
* **What is missing is exactly the gap this event fell through:** the *recovery* park
  leaves the hand in CLOSED_LOOP holding, so a park that "succeeded" onto a stop keeps
  pressing indefinitely. Adding the IDLE after a *confirmed* park converts an
  11-second 4.7 A push into a 0 A rest — and it does so whichever stop it is.
* **What a monitor should do instead of IDLE on a mid-stroke abort:** freeze in place,
  not de-energise. Re-issue `set_input_pos(current encoder)` and report FAILED, leaving
  the axis energised at roughly the gravity hold. De-energising is correct **at the
  park** and wrong **anywhere else**.

## 6. Fixes, re-ranked for an upward push (and valid for either stop)

**N1 — make the park's success criterion stall-aware.** *(new #1; this event's core defect)*
`ActivateMonitor` accepted a stalled axis as parked. Add terminal failures for
(a) traverse time ≫ the TRAP_TRAJ prediction for the commanded distance,
(b) holding current above a threshold at "arrival", and
(c) zero progress with non-zero error for > N ticks.
(b) needs `iq` plumbed into `AxisStatus` — `_activate_axis_status`
(`teensy_bridge_node.py:6746+`) currently builds only `axis_state / pos_rev / vel_rps /
active_errors` from the telemetry+diagnostic cache, and `iq_meas` is already in the
same `Telemetry` frame. Probe each trip condition before writing tests (CLAUDE.md rule).

**N2 — de-energise after a CONFIRMED park** (the owner's proposal, in its safe form).
`_park_hand`: park → verify inside `_HAND_PARK_BAND_REV` → `set_axis_state(6, IDLE)`.
Never IDLE without that confirmation. This alone would have ended the push at 660.4.

**N3 — replace the firmware abort's IDLE with a hold-in-place for axis 6**
(`leg_activate.cpp` `abort_all`, `:104-112`), and add the same stall test to the
firmware MONITOR so the abort comes at contact rather than at 10 s. Today an abort
from mid-stroke free-drops a loaded hand.

**N4 — move the park target off the metal.** `REST_HAND_REV = 0.3071` (0.414 rev /
13.5 mm of clearance) instead of `JB_OP_HAND_ACTIVATE_POSITION_REV = 0.0`
(0.107 rev / 3.5 mm). Unchanged from the first report; still the cheapest structural
improvement, and it removes the "final approach ends 3.5 mm from metal at 81 mm/s"
geometry at whichever end that is. Same grep sweep caveat.

**N5 — stop the recovery blanking the node.** Unchanged and now sharper: the operator
watched an 11-second 4.7 A push on a display frozen since 639.28, and reached for the
E-stop because nothing else was telling them anything.

**N6 — slow the final approach; bound the push time; serialise the recovers.**
Unchanged from the first report (previously R2 / R5 / R6).

## 7. What the bag cannot show

1. **The ODrive's `input_pos`**, so the PASSTHROUGH→TRAP_TRAJ planner-seed hypothesis
   for an upward command cannot be excluded — only the Jetson's stale command *echo*
   is recorded, and it is demonstrably not what the drive held.
2. **Anything at all between 1790135639.28 and 1790135659.64** — the node published
   nothing on any topic. Direction of travel during the first park, the moment of
   contact, the current while pushing, and whether the firmware's 10 s abort actually
   fired are all unrecorded.
3. **Which physical stop the 0-rev end is.** The encoder frame is self-consistent and
   intact; the mapping from that frame to the machine's top and bottom is not in any
   recorded signal. See the bench test in §1.
4. **What caused the 661.76–662.18 disturbance** during the hold.

---
---

# SECOND RE-EXAMINATION — owner facts: manual lowering after the E-stop, and IDLE is not a fall hazard

Both owner facts are confirmed by the bag, and together they close the reconstruction
quantitatively. **The carriage never moved during the park.** The motor executed the
whole commanded 9.964 → 0.0 rev descent while the carriage stayed at catch prime,
324 mm up. Everything else follows.

## 1. The manual lowering IS in the bag — and it proves the decoupling

`/hand_telemetry`, after the E-stop at 1790135671.38 (bus down; the `iq` field is
frozen at +2.506 from the last powered frame — it is stale, not a real current):

| t | pos_meas | what |
|---|---|---|
| 671.4 – **678.65** | **−0.0263 … −0.0268**, flat | de-energised, 7.2 s, **zero drift** (carriage at the TOP per owner) |
| **678.65 → 680.16** | −0.009 → **+2.1665**, vel bursts to +6.2 rev/s | **the operator handling the carriage** — jerky, bidirectional, unpowered |
| 680.2 – 682.7 | +2.12 … +3.2462 → +2.7622 | more handling |
| **681.7 → 709.0** | **+2.6886 / +2.6887**, flat | **27.3 s de-energised, zero drift** (carriage now at the BOTTOM per owner) |
| 709.5 → 710.5 | +2.4411 → +1.1815 → **−0.0129**, vel ≈ −1.87 rev/s, iq real again (−1.14, −0.81, −0.05) | powered slew back to the ODrive's **stale `input_pos` = 0.0** left by the recovery park |
| 711 – 727.2 | +0.0000 … +0.0004, iq +3.67 → +2.65 A | holding |
| 727.69 → 728.70 | +0.1297 → +0.2490 → +0.3083 | the schedule's opening REST walk to +0.3071 |
| 729.2 onward | ±90 rev/s swings, 3.0 ↔ 9.5 rev | **normal throws resume** |

### The slip test — three independent numbers, all failing an intact coupling

1. **Span.** `config/hardware_config.yaml:484-494` records the owner's own two
   stop-to-stop measurements: **352.0 mm = 10.809 rev** (mean), which is where
   `hand_mm_per_rev: 32.567` comes from. A top-to-bottom manual move on an intact
   coupling *must* register 10.809 rev. It registered a **net ≈ 0 rev**
   (−0.0263 → +2.6886 → back to −0.0129 under power). Even the raw excursion is
   **+2.715 rev = 25 %** of the required span.
2. **Direction.** Throws drive **0.31 → 9.5 rev** as the hand accelerates the ball
   upward, so +rev is the up/extend direction. A manual **lowering** on an intact
   coupling must register **−rev**. It registered **+2.715 rev**.
3. **Reference error before vs after.** Before the manual move the encoder read
   ~0.000 with the carriage at the TOP; the correct reading there is ~+10.70 rev — a
   **−10.7 rev (348 mm) error**. After the manual move, with **no home anywhere in the
   session** (§1 of the previous re-examination), it read +0.0000 with the carriage at
   the BOTTOM — correct to **~0.1 rev**. A reference cannot fix itself.

**All three are explained by one thing: motor rotation and carriage travel were
decoupled, and the decoupling arithmetic closes exactly.**

```
carriage stays at catch prime  (9.964 rev-equivalent, 324 mm up, 24 mm below the top metal)
motor:   9.9640  --park #1-->  -0.1101   (-10.074 rev)
                 --park #2-->  +0.0003   (+0.111 rev)      <- the 40 deg the owner saw
                 --handling--> +2.6886   (+2.688, net of back-and-forth)
                 --re-tension->-0.0129   (-2.702)
         net motor rotation from 9.9640 = -9.98 rev
carriage: catch prime -> bottom, BY HAND = -9.96 rev-equivalent
         => residual reference error after the accident ~= 0.1 rev (3.5 mm)
```

The pre-existing −9.96 rev error was cancelled **by coincidence**, because the
operator moved the carriage by exactly the distance the motor had over-rotated
(catch-prime-to-bottom is the same journey). **That is why 25+ throws worked
afterwards with no re-home — luck, not recovery.** The residual ~0.75 rev of margin
loss is also why the post-recovery throws topped out at 9.45–9.66 rev rather than the
9.96 they had been reaching.

### Does 4.62 A match a stalled push at the ODrive's pos gain? YES — exactly

`hand_pos_gain: 35.0` (`:630`), `hand_vel_gain: 0.007` N·m per rev/s (`:631`),
`hand_torque_constant_nm_per_a: 0.0055133` (`:1666`).

A standing position error **e** commands `35·e` rev/s of velocity error →
`0.007 × 35 × e = 0.245·e` N·m → `0.245·e / 0.0055133` = **44.4·e amps**.

* observed **4.62 A ⇒ e = 0.104 rev**
* the actual standing error at that moment: **0.11 rev** (the second park's traverse
  from −0.1101 to the 0.0 target)

**Exact fit.** For comparison the same axis holds **0.88 A** at 9.964 rev mid-stroke
and `gravity_hold_current_a: 1.50` ball-free (`:1692`) — so 4.62 A is **not** a
gravity hold, it is the P-path of a 0.10 rev standing error, i.e. a **stall
signature**. It then decays 4.70 → 2.51 A over the 11 s to the E-stop as the velocity
integrator (`hand_vel_int_gain: 0.07`, `:632`) unwinds.

### Coupling and sensing

* **It is a spool (cable/belt on a drum), not a lead screw and not a geared stage.**
  `config/hardware_config.yaml:482-494`: the hand's gain is "the hand's mm/rev **spool
  gain** — MEASURED, replacing the retired `teensy_trajectory.linear_gain_factor` /
  `hand_spool_radius_m` fudge pair", `hand_mm_per_rev: 32.567` = 352.0 mm / 10.809 rev.
  Effective drum radius **5.183 mm**. No reduction ratio appears anywhere for the
  Jugglebot hand (`hand_spool_radius_m: 0.0052493` at `:1803` is Ball Butler's).
* **The encoder is motor-side only.** The telemetry field is `pos_rev` in motor
  revolutions and the *only* thing converting it to carriage millimetres is that one
  software constant. **There is no carriage-side position sensor anywhere in the
  config.** So a decoupled or slipping spool is **undetectable by construction** — the
  ODrive, the firmware deviation guard and every Jetson monitor all see a perfectly
  healthy axis following its target. That is the deepest finding of this
  investigation.

### What the bag still cannot show

The carriage. Every statement about where the carriage was comes from the owner; the
instrumentation only ever sees the motor. What the bag *does* show is that motor
rotation and carriage travel demonstrably disagreed by ~10 rev, and by how much.
It also cannot show **why** the spool let go (the 20 s blackout covers the whole
descent): a cable paying out into slack against a carriage that does not fall, a
jumped/derailed cable, or a loosened drum grub screw are all consistent. **That is a
bench question, and it is now the highest-priority one** — the machine has been
running since on a reference restored by accident.

## 2. Retracting the fall claim — the bag confirms the owner

**I withdraw "IDLE mid-stroke drops the carriage up to 325 mm".** The bag contradicts
it directly, twice, and one of those is at height:

* **671.4 → 678.65, de-energised at the TOP (324 mm up): 7.2 s, pos −0.0263 → −0.0268,
  drift 0.0005 rev = 0.016 mm.**
* **681.7 → 709.0, de-energised at the bottom: 27.3 s, pos +2.6886 ↔ +2.6887, drift
  ±0.0001 rev = ±0.003 mm.**

The hand does not back-drive. `gravity_hold_current_a: 1.50 A` is the *closed-loop*
holding current (friction/preload against a commanded position), not evidence that the
axis falls when de-energised. **IDLE is a safe state at any position on axis 6**, and
the design principle "hand in IDLE whenever it is not actively being driven" is sound.

Consequently **N3 of the previous section is WITHDRAWN**: `abort_all`'s
`encode_set_state(i, IDLE)` (`leg_activate.cpp:104-112`) is *correct* behaviour for
the hand, not a hazard. (It remains a hazard for the legs, which is why this must stay
axis-6-scoped.)

## 3. Could the firmware IDLE the hand at the MAX_DEVIATION latch? Yes — and it should

Today the latch suppresses leg output and the hand ODrive is simply **left in
CLOSED_LOOP holding its last `input_pos`**. The bag shows exactly that: from 631.9 to
639.28 it held 9.9639 rev at 0.28–0.88 A for 7.5 s.

Two concrete harms in this very event:
* the stale `input_pos` survived the whole incident and **fired again at 709.5**, when
  the re-enabled hand slewed −2.70 rev at 1.87 rev/s with nobody commanding it. That
  is an un-commanded motion on a machine the operator believed was parked.
* holding at a stale target is what the recovery park then had to undo, at speed,
  toward a position the carriage could not reach.

**Where:** `fault_machine.cpp`, on the edge that sets `state = MAX_DEVIATION` /
`estop = true` — the hand term at `:504` (and the leg term at `:459-460`) — send
`ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE)` on the same edge that
suppresses output. **Axis 6 only**; the legs must keep holding or the platform drops.
Cost: a held ball is no longer actively cupped — but the carriage does not move, and
the latch already ended the throw.

## 4. Where the bridge should IDLE the hand

1. **On the disarm edge — `_stop_setpoint_output` (`teensy_bridge_node.py:4243-4268`).**
   Highest value. Today the disarm stops the Jetson's 40 Hz feed, stages `mpc_active=0`
   and closes the :5557 source, and **writes nothing to the ODrive** — so the hand
   stays energised on a stale target indefinitely. `_park_hand_disarmed`'s docstring
   (`:6683-6690`) asserts "the ODrives hold every axis in place", which is true but
   means *actively, at a stale target*. Adding `set_axis_state(6, IDLE)` here makes
   "wire disarmed" mean "hand not driven", which is what every caller already assumes.
2. **At the end of a CONFIRMED recovery park — `_park_hand`**, after verifying the
   position is inside `_HAND_PARK_BAND_REV`. This alone would have ended the 4.6 A
   push at 660.4 instead of 671.4.
3. **At schedule END / stop** (skill_node's `END … hold installed` path). A schedule
   that has ended has no reason to hold the hand energised.
4. **Already correct:** `_run_deactivate` (`:7025-7031`) and `leg_activate.cpp`'s
   `abort_all`.

**Mirror obligation on the arming side:** if the hand can be IDLE, then
`_arm_setpoint_output` must **re-seed `input_pos` from the measured encoder and
re-enter CLOSED_LOOP before the first streamed frame**, or the first frame steps it.
The 709.5 slew is the un-seeded case, observed.

## 5. Fixes, re-ranked with both owner facts

**F1 — Detect the decoupling. (new #1; nothing else protects against this class.)**
A motor-side-only encoder on a spool drive means a slipped or decoupled carriage is
invisible to every guard in the system. Two cheap, independent checks, neither needing
new hardware:
 * **A stall/contact test at every op arrival** (was N1). `ActivateMonitor`
   (`teensy_link/activate.py:157-170`) accepts `CLOSED_LOOP` + `|pos−target| ≤ 0.05`
   + `|vel| ≤ 0.1`, all three of which a jammed drive satisfies. Add: traverse time vs
   the TRAP_TRAJ prediction (this park took 0.75 s for 0.11 rev against a ~0.12 s
   prediction — **6×**), and holding current at arrival (4.62 A against a 0.88 A
   mid-stroke hold — **5×**, and `44.4 A/rev` makes the current a direct readout of
   the standing error). `iq_meas` is already in the same `Telemetry` frame but is not
   plumbed into `AxisStatus` — `_activate_axis_split`/`_activate_axis_status`
   (`teensy_bridge_node.py:6746+`) carries only axis_state/pos/vel/errors.
 * **A stroke-consistency check at each schedule start.** The opening REST already
   prints the measured hand position; an end-to-end reference check (drive gently to
   the retract stop, confirm it reads −0.107 ± band) once per session would have
   caught a 10 rev error instantly. Today nothing ever re-validates the reference —
   `is_homed` came from a Platform-Teensy flag set before this launch.

**F2 — IDLE the hand whenever it is not being driven** (the owner's principle).
Firmware: on the MAX_DEVIATION latch edge (§3). Bridge: disarm edge, confirmed park
end, schedule end (§4). Plus the arming-side re-seed. This removes both the 11 s push
and the un-commanded 709.5 slew.

**F3 — Never command the hand toward a position the carriage may not be able to
reach.** The park drives to `JB_OP_HAND_ACTIVATE_POSITION_REV = 0.0`, 0.107 rev
(3.5 mm) from metal, from up to 9.96 rev away, open-loop with respect to the carriage.
Park to `REST_HAND_REV = 0.3071` instead (0.414 rev / 13.5 mm of clearance) and cap
the commanded travel, so a decoupling costs centimetres rather than the whole stroke.

**F4 — Stop the recovery blanking the node.** Unchanged, and now the sharpest
operational lesson: the operator watched an 11 s 4.6 A push against a display frozen
since 639.28, with `/robot_state`, `/hand_telemetry` and `/link_status` all dead, and
reached for the E-stop because nothing else was telling them anything.

**F5 — Bound the push: monitor timeout below the firmware's 10 s; serialise
`/clear_errors`; slow the final approach.** Unchanged (previously R2/R5/R6).

**WITHDRAWN:** the previous N3 ("replace the firmware abort's IDLE with a
hold-in-place") — the owner's fact (2), confirmed by 34.5 s of zero-drift de-energised
telemetry at two different heights, makes the existing IDLE correct for axis 6.

## 6. Bench actions before the next sitting

1. **Inspect the hand spool/cable/drum coupling.** The reconstruction says the motor
   turned ~10 rev that the carriage did not. Find out why. Until that is understood
   the machine can lose its hand reference silently at any moment.
2. **Re-home the hand.** It has been running since 1790135726 on a reference restored
   by coincidence, ~0.1 rev out, with ~0.75 rev less top-stop margin than before.
3. **Read `pos_estimate` at each hard stop** (launch down, hand de-energised) and
   confirm the span is 10.809 rev and the retract end reads ≈ −0.107. That is both the
   reference check and the confirmation that the coupling is sound again.
