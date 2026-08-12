---
title: Hand ball-present sensor — G02 wiring through firmware, protocol, ROS, GUI
created: 2026-07-28
status: active
related_logbook:
  - 2026-07-28-anomaly-fixes-validation-sitting.md
  - 2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md
  - 2026-07-23-phase7-third-sitting-verdicts-stutter-pretilt.md
related_config:
  - config/protocol_config.yaml → endpoints (restructured in Phase 2)
  - config/hardware_config.yaml → jugglebot_ball_detect (new block, Phase 1)
  - config/ODrive config Files/odrive_pro_hand_config.json → config.gpio2_mode
related_code:
  - ros_ws/docs/ball_possession_contract.md::C-POSSESS-1
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp::version_check_step
  - config/generate_udp_protocol.py::HAND_AXIS6_PERMITTED
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_publish_hand_telemetry
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py::SOURCE_HAND_BALL_SENSOR
---

# Plan — Hand ball-present sensor: wire G02 through firmware, protocol, ROS, GUI

**Scope boundary:** this plan ends at the ROS/GUI surface. The possession-source
implementation (`SOURCE_HAND_BALL_SENSOR`), the `toss_require_ball_evidence`
flip, the post-release-decel work, and the end-stop anchor question are owned
elsewhere (§ Notes for collaborators → Out of scope). Design approved by the
operator 2026-07-28; the ODrive-side pin config is already flashed and
NVM-persisted (recorded in commit `64d2a8f`); sensor code has landed
through Phase 4 — see the phase table for per-phase status.

## Context

A ball-present sensor was physically installed in Jugglebot's hand on
2026-07-28: a switch that, when pressed by a seated ball, shorts the Hand
ODrive Pro's GPIO **G02** to GND. The mechanism is identical to BallButler's
G03 sensor (same sliding-rail contact design; operator-confirmed 2026-07-28),
except for the pin and the board family — BallButler's hand drive is an ODrive
**S1**, Jugglebot's is an ODrive **Pro** (hw 4.4.58, fw **0.6.11-1**;
operator-confirmed, no reflash since the March 2026 config backups).

The primary consumer is the **`ball_seated` toss precondition**
(`reload_coordinator_node.py` builds `ball_seated` unconditionally-True today
because `JB_OP_TOSS_REQUIRE_BALL_EVIDENCE = false`, with the sensor's absence
given as the YAML reason). The earliest design intent on record predates the
install: the 2026-07-23 third sitting already named this sensor ("switch on
a hand-ODrive GPIO, BB-pattern") as "the clean long-term verdict source",
one that "will subsume most of the plausibility-gate machinery"
(`logbook/2026-07-23-phase7-third-sitting-verdicts-stutter-pretilt.md:221-223`).
Secondary consumers, in priority order: `RETENTION_REJECTED` for bounce-outs
via the C-POSSESS-1 possession seam
(`ball_possession.py`, landed 2026-07-28 by the possession workstream, which
reserves `SOURCE_HAND_BALL_SENSOR` by name), carry-phase seating diagnostics
(the operator wants spotty "held" signals observable, to track a ball moving
around in the cup), and — later, in the possession workstream — contact
timestamping for the pre-registered sensor-era timing experiment
(`logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md`,
announced-to-contact bias +90–120 ms, core σ ≈ 35 ms).

### How BallButler does it (the reference pattern, verified 2026-07-28)

BallButler's own Teensy polls its hand ODrive over CAN using CANSimple
RxSdo/TxSdo arbitrary-parameter access: an RxSdo frame with **`OPCODE_WRITE`
invoking the `get_gpio_states` function endpoint** (the documented
function-invoke idiom — a function's return value arrives in the TxSdo reply
as a `uint32` GPIO bitmask). Poll interval 50 ms (≤20 Hz; stamped from
reply-or-timeout, so the true period is 50 ms + round trip), 100 ms reply
timeout, and an active-low decode (`ball_in_hand = !((states >> pin) & 1)` —
the switch shorts to GND against the ODrive's internal pull-up,
`gpioN_mode = 1` = `DIGITAL_PULL_UP`). The reported bit is **raw and
undebounced** — `ball_in_hand_` is overwritten from every reply; BallButler's
`max_missing_samples = 5` counter gates only a *recovery action* (escalate to
its CHECKING_BALL re-seat state), never the reported signal.

Three BallButler properties are **deliberately not copied**:

1. **Fail-open boot default** (`ball_in_hand_ = true` before any GPIO read).
   For BallButler the failure mode is a wasted throw; for a catch/`ball_seated`
   gate it mis-reports possession. Jugglebot's signal is tri-state from birth
   (§ Architecture).
2. **No timestamp / no staleness discipline** — BallButler's bit is the only
   state variable in its `CanInterface` with no age accessor, and a dead ODrive
   republishes the stale value forever. Jugglebot's pipeline carries a source
   timestamp and explicit validity on **both** hops (ODrive→bridge and
   bridge→Jetson; § Architecture and Phase 5).
3. **State-gated polling** (BallButler never polls during BOOT/THROWING/
   CALIBRATING/ERROR). Jugglebot polls continuously (§ Approved decisions,
   row 2's resolution).

### The blocking finding: the repo's existing endpoint id is wrong for the Pro

`config/protocol_config.yaml` already carries `endpoints: GPIO_STATES: 700`,
propagated by codegen into `config/generated/protocol_config.{h,py}`; the
generated **`.h`** is copied into all four firmware dirs
(`generate_config.py:998-1001` as of `386ade5`) while the **`.py`** goes only
to the ROS package (`:1002`), which is what backs the name→id table in the live
`jugglebot/can/odrive.py` (`ENDPOINT_IDS`, dereferenced at import by
`teensy_bridge_node`). No Jugglebot runtime path resolves `get_gpio_states`
today — but **BallButler's shipping firmware consumes both flat constants**
out of its generated `protocol_config.h`
(`BallButler/ball_butler_main/CanInterface.h:94-96` aliases them; the
ball-detect poller and encoder-search use them at six call sites), and four
live assertions in `tests/ros/test_odrive.py` read
`ENDPOINT_COMMUTATION_MAPPER_POS_ABS`. Any restructure must carry these
consumers (Phase 2).

**700 is the ODrive S1 0.6.11 id — correct for BallButler, wrong for the
Pro.** ODrive endpoint ids are firmware-build-specific; for the Pro 4.4-58V
the `get_gpio_states` id is:

| fw | `get_gpio_states` id |
|---|---|
| 0.6.10 | 675 |
| **0.6.11 / 0.6.11-1** | **726** (identical endpoint trees, CRC 55416) |
| 0.6.12 | 764 |

Worse than a miss: **on Pro 0.6.11, id 700 exists and is
`encoder_estimator1.status`** (a read-only `uint8`). A poller using 700
against the Pro would receive well-formed TxSdo replies carrying an
unrelated, probably-constant status byte — a live-looking sensor that never
changes, with **no timeout to diagnose**. This defeats any commissioning
check of the form "no reply ⇒ wrong endpoint" and is why the Phase 7 gate
requires a raw-word toggle test, never a decoded-boolean check.

The sibling constant `commutation_mapper_pos_abs: 488` is a related but
distinct case: its **only production consumer is BallButler's S1
encoder-search, where it works on the shipping S1 0.6.11 firmware** — so 488
is treated as the S1 value and moves under the S1 group verbatim in Phase 2.
(Jugglebot's Pro-facing `can/odrive.py` also *names* the endpoint in
`ENDPOINT_IDS`, but nothing in production dereferences it — Phase 2 item 3
**deletes** that name rather than re-pointing it at either board's id.) (The Pro-0.6.11 id for the
same endpoint is 461; it is recorded here for future use and deliberately
**not emitted** — it has no consumer and would ship unverified.) The class
defect is that neither constant was board+fw-qualified; Phase 2 closes the
class.

### Why push is impossible (researched 2026-07-28, ODrive docs + release API)

No CANSimple message on any released firmware 0.6.7–0.6.12 carries GPIO
state; the cyclic-broadcast set is closed to the fixed `Get_` message list
(nine rate-config fields, none GPIO); no facility exists to broadcast an
arbitrary endpoint. Indirect mappings were evaluated and rejected: `enable_pin`
(state visible in heartbeat but **disarms the hand motor when the ball
lands**), endstops (state reaches no broadcast field — collapses back into
SDO with homing side effects), the thermistor divider (the only true native
push; forfeits G02 and destroys the hand's thermal protection), custom
firmware (0.6.x Pro ships as signed binaries; no public source). **Polling is
the only mechanism.** The fallback, if SDO polling proves unreliable on the
heavily loaded CAN3, is rewiring the switch to a Teensy GPIO (zero bus cost,
no endpoint fragility, discards the installed G02 run) — kept on record, not
planned.

### Approved decisions (operator, 2026-07-28)

| # | Decision | Notes |
|---|---|---|
| 1 | **Poll host = can-bridge Teensy** | Modeled on `version_check.cpp`. The Platform Teensy is rejected for observability work: no scheduler, a CAN RX callback that can block ~300 ms, and no Jetson path except the bridge's two-id relay (two flashes instead of one). Revisit only if firmware must react to contact mid-stroke. |
| 2 | **Poll continuously, config-driven rate, default 50 Hz** | The operator's initial hold-phase-only gating proposal was withdrawn after review: a catch, a BB reload throw, and a bounce-out all transition **during** the would-be-unpolled window, so gating creates a "blind exactly at the interesting transition" class (a bounce-out during an unpolled window reads as held when polling resumes — a false `ball_seated`, the exact defect the sensor exists to kill). Gating saves ~1 pp of CAN3 bandwidth and requires new phase-downlink plumbing. A stroke-window suppression knob is a cheap follow-up **if** Phase 7 jitter data orders it. |
| 3 | **5-sample miss threshold for the debounced verdict** | The debounced verdict is **new to Jugglebot** — BallButler's reported bit is raw, and its `max_missing_samples = 5` sizes a recovery action, not a signal filter (§ Context). The 5 is a starting point inherited by analogy only; at 50 Hz it is a 100 ms window vs the 250 ms the same count gives BallButler at 20 Hz. Both knobs are config values; Phase 7 step 5 is the first thing that actually sizes them. |
| 4 | **Uplink raw AND debounced bits** | A consumer will track whether the ball moves around in the hand during carry, so the raw per-sample bit must survive to ROS; debounce applies at the verdict layer only. |
| 5 | **G02 final** | Thermistor occupies G03 on the Pro; G02 was free (`gpio2_mode` was 17 = AUTO, no endstop/brake/step-dir claim). |
| 6 | **fw 0.6.11-1 confirmed** | No `-1` is expected in CAN `Get_Version` frames; endpoint tree identical to 0.6.11 (CRC 55416) ⇒ **726** stands. Phase 0 surfaces the reply's fourth byte (`fw_unreleased`) — landed in `aa14098`; empirical confirmation is Phase 7 step 1. |
| 7 | **`gpio2_mode = 1` (DIGITAL_PULL_UP) flashed + NVM-persisted** | Done by the operator via the ODrive GUI, `save_configuration` + reboot, 2026-07-28. Config backup updated in `64d2a8f`. |
| 8 | **Scope ends at ROS + GUI pill** | Possession-source implementation stays with the possession workstream (C-POSSESS-1 forbids speculative sensor code and reserves the seam). |
| 9 | **Boot/unknown default = tri-state UNKNOWN** | Never a bare bool; `ball_seated` must treat UNKNOWN/stale as not-seated once `toss_require_ball_evidence` flips (that flip is NOT this plan's). **Honoured 2026-08-10 when the flip landed**: UNKNOWN/stale mints `REJECTED_BALL_UNKNOWN` and refuses the throw. |
| 10 | **Decel fix / end-stop anchor stay separate work items** | Per decision (b) of the 2026-07-28 sitting. |

## Architecture

```
Hand ODrive Pro (node 6, CAN3, fw 0.6.11-1, gpio2_mode=1)
      ▲ RxSdo 0x0C4: OPCODE_WRITE, endpoint 726 (get_gpio_states)   ← gpio_poll.cpp @50 Hz
      ▼ TxSdo 0x0C5: uint32 bitmask, bit 2 (active-low)             → typed decode, cache
can-bridge Teensy 4.1  (gpio_poll on task_homing; uplink on task_telem; FW_VERSION 3→4)
      ▼ UDP :5005, new additive MsgType HAND_SENSOR {t_bridge_us, raw_states, flags, miss_count}
teensy_bridge_node (Jetson; RX-age staleness gate)
      ▼ /hand_telemetry gains ball_held / ball_held_raw / ball_held_valid / ball_held_stamp
      ▼ /link_status gains a hand_ball_sensor KeyValue (incl. raw word in hex)
GUI ball-in-hand pill (ros_ws/gui, existing /hand_telemetry subscription)
      ▼ (later, possession workstream) SOURCE_HAND_BALL_SENSOR at the C-POSSESS-1 seam
```

**Signal semantics, end to end — normative:**

- **Tri-state.** `HELD` / `EMPTY` are asserted only from a fresh,
  endpoint-verified TxSdo reply; `UNKNOWN` covers boot (before the first
  reply), reply staleness, un-anchored bridge time, and the `Get_Version`
  gate not passing.
- **A reply timeout is NOT a miss.** Timeouts advance staleness only; the
  debounced verdict and `miss_count` change only on good replies. (Two
  readings of "miss" give opposite safety behaviour: timeout-as-miss would
  flip a healthy seated ball to EMPTY after 5 lost replies on a busy bus,
  violating the tri-state rule. BallButler behaves the same way: its timeout
  branch touches neither the bit nor the counter.)
- **`miss_count` counts consecutive EMPTY readings from good replies**,
  saturating, frozen during a stale window (so a consumer can distinguish
  "stale at 3 misses" from "stale at 0").
- **Debounce asymmetry:** the 5-miss rule applies to HELD→EMPTY only; any
  single HELD reading restores HELD. (Sized for a sliding contact whose
  failure mode is spurious opens, never spurious shorts — the BallButler
  contact-reliability data.)
- **Clock discipline** (`time_base.h:46-63` invariant): all staleness/interval
  arithmetic uses `micros64()` monotonic stamps that never leave the Teensy;
  the wire field `t_bridge_us` is **wall-clock by contract**
  (`now_wall_us()`), Jetson-epoch microseconds, exactly like every existing
  `t_bridge_us` field. When the bridge's wall anchor is not set
  (`time_synced()` false, `now_wall_us() == micros64()`), the sample is
  marked not-time-synced and the Jetson must not trust the stamp.

**Bus cost** (ADR-0013 § Decision: CAN3 carries ~5,340 frames/s steady,
~5,840 with a throw active, ~64–78 % of the classical 1 Mbps ceiling): at
50 Hz the poll adds 100 frames/s ≈ 100 × 111 µs = **+1.1 pp of bus time**
(derived from `bits_per_frame_approx: 111`, pre-stuffing — an estimate, not
a measurement). Arbitration: the SDO pair (0x0C4/0x0C5) ranks below all six
legs' traffic but **above** the hand's own 0x0CC stroke setpoints — the
injected jitter into the 500 Hz stroke stream is unquantified and is a
Phase 7 measurement, not a blocker.

## Implementation phase summary

| Phase | Deliverable | Deploy unit | Status |
|---|---|---|---|
| 0 | Surface decoded ODrive fw versions (log + `/link_status`) | Jetson only | done (`aa14098`) |
| 1 | `jugglebot_ball_detect` YAML block + codegen + `gpio2_mode` drift test | repo (+ regenerated headers in BallButler's tree) | done (`2b3ab78`; JSON record `64d2a8f`) |
| 2 | Endpoint-id contract: (board, fw)-qualified ids; 726; consumer migration incl. BallButler + native-golden regen | repo (incl. `tests/firmware/native/` golden) + BallButler lockstep commit | done (`386ade5`; BB `93a91fb`+`334af82`) |
| 3 | Bridge firmware: `gpio_poll.cpp`, typed TxSdo decode, `Get_Version` gate, FW_VERSION 4 | bridge flash | done (`7dc347f`; flashed by the operator 2026-07-29) |
| 4 | Additive `MsgType HAND_SENSOR` uplink | bridge flash + Jetson (independent) | done (`6cc38f7`; flashed by the operator 2026-07-29) |
| 5 | ROS surface: `/hand_telemetry` fields + `/link_status` KeyValue + RX-age gate | Jetson (colcon) | done (`fafcee0`; colcon-deployed by the operator 2026-07-29) |
| 6 | GUI ball-in-hand pill + `tests/hardware/session_hand_ball_sensor.md` runbook | Jetson (static files) | done (`73d70c6`; P3 browser check outstanding) |
| 7 | Hardware commissioning: raw-word toggle gate, SDO RTT, soak | operator-run | steps 4-5 open (bench tuning, **not blockers** — the operator validated the sensor in situ 2026-08-10 and the possession flip landed on that authorisation; see § Out of scope) |

Phases 0–2 are pure-repo and independently committable (Phase 2 spans both
repos in lockstep). Phases 3–4 ship in one bridge flash. Phase 4's message is
additive (LegCmd precedent: no existing frame changes ⇒ **no
`PROTOCOL_VERSION` bump**), so the Jetson and firmware deploy independently
in either order; an old Jetson silently ignores the unknown msg_type, a new
Jetson treats "never seen" as UNKNOWN.

**Flash note (2026-07-29):** the operator's FW 4 flash + colcon build + live sitting surfaced a CAN3 bus-health regression (`bus1_health` flapping OK↔WARN) attributed to the poller's interaction with the shared CAN3 command gate — under separate investigation (`logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md`), with a FW_VERSION 5 fix pending flash.

**Cross-repo caution (applies to Phases 1 AND 2):** `generate_config.py`
copies BOTH `protocol_config.h` and `hardware_config.h` into four firmware
dirs including `../BallButler/ball_butler_main`. Any codegen run dirties the
sister repo's working tree — check for another session's uncommitted work
there before regenerating, and commit the BallButler side in lockstep where
its firmware consumes the changed symbols (Phase 2).

## Implementation phases

### Phase 0 — surface the firmware versions that the endpoint id depends on

The bridge sweeps `Get_Version` once per **Teensy boot** for every axis that
has heartbeated (`version_check.cpp:70-96`: 100 Hz homing task, one frame
per tick, 1 s re-query until every present axis replies, then idles; the
sent/received masks reset only in `version_check_init()`). The Jetson pulls
that Teensy-local cache at each launch via the `GET_AXIS_VERSIONS` RPC and
decodes `(fw_major, fw_minor, fw_rev)` per axis into
`MotorStateTracker.firmware_versions` — and, before Phase 0, discarded them:
the PASS log line printed no numbers and `/link_status` had no
**ODrive**-version row (only `platform_fw_version`). `RobotState.msg` still
carries only `bool firmware_validated` (Phase 0 added no message field).
Note `MotorStateTracker.validate_group` checks
firmware only for *internal consistency within a hardware group* — the
existing "firmware check PASSED" log does **not** assert fw == 0.6.11.

Work: in `teensy_bridge_node._version_check_poll` (`teensy_bridge_node.py:1773`
as of `fafcee0`; PASS/FAIL log calls at `:1837-1839` and `:1833-1834`), format
the decoded per-axis fw versions into the log line — **including the fourth
byte, `fw_unreleased`, previously discarded as `_unrel` — decoded at `:1810`,
stored at `:1813`**
(the only wire evidence that could carry the `-1`). Add an `odrive_fw_versions`
KeyValue in `_publish_link_status` (`:2440` region), distinct from the existing
`platform_fw_version` row, rendering absent axes explicitly (e.g. `?`)
rather than omitting them.

Done when: a unit test drives the formatting path with a mocked tracker (all
seven axes populated, and a partial-rig case with absent axes) and asserts
both the log string and the KeyValue content; `pytest tests/ -q` green. Live
confirmation on powered hardware is Phase 7 step 1 (a partial bench rig
legitimately shows fewer axes — the sweep only queries axes that heartbeat).

### Phase 1 — config groundwork

1. New YAML block, sibling of `ball_butler_ball_detect`
   (`config/hardware_config.yaml`), registered in `config/generate_config.py`
   (the `(yaml_key, prefix, namespace, title)` registration table at `:476`
   as of `386ade5`)
   with prefix `JB_BD_` / C++ namespace `JBBallDetect` (as landed in
   `2b3ab78` — uppercase initialism per the generated header's convention,
   e.g. `JBOp`, `BBBallDetect`; the plan originally spelt it `JbBallDetect`):

   ```yaml
   jugglebot_ball_detect:
     enabled: true              # false ⇒ gpio_poll compiles out entirely (kill switch;
                                #   requires a reflash). The Phase 7 A/B uses the RUNTIME
                                #   serial toggle (`gpio_poll on|off`), not this flag.
     gpio_pin: 2                # G02 on the hand ODrive Pro (switch to GND, internal pull-up)
     check_interval_ms: 20      # 50 Hz continuous poll (operator decision 2026-07-28)
     max_missing_samples: 5     # HELD→EMPTY debounce; 100 ms window at 50 Hz
     check_timeout_ms: 100      # SDO reply timeout per request
     expected_fw: [0, 6, 11]    # ODrive Pro fw the endpoint-id table is pinned against (Get_Version gate)
   ```

   `expected_fw` emits as a Python list / C++ array (`JB_BD_EXPECTED_FW` /
   `JBBallDetect::EXPECTED_FW`). The board half of the qualification is NOT
   restated here — the axis-6 board identity already lives in the
   `ODRIVE_VER_AXIS_*` registry (`config/generated/hardware_config.py:100-108`)
   and is validated Jetson-side by `EXPECTED_HW_VERSIONS`; the firmware gate
   compares the fw triple only.

2. Drift test closing the silent-divergence class for the flashed pin mode
   (pattern: `tests/motion/test_leg_torque_ff.py::`
   `test_yaml_kt_odrive_config_matches_the_flashed_odrive_json`): assert
   `config/ODrive config Files/odrive_pro_hand_config.json →
   config.gpio2_mode == 1`, keyed off the new YAML block's `gpio_pin`, so a
   future re-dump of the hand config or a pin move breaks loudly.

Done when: codegen emits the `JB_BD_*` Python constants and the C++
`JBBallDetect` namespace, the drift test passes, and the phase's scoped
checks are green (full suite at end of plan, per the Testing-plan gate).

### Phase 2 — endpoint-id contract (the class fix)

1. Restructure the `endpoints:` section of `config/protocol_config.yaml` so
   every ODrive endpoint id is **qualified by (board, fw)**:

   ```yaml
   endpoints:
     # ODrive endpoint ids are firmware-build-specific (flat_endpoints.json).
     # Qualified by (board, fw). A consumer MUST verify Get_Version before use.
     odrive_pro_0_6_11:            # hw 4.4.58, fw 0.6.11 and 0.6.11-1 (tree CRC 55416)
       get_gpio_states: 726
     odrive_s1_0_6_11:             # BallButler hand (hw 5.2.0) — values proven in production on BB
       get_gpio_states: 700
       commutation_mapper_pos_abs: 488
   ```

   **488 moves verbatim** — its only production consumer is BallButler's S1
   encoder-search, where it works on the shipping S1 0.6.11 firmware
   (Jugglebot's dangling name for it is deleted, not migrated — item 3). It is
   not re-pointed to the Pro value (461); a Pro entry for that endpoint is
   added only when a Jugglebot consumer appears, and would ship unverified
   until commissioned.

2. Teach the two flat emitters the nested shape — they currently interpolate
   `cfg["endpoints"].items()` values directly and would emit invalid C++ for
   a nested dict: `config/generate_config.py:242-245` (C++
   `namespace EndpointId`) and `:421-423` (Python `ENDPOINT_*`; post-change
   line numbers as of `386ade5`). Pinned
   generated names, quoted so Phase 3 and consumers can reference them
   verbatim: C++ `EndpointId::odrive_pro_0_6_11::get_gpio_states` (nested
   namespace), Python `ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES`.

3. Migrate every consumer of the removed flat names **in the same change**:
   - `ros_ws/src/jugglebot/jugglebot/can/odrive.py:75-78` — `ENDPOINT_IDS`
     is dereferenced at module import by `teensy_bridge_node`; a stale name
     here is an `AttributeError` that kills the whole ROS launch.
     **DELETE the `'commutation_mapper.pos_abs'` entry; do not re-point it.**
     The table's only readers are this module's `encode_sdo_read` /
     `encode_sdo_write` (`:239`, `:247`), which have no production caller
     (tests only, driving axis 0 — a Jugglebot Pro leg). Re-pointing the name
     at the S1 group would leave a Pro-facing module handing an S1 endpoint to
     Pro axes: the exact class defect Phase 2 exists to close, re-created one
     line below the fix. `ENDPOINT_IDS` is left with a single entry,
     `'get_gpio_states' → ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES`.
   - `BallButler/ball_butler_main/CanInterface.h:94-96` — the two
     `EndpointId::` aliases (S1 group names now); **separate git tree,
     committed in lockstep; BallButler `pio` build must pass.**
   - `tests/ros/test_odrive.py:192-210` — the `encode_sdo_read` /
     `encode_sdo_write` cases key on the deleted `'commutation_mapper.pos_abs'`
     name (a `KeyError` once it is gone); re-target them to `'get_gpio_states'`.
     `:315` and `:322` re-target the constant itself to
     `proto.ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES`.
   - `tests/firmware/test_odrive_protocol_xref.py:49` — `_SDO_PARAM` and its
     value comment become `"get_gpio_states"` / 726, which changes the bytes
     `:84-85` reproduce; `:135`'s decode literal moves 488 → 726 with them.
     **The native golden is therefore NOT unaffected.**
     `tests/firmware/native/test_odrive_protocol.cpp:58-59` passes the endpoint
     as a bare literal `488` and must become `726`;
     `tests/firmware/native/odrive_protocol_golden.json` is then regenerated
     (`python tests/firmware/native/build.py --odrive-golden
     tests/firmware/native/odrive_protocol_golden.json`) and committed in the
     same change — otherwise
     `test_odrive_committed_golden_matches_live_firmware` and the xref
     byte-parity cases both fail. `test_rpc_dispatch.cpp:189`'s `488` is an
     arbitrary endpoint on a *rejection* path (the value never reaches the
     wire) — cosmetic, but update it in the same pass so no bare 488 survives
     against a Pro axis.
   - `jugglebot/archived/can_interface.py:108-109` — archived; update or leave
     with a stale-name comment, implementer's call.

4. Specify the runtime guard that Phase 3 implements — **three states, not
   two**: version UNRECEIVED (normal for seconds after boot, permanent for an
   unpowered hand axis — `version_check.cpp` skips axes without
   `heartbeat_seen`) ⇒ poller quiet, zero RxSdo sent, uplink UNKNOWN, retry
   forever, **no fault**; MATCH ⇒ poll; MISMATCH ⇒ latched loud park (serial
   + uplinked invalid). The poller distinguishes the states via
   `version_received_mask()`.

Done when: codegen round-trips; the `488 → 726` literals in
`tests/firmware/native/` land together with a regenerated, committed
`odrive_protocol_golden.json`; `pytest tests/ros/test_odrive.py
tests/firmware/test_odrive_protocol_xref.py
tests/firmware/test_native_firmware.py -q` pass against the new symbol
names; the BallButler `ball_butler_main` build compiles; a grep for the
removed flat constant names returns zero hits outside `jugglebot/archived/`;
the phase's scoped checks are green (full suite at end of plan).

### Phase 3 — bridge firmware poller

New TU `Teensy_code_canbridge/gpio_poll.cpp`, hosted on **`task_homing`**
(`Teensy_code_canbridge.ino:222-233` as of `7dc347f`, `HOMING_RATE_HZ = 100`, `PRIO_HOMING =
2`) alongside `version_check_step()`, with `gpio_poll_init()` called once
from `setup()` (the `version_check_init()` pattern). That task's header
comment currently reads "idle the rest of the time — rare bench/cold-start
ops"; a continuous poller amends that — update the comment and confirm
`STACK_HOMING` (256 words) still suffices. Discipline, per bullet:

- Internally rate-limited to `JBBallDetect::CHECK_INTERVAL_MS`; at most one CAN TX
  per tick (bus pacing). **Two distinct off-switches, deliberately separate:**
  (a) `JBBallDetect::ENABLED == false` compiles the poller's body out — a
  build-time kill switch, so flipping it costs a reflash and it is *not* the
  A/B mechanism. **Mechanism constraint (Phase 1 finding):** `ENABLED` emits
  as a `constexpr bool`, not a macro, so this must be constexpr-gated code
  (early returns on `!ENABLED`; the compiler eliminates the dead body) —
  never `#if JB_BD_ENABLED`, which evaluates an undefined identifier to 0
  and silently compiles the poller out even when enabled;
  (b) when the TU is compiled in, the poller boots ON, and a serial-console
  toggle (`gpio_poll on|off`, diag console) flips it live. Phase 7 step 4's
  A/B uses (b) — no reflash between arms. A compiled-in-but-boots-OFF key
  (`start_enabled`) was considered and rejected: Phase 7 step 2 needs the
  poller live immediately after the first flash, and the partner-presence,
  commands-allowed, and `Get_Version` gates already cover the risky windows;
  the serial toggle is the manual off.
- Gated on `jugglebot_commands_allowed()` and the CAN3 partner-presence gate
  (never TX into a dead bus; the un-ACKed-TX TEC climb is a known hazard).
- Gated on the Phase 2 three-state `Get_Version` check. **This amends a
  documented contract and must do so explicitly:** `version_check.h:10-20`
  (pre-`7dc347f`) declared "ZERO version SEMANTICS in firmware — validation
  policy stays in tested Python"; the amendment now occupies `:17-27`. The amendment's root cause: the wrong-id failure mode
  *answers plausibly*, so the refusal must happen before the RxSdo leaves
  the Teensy — only firmware can do that. The firmware gains exactly one
  narrow compare (cached fw triple vs `JBBallDetect::EXPECTED_FW`), not the
  validation policy; update `version_check.h`'s split paragraph in the same
  commit.
- Two-phase non-blocking request/await state machine (BallButler's
  production shape; its blocking `readGpioStates()` helper must not be
  ported). **Invalidate the cached response before each send** — else the
  await phase re-reads the prior reply as fresh and the timeout is
  unreachable (BallButler does this at `CanInterface.cpp:792-793`).
- Request: RxSdo to node 6, `OPCODE_WRITE`, endpoint
  `EndpointId::odrive_pro_0_6_11::get_gpio_states`. Reply: new
  `case ODriveCmd::TxSdo:` in the CAN3 decode path, matching on
  `endpoint_id`, decoding the value as **`uint32`**. The existing
  `decode_sdo_response` unpacks `float32` and must not be reused; add a typed
  variant. Fix **both** stale comments claiming an encoder-search TxSdo
  consumer exists (`can_buses.cpp:123` and `rpc.cpp:251` pre-`7dc347f`; the
  replacements now sit at `:125-129` and `:251-255`) — neither handler has
  ever existed.
- Cache (one writer on `task_homing`, one reader on `task_telem` — publish
  via a seqlock in the `axis_state.h` style, or a critical-section snapshot;
  the cache holds 64-bit stamps, which tear without one): **last raw
  `uint32` word** (Phase 7's gate reads it), raw decoded bit, debounced
  verdict, saturating consecutive-miss count, a `micros64()` monotonic stamp
  of the last good reply (staleness arithmetic), and a `now_wall_us()`
  wall stamp of the same reply (the wire field — wall by contract,
  `time_base.h:46-63`).
- **Allow-table: no change.** The poller is firmware-internal (it sends via
  `can_jugglebot_send()`, like `version_check_step` — `send_axis_frame` and
  `hand_axis6_permitted()` are never involved), so the operator-locked
  `_PERMIT` list and its mirror test stay untouched. Nothing in the repo
  needs an ad-hoc hand SDO read today (`teensy_sdo_read` has no production
  caller — only `tests/ros/test_teensy_bridge_node_rpc.py:98` exercises it —
  is already rejected for axis 6, and its TxSdo reply has no return path),
  so this forecloses nothing. Any future widening of `hand_axis6_permitted`
  requires an operator re-lock of the policy, with the failure-mode
  justification written — it is not an implementer decision.
- `CanBridge::FW_VERSION` 3 → 4 with an inline history line.

Done when: `pio` build passes; firmware cross-reference tests pass; the
`enabled: false` build compiles the poller out. Landed `7dc347f`, **flashed
by the operator 2026-07-29** (first time this firmware ran on a Teensy) — the
serial toggle's live flip on the bench is still deferred to Phase 7 step 4.

### Phase 4 — additive uplink message

New `Message("HandSensor", "HAND_SENSOR", ...)` in
`config/generate_udp_protocol.py`, id **0x8B** — the next free slot in the
reserved 0x89–0x8F telemetry block (`generate_udp_protocol.py:163-168` as of
`6cc38f7`; the block's generic owner comment needed no change — the enum
entries are the ledger). Payload (packed, 14 B):

```
struct HandSensor {
  uint64_t t_bridge_us;   // Bridge WALL-clock (now_wall_us()) at the last good TxSdo reply (us)
                          // — wire-bound absolute timestamp, wall by contract (time_base.h:46-63)
  uint32_t raw_states;    // last raw get_gpio_states word, verbatim (commissioning + diagnostics)
  uint8_t  flags;         // bit0 raw_held, bit1 debounced_held, bit2 valid (=not UNKNOWN),
                          // bit3 stale, bit4 time_synced (bridge wall anchor set)
  uint8_t  miss_count;    // consecutive EMPTY readings from good replies (saturating)
}
```

Emitted from `task_telem` (`TELEM_RATE_HZ`: 100 Hz production, 250 Hz under
`BENCH_SYSID_BUILD`) alongside `hand_cmd_echo_uplink_step()`, rate-limited to
the poll rate (new good reply ⇒ new frame); while UNKNOWN/stale, a 1 Hz
keepalive frame carries the flags so staleness is itself observable.

Additive message ⇒ **no `PROTOCOL_VERSION` bump** (LegCmd precedent,
`udp_protocol.h:261` as of `6cc38f7`); `test_wire_layout_frozen`'s pinned digest changes and
is re-pinned in the same commit (the test prints the new digest);
`test_protocol_version_frozen` stays at 4. File list: the generator, the
regenerated/delivered artifacts, and `controller/teensy_link/protocol.py` —
the hand-maintained re-export shim; a message absent there is unimportable
from the stable path — plus `controller/teensy_link/__init__.py` (the
package-root re-export; both prior additive messages touched both files —
see the Phase 4 entry's Discussion).

Done when: xlang codec tests pass in both languages with the new message;
frozen-layout re-pin lands in the same commit.

### Phase 5 — ROS surface

- `teensy_bridge_node` subscribes to the new msg_type;
  `jugglebot_interfaces/HandTelemetryMessage` gains: `bool ball_held`
  (debounced), `bool ball_held_raw`, `bool ball_held_valid`, and
  `builtin_interfaces/Time ball_held_stamp`.
- **Stamp conversion:** `t_bridge_us` arrives already in Jetson-epoch
  microseconds — the bridge is the time-sync master, anchored via
  `TIME_OF_DAY_QUERY`. Convert exactly as `_publish_bb_axis_estimates` does
  (`teensy_bridge_node.py:1214` region: `sec = t_us // 1_000_000`,
  `nanosec = (t_us % 1_000_000) * 1000`). **No offset is applied on the
  Jetson side.**
- **Jetson-side staleness gate (closes the hop BallButler leaves open one
  layer up):** `ball_held_valid = False` unless (a) the frame's own
  `valid && time_synced` flags are set, AND (b) a `HAND_SENSOR` frame
  arrived within the last 3 s measured against the Jetson's monotonic clock
  at RX (≈3× the keepalive period) — never against `t_bridge_us`, which is a
  foreign wall clock that can step. Without (b), a dead bridge or dropped
  link would leave `/hand_telemetry` republishing `ball_held=True,
  valid=True` from cache forever (the publisher is a free-running 100 Hz
  timer over cached state).
- `/link_status` gains a `hand_ball_sensor` KeyValue:
  `held|empty|unknown|stale`, miss count, and the **raw word in hex** (the
  Phase 7 gate's observable).
- Deployment note (repeatedly earned in this repo): a
  `jugglebot_interfaces` change requires `colcon build` of **both**
  `jugglebot_interfaces` and `jugglebot`, then relaunch — the launch runs
  the installed copy, and a stale interfaces build kills consumers at
  startup.

Done when: `tests/ros/` covers fresh, stale-by-flags, stale-by-RX-age, and
never-seen (all three non-fresh cases ⇒ `ball_held_valid == False`), plus
the `/link_status` row; the phase's scoped checks are green (full suite at
end of plan).

### Phase 6 — GUI ball-in-hand pill + session runbook

- Build the pill in `ros_ws/gui/js/panels.js` next to the existing
  `flag-level-pill` precedent (`panels.js:~204-211` as of `73d70c6`), styled in
  `ros_ws/gui/css/panels.css` (follow the `.flag-level-pill` rule at
  `:301-304`), driven from the existing `onHandTelemetry` handler
  (`js/main.js:282` as of `73d70c6`). Four visual states: HELD (filled), EMPTY (hollow),
  UNKNOWN/STALE (greyed — never rendered as EMPTY), and a flicker marker
  when `ball_held_raw` disagrees with `ball_held` (the operator's
  spotty-contact observability). Static files, no build step; served by the
  existing `jugglebot-gui.service`.
- **This phase's implementer also writes
  `tests/hardware/session_hand_ball_sensor.md`** (pattern:
  `tests/hardware/session_phase7_reload.md`) covering Phase 7 steps 1–5 with
  exact commands and PASS/ABORT criteria, committed before the sitting —
  the operator runs robot-actuating commands from the runbook, not ad hoc.

Done when: the pill renders all four states driven from a synthetic
`HandTelemetryMessage` publisher, verified in the browser against
`jugglebot-gui.service`; the runbook is committed; the end-of-plan
full-suite gate (`pytest tests/ -q`) is green.
Live confirmation against the hand-toggled switch is Phase 7 step 2.

### Phase 7 — hardware commissioning (operator-run)

Run from `tests/hardware/session_hand_ball_sensor.md` (written in Phase 6).

1. **Flash** the Phase 3/4 bridge firmware; verify FW_VERSION 4 on the
   serial console and Phase 0's fw-version line (incl. `fw_unreleased`) on
   launch.
2. **Raw-word toggle gate (mandatory, blocking):** with the hand parked,
   press and release the switch by hand; require the **raw `uint32` word**
   — surfaced in hex on the `hand_ball_sensor` `/link_status` KeyValue
   (Phase 5) and on the serial console — to change **in bit 2
   specifically**, both edges, ≥5 cycles, with the GUI pill tracking. A
   wrong-but-existing endpoint answers with a plausible constant — only a
   moving bit 2 proves the id, the mode, and the wiring end to end. PASS ⇒
   proceed; any other bit moving, or no movement ⇒ ABORT (wrong endpoint
   table, wrong pin, or wiring fault).
3. **SDO round-trip measurement:** log request→reply latency over ≥1000
   polls on the loaded bus (idle vs during leg motion vs during a hand
   stroke). No documented figure exists anywhere; this retires the plan's
   only unmeasured term and sizes the staleness window.
4. **Stroke-jitter observation:** compare 0x0CC inter-frame timing during a
   throw with polling on vs off (the Phase 3 serial toggle — no reflash).
   If jitter is material, order the stroke-window suppression knob
   (§ Approved decisions row 2).
5. **Ball soak:** seated ball through carry/tilt motions — record raw-bit
   dropout statistics to validate or retune `max_missing_samples` (the
   100 ms window at 50 Hz vs BallButler's 250 ms is the open tuning
   question).
6. Results are recorded by the session that runs the sitting into a new
   `logbook/YYYY-MM-DD-hand-ball-sensor-commissioning.md` with (date,
   command, result) triples, then the validated `/hand_telemetry` signal is
   handed to the possession workstream.

## Testing plan

| Layer | Tests |
|---|---|
| Config | drift test JSON `gpio2_mode` ↔ YAML (Phase 1); codegen round-trip for the nested endpoint shape; grep-zero for removed flat constant names outside `jugglebot/archived/` (Phase 2) |
| Cross-repo | BallButler `ball_butler_main` `pio` build compiles against the migrated endpoint symbols (Phase 2) |
| Protocol | xlang codec tests for `HandSensor` (both languages); `test_wire_layout_frozen` re-pin; `test_protocol_version_frozen` unchanged at 4; `tests/firmware/test_hand_axis6_allow.py` untouched (allow-table unchanged by design) |
| Firmware xref | endpoint symbols + `expected_fw` cross-referenced between generated headers and `gpio_poll.cpp` (pattern: `test_odrive_protocol_xref.py`); `tests/ros/test_odrive.py` + `test_odrive_protocol_xref.py` re-targeted to `get_gpio_states`; native odrive golden regenerated for the `488 → 726` SDO rows and `test_odrive_committed_golden_matches_live_firmware` green (Phase 2) |
| ROS | new-field population fresh / stale-by-flags / stale-by-RX-age / never-seen; `ball_held_valid` gating; `/link_status` row; Phase 0 version formatting with mocked tracker |
| Hardware | Phase 7 gates 2–5, operator-run from the session runbook, results recorded with (date, command, result) triples |

Gate per commit: the scoped checks named in each phase. Full-suite gate
(`pytest tests/ -q`) **once at the end of the plan**, then once more only if
its failures force significant changes — operator direction 2026-07-29,
superseding the per-commit default in CLAUDE.md for this plan. (Phase 0
predates the direction and ran the full gate: 4256 passed, 3 xfailed,
2026-07-29.)

## Notes for collaborators

- **The endpoint id is the sharpest edge in this plan.** Do not "fix" 726
  back to 700 because BallButler uses 700 — the boards differ, and 700 on
  the Pro resolves to an unrelated register that answers plausibly. Do not
  trust a reply's existence; trust only the Phase 7 raw-word toggle.
  Symmetrically, do not "modernise" the S1 group's 488 to the Pro's 461 —
  its only production consumer is BallButler's S1, where 488 is proven in
  production. Jugglebot's own dangling name for that endpoint is **deleted**,
  not migrated (Phase 2 item 3): a name with no production caller is not worth
  binding to either board's id.
- BallButler's poll/threshold numbers are sized for a degrading sliding
  contact on a lightly loaded bus and a robot that never catches — ADR-0013
  puts CAN1's *Jugglebot-visible* load at ~1.5 %
  (`docs/adr/0013-three-can-buses.md:23`), but that figure counts only the
  BB Teensy plus time-sync and **excludes BallButler's own ODrive traffic**,
  which shares the same wire (BB runs a single `FlexCAN_T4<CAN1>` instance
  for both). Jugglebot shares the contact design (operator-confirmed) but
  not the bus load or
  the consumer cost function — treat every `jugglebot_ball_detect` value as
  provisional until Phase 7 data lands.
- Multiple Claude sessions work `mvp-trajectory-bringup` in parallel. Before
  regenerating config (Phases 1–2) or touching shared headers,
  `git fetch && git status -sb` in BOTH repos and check for another
  session's uncommitted work — codegen dirties the BallButler tree too.
- The recon behind this plan (five-agent read-only investigation,
  2026-07-28: BallButler mechanism map, ODrive push-impossibility survey,
  endpoint-id fingerprinting against the ODrive release API, comms-topology
  map, consumer/contract map) is summarised inline in § Context; primary
  sources are cited per claim.

### Out of scope (owned elsewhere)

- **Possession-source implementation** (`SOURCE_HAND_BALL_SENSOR` at the
  C-POSSESS-1 seam): owned by the possession workstream. The contract's § 3
  enumerates what a sensor-primary source inherits — three consequences (a
  source cannot originate a claim, cannot answer late, **cannot clear the
  latch**) plus one reporting wart it flags as likely to survive unnoticed —
  see `ros_ws/docs/ball_possession_contract.md` § 3 directly rather than any
  paraphrase here; the pointer is safer against contract drift. Two further
  seam facts recorded during this plan's recon: all three seam call sites
  are gated on a tracker `CAUGHT`, and `judge()` carries no time argument.

  > **LANDED 2026-08-10** as `ball_possession.HandBallSensorSource`, in
  > `plans/active/catch-robustness.md` Phase 1. Both recon facts held and both
  > shaped the design: the source is **tick-driven** (`observe(now, landing_t)`),
  > a second source kind the contract now specifies in § 3.2, precisely because
  > `judge()` carries no time; and the ball-evidence precondition became a live
  > `evidence(now)` read (§ 3.3) precisely because the seam call sites are gated
  > on a tracker `CAUGHT`.

- **`toss_require_ball_evidence` flip**: deferred to whoever validates the
  sensor, per decision (e) of the 2026-07-28 sitting. Not flipped by any
  phase of this plan.

  > **⚠ GATE SUPERSEDED 2026-08-10 — operator authorisation.** This plan's
  > position that the flip is forbidden until **Phase 7** validates the sensor is
  > superseded: the operator validated the sensor **in situ** and authorised both
  > the sensor-primary possession source and the flip (recorded in
  > `plans/active/catch-robustness.md` § Owner decisions). The default is now
  > `true`. **Phase 7 steps 4–5 remain open as bench work, not as blockers** —
  > they size the debounce/poll knobs and measure SDO RTT, which is tuning on a
  > sensor already known to work, not permission to use it. The supporting
  > evidence for the authorisation is in the bags rather than the bench: 203,922
  > `/hand_telemetry` samples across three 2026-08-10 sessions at **100 %
  > `ball_held_valid`**, with the sensor's arrival edge landing +137…+798 ms after
  > every announced catch and no non-catch edge closer than +3194 ms
  > (`tools/probes/hand_sensor_verdict_replay.py`).
  >
  > Note what this does **not** waive: the plan's decision-9 requirement that the
  > signal be tri-state and that `ball_seated` treat UNKNOWN/stale as *not*
  > seated is honoured exactly — an UNKNOWN sensor mints `REJECTED_BALL_UNKNOWN`
  > and refuses the throw.
- **Post-release decel fix + end-stop anchor**: separate ordered work
  (decision (b) of the same sitting).
- **BallButler fail-open reload-skip defect** (found during this
  investigation, shipping today): BallButler heartbeats `ball_in_hand = true`
  from boot before its first GPIO read, and `reload_coordinator_node` gates
  that bit only on heartbeat freshness, so a freshly-rebooted BallButler
  makes the reload FSM skip `ACTION_CALL_RELOAD`. Own investigation/logbook;
  the Phase 5 validity pattern is the shape of its fix.
