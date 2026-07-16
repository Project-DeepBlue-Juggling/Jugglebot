#pragma once
// =============================================================================
//  canbridge_config.h — firmware-wide configuration for the can-bridge Teensy
// =============================================================================
//  Hand-authored (NOT codegen). Pulls protocol/hardware constants from the
//  generated headers and adds firmware-only configuration: pin map, FreeRTOS
//  task table, and the control/safety constants ported 1:1 from the Jetson
//  Python (motor_guard.py / can_node.py) so behaviour is identical.
//
//  Single source of truth for any value that also lives in Python is the
//  generated config; values mirrored here from Python source files are marked
//  with their origin so a future codegen pass can hoist them.
// =============================================================================

#include <cstdint>
#include "protocol_config.h"   // CAN protocol: CanBus, ODriveCmd, NodeId, SharedCanId, InputScale...
#include "hardware_config.h"   // Geometry, ODrive limits, operational constants
#include "udp_protocol.h"      // JbUdp:: wire protocol (ports, IPs, framing)

// ── BENCH_SYSID_BUILD — one-off bench-measurement variant (NEVER production) ──
// DEFAULT OFF. A dedicated compile-time flag that lifts three measurement blind
// spots for on-wire gain-ladder / system-ID runs on the single-leg bench rig:
//   * 250 Hz telemetry (was 100), so a pos_gain-90 loop (~14 Hz closed-loop BW)
//     is not telemetry-aliased;
//   * 100 Hz knots (SEGMENT_T_S 0.010, was 0.025) — the honest chirp ceiling is
//     otherwise KNOT-bound at 8 Hz (knot_stream_top_freq = knot_hz/5); the Jetson
//     harness MUST then run --knot-hz 100 to match this segment duration;
//   * a 250 Hz forced DIAGNOSTIC on axis 0, so iq_measured (only carried by the
//     on-change-gated DIAGNOSTIC) is observable at rate for the current-rail and
//     vel_gain-buzz ceilings the on-change gate hides.
// It changes CADENCE and internal timing ONLY — ZERO wire-format changes (no
// struct/field/protocol-version edit). With the flag OFF the binary is IDENTICAL
// to production (every use sits behind #if BENCH_SYSID_BUILD). Build the variant
// with `pio run -e teensy41_bench_sysid` (platformio.ini adds -DBENCH_SYSID_BUILD=1);
// DO NOT flash it to the assembled robot.
#ifndef BENCH_SYSID_BUILD
#define BENCH_SYSID_BUILD 0
#endif

namespace CanBridge {

// ── Identity ────────────────────────────────────────────────────────────────
constexpr char     FW_NAME[]    = "jugglebot-canbridge";
constexpr uint16_t FW_VERSION   = 2;          // bump on behavioural change (1→2: 2026-07-16 MAX_DEVIATION_REV 0.5→1.0 guard raise)

// ── Network (static, point-to-point /30) ─────────────────────────────────────
// IP octets and ports come from the generated udp_protocol.h (JbUdp::).
constexpr uint8_t  TEENSY_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::TEENSY_IP_3 };
constexpr uint8_t  JETSON_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::JETSON_IP_3 };
constexpr uint8_t  NETMASK[4]   = { 255, 255, 255, 252 };   // /30
// No gateway, no DHCP, no mDNS (static point-to-point link).

// ── CAN bus wiring (Teensy 4.1) — three subsystem-isolated buses (ADR-0013) ───
//  One FlexCAN_T4 peripheral per robot subsystem (supersedes the old two-bus
//  shared-aux / private-leg split). Pin directions below are the FlexCAN_T4
//  silicon-fixed DEF mux (FlexCAN_T4.tpp setTX/setRX, invoked by begin()) — the
//  library default that can_buses.cpp actually runs; these constants are
//  documentation only (nothing reads them; the peripheral picks its own pads).
//
//    CAN1 = Ball Butler bus     (BB Teensy only)                  TX 22 / RX 23
//    CAN2 = catching cone bus   (cone Teensy, often disconnected) TX  1 / RX  0
//    CAN3 = Jugglebot core bus  (6 leg ODrives + Hand ODrive +    TX 31 / RX 30
//           platform Teensy 4.0 + can-bridge)
//
//  CAN3 is the FD-capable peripheral, run classical 1 Mbps today to match the
//  classical-only ODrive firmware; a future CAN-FD upgrade is a config change,
//  not a rewire. All three buses carry the 100 Hz 0x7DD time-sync broadcast.
//  NB: ADR-0013 lists CAN2 and CAN3 TX/RX reversed — the
//  FlexCAN_T4 silicon mux above is authoritative.
constexpr uint32_t CAN_BITRATE  = CanBus::BAUD_RATE;   // 1 Mbps, all nodes agree
constexpr uint8_t  CAN1_TX_PIN  = 22;   // Ball Butler
constexpr uint8_t  CAN1_RX_PIN  = 23;
constexpr uint8_t  CAN2_TX_PIN  = 1;    // catching cone (FlexCAN_T4 DEF: TX=1, RX=0)
constexpr uint8_t  CAN2_RX_PIN  = 0;
constexpr uint8_t  CAN3_TX_PIN  = 31;   // Jugglebot core (FlexCAN_T4 DEF: TX=31, RX=30)
constexpr uint8_t  CAN3_RX_PIN  = 30;

// ── Status LED ────────────────────────────────────────────────────────────────
constexpr uint8_t  LED_PIN      = 13;          // Teensy on-board LED

// ── Axis layout (mirrors odrive.py groupings) ────────────────────────────────
constexpr uint8_t  NUM_LEGS     = JbUdp::NUM_LEGS;   // 6, node ids 0..5
constexpr uint8_t  HAND_AXIS    = NodeId::JUGGLEBOT_HAND;  // 6
constexpr uint8_t  NUM_AXES     = JbUdp::NUM_AXES;   // 7 (legs + hand) — cache + telemetry breadth

// ── Loop rates ────────────────────────────────────────────────────────────────
constexpr uint32_t INTERP_RATE_HZ      = 500;     // motor_guard DEFAULT_RATE_HZ
constexpr uint32_t INTERP_PERIOD_US    = 1000000u / INTERP_RATE_HZ;   // 2000 us
constexpr uint32_t TIME_SYNC_RATE_HZ   = 100;     // 0x7DD broadcast (can_node ts_period 0.01)
constexpr uint32_t TIME_SYNC_PERIOD_US = 1000000u / TIME_SYNC_RATE_HZ;
#if BENCH_SYSID_BUILD
constexpr uint32_t TELEM_RATE_HZ       = 250;     // bench-only: un-alias the ~14 Hz pos_gain-90 loop (4 ms task tick; configTICK_RATE_HZ=1000)
#else
constexpr uint32_t TELEM_RATE_HZ       = 100;     // motor-state uplink
#endif
constexpr uint32_t TELEM_PERIOD_US     = 1000000u / TELEM_RATE_HZ;
constexpr uint32_t HEARTBEAT_RATE_HZ   = JbUdp::HEARTBEAT_HZ;   // 10
constexpr uint32_t DIAG_HEARTBEAT_HZ   = 1;       // 1 Hz forced diagnostic/profile refresh
constexpr uint32_t FAULT_TASK_HZ       = 10;      // can_node fault cadence
constexpr uint32_t WATCHDOG_HZ         = 1;       // can_node 1 Hz heartbeat watchdog
// Homing monitor. 100 Hz ≈ the Iq update rate so the current EMA tracks
// can_node's per-reading cadence (a slower tick over-smooths → laggier, but still
// safe: it detects the stall later, never harder than the ODrive current limit).
constexpr uint32_t HOMING_RATE_HZ      = 100;

// Time-of-day drift re-sync interval (design range 10-60 s).
constexpr uint32_t TIMEOFDAY_RESYNC_MS = 30000u;  // 30 s

// ── FreeRTOS task table (priorities + stacks) ────────────────────────────────
//  Priority 6 (interp) is the HIGHEST — nothing preempts the 500 Hz tick.
//  Stacks are in StackType_t WORDS (4 bytes on Cortex-M7). Budgets are in
//  bytes; *_STACK is the word count = bytes / 4.
constexpr uint8_t PRIO_INTERP      = 6;   // leg_interp (hard deadline)
constexpr uint8_t PRIO_CAN_TX      = 5;   // CAN TX mailbox drain
constexpr uint8_t PRIO_CAN_RX      = 5;   // can_rx task (pumps all three buses)
constexpr uint8_t PRIO_UDP_RX      = 4;   // usb_rx (UDP downlink)
constexpr uint8_t PRIO_TIME_SYNC   = 4;   // time_sync_master
constexpr uint8_t PRIO_UDP_TX      = 3;   // usb_tx (telemetry uplink)
constexpr uint8_t PRIO_FAULT       = 3;   // fault_state
constexpr uint8_t PRIO_HOMING      = 2;   // leg_homing monitor (rare bench op)
constexpr uint8_t PRIO_WATCHDOG    = 2;   // watchdog
constexpr uint8_t PRIO_DIAG        = 1;   // diag / profiling

constexpr uint16_t STACK_INTERP    = 2048 / 4;   // 512 words
constexpr uint16_t STACK_CAN_TX    = 1024 / 4;
constexpr uint16_t STACK_CAN_RX    = 2048 / 4;
constexpr uint16_t STACK_UDP_RX    = 4096 / 4;
constexpr uint16_t STACK_TIME_SYNC = 1024 / 4;
constexpr uint16_t STACK_UDP_TX    = 4096 / 4;
constexpr uint16_t STACK_FAULT     = 2048 / 4;
constexpr uint16_t STACK_HOMING    = 1024 / 4;   // state machine + a few CAN sends
constexpr uint16_t STACK_WATCHDOG  = 1024 / 4;
constexpr uint16_t STACK_DIAG      = 8192 / 4;   // bumped from 1024 (7-arg printf overflowed), again from 4096
                                                 // for the 13-arg [canerrs] diagnostic printf (2026-07-05)

// ── Interpolator constants (PORTED 1:1 from motor_guard.py) ──────────────────
//  Source: ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py constants block.
#if BENCH_SYSID_BUILD
// Bench-only: 100 Hz knots. SEG_T is the Hermite segment span and MUST equal the
// actual inter-knot interval — the interpolated velocity uses invT = 1/SEG_T and
// v1 = (u2-u1)/SEG_T, so a SEG_T that disagrees with the send cadence distorts the
// velocity profile (each segment re-latches at s = dt/SEG_T before completing). The
// Jetson harness must run --knot-hz 100 so knots actually arrive every 10 ms.
constexpr float    SEGMENT_T_S          = 0.010f;  // 100 Hz knots (matches --knot-hz 100)
#else
constexpr float    SEGMENT_T_S          = 0.025f;  // _mpc_segment_T (nominal MPC fine step)
#endif
// MAX_EXTRAP / EXTRAP_DECAY / JERK_EMA are NOT knot-cadence-derived: they are the
// missed-knot graceful-degradation windows (Taylor-extrapolate then decay velocity
// to a safe stop) and the jerk-EMA memory, all consumed ONLY when has_next is false.
// The bench harness streams Hermite (has_next true), where s is clamped to 1.0 and
// these never fire; and they are wall-clock safety ramps (a leg coasting to rest),
// not knot-count. Justify-KEEP at their absolute production values under both builds.
constexpr float    MAX_EXTRAP_DT_S      = 0.05f;   // MAX_EXTRAP_DT_S
constexpr float    EXTRAP_DECAY_DT_S    = 0.06f;   // EXTRAP_DECAY_DT_S
constexpr float    JERK_EMA_ALPHA       = 0.3f;    // JERK_EMA_ALPHA
// MAX_LEAD_REV lead-clamp: cap the commanded position at encoder ± this.
// Lowered 0.15 → 0.10 (2026-07-10 MAX_DEVIATION-runaway forensics). The ODrive
// legs run POSITION/PASSTHROUGH with LEG_POS_GAINS = 40 and the LEG_VEL_LIMIT_RPS
// pushed from hardware_config leg_vel_limit_rps (4.0 through 2026-07-15; 6.0 then
// 12.0 on 2026-07-16 — see logbook 2026-07-16-max-deviation-guard-tracking-lag.md).
// While the clamp is engaged the position error is pinned at MAX_LEAD_REV, so the
// position loop's velocity command is pos_gain × lead. At the old 0.15 that was
// 40 × 0.15 = 6.0 rev/s — ABOVE the THEN-4.0 vel_limit — so the loop saturated the
// limit, sprinted, overshot, braked (measured negative iq), and re-lagged: a ~6 Hz
// bang-bang limit cycle whose accumulated lead crossed MAX_DEVIATION_REV and latched
// the guard.
// At 0.10, 40 × 0.10 = 4.0 bounds the position-loop P-term (pos_gain × lead) to
// 4.0 rev/s — but that is the P-TERM ALONE. The ODrive's velocity setpoint is
// vel_ff + pos_gain × lead, and vel_ff is now PASSED THROUGH when the clamp engages
// (bounded to LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5, no longer zeroed — see below).
// Against the OLD 4.0 vel_limit the P-term (4.0) EXACTLY equalled the limit, so when
// the clamp engaged the setpoint was already pinned at 4.0 by the P-term and vel_ff
// had ZERO room to add — every rev/s of feedforward was clipped, and the steady
// clamped sprint stayed vel_limit-bound. With the 2026-07-16 raises (4.0 → 6.0 → 12.0)
// the constraint becomes an INEQUALITY: 40 × 0.10 = 4.0 ≤ vel_limit, so the P-term sits
// well BELOW the limit and vel_ff regains genuine catch-up authority — the full 3.5
// VELFF cap now passes unclipped, making the clamp-engaged velocity setpoint top out at
// 4.0 + 3.5 = 7.5 rev/s (the VELFF cap, not vel_limit, is the binding bound). What the 0.10 +
// vel_ff-pass-through pair bought at 2026-07-10 was BOUNDARY CONTINUITY (no vel_ff
// discontinuity at clamp engage, a P-term no longer above the limit, killing the ~6 Hz
// limit cycle); the 6.0 raise converts the remaining P-term-EQUALS-limit saturation
// into usable catch-up headroom. MAX_LEAD stays 0.10 — do NOT change it; Kp·MAX_LEAD ≤
// vel_limit (the v2-bug constraint) is exactly what un-saturates the velocity setpoint.
constexpr float    MAX_LEAD_REV         = 0.10f;
// Feed-forward velocity cap applied to the interpolated vel_ff before it reaches the
// leg ODrive (2026-07-10 forensics). The lead clamp used to ZERO vel_ff whenever it
// engaged; that discontinuity (and the pos-loop sprint above) seeded the stutter.
// leg_interp now sends the TRUE interpolated vel_ff instead of zeroing it, bounded to
// this magnitude — kept below the drive vel_limit so a runaway command can never inject
// an over-vel_limit feedforward. NOTE (2026-07-16, vel_limit now 12): this 3.5 cap is now
// the BINDING clamp-engaged catch-up bound (Kp·MAX_LEAD + 3.5 = 7.5 rev/s ≈ 529 mm/s of
// leg speed); raising it toward ~10 is a flash item for the accel-FF chapter — keep it
// strictly below the drive vel_limit when raised.
constexpr float    LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5f;
// MAX_DEVIATION_REV (E-stop): raw streamed u0 vs encoder guard, checked at FAULT_TASK_HZ
// (fault_machine.cpp). Raised 0.5 → 1.0 (2026-07-16 max-deviation-guard-tracking-lag
// forensics). Three guard E-STOPs latched during the S4 limit-ramp bench session at
// leg-vel 200 mm/s (trip deviations 0.52 / 0.55 / 0.56 rev, always leg 1 first, always
// at move onset). Those deviations were LEGITIMATE velocity-loop lag under coordinated-
// move load — the reflected platform inertia (~5-20× the bare rotor J_eff) makes the
// velocity loop lag the commanded ramp, and the deficit integrates into position
// deviation superlinearly with velocity (0.17-0.22 rev at 100 mm/s, ~0.41 at 160,
// 0.52-0.56 at ~190) — NOT runaway (peak iq_setpoint 8.7 A of 10 A, peak vel 2.2 of
// 4.0 rev/s: never current- nor velocity-railed). The guard watches the RAW streamed
// u0 against the encoder PRE lead-clamp, so it counts exactly the command-space lag the
// lead clamp deliberately tolerates. Raising it adds ZERO physical excursion for
// command-side faults: the executed command is bounded independently by the lead clamp
// (enc ± MAX_LEAD 0.10) + the stroke clamp + the ODrive position clip. The accepted cost
// is the encoder-side-runaway detect distance: 0.5→1.0 rev ⇒ 35→70 mm before a true
// encoder-side runaway latches (still caught; the firmware MAX_MOTOR_VEL_RPS = 16.5
// overspeed guard is unchanged). NOTE: the legacy Jetson-side MPC-path guard
// (motor_guard.py MAX_DEVIATION_REV) deliberately stays 0.5 — it has seen none of this
// data and its path is dormant on the mvp branch; re-decide when the MPC path revives.
// See logbook 2026-07-16-max-deviation-guard-tracking-lag.md.
constexpr float    MAX_DEVIATION_REV    = 1.0f;
// MAX_MOTOR_VEL_RPS = ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1 (10% noise margin).
constexpr float    MAX_MOTOR_VEL_RPS    = ODriveDefaults::TRAP_VEL_LIMIT_RPS * 1.1f;  // 16.5

// ── Staleness / liveness thresholds (PORTED 1:1) ─────────────────────────────
//  motor_guard.py: MPC_CMD_STALENESS_S=0.25, MOTOR_FB_STALENESS_S=0.15.
//  can_node.py:   _HEARTBEAT_TIMEOUT_S=2.0 (CAN heartbeat watchdog).
constexpr uint32_t MPC_CMD_STALENESS_US = 250000u;   // 0.25 s → E-STOP (link-fault trigger)
constexpr uint32_t MOTOR_FB_STALENESS_US = 150000u;  // 0.15 s → suppress commands
constexpr uint32_t CAN_HEARTBEAT_TIMEOUT_US = 2000000u;  // 2.0 s → fatal_can_error
// Ball Butler heartbeat (CAN1) is tighter: BB broadcasts at 10 Hz and the
// production timeout (can_node._bb_heartbeat_timeout_s = proto.BB_HEARTBEAT_TIMEOUT_MS
// / 1000) is 0.5 s — only 5 missed heartbeats vs the ODrive's 20. Reused 1:1 here.
constexpr uint32_t BB_HEARTBEAT_TIMEOUT_US = BallButler::HEARTBEAT_TIMEOUT_MS * 1000u;
// Bus-partner presence window (generalised 2026-07-05 from CONE_PRESENT_STALENESS_US).
// EVERY can_*_send() gates its TX on "some partner frame arrived on this bus within
// this window" — never transmit into a bus with no recently-seen partner. Un-ACKed TX
// into a dead/unpowered bus retransmits forever, pins TEC at the error-passive
// threshold (128), and during supply transitions can push through to bus-off — the
// sticky BUSOFF/tec=254 pollution diagnosed in the 2026-07-05 marginal-CAN3
// investigation. Originally shipped for CAN2 only (cone-absent tolerance); the failure class is identical on
// all three buses (CAN1 pinned tec=128/passive from a Ball-Butler-absent window, CAN3
// reached BUSOFF across robot-supply cycles). A partner counts as "present" if any
// frame arrived within this window; generous so a brief silence (e.g. the ~3.5 s
// worst-case ODrive reboot gap) doesn't blip the gate — and on CAN3 the Platform
// Teensy's 2 Hz TRAFFIC_REPORT keeps the bus "present" through ODrive-only outages.
constexpr uint32_t BUS_PARTNER_STALENESS_US = 5000000u;  // 5.0 s
// Jetson UDP link: declare lost after LINK_LOST_MISSES missed heartbeats.
constexpr uint32_t JETSON_LINK_TIMEOUT_US =
    (1000000u / HEARTBEAT_RATE_HZ) * JbUdp::LINK_LOST_MISSES;   // 500 ms

// ── Fault state machine constants (PORTED 1:1 from can_node.py / motor_state.py) ─
//  motor_state.py: max_soft_reset_attempts = 1 (the cap exists because of a real
//  soft-reset bounce-loop incident — DO NOT raise it without that context).
constexpr uint8_t  MAX_SOFT_RESET_ATTEMPTS = 1;
//  odrive.py ERR_DC_BUS_UNDER_VOLTAGE = 512.
constexpr uint32_t ERR_DC_BUS_UNDER_VOLTAGE = 512u;

// Reboot-in-progress watchdog-suppression window (see logbook 2026-06-30-canbridge-phase6-reboot-latch). A REBOOT_ODRIVES RPC
// arms fault_notify_reboot_started(), which suppresses the CAN3 CAN-loss detector
// for this bounded window so the deliberate ODrive-reboot silence is NOT read as a
// real CAN loss (which would falsely arm the 2026-05-19 deferred stow). Sized to the
// MEASURED reboot→first-leg-heartbeat latency + margin: 2.3–3.5 s across 3 bench runs
// (motor power off, 2026-06-30, tools/probes/canbridge_reboot_latch_probe.py) + ~2.5 s
// headroom for reboot variance / ODrive cold-boot — NOT the untuned ~10 s can_node
// assumed. With the fresh-heartbeats-after-stale release, a normal reboot releases the
// latch the instant the legs return (well before this deadline), so this deadline only
// bounds the FAILURE case (legs never return); during it the ODrives autonomously hold
// their last setpoint (safe — the deferred-stow inversion), so the blind-spot cost is
// a bounded delay of the eventual safe-stow, never a collapse.
constexpr uint64_t REBOOT_WATCHDOG_SUPPRESS_US = 6000000ULL;  // 6.0 s

// Deferred-stow profiled descent (mirrors can_node _gently_move_to_setpoint):
// velocity-limited move to the off pose at the same limit on_shutdown uses.
constexpr float GENTLE_MOVE_VEL_LIMIT_RPS = JBOp::GENTLE_MOVE_VEL_LIMIT_RPS;  // 2.5
constexpr float STOW_OFF_POSE_REV = 0.0f;    // off pose = fully retracted (can_node stows to 0.0)
constexpr float STOW_DONE_EPS_REV = 0.01f;   // legs within this of the off pose ⇒ stow complete
// Accel limit for the stow descent so the commanded velocity ramps (no startup
// step). Reaches the gentle-move velocity limit in ~0.25 s.
constexpr float STOW_ACCEL_RPS2 = 10.0f;

// ── Re-enable recovery slew (2026-07-11 clear-errors jolt forensics) ──────────
// On the s_output_enabled false→true edge (every /clear_errors, /recover, arm, and
// fb-stale/stow resumption) the ISR re-baselines the transmitted command to the LIVE
// encoder and slews toward the streamed (lead-clamped) command at this bounded
// velocity + accel. Root cause it defends: while output is suppressed the ODrive
// holds its frozen setpoint (enc_freeze + lead) while the leg physically drifts onto
// the live encoder, so at the re-enable instant the streamed command can sit up to a
// full lead clamp (0.10 rev) BELOW the encoder. Transmitting it directly injects
// pos_gain × lead = 40 × 0.10 = a 4 rev/s velocity step straight to the -10 A current
// rail on every leg (measured on both clear events, 2026-07-11 forensics RESULT 3).
// The bounded slew replaces that step with a visibly-gentle ramp. VEL is kept well
// under the gentle-move limit (2.5) so the recovery reads as deliberate, not a lurch;
// ACCEL smooths the onset so the very first re-enabled frame is at the encoder with
// zero vel_ff (no velocity step either). At 5 rev/s² a worst-case full-clamp 0.10 rev
// slew peaks at exactly √(2·5·0.10) = 1.0 rev/s and completes in ~0.2 s.
constexpr float RECOVER_SLEW_VEL_RPS       = 1.0f;    // bounded recovery slew velocity (rev/s)
constexpr float RECOVER_SLEW_ACCEL_RPS2    = 5.0f;    // onset accel so there is no vel step at the edge
constexpr float RECOVER_SLEW_DONE_EPS_REV  = 0.002f;  // per-leg converged band → hand back to normal streaming

// ── Per-leg stroke clamp bounds (rev) ────────────────────────────────────────
//  Backstop matching motor_guard's _stroke_min_rev / _stroke_max_rev, which are
//  WorkspaceLimits.from_geometry(leg_hard_{min,max}_mm) * GEOM_MM_TO_REV (per-leg).
//  Captured 2026-06-01 from the live MotorGuard; the xref harness asserts these
//  equal the running guard's values so drift is caught.
//  TODO: hoist into codegen (derive from workspace limits in YAML).
constexpr float STROKE_MIN_REV[NUM_LEGS] =
    { 0.070917f, 0.070954f, 0.070448f, 0.070934f, 0.071340f, 0.071248f };
constexpr float STROKE_MAX_REV[NUM_LEGS] =
    { 3.900413f, 3.902459f, 3.874629f, 3.901381f, 3.923703f, 3.918615f };

// ── ODrive position clip bounds (odrive.clip_position) ───────────────────────
constexpr float LEG_MOTOR_MAX_POSITION  = Geometry::LEG_MOTOR_MAX_POSITION_REVS;   // 4.2
constexpr float HAND_MOTOR_MAX_POSITION = Geometry::HAND_MOTOR_MAX_POSITION_REVS;  // 11.1

// ── ODrive int16 input scaling (matches protocol_config InputScale) ──────────
constexpr float LEG_VEL_SCALE  = InputScale::leg_vel;   // 1000.0
constexpr float LEG_TOR_SCALE  = InputScale::leg_tor;   // 10000.0
constexpr float HAND_VEL_SCALE = InputScale::hand_vel;  // 100.0
constexpr float HAND_TOR_SCALE = InputScale::hand_tor;  // 100.0

}  // namespace CanBridge
