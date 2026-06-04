// =============================================================================
//  can_buses.cpp — three subsystem-isolated FlexCAN_T4 buses + RX decode + TX
// =============================================================================
#include "can_buses.h"

#include <FlexCAN_T4.h>
#include "canbridge_config.h"
#include "protocol_config.h"
#include "axis_state.h"
#include "time_base.h"
#include "odrive_protocol.h"

namespace CanBridge {

// Peripheral identity is confined to these three lines (ADR-0013 / HANDOFF D3):
//   bb = CAN1 (pins 22/23), cone = CAN2 (pins 1/0), jugglebot = CAN3 (pins 31/30).
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_bb;
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_cone;
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_jugglebot;

static volatile uint32_t s_bb_rx = 0, s_bb_tx = 0, s_cone_rx = 0, s_cone_tx = 0,
                         s_jugglebot_rx = 0, s_jugglebot_tx = 0;
static volatile uint64_t s_bb_last_rx_us = 0, s_cone_last_rx_us = 0, s_jugglebot_last_rx_us = 0;

// RX-health observability counters (see can_buses.h "RX-health observability").
// Written only in task_can_rx context — the service loop (depth/cap/bus-error
// fields) and the decode callback (decode_* fields) — and read by task_diag. Every
// field is a single word (atomic on Cortex-M7) and cumulative/sticky, so the
// cross-task read needs no lock and no reset clears a rare transient.
static volatile BusRxHealth s_bb_rxh{}, s_cone_rxh{}, s_jugglebot_rxh{};
static volatile uint32_t s_decode_short = 0, s_decode_bad_axis = 0;

// Decode an ODrive frame into the per-axis cache. Used by the Jugglebot bus only
// (CAN3 carries all 6 leg ODrives + the Hand ODrive).
static void decode_into_cache(const CAN_message_t& msg) {
  const uint8_t axis = ODrive::axis_of(msg.id);
  const uint8_t cmd  = ODrive::cmd_of(msg.id);
  if (axis >= NUM_AXES) { s_decode_bad_axis++; return; }   // node id we don't cache (stray/garbled)
  if (msg.len < 8) { s_decode_short++; return; }           // truncated (mirrors odrive.py _check_len;
                                                           // every ODrive telemetry frame we decode is DLC 8)
  AxisState& a = axes[axis];
  const uint8_t* d = msg.buf;

  switch (cmd) {
    case ODriveCmd::heartbeat_message: {
      auto h = ODrive::decode_heartbeat(d);
      a.axis_state       = h.state;
      a.procedure_result = h.procedure_result;
      a.trajectory_done  = h.trajectory_done;
      a.last_heartbeat_us = now_wall_us();
      a.heartbeat_seen   = true;
      a.heartbeat_stale  = false;
      break;
    }
    case ODriveCmd::get_error: {
      auto e = ODrive::decode_error(d);
      a.active_errors = e.active_errors;
      a.disarm_reason = e.disarm_reason;
      break;
    }
    case ODriveCmd::get_encoder_estimate: {
      auto p = ODrive::decode_encoder_estimate(d);   // (pos, vel) in ODrive convention
      // Sign-flip legs to the Jugglebot convention (positive = extension),
      // mirroring can_node._handle_encoder's _leg_sign.
      const float pos = ODrive::leg_sign(axis, p.a);
      const float vel = ODrive::leg_sign(axis, p.b);
      write_pos_vel(a, pos, vel, now_wall_us());
      break;
    }
    case ODriveCmd::get_iq: {
      auto q = ODrive::decode_iq(d);
      a.iq_setpoint = q.a; a.iq_measured = q.b;
      break;
    }
    case ODriveCmd::get_temps: {
      auto t = ODrive::decode_temps(d);
      a.temp_fet = t.a; a.temp_motor = t.b;
      break;
    }
    case ODriveCmd::get_bus_voltage_current: {
      auto v = ODrive::decode_bus_voltage_current(d);
      a.bus_voltage = v.a; a.bus_current = v.b;
      break;
    }
    // get_version / TxSdo handled elsewhere (encoder-search Phase 9). Ignore here.
    default:
      break;
  }
}

// CAN1 Ball Butler: no ODrive on this bus → count only, never decode.
static void on_bb_rx(const CAN_message_t& /*msg*/) {
  s_bb_rx++;
  atomic_write_u64(&s_bb_last_rx_us, now_wall_us());   // 64-bit; read by health_of
}

// CAN2 catching cone: no ODrive on this bus → count only. The RX timestamp also
// drives the cone-presence gate in can_cone_send() (cone-absent tolerance).
static void on_cone_rx(const CAN_message_t& /*msg*/) {
  s_cone_rx++;
  atomic_write_u64(&s_cone_last_rx_us, now_wall_us());   // 64-bit; read by the cone gate + health_of
}

// CAN3 Jugglebot core: every ODrive frame (legs 0..5 + hand) decodes into the cache.
static void on_jugglebot_rx(const CAN_message_t& msg) {
  s_jugglebot_rx++;
  atomic_write_u64(&s_jugglebot_last_rx_us, now_wall_us());   // 64-bit; read by health_of
  decode_into_cache(msg);
}

void can_buses_init() {
  can_bb.begin();
  can_bb.setBaudRate(CAN_BITRATE);
  can_bb.setMaxMB(16);
  can_bb.enableFIFO();
  can_bb.enableFIFOInterrupt();
  can_bb.onReceive(on_bb_rx);

  // CAN2 cone-absent tolerance (candidate 3, "gated broadcast" — HANDOFF D2).
  // The bus is brought up identically to the others; the tolerance lives in
  // can_cone_send() below, which withholds TX whenever no cone frame has arrived
  // within CONE_PRESENT_STALENESS_US. With the cone disconnected we therefore
  // never transmit into an un-ACKed bus, so the FlexCAN TEC never climbs and CAN2
  // never enters bus-off — the failure is PREVENTED, not recovered from.
  // Why not the other candidates: the pinned FlexCAN_T4 exposes no one-shot-TX
  // API (candidate 1) and no bounded bus-off-recovery setter (candidate 2 would
  // need raw ESR1/ECR register work); the software gate is the least-risky option
  // that compiles cleanly against the pinned library.
  can_cone.begin();
  can_cone.setBaudRate(CAN_BITRATE);
  can_cone.setMaxMB(16);
  can_cone.enableFIFO();
  can_cone.enableFIFOInterrupt();
  can_cone.onReceive(on_cone_rx);

  can_jugglebot.begin();
  can_jugglebot.setBaudRate(CAN_BITRATE);
  can_jugglebot.setMaxMB(16);
  can_jugglebot.enableFIFO();
  can_jugglebot.enableFIFOInterrupt();
  can_jugglebot.onReceive(on_jugglebot_rx);
}

// Per-tick RX drain budget. FlexCAN_T4::events() pops EXACTLY ONE frame from the
// peripheral's RX_SIZE_256 rxBuffer per call (FlexCAN_T4.tpp: a single
// `if (rxBuffer.size()) { pop_front; mbCallbacks; }`, not a loop), so one call per
// 1 kHz task_can_rx tick caps RX decode at ~1,000 fps. CAN3 receives ~2,240 fps of
// ODrive telemetry steady (~2,740 with a throw) — the ~5,340 fps bus total minus our
// own ~3,100 fps setpoint + time-sync TX, which the peripheral's SRX_DIS self-reception
// disable never feeds back to RX. At a 1,000 fps drain the 256-deep rxBuffer saturates
// within ~50 ms and then overwrites oldest-first: ~55% of telemetry is dropped before
// decode and the survivors reach the cache ~110 ms stale (buffer depth / arrival rate).
// We instead drain each bus to empty, capped at this budget per tick. 32 → 32,000 fps
// drain capacity, ~4x the ~7,700 fps physical 1 Mbps classical-CAN frame ceiling, so
// even a saturated/babbling bus cannot grow the buffer (steady state empties in ~3
// iterations; a full buffer clears in ~8 ticks). The cap also bounds the worst-case
// per-tick decode cost during a backlog burst.
//
// Why this keeps the 500 Hz interp ISR un-starved: decode runs in this priority-5
// FreeRTOS task, BELOW the leg-setpoint IntervalTimer ISR (leg_interp.cpp, NVIC
// priority 32, above the FreeRTOS syscall ceiling), which preempts the task at any
// instruction. A larger per-tick drain therefore cannot delay the leg setpoints; it
// only shortens the tick slice left to LOWER-priority tasks (net, telem, fault), and
// only while a backlog actually exists — the loop breaks the instant the buffer empties.
// The bounded drain also REDUCES the seqlock/rxBuffer race surface vs today: with the
// buffer kept near-empty, the ISR's overwrite-oldest head advance (which races task-side
// pops) effectively never fires, where today the always-full buffer races on every push.
static constexpr uint8_t CAN_RX_DRAIN_BUDGET = 32;

// Drain one bus's rxBuffer to empty (or the budget), decoding each frame via its
// onReceive callback. events() returns (rxBuffer.size() << 12) | txBuffer.size();
// stop once no RX frames remain. The first call always runs, so the TX-mailbox-drain
// half of events() is serviced every tick exactly as before — no TX regression.
// Drain the FlexCAN ESR1-change history (≤16 deep, captured by the library ISR on
// every error/bus-state change — CTRL_ERR_MSK is enabled) into the per-bus health
// counters. Polled every service tick so the history never overflows and no event is
// missed. error() returns false once the history is empty (cheap — a size check, no
// IRQ mask), so the steady-state cost is one read per bus per tick.
template <typename BusT>
static inline void poll_bus_errors(BusT& bus, volatile BusRxHealth& h) {
  CAN_error_t e;
  while (bus.error(e, /*printDetails=*/false)) {
    h.err_events++;
    uint8_t f = h.err_flags;
    if (e.ACK_ERR)  f |= BusErrFlag::ACK;
    if (e.CRC_ERR)  f |= BusErrFlag::CRC;
    if (e.FRM_ERR)  f |= BusErrFlag::FORM;
    if (e.STF_ERR)  f |= BusErrFlag::STUFF;
    if (e.BIT0_ERR) f |= BusErrFlag::BITERR0;
    if (e.BIT1_ERR) f |= BusErrFlag::BITERR1;
    h.err_flags = f;
    if (e.RX_ERR_COUNTER > h.rec_max) h.rec_max = e.RX_ERR_COUNTER;
    // Fault-confinement from ESR1 FLTCONF (bits 5:4): 0 active, 1 passive, >=2 bus-off.
    // Derive it from ESR1 directly — the library's FLT_CONF string compare is buggy
    // (it tests `(ESR1 & 0x30) == 0x1`, which the 0x30 mask can never satisfy, so it
    // mislabels passive as bus-off).
    const uint8_t fc = (uint8_t)((e.ESR1 >> 4) & 0x3);
    const uint8_t fc2 = (fc >= 2) ? 2 : fc;
    if (fc2 > h.fault_conf) h.fault_conf = fc2;
  }
}

// Service one bus for a tick: record the rxBuffer pressure (occupancy BEFORE
// draining = the backlog that accumulated since last tick), drain to empty capped at
// CAN_RX_DRAIN_BUDGET, flag a budget-bound (the overflow precursor), and fold in any
// bus errors. All work stays in the priority-5 CAN-RX task — see can_buses_service.
template <typename BusT>
static inline void service_bus(BusT& bus, volatile BusRxHealth& h) {
  const uint16_t pre = (uint16_t)bus.getRXQueueCount();
  if (pre > h.depth_hwm) h.depth_hwm = pre;
  uint8_t n = 0;
  uint64_t r;
  // events() pops one rx frame (if any) + services tx; loop while rx frames remain.
  // The do-while always runs once, so tx is serviced every tick even when rx is idle.
  do { r = bus.events(); } while (++n < CAN_RX_DRAIN_BUDGET && (r >> 12) != 0);
  if ((r >> 12) != 0) h.cap_hits++;        // budget bound with frames still queued
  poll_bus_errors(bus, h);
}

void can_buses_service() {
  service_bus(can_bb, s_bb_rxh);
  service_bus(can_cone, s_cone_rxh);
  service_bus(can_jugglebot, s_jugglebot_rxh);
}

// Each FlexCAN template instance is a distinct type, so a small overload per bus
// rather than a template (mirrors the original two-bus pattern).
static bool send_on(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}
static bool send_on(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}
static bool send_on(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}

// ── TX serialisation (interp-ISR-safe) ───────────────────────────────────────
// All three FlexCAN writes run inside a short IRQ-off (PRIMASK) critical section.
// can_jugglebot is written by BOTH the 500 Hz leg-setpoint IntervalTimer ISR
// (leg_interp.cpp) AND several FreeRTOS tasks (time-sync 0x7DD fan-out, fault,
// RPC). FlexCAN_T4::write() is NOT reentrant — it scans the TX mailboxes and
// stores registers, with a non-atomic txBuffer.push_back on overflow — so a
// preempting writer can clobber a half-written mailbox and corrupt or drop a leg
// setpoint. A FreeRTOS mutex cannot exclude the ISR, so we mask IRQs (the same
// idiom as time_base.h atomic_read_u64 / leg_interp.cpp interp_on_setpoint). bb
// and cone have only the single time-sync writer today, but they are guarded too
// so the "every FlexCAN write is interrupt-serialised" contract is uniform and a
// future high-priority / ISR writer can't silently race. The masked region is one
// mailbox write (a few us) — well under the 500 us interp jitter budget — and it
// also makes the s_*_tx counter increment atomic.
// TODO(bench): confirm interp_max_jitter_us / interp_deadline_misses stay within
// budget with the broadcast fan-out active.
bool can_bb_send(const ODrive::CanFrame& f) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_bb, f);
  if (ok) s_bb_tx++;
  __set_PRIMASK(pm);
  return ok;
}

// Cone-absent tolerance (HANDOFF D2): transmit only when a cone has been seen
// recently. With no cone on CAN2 an un-ACKed TX would climb the TEC → bus-off;
// gating here (not in the time-sync master) keeps the master bus-agnostic about
// slave presence. FlexCAN self-reception is disabled, so our own 0x7DD does not
// count as cone presence — only real cone frames open the gate.
// TODO(bench): validate cone-absent on real hardware — disconnect the cone and
// confirm CAN2 never enters bus-off while CAN1/CAN3 keep broadcasting 0x7DD.
bool can_cone_send(const ODrive::CanFrame& f) {
  const uint64_t last = atomic_read_u64(&s_cone_last_rx_us);
  if (last == 0 || now_wall_us() - last > CONE_PRESENT_STALENESS_US) {
    return false;   // cone absent → skip TX (no NACK, no TEC climb, no bus-off)
  }
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_cone, f);
  if (ok) s_cone_tx++;
  __set_PRIMASK(pm);
  return ok;
}

bool can_jugglebot_send(const ODrive::CanFrame& f) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_jugglebot, f);
  if (ok) s_jugglebot_tx++;
  __set_PRIMASK(pm);
  return ok;
}

static uint8_t health_of(uint64_t last_rx_us) {
  if (last_rx_us == 0) return JbUdp::BusHealth::UNKNOWN;
  // OK if we've seen a frame within the CAN heartbeat window.
  // TODO(bench): read the FlexCAN error/bus-off registers for WARN/BUS_OFF.
  if (now_wall_us() - last_rx_us > CAN_HEARTBEAT_TIMEOUT_US) return JbUdp::BusHealth::WARN;
  return JbUdp::BusHealth::OK;
}

CanStats can_buses_stats() {
  CanStats s;
  s.bb_rx = s_bb_rx; s.bb_tx = s_bb_tx;
  s.cone_rx = s_cone_rx; s.cone_tx = s_cone_tx;
  s.jugglebot_rx = s_jugglebot_rx; s.jugglebot_tx = s_jugglebot_tx;
  // 64-bit timestamps: read atomically (written atomically in the RX callbacks).
  // can_buses_stats runs in lower-priority tasks than the CAN-RX writer, so a
  // plain two-word load could tear across a writer preemption (esp. at the ~71 min
  // micros64 high-word wrap) and mis-classify bus health.
  s.bb_health = health_of(atomic_read_u64(&s_bb_last_rx_us));
  s.cone_health = health_of(atomic_read_u64(&s_cone_last_rx_us));
  s.jugglebot_health = health_of(atomic_read_u64(&s_jugglebot_last_rx_us));
  return s;
}

// Field-by-field copy of a volatile health block. Each field is a single word, so
// each load is atomic; a torn read ACROSS fields is harmless for 1 Hz debug output.
static BusRxHealth snapshot_bus(const volatile BusRxHealth& h) {
  BusRxHealth o;
  o.depth_hwm  = h.depth_hwm;
  o.cap_hits   = h.cap_hits;
  o.err_events = h.err_events;
  o.err_flags  = h.err_flags;
  o.rec_max    = h.rec_max;
  o.fault_conf = h.fault_conf;
  return o;
}

CanRxHealth can_buses_rx_health() {
  CanRxHealth s;
  s.bb        = snapshot_bus(s_bb_rxh);
  s.cone      = snapshot_bus(s_cone_rxh);
  s.jugglebot = snapshot_bus(s_jugglebot_rxh);
  s.decode_short    = s_decode_short;
  s.decode_bad_axis = s_decode_bad_axis;
  return s;
}

}  // namespace CanBridge
