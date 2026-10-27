#!/usr/bin/env python3
"""Lead-clamp encoder-frame-gap bench A/B (plans/archived/leg-lead-clamp-age-aware.md P2).

MOTIVATION
    The can-bridge leg lead clamp (Teensy_code_canbridge/leg_interp.cpp,
    leg_clamp_anchor + the per-tick clamp loop) bounds the executed command to
    anchor +- MAX_LEAD_REV (0.10 rev). FW 30 anchors to the LAST ENCODER FRAME, so
    during an encoder-frame gap the command pins at stale-frame + 0.10 rev and then
    jumps at the next frame (a staircase) -- the amplifier behind the two single-leg
    ~10 Hz vibrations of the 2026-10-24 oval A/B sitting. FW 31/32 dead-reckon the
    anchor so the command keeps following the smooth stream (FW 32 adds a shift slew
    bound at vel_limit x tick). This probe is the bench A/B: same rig, same stream,
    same injected gaps, flown once on FW 30 and once on FW 32.

RIG
    One ODrive Pro leg, CAN node id 0, alone on the bridge's Jugglebot leg bus,
    production firmware env teensy41 (absent axes 1-5 are tolerated by leg_present()).
    NO teensy_bridge_node / ros2 launch may be running (single-owner UDP hazard; the
    probe refuses). Drive + guard handling are tests/hardware/bench_leg_sysid.py's
    BridgeSysID verbatim (subclassed, never modified): bring-up, warm/arm, hold at
    centre, then ONE triangle track series (default 2.0 rev/s, +-0.4 rev, 12 s) at the
    production 40 Hz knot rate.

GAP INJECTION
    CAN SDO write of axis0.config.can.encoder_msg_rate_ms (uint32 ms, nominal 10) via
    RpcMethod.SDO_WRITE. The wire arg is '<BHf' and the firmware forwards the 4 value
    bytes VERBATIM, so uint32 N is sent as the float32 whose bit pattern is N
    (``uint32_as_f32``; small N are denormals). The Pro's endpoint id is UNVERIFIED in
    this repo (the S1 table says 265; on the S1 table 275 is
    axis0.config.commutation_encoder -- a destructive target), so ``--endpoint`` has no
    default and ``--endpoint-verified-name`` must equal
    'axis0.config.can.encoder_msg_rate_ms', typed by the operator after confirming it on
    the device, or the probe refuses before opening the link.
    Schedule: ``--gaps start_s:rate_ms:duration_s,...`` relative to the series start;
    each entry writes rate at start and 10 at start+duration. rate 100 mimics the
    robot's ~10 Hz frame drop; rate 0 stops frames entirely (>150 ms trips the
    RECOVERABLE MOTOR_FB_STALE output suppression, which must self-clear when frames
    return). A finally (and SIGINT) writes 10 three times, 50 ms apart, then disarms.

RUN RECIPE (launch DOWN, bench ODrive powered, venv active)
    python tools/probes/lead_clamp_gap_bench.py --self-test
    python tools/probes/lead_clamp_gap_bench.py --dry-run --endpoint <ID> \\
        --endpoint-verified-name axis0.config.can.encoder_msg_rate_ms
    python tools/probes/lead_clamp_gap_bench.py --endpoint <ID> \\
        --endpoint-verified-name axis0.config.can.encoder_msg_rate_ms   # FW 30, then FW 32
    python tools/probes/lead_clamp_gap_bench.py --analyse temp/probes/lead_clamp_gap_bench/<run>

OUTPUTS
    temp/probes/lead_clamp_gap_bench/<YYYYmmdd_HHMMSS>_fw<fw>/
      meta.json, summary.json, and one JSONL per stream (host perf_counter ``t`` +
      time.time ``wall`` on every row): leg_cmd (MsgType.LEG_CMD, 100 Hz post-clamp
      executed command), clamp_diag (LEG_CLAMP_DIAG 0x94, 1 Hz cumulative; an FW 30
      board never sends it -> recorded as never-seen), heartbeat (fault_state,
      lead_clamp_mask, uptime_ms), telemetry (pos/vel), identity (BRIDGE_IDENTITY),
      knots (every streamed u0/v0) and events (every SDO write, planned + actual).
    No test currently reads this probe; it underpins the P2 bench verdict.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
_HW = os.path.join(_ROOT, 'tests', 'hardware')
for _p in (_HW, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bench_leg_sysid as bls  # noqa: E402
import sysid_lib as sid  # noqa: E402

AXIS = 0
NOMINAL_RATE_MS = 10
VERIFIED_NAME = 'axis0.config.can.encoder_msg_rate_ms'
S1_TABLE = {265: 'axis0.config.can.encoder_msg_rate_ms (uint32)',
            275: 'axis0.config.commutation_encoder (uint8) -- DESTRUCTIVE on an S1'}
MAX_LEAD_REV = bls.BRIDGE_MAX_LEAD_REV          # 0.10, canbridge_config.h MAX_LEAD_REV
MM_PER_REV = 70.7
MOTOR_FB_STALENESS_S = 0.150                    # canbridge_config.h MOTOR_FB_STALENESS_US
STAIR_MIN_STEPS = 3   # a staircase repeats per frame; the stream alone throws ~1 such step per 1200 samples
KNOT_HZ = bls.BRIDGE_SETPOINT_HZ                # production 40 Hz -- never 100 here
PEAK_VEL_ENVELOPE_RPS = 2.0                     # owner-approved envelope
DEFAULT_GAPS = '2.0:100:1.0,5.0:0:0.3,8.0:0:1.0'
MAX_RATE0_S = 1.5                               # frames fully off: cap the stale window
MAX_GAP_S = 5.0
FAULT_MOTOR_FB_STALE = sid.FAULT_MOTOR_FB_STALE  # 7
OUT_BASE = os.path.join(_ROOT, 'temp', 'probes', 'lead_clamp_gap_bench')


# ---------------------------------------------------------------------------
# SDO packing + schedule parsing (pure)
# ---------------------------------------------------------------------------

def uint32_as_f32(n: int) -> float:
    """The float32 whose bit pattern is uint32 ``n`` (the firmware forwards the
    SDO value bytes verbatim, so a uint32 endpoint needs this reinterpretation)."""
    if not 0 <= int(n) <= 0xFFFFFFFF:
        raise ValueError(f"uint32 out of range: {n}")
    return struct.unpack('<f', struct.pack('<I', int(n)))[0]


def encode_rate_write(endpoint: int, rate_ms: int) -> bytes:
    from teensy_link import rpc_args
    return rpc_args.encode_sdo_write(AXIS, int(endpoint), uint32_as_f32(rate_ms))


def parse_gaps(spec: str, secs: float) -> List[Tuple[float, int, float]]:
    """'start_s:rate_ms:duration_s,...' -> sorted [(start, rate, dur)]; refuses
    overlap, a restore later than secs - 0.5, a rate-0 window > MAX_RATE0_S."""
    out = []
    for tok in [t for t in spec.split(',') if t.strip()]:
        parts = tok.strip().split(':')
        if len(parts) != 3:
            raise ValueError(f"gap '{tok}' is not start_s:rate_ms:duration_s")
        st, rate, dur = float(parts[0]), int(parts[1]), float(parts[2])
        if st < 0.5 or dur <= 0.0:
            raise ValueError(f"gap '{tok}': start must be >= 0.5 s, duration > 0")
        if not 0 <= rate <= 1000:
            raise ValueError(f"gap '{tok}': rate_ms must be 0..1000")
        if rate == NOMINAL_RATE_MS:
            raise ValueError(f"gap '{tok}': rate {rate} is nominal -- not a gap")
        if rate == 0 and dur > MAX_RATE0_S:
            raise ValueError(f"gap '{tok}': rate 0 for {dur} s > {MAX_RATE0_S} s cap")
        if dur > MAX_GAP_S:
            raise ValueError(f"gap '{tok}': duration > {MAX_GAP_S} s")
        if st + dur > secs - 0.5:
            raise ValueError(f"gap '{tok}': restore at {st + dur:.2f} s is past "
                             f"--secs {secs} - 0.5 s")
        out.append((st, rate, dur))
    out.sort()
    for (a, _, da), (b, _, _) in zip(out, out[1:]):
        if a + da + 0.3 > b:
            raise ValueError(f"gaps at {a} and {b} overlap (need 0.3 s between "
                             f"restore and next start)")
    if not out:
        raise ValueError("empty gap schedule")
    return out


def guard_grace_ticks(gaps) -> int:
    """Recoverable-fault grace sized to the longest injected rate-0 window + 1 s at
    the knot rate (the harness default 20 ticks = 0.5 s would abort a 1.0 s window
    of the very MOTOR_FB_STALE we are injecting)."""
    longest = max([d for _, r, d in gaps if r == 0] or [0.0])
    return max(20, int(math.ceil((longest + 1.0) * KNOT_HZ)))


def running_bridge_owners() -> List[str]:
    """Live processes that would co-own the Teensy UDP link (excluding ourselves)."""
    hits = []
    for pat in ('teensy_bridge_node', 'ros2 launch'):
        try:
            r = subprocess.run(['pgrep', '-af', pat], capture_output=True, text=True,
                               timeout=5)
        except Exception as e:  # noqa: BLE001
            hits.append(f"pgrep failed ({e}) -- cannot prove the link is free")
            continue
        for line in r.stdout.splitlines():
            pid = line.split(' ', 1)[0]
            if pid.isdigit() and int(pid) in (os.getpid(), os.getppid()):
                continue
            if 'lead_clamp_gap_bench' in line or 'pgrep' in line:
                continue
            hits.append(line.strip())
    return hits


# ---------------------------------------------------------------------------
# The live runner: BridgeSysID + extra recording + the gap schedule
# ---------------------------------------------------------------------------

class GapBench(bls.BridgeSysID):

    def __init__(self, args, gaps):
        super().__init__(
            axis_id=AXIS, stroke_cap_rev=args.stroke_cap, center_rev=args.center,
            current_limit_a=args.current_limit, vel_cap_rps=args.vel_cap,
            output_dir=OUT_BASE, dry_run=args.dry_run,
            chirp_f0=bls.DEFAULT_CHIRP_F0_HZ, chirp_f1=bls.DEFAULT_CHIRP_F1_HZ,
            chirp_dur=bls.DEFAULT_CHIRP_DURATION_S, chirp_amp=bls.DEFAULT_CHIRP_AMP_REV,
            pos_steps=bls.DEFAULT_POS_STEPS_REV, ladder_step=bls.DEFAULT_LADDER_STEP_REV,
            zeta_target=0.7, bw_clear_hz=None, n_vel=4,
            ripple_threshold=bls.DEFAULT_RIPPLE_RMS_THRESHOLD,
            osc_threshold=bls.DEFAULT_OSC_SCORE_THRESHOLD,
            iq_ripple_threshold=bls.DEFAULT_IQ_RIPPLE_THRESHOLD, do_home=False,
            max_recoveries=args.max_recoveries, knot_hz=KNOT_HZ,
            track_wave='triangle', track_peak_vel=args.peak_vel, track_secs=args.secs,
            track_gains=bls._parse_track_gains(args.gains))
        self.args = args
        self.gaps = gaps
        self.endpoint = int(args.endpoint)
        self.guard = sid.GuardLatchBackoff(max_recoveries=args.max_recoveries,
                                           recoverable_grace=guard_grace_ticks(gaps))
        self.amp = min(sid.TRACK_AMP_MAX_REV,
                       sid.symmetric_amplitude_limit(self.center_rev, self.bounds))
        self._rec_lock = threading.Lock()
        self._raw: List[tuple] = []          # (kind, perf, wall, payload bytes)
        self._knots: List[tuple] = []        # (perf, wall, u0, v0)
        self._events: List[dict] = []
        self._stop_evt = threading.Event()
        self._stream_t0: Optional[Tuple[float, float]] = None
        self._outcome = 'not-started'

    def series(self) -> np.ndarray:
        return sid.track_reference_series(
            center_rev=self.center_rev, amplitude_rev=self.amp,
            peak_vel_rps=self.track_peak_vel, duration_s=self.track_secs,
            seg_t_s=self.seg_t_s, wave='triangle', bounds=self.bounds)

    # -- recording (RX callbacks only append under a lock) --------------------

    def _rec(self, kind):
        def cb(mt, seq, payload, addr):
            p, w = time.perf_counter(), time.time()
            with self._rec_lock:
                self._raw.append((kind, p, w, bytes(payload)))
        return cb

    def _setup_transport(self):
        super()._setup_transport()
        from teensy_link import protocol as pr
        mt = pr.MsgType
        for kind, t in (('leg_cmd', mt.LEG_CMD), ('clamp_diag', mt.LEG_CLAMP_DIAG),
                        ('identity', mt.BRIDGE_IDENTITY), ('heartbeat', mt.HEARTBEAT_T2J),
                        ('telemetry', mt.TELEMETRY)):
            self._client.subscribe(int(t), self._rec(kind))

    def _send_knot(self, u0, u1, u2, v0, t_origin_us):
        super()._send_knot(u0, u1, u2, v0, t_origin_us)
        with self._rec_lock:
            self._knots.append((time.perf_counter(), time.time(), float(u0), float(v0)))

    def _identity(self, timeout=2.5) -> Optional[Tuple[int, int]]:
        from teensy_link import protocol as pr
        t_end = time.perf_counter() + timeout
        while time.perf_counter() < t_end:
            with self._rec_lock:
                ids = [r for r in self._raw if r[0] == 'identity']
            if ids:
                b = pr.BridgeIdentity.unpack(ids[-1][3])
                return int(b.fw_version), int(b.protocol_version)
            time.sleep(0.05)
        return None

    # -- SDO -------------------------------------------------------------------

    def _write_rate(self, rate_ms, kind, gap_index=None, planned=None):
        ev = {'kind': kind, 'rate_ms': int(rate_ms), 'gap_index': gap_index,
              'planned_rel_s': planned, 'endpoint': self.endpoint,
              'payload_hex': encode_rate_write(self.endpoint, rate_ms).hex()}
        ev['t'], ev['wall'] = time.perf_counter(), time.time()
        try:
            self._rpc.call(int(self._RpcMethod.SDO_WRITE),
                           encode_rate_write(self.endpoint, rate_ms))
            ev['ok'] = True
        except Exception as e:  # noqa: BLE001
            ev['ok'], ev['err'] = False, repr(e)
        ev['t_done'] = time.perf_counter()
        with self._rec_lock:
            self._events.append(ev)
        return ev['ok']

    def _schedule_worker(self, t0):
        plan = []
        for i, (st, rate, dur) in enumerate(self.gaps):
            plan.append((st, rate, 'gap', i))
            plan.append((st + dur, NOMINAL_RATE_MS, 'restore', i))
        plan.sort(key=lambda x: x[0])
        for rel, rate, kind, i in plan:
            if self._stop_evt.wait(timeout=max(0.0, t0 + rel - time.perf_counter())):
                return
            self._write_rate(rate, kind, i, rel)

    # -- run ---------------------------------------------------------------------

    def run_gap(self) -> int:
        def sigint(signum, frame):
            print("\n[SIGINT] abort -> restore encoder rate, disarm")
            self._sigint = True
            self._stop_evt.set()
            try:
                self._disarm()
            except Exception:
                pass
        signal.signal(signal.SIGINT, sigint)
        fw = None
        meta: Dict = {'created': datetime.datetime.now().isoformat(timespec='seconds'),
                      'args': vars(self.args), 'endpoint': self.endpoint,
                      'endpoint_verified_name': self.args.endpoint_verified_name,
                      'gaps': [{'start_s': s, 'rate_ms': r, 'duration_s': d}
                               for s, r, d in self.gaps],
                      'guard_recoverable_grace_ticks': self.guard.recoverable_grace,
                      'max_lead_rev': MAX_LEAD_REV, 'mm_per_rev': MM_PER_REV,
                      'knot_hz': KNOT_HZ, 'center_rev': self.center_rev,
                      'amplitude_rev': self.amp, 'peak_vel_rps': self.track_peak_vel}
        rc = 2
        link_up = False
        sched = None
        try:
            self._setup_transport()
            link_up = True
            t_w = time.time()
            while self._sample()[0] is None and time.time() - t_w < 3.0:
                time.sleep(0.05)
            if self._sample()[0] is None:
                self._outcome = 'no telemetry'
                return 2
            ident = self._identity()
            fw = ident[0] if ident else None
            meta['fw_version'] = fw
            meta['protocol_version'] = ident[1] if ident else None
            with self._lock:
                hb = self._cache['hb']
            meta['uptime_ms_start'] = None if hb is None else int(hb.uptime_ms)
            print(f"  link up: fw={fw if fw is not None else 'never-seen'} "
                  f"uptime_ms={meta['uptime_ms_start']} fault={self._fault_state()}")
            rate = self._measure_telemetry_rate()
            print(f"  telemetry {rate.mean_hz:.1f} Hz")
            if not self._startup_clear_latch():
                self._outcome = 'startup latch: ' + self._abort_reason
                return 2
            entry = self._enter_hold_at_center()
            if entry['aborted'] or entry['guard_latched']:
                self._outcome = 'approach: ' + (self._abort_reason or 'guard latch')
                return 2
            gp = self.track_gains[0]
            self._disarm()
            self._apply_gains(gp)
            cur = self._sample()[0]
            cur = self.center_rev if cur is None else cur
            self._warm_and_arm(cur)
            self._stream_and_sample(
                self._knot_ramp(cur, self.center_rev, hold_frames=int(0.4 / self.seg_t_s)),
                guard=True, revert_gains=bls.BASELINE_GAINS)
            series = self.series()
            print(f"  streaming triangle: {len(series)} knots, gaps {self.gaps}")
            t0 = time.perf_counter()
            self._stream_t0 = (t0, time.time())
            sched = threading.Thread(target=self._schedule_worker, args=(t0,),
                                     daemon=True)
            sched.start()
            res = self._stream_and_sample(series, guard=True,
                                          revert_gains=bls.BASELINE_GAINS)
            if res['aborted'] or res['guard_latched']:
                self._outcome = 'stream: ' + (self._abort_diag(res) or self._abort_reason
                                              or 'guard latch')
            else:
                self._outcome = 'completed'
                rc = 0
        except Exception as exc:  # noqa: BLE001
            self._outcome = f'FATAL {exc!r}'
            print(f"FATAL: {exc!r}", file=sys.stderr)
            rc = 1
        finally:
            self._stop_evt.set()
            if sched is not None:
                sched.join(timeout=3.0)
            if link_up and self._rpc is not None:
                for _ in range(3):
                    self._write_rate(NOMINAL_RATE_MS, 'final_restore')
                    time.sleep(0.05)
            try:
                self._disarm()
            except Exception:
                pass
            self._teardown_transport()
            meta['outcome'] = self._outcome
            if self._stream_t0:
                meta['stream_t0_perf'], meta['stream_t0_wall'] = self._stream_t0
            run_dir = self._dump(meta, fw)
            print(f"  outcome: {self._outcome}\n  run dir: {run_dir}")
        try:
            analyse(run_dir)
        except Exception as e:  # noqa: BLE001
            print(f"  analyse failed: {e!r}", file=sys.stderr)
        return rc

    def _dump(self, meta, fw) -> str:
        from teensy_link import protocol as pr
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(OUT_BASE, f"{stamp}_fw{fw if fw is not None else 'unknown'}")
        os.makedirs(run_dir, exist_ok=True)
        with self._rec_lock:
            raw, knots, events = list(self._raw), list(self._knots), list(self._events)
        files = {k: open(os.path.join(run_dir, f'{k}.jsonl'), 'w') for k in
                 ('leg_cmd', 'clamp_diag', 'identity', 'heartbeat', 'telemetry',
                  'knots', 'events')}
        try:
            for kind, p, w, pl in raw:
                row = {'t': p, 'wall': w}
                try:
                    if kind == 'leg_cmd':
                        m = pr.LegCmd.unpack(pl)
                        row.update(t_teensy_us=m.t_teensy_us, pos=list(m.cmd_pos_rev),
                                   vel=list(m.cmd_vel_rps))
                    elif kind == 'clamp_diag':
                        m = pr.LegClampDiag.unpack(pl)
                        row.update(lead_clamp_ticks=list(m.lead_clamp_ticks),
                                   stale_anchor_ticks=list(m.stale_anchor_ticks))
                    elif kind == 'identity':
                        m = pr.BridgeIdentity.unpack(pl)
                        row.update(fw_version=m.fw_version,
                                   protocol_version=m.protocol_version)
                    elif kind == 'heartbeat':
                        m = pr.HeartbeatT2J.unpack(pl)
                        row.update(fault_state=int(m.fault_state), flags=int(m.flags),
                                   lead_clamp_mask=int(m.lead_clamp_mask),
                                   uptime_ms=int(m.uptime_ms),
                                   live_deviation0=float(m.live_deviation[AXIS]))
                    elif kind == 'telemetry':
                        m = pr.Telemetry.unpack(pl)
                        row.update(t_teensy_us=m.t_teensy_us, pos=list(m.pos_rev),
                                   vel=list(m.vel_rps))
                except Exception as e:  # noqa: BLE001
                    row.update(decode_error=repr(e), payload_hex=pl.hex())
                files[kind].write(json.dumps(row) + '\n')
            for p, w, u0, v0 in knots:
                files['knots'].write(json.dumps({'t': p, 'wall': w, 'u0': u0,
                                                 'v0': v0}) + '\n')
            for ev in events:
                files['events'].write(json.dumps(ev) + '\n')
        finally:
            for f in files.values():
                f.close()
        meta['counts'] = {k: sum(1 for r in raw if r[0] == k) for k in
                          ('leg_cmd', 'clamp_diag', 'identity', 'heartbeat', 'telemetry')}
        meta['counts'].update(knots=len(knots), events=len(events))
        meta['clamp_diag_seen'] = meta['counts']['clamp_diag'] > 0
        with open(os.path.join(run_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2, default=str)
        return run_dir


# ---------------------------------------------------------------------------
# Offline analysis
# ---------------------------------------------------------------------------

def _load(run_dir, name) -> List[dict]:
    path = os.path.join(run_dir, f'{name}.jsonl')
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [r for r in (json.loads(l) for l in f if l.strip())
                if 'decode_error' not in r]


def _windows(meta, events) -> List[dict]:
    t0 = meta.get('stream_t0_perf')
    wins = []
    for i, g in enumerate(meta['gaps']):
        on = [e for e in events if e.get('kind') == 'gap' and e.get('gap_index') == i]
        off = [e for e in events if e.get('kind') == 'restore' and e.get('gap_index') == i]
        t_on = on[0]['t'] if on else (None if t0 is None else t0 + g['start_s'])
        t_off = off[0]['t'] if off else (
            None if t0 is None else t0 + g['start_s'] + g['duration_s'])
        wins.append({'index': i, 'rate_ms': g['rate_ms'], 'duration_s': g['duration_s'],
                     't_on': t_on, 't_off': t_off,
                     'write_ok': bool(on and on[0].get('ok')) if on else None,
                     'restore_ok': bool(off and off[0].get('ok')) if off else None})
    return wins


def _gap_metrics(w, leg, knots, hb, cd, tel) -> dict:
    t_on, t_off = w['t_on'], w['t_off']
    lo, hi = t_on, t_off + 0.2
    out = dict(w)
    # Executed command steps.
    lt = np.array([r['t'] for r in leg if lo <= r['t'] <= hi])
    lp = np.array([r['pos'][AXIS] for r in leg if lo <= r['t'] <= hi])
    lv = np.array([r['vel'][AXIS] for r in leg if lo <= r['t'] <= hi])
    kt = np.array([r['t'] for r in knots])
    kv = np.array([r['v0'] for r in knots])
    # LEG_CMD reports axes[].target_pos_rev, written EVERY tick, but the CAN TX is
    # gated on s_output_enabled (leg_interp.cpp), which the 10 Hz fault task holds
    # shut through MOTOR_FB_STALE until after frames return. Those samples never
    # reach the ODrive, so the step metrics drop them: from t_on + 150 ms (the
    # earliest trip) to the recovery slew's re-baseline sample (cmd ~= encoder,
    # |vel_ff| <= 0.15 rev/s, the slew's first ramp tick). Bench 2026-10-27: the
    # 7-11 mm "steps" at frame return on FW 30 AND FW 33 were exactly these.
    tx = np.ones(lt.size, dtype=bool)
    hw_stale = any(r['fault_state'] == FAULT_MOTOR_FB_STALE
                   for r in hb if lo <= r['t'] <= hi)
    out['untransmitted_span_s'] = 0.0
    if hw_stale and lt.size:
        tel_t = np.array([r['t'] for r in tel])
        tel_p = np.array([r['pos'][AXIS] for r in tel])
        j = np.clip(np.searchsorted(tel_t, lt, side='right') - 1, 0, max(0, tel_t.size - 1))
        enc = tel_p[j] if tel_t.size else np.full(lt.size, np.nan)
        base = np.where((lt > t_off) & (np.abs(lp - enc) * MM_PER_REV < 3.0)
                        & (np.abs(lv) <= 0.15))[0]
        t_end = lt[base[0]] if base.size else hi
        tx = ~((lt >= t_on + MOTOR_FB_STALENESS_S) & (lt < t_end))
        out['untransmitted_span_s'] = float(max(0.0, t_end - (t_on + MOTOR_FB_STALENESS_S)))
    if lp.size >= 2:
        # Steps only between consecutive TRANSMITTED samples (a pair straddling the
        # untransmitted span is not a tick-to-tick step either).
        pair = tx[1:] & tx[:-1]
        d_all = np.abs(np.diff(lp))
        out['untransmitted_max_step_mm'] = (float(d_all[~pair].max() * MM_PER_REV)
                                            if np.any(~pair) else 0.0)
        d = np.where(pair, d_all, 0.0)
        dt_med = float(np.median(np.diff(lt)))
        kin = (kt >= lo) & (kt <= hi)
        vmax = float(np.max(np.abs(kv[kin]))) if np.any(kin) else float('nan')
        expected = vmax * dt_med
        out['max_step_rev'] = float(d.max())
        out['max_step_mm'] = float(d.max() * MM_PER_REV)
        out['leg_cmd_dt_ms'] = dt_med * 1e3
        out['stream_vmax_rps'] = vmax
        out['expected_step_rev'] = expected
        out['step_over_expected'] = float(d.max() / expected) if expected > 0 else None
        out['step_over_max_lead'] = float(d.max() / MAX_LEAD_REV)
        # Longest flat run while the stream is moving (> 0.5 rev/s).
        idx = np.searchsorted(kt, lt[1:], side='right') - 1
        vk = np.where(idx >= 0, np.abs(kv[np.clip(idx, 0, max(0, kv.size - 1))]), 0.0) \
            if kv.size else np.zeros(d.size)
        flat = (d < 1e-6) & (vk > 0.5)
        best = run = 0.0
        for k in range(d.size):
            run = run + (lt[k + 1] - lt[k]) if flat[k] else 0.0
            best = max(best, run)
        out['longest_flat_s'] = best
        thr = max(2.5 * expected, 0.4 * MAX_LEAD_REV) if expected > 0 else 0.4 * MAX_LEAD_REV
        out['n_big_steps'] = int(np.sum(d > thr))
        stair = out['n_big_steps'] >= STAIR_MIN_STEPS
        out['verdict'] = 'STAIRCASE' if stair else 'SMOOTH'
    else:
        out['verdict'] = 'NO-LEG_CMD'
    # Heartbeat lead-clamp bit + fault timeline.
    hw_ = [r for r in hb if lo <= r['t'] <= hi]
    out['hb_samples'] = len(hw_)
    out['hb_lead_clamp_bit0'] = sum(1 for r in hw_ if r['lead_clamp_mask'] & 1)
    out['fb_stale_seen'] = any(r['fault_state'] == FAULT_MOTOR_FB_STALE for r in hw_)
    after = [r for r in hb if t_off <= r['t'] <= t_off + 1.5]
    out['faults_in_window'] = sorted({r['fault_state'] for r in hw_})
    out['fault_none_after_restore'] = (
        any(r['fault_state'] == 0 for r in after) if after else None)
    clr = [r['t'] for r in after if r['fault_state'] == 0]
    out['clear_after_restore_s'] = (clr[0] - t_off) if (clr and out['fb_stale_seen']) else None
    # LegClampDiag deltas.
    before = [r for r in cd if r['t'] <= lo]
    aft = [r for r in cd if r['t'] >= hi]
    if before and aft:
        a, b = before[-1], aft[0]
        out['diag_lead_clamp_ticks_delta'] = b['lead_clamp_ticks'][AXIS] - a['lead_clamp_ticks'][AXIS]
        out['diag_stale_anchor_ticks_delta'] = (b['stale_anchor_ticks'][AXIS]
                                                - a['stale_anchor_ticks'][AXIS])
    else:
        out['diag_lead_clamp_ticks_delta'] = None
        out['diag_stale_anchor_ticks_delta'] = None
        out['clamp_diag'] = 'never-seen' if not cd else 'no bracketing samples'
    # Encoder frame interval from TELEMETRY pos changes (skip SDO latency 50 ms).
    tt = [(r['t'], r['pos'][AXIS]) for r in tel if t_on + 0.05 <= r['t'] <= t_off]
    chg = [tt[k][0] for k in range(1, len(tt)) if tt[k][1] != tt[k - 1][1]]
    iv = np.diff(chg) if len(chg) >= 2 else np.array([])
    span = (t_off - t_on - 0.05)
    longest = 0.0
    if tt:
        marks = [t_on + 0.05] + chg + [t_off]
        longest = float(np.max(np.diff(marks)))
    out['enc_interval_median_ms'] = float(np.median(iv) * 1e3) if iv.size else None
    out['enc_longest_nochange_ms'] = longest * 1e3
    if w['rate_ms'] == 0:
        out['rate_confirmed'] = bool(tt) and longest >= 0.8 * span
    else:
        m = out['enc_interval_median_ms']
        out['rate_confirmed'] = (m is not None
                                 and 0.6 * w['rate_ms'] <= m <= 1.6 * w['rate_ms'])
    return out


def analyse(run_dir: str, quiet: bool = False) -> dict:
    with open(os.path.join(run_dir, 'meta.json')) as f:
        meta = json.load(f)
    leg, knots, hb = _load(run_dir, 'leg_cmd'), _load(run_dir, 'knots'), _load(run_dir, 'heartbeat')
    cd, tel, ev = _load(run_dir, 'clamp_diag'), _load(run_dir, 'telemetry'), _load(run_dir, 'events')
    wins = [_gap_metrics(w, leg, knots, hb, cd, tel) for w in _windows(meta, ev)
            if w['t_on'] is not None]
    # Baseline encoder interval outside every window.
    tt = [(r['t'], r['pos'][AXIS]) for r in tel
          if not any(w['t_on'] - 0.1 <= r['t'] <= w['t_off'] + 0.3 for w in wins)]
    chg = [tt[k][0] for k in range(1, len(tt)) if tt[k][1] != tt[k - 1][1]]
    base_ms = float(np.median(np.diff(chg)) * 1e3) if len(chg) >= 2 else None
    trans, prev = [], None
    t0 = meta.get('stream_t0_perf') or (hb[0]['t'] if hb else 0.0)
    for r in hb:
        if r['fault_state'] != prev:
            trans.append((round(r['t'] - t0, 3), sid.fault_name(r['fault_state'])))
            prev = r['fault_state']
    summary = {'run_dir': run_dir, 'fw_version': meta.get('fw_version'),
               'uptime_ms_start': meta.get('uptime_ms_start'),
               'outcome': meta.get('outcome'),
               'clamp_diag_seen': bool(cd), 'baseline_enc_interval_ms': base_ms,
               'fault_timeline_rel_s': trans, 'windows': wins}
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    if not quiet:
        print(f"\n  fw={summary['fw_version']} outcome={summary['outcome']} "
              f"clamp_diag={'seen' if cd else 'NEVER-SEEN'} "
              f"baseline enc interval={base_ms if base_ms is None else round(base_ms, 1)} ms")
        print("  gap rate dur  | maxstep mm  /exp  /lead | flat s | hb lc | "
              "diag lc/stale | faults  clr s | enc med/long ms ok | nbig untx s (max mm) | verdict")
        for w in wins:
            def g(k, fmt):
                v = w.get(k)
                return '-' if v is None else fmt.format(v)
            print(f"  {w['index']:3d} {w['rate_ms']:4d} {w['duration_s']:4.1f} | "
                  f"{g('max_step_mm', '{:7.2f}')} {g('step_over_expected', '{:5.1f}')} "
                  f"{g('step_over_max_lead', '{:5.2f}')} | {g('longest_flat_s', '{:6.3f}')} | "
                  f"{w['hb_lead_clamp_bit0']:2d}/{w['hb_samples']:<2d} | "
                  f"{g('diag_lead_clamp_ticks_delta', '{}')}/"
                  f"{g('diag_stale_anchor_ticks_delta', '{}')} | "
                  f"{','.join(str(x) for x in w['faults_in_window']) or '-':7s} "
                  f"{g('clear_after_restore_s', '{:5.2f}')} | "
                  f"{g('enc_interval_median_ms', '{:5.0f}')}/"
                  f"{g('enc_longest_nochange_ms', '{:5.0f}')} "
                  f"{'Y' if w['rate_confirmed'] else 'N'} | "
                  f"{g('n_big_steps', '{:3d}')} {w.get('untransmitted_span_s', 0.0):5.2f} "
                  f"({g('untransmitted_max_step_mm', '{:5.2f}')}) | {w['verdict']}")
        print(f"  fault timeline (s rel stream start): {trans}")
    return summary


# ---------------------------------------------------------------------------
# Self-test (synthetic FW30 staircase vs FW32 smooth)
# ---------------------------------------------------------------------------

def _synth(run_dir: str, fw: int, gaps) -> None:
    os.makedirs(run_dir, exist_ok=True)
    seg = 1.0 / KNOT_HZ
    secs = 10.0
    t0 = 1000.0
    ser = sid.track_reference_series(center_rev=1.5, amplitude_rev=0.4, peak_vel_rps=2.0,
                                     duration_s=secs, seg_t_s=seg, wave='triangle')
    kt = t0 + np.arange(ser.size) * seg
    kv = np.append(np.diff(ser) / seg, 0.0)
    stream = lambda t: float(np.interp(t, kt, ser))  # noqa: E731
    events = []
    for i, (st, rate, dur) in enumerate(gaps):
        events.append({'kind': 'gap', 'gap_index': i, 'rate_ms': rate, 't': t0 + st,
                       'ok': True})
        events.append({'kind': 'restore', 'gap_index': i, 'rate_ms': 10,
                       't': t0 + st + dur, 'ok': True})

    def rate_at(t):
        for st, rate, dur in gaps:
            if t0 + st + 0.02 <= t < t0 + st + dur:
                return rate
        return 10
    # Encoder frames (2 ms sim tick), leg follows the executed cmd exactly.
    dt = 0.002
    ts = np.arange(t0, t0 + secs, dt)
    enc, last_frame, cmd_hist, enc_hist, clamp_hist, stale_hist = stream(t0), t0, [], [], [], []
    cmd = enc
    for t in ts:
        r = rate_at(t)
        if r and t - last_frame >= r / 1000.0 - 1e-9:
            enc, last_frame = cmd, t
        s = stream(t)
        if fw >= 31:
            cmd = s
            clamped = False
        else:
            cmd = min(max(s, enc - MAX_LEAD_REV), enc + MAX_LEAD_REV)
            clamped = cmd != s
        cmd_hist.append(cmd)
        enc_hist.append(enc)
        clamp_hist.append(clamped)
        stale_hist.append(t - last_frame > 0.15)
    with open(os.path.join(run_dir, 'leg_cmd.jsonl'), 'w') as f:
        for k in range(0, ts.size, 5):
            f.write(json.dumps({'t': ts[k] + 0.001, 'pos': [cmd_hist[k]] + [0.0] * 5,
                                'vel': [0.0] * 6}) + '\n')
    with open(os.path.join(run_dir, 'telemetry.jsonl'), 'w') as f:
        for k in range(0, ts.size, 5):
            f.write(json.dumps({'t': ts[k] + 0.002, 'pos': [enc_hist[k]] + [0.0] * 6,
                                'vel': [0.0] * 7}) + '\n')
    with open(os.path.join(run_dir, 'heartbeat.jsonl'), 'w') as f:
        for k in range(0, ts.size, 50):
            f.write(json.dumps({'t': ts[k] + 0.003,
                                'fault_state': FAULT_MOTOR_FB_STALE if stale_hist[k] else 0,
                                'lead_clamp_mask': int(clamp_hist[k]), 'uptime_ms': 5000,
                                'flags': 0}) + '\n')
    with open(os.path.join(run_dir, 'clamp_diag.jsonl'), 'w') as f:
        if fw >= 31:
            lc = st_ = 0
            for k in range(ts.size):
                lc += int(clamp_hist[k])
                st_ += int(stale_hist[k])
                if k % 500 == 0:
                    f.write(json.dumps({'t': ts[k], 'lead_clamp_ticks': [lc] + [0] * 5,
                                        'stale_anchor_ticks': [st_] + [0] * 5}) + '\n')
    with open(os.path.join(run_dir, 'knots.jsonl'), 'w') as f:
        for t, u, v in zip(kt, ser, kv):
            f.write(json.dumps({'t': t, 'u0': u, 'v0': v}) + '\n')
    with open(os.path.join(run_dir, 'events.jsonl'), 'w') as f:
        for e in events:
            f.write(json.dumps(e) + '\n')
    with open(os.path.join(run_dir, 'meta.json'), 'w') as f:
        json.dump({'fw_version': fw, 'uptime_ms_start': 5000, 'outcome': 'synthetic',
                   'stream_t0_perf': t0,
                   'gaps': [{'start_s': s, 'rate_ms': r, 'duration_s': d}
                            for s, r, d in gaps]}, f)


def self_test() -> int:
    fails = []

    def check(cond, msg):
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            fails.append(msg)
    print("SDO packing (uint32 bit pattern through encode_sdo_write '<BHf'):")
    for n in (0, 10, 100, 1000):
        pl = encode_rate_write(265, n)
        check(len(pl) == 7 and pl[-4:] == struct.pack('<I', n) and pl[0] == AXIS
              and struct.unpack('<H', pl[1:3])[0] == 265,
              f"N={n}: payload {pl.hex()} tail == {struct.pack('<I', n).hex()}")
    print("schedule parse:")
    g = parse_gaps(DEFAULT_GAPS, 12.0)
    check(g == [(2.0, 100, 1.0), (5.0, 0, 0.3), (8.0, 0, 1.0)], f"default -> {g}")
    check(guard_grace_ticks(g) == 80, f"guard grace ticks {guard_grace_ticks(g)} == 80")
    for bad, why in (('2.0:100:1.0,2.5:0:0.3', 'overlap'), ('2.0:0:2.0', 'rate-0 > cap'),
                     ('11.0:100:1.0', 'past --secs'), ('2.0:10:1.0', 'nominal rate'),
                     ('2.0:100', 'malformed')):
        try:
            parse_gaps(bad, 12.0)
            check(False, f"'{bad}' refused ({why})")
        except ValueError:
            check(True, f"'{bad}' refused ({why})")
    print("analyse() on synthetic FW30 staircase vs FW32 smooth:")
    gaps = [(2.0, 100, 1.0), (5.0, 0, 0.3), (7.0, 0, 1.0)]
    with tempfile.TemporaryDirectory() as td:
        res = {}
        for fw in (30, 32):
            d = os.path.join(td, f'fw{fw}')
            _synth(d, fw, gaps)
            res[fw] = analyse(d)
        w30, w32 = res[30]['windows'], res[32]['windows']
        check(w30[0]['verdict'] == 'STAIRCASE', "FW30 rate-100 window STAIRCASE")
        # Rate-0 windows: the steps past t_on + 150 ms are the fault-suppressed span
        # (never transmitted, bench 2026-10-27), so they are excluded, not judged.
        check(all(w['untransmitted_span_s'] > 0 for w in (w30[1], w30[2])),
              "rate-0 windows: suppressed span excluded from the step metrics")
        check(all(w['verdict'] == 'SMOOTH' for w in w32), "FW32 every window SMOOTH")
        check(all(w['rate_confirmed'] for w in w30 + w32), "rate change confirmed (both)")
        check(w30[2]['fb_stale_seen'] and w30[2]['fault_none_after_restore'],
              "rate-0 1.0 s: MOTOR_FB_STALE seen and cleared after restore")
        check(not w30[0]['fb_stale_seen'], "rate-100: no MOTOR_FB_STALE")
        check(not res[30]['clamp_diag_seen'] and w30[0].get('clamp_diag') == 'never-seen',
              "FW30 LegClampDiag recorded as never-seen")
        check(res[32]['clamp_diag_seen'] and w32[2]['diag_stale_anchor_ticks_delta'] > 0,
              "FW32 LegClampDiag stale_anchor delta > 0 over the rate-0 window")
        check(w30[0]['hb_lead_clamp_bit0'] > 0 and w32[0]['hb_lead_clamp_bit0'] == 0,
              "heartbeat lead_clamp bit0 counted (FW30 > 0, FW32 == 0)")
    print(f"\nself-test: {'PASS' if not fails else 'FAIL (%d)' % len(fails)}")
    return 0 if not fails else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def dry_run(bench: GapBench, gaps) -> int:
    ser = bench.series()
    pv = float(np.max(np.abs(np.diff(ser))) / bench.seg_t_s) if ser.size > 1 else 0.0
    print("=== lead_clamp_gap_bench DRY RUN (no socket opened) ===")
    print(f"  axis {AXIS}, endpoint {bench.endpoint} "
          f"(S1-table name: {S1_TABLE.get(bench.endpoint, 'not in S1 table')}); "
          f"operator-verified as '{bench.args.endpoint_verified_name}'")
    print(f"  caps: stroke {bench.stroke_cap_rev} rev, bounds [{bench.bounds.lo_rev:.3f},"
          f"{bench.bounds.hi_rev:.3f}], current {bench.current_limit_a} A, vel cap "
          f"{bench.vel_cap_rps} rev/s, gains {bench.track_gains[0]}")
    print(f"  series: triangle {ser.size} knots @ {KNOT_HZ:.0f} Hz ({ser.size / KNOT_HZ:.2f} s),"
          f" centre {bench.center_rev:.3f} amp {bench.amp:.3f} rev, peak vel {pv:.3f} rev/s,"
          f" range [{ser.min():.3f},{ser.max():.3f}]")
    print(f"  guard recoverable grace: {bench.guard.recoverable_grace} ticks "
          f"({bench.guard.recoverable_grace / KNOT_HZ:.2f} s; harness default 20)")
    print("  plan: link up -> identity/uptime -> telemetry rate -> startup latch clear ->"
          " hold at centre -> apply gains, warm/arm, settle -> stream + schedule")
    for i, (st, rate, dur) in enumerate(gaps):
        print(f"    gap {i}: t={st:5.2f} s write {rate:4d} ms  "
              f"[{encode_rate_write(bench.endpoint, rate).hex()}]  -> t={st + dur:5.2f} s "
              f"write 10 ms [{encode_rate_write(bench.endpoint, 10).hex()}]")
    print("  finally: write 10 x3 (50 ms apart), disarm, teardown")
    owners = running_bridge_owners()
    print(f"  link owners now: {owners or 'none'}"
          + ("  (a LIVE run would REFUSE)" if owners else ""))
    print(f"  output: {OUT_BASE}/<YYYYmmdd_HHMMSS>_fw<fw>/")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--endpoint', type=int, default=None,
                   help='REQUIRED: ODrive endpoint id of axis0.config.can.encoder_msg_rate_ms '
                        'on THIS device (no default -- S1 table 265; 275 is '
                        'commutation_encoder on an S1)')
    p.add_argument('--endpoint-verified-name', default=None,
                   help=f"REQUIRED: must equal '{VERIFIED_NAME}' (typed after confirming "
                        f"the id on the device)")
    p.add_argument('--gaps', default=DEFAULT_GAPS,
                   help=f"start_s:rate_ms:duration_s,... (default '{DEFAULT_GAPS}')")
    p.add_argument('--secs', type=float, default=12.0, help='track duration (default 12)')
    p.add_argument('--peak-vel', type=float, default=PEAK_VEL_ENVELOPE_RPS,
                   help=f'triangle peak vel rev/s (default/max {PEAK_VEL_ENVELOPE_RPS})')
    p.add_argument('--gains', default=bls.DEFAULT_TRACK_GAINS,
                   help=f"ONE pg:vg:vint (default '{bls.DEFAULT_TRACK_GAINS}', production)")
    p.add_argument('--stroke-cap', type=float, default=bls.HARD_STROKE_CAP_REV)
    p.add_argument('--confirm-stroke', action='store_true')
    p.add_argument('--center', type=float, default=None)
    p.add_argument('--current-limit', type=float, default=bls.HARD_CURRENT_LIMIT_A)
    p.add_argument('--vel-cap', type=float, default=bls.DEFAULT_VEL_CAP_RPS)
    p.add_argument('--max-recoveries', type=int, default=bls.BRIDGE_MAX_RECOVERIES)
    p.add_argument('--dry-run', action='store_true', help='print the plan; open no socket')
    p.add_argument('--self-test', action='store_true', help='offline checks only')
    p.add_argument('--analyse', metavar='RUN_DIR', default=None,
                   help='re-run analyse() on a saved run dir')
    args = p.parse_args()

    if args.self_test:
        return self_test()
    if args.analyse:
        analyse(args.analyse)
        return 0
    if args.endpoint is None or args.endpoint_verified_name is None:
        print("REJECT: --endpoint and --endpoint-verified-name are both required",
              file=sys.stderr)
        return 1
    if args.endpoint_verified_name != VERIFIED_NAME:
        print(f"REJECT: --endpoint-verified-name '{args.endpoint_verified_name}' != "
              f"'{VERIFIED_NAME}' -- confirm the id on the device first", file=sys.stderr)
        return 1
    if args.peak_vel <= 0.0 or args.peak_vel > PEAK_VEL_ENVELOPE_RPS + 1e-9:
        print(f"REJECT: --peak-vel must be in (0, {PEAK_VEL_ENVELOPE_RPS}] (owner envelope)",
              file=sys.stderr)
        return 1
    if args.stroke_cap > bls.HARD_STROKE_CAP_REV and (
            not args.confirm_stroke or args.stroke_cap > bls.HARD_STROKE_CEIL_REV):
        print("REJECT: --stroke-cap above the harness cap needs --confirm-stroke and "
              f"never exceeds {bls.HARD_STROKE_CEIL_REV}", file=sys.stderr)
        return 1
    if args.current_limit > bls.HARD_CURRENT_LIMIT_A:
        print(f"REJECT: --current-limit > {bls.HARD_CURRENT_LIMIT_A} A", file=sys.stderr)
        return 1
    try:
        gains = bls._parse_track_gains(args.gains)
        if len(gains) != 1:
            raise ValueError("--gains takes exactly ONE pg:vg:vint")
        gaps = parse_gaps(args.gaps, args.secs)
    except ValueError as e:
        print(f"REJECT: {e}", file=sys.stderr)
        return 1
    bench = GapBench(args, gaps)
    if args.dry_run:
        return dry_run(bench, gaps)
    owners = running_bridge_owners()
    if owners:
        print("REJECT: another link owner is running (single-owner UDP hazard):\n  "
              + "\n  ".join(owners), file=sys.stderr)
        return 1
    return bench.run_gap()


if __name__ == '__main__':
    sys.exit(main())
