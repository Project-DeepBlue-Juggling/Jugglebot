"""Capture PRE-v6 (PROTOCOL_VERSION 5) byte-exact wire fixtures for T-R1.

WHAT THIS CAPTURES
------------------
Two fixture families, produced by RUNNING THE CURRENT (pre-v6) production
code, frozen as hex so the Phase 2 v6 widening can prove legacy
byte-exactness (T-R1 / T-U7 / T-U8 of
``plans/active/unified-7dof-planner.md`` § 5):

1. **Pump wire fixtures** (``tests/teensy_link/data/v5_pump_wire_fixtures.json``)
   — representative :5557 ``mpc_cmd`` dicts driven through the real
   ``teensy_link.setpoint_pump.SetpointPump`` and the real codec
   (``teensy_link.protocol``): the 156-byte v5 ``Setpoint`` payload AND the
   full ``encode_frame`` UDP frame (header + payload + CRC-16), both as hex,
   with the input dict and pump constructor args stored alongside. Covers
   full Mode-1 (flags 0x3), the ``ext_mm`` fallback, a no-lookahead frame
   (flags 0x0), a torque-FF ramp-in sequence (clamp + ramp progression
   byte-exact), a default-pump multi-frame sequence, and one frame chained
   from the real emitter through ipc wire form.

2. **Emitter frame fixtures** (``tests/motion/data/v5_emitter_frame_fixtures.json``)
   — legacy plans (``HoldPlan``, ``planner.build_return_to_neutral``) sampled
   through the real ``KnotEmitter.frame`` and serialised with the production
   ``jugglebot.motion.ipc._pack``; the stored hex is ``frames[1]`` (the
   msgpack blob), which pins key order AND float bits — the true
   byte-identity the T-U8/T-R1 regression needs. The exact plan-construction
   recipe is embedded in the fixture.

⚠ ONLY MEANINGFUL AT A PRE-v6 CHECKOUT. The whole point of these fixtures is
that they are IRREPRODUCIBLE after the v6 wire change lands (Setpoint 6 → 7
+ ``v1``, ``SETPOINT_SIZE`` 156 → 208, ``PROTOCOL_VERSION`` 5 → 6): they are
the frozen ground truth the post-change code is compared against. The script
therefore REFUSES to run unless ``PROTOCOL_VERSION == 5``. Captured
2026-09-01 at commit 2aaaae1 (branch ``mvp-trajectory-bringup``). Do NOT
regenerate at a later checkout — a regenerated file is a different baseline,
not a refresh.

MOTIVATING PLAN / LOGBOOK
    ``plans/active/unified-7dof-planner.md`` § 4 Phase 2, § 5 T-R1;
    ``logbook/2026-09-01-unified-7dof-planner-phase1-planner-core.md``
    (the ``CyclePlan`` contract these regressions fence).

UNDERPINS (written in Phase 2, after this capture)
    the T-U7 pump legacy-byte-identity test and the T-U8 emitter
    legacy-byte-identity test.

INVOCATIONS (venv, from the repo root)::

    source ~/Desktop/PDJ_venv/venv/bin/activate

    # survey run — timestamped output dir under temp/probes/
    python tools/probes/capture_v5_wire_fixtures.py

    # determinism check — write to explicit dirs, then cmp
    python tools/probes/capture_v5_wire_fixtures.py --out-dir /tmp/run1
    python tools/probes/capture_v5_wire_fixtures.py --out-dir /tmp/run2

    # EMIT THE COMMITTED FIXTURES (tests/teensy_link/data/ + tests/motion/data/)
    python tools/probes/capture_v5_wire_fixtures.py --emit-fixtures

``--emit-fixtures`` is the explicit-output regeneration flag
``tools/probes/README.md`` mandates for committed reference data — but note
the warning above: for THIS probe the regeneration recipe is only valid at
the pre-v6 checkout it documents.

Output is byte-deterministic by construction (fixed literal inputs, no
timestamps beyond the capture date) — two runs at the same checkout must
produce byte-identical JSON, per the repo's determinism discipline.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (REPO_ROOT,
           os.path.join(REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(REPO_ROOT, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import msgpack                                                  # noqa: E402
import numpy as np                                              # noqa: E402

from teensy_link import protocol as p                           # noqa: E402
from teensy_link.setpoint_pump import SetpointPump              # noqa: E402

import jugglebot.hardware_config as hw                          # noqa: E402
from jugglebot.motion import ipc                                # noqa: E402
from jugglebot.motion.geometry import StewartGeometry           # noqa: E402
from jugglebot.motion.trajectory import (                       # noqa: E402
    HoldPlan, KnotEmitter, TrajectoryLimits,
)
from jugglebot.motion.trajectory import planner                 # noqa: E402

#: Committed fixture destinations (written only under --emit-fixtures).
PUMP_FIXTURE_PATH = os.path.join(
    REPO_ROOT, 'tests', 'teensy_link', 'data', 'v5_pump_wire_fixtures.json')
EMITTER_FIXTURE_PATH = os.path.join(
    REPO_ROOT, 'tests', 'motion', 'data', 'v5_emitter_frame_fixtures.json')
#: Survey runs land here, timestamped (tools/probes/README.md convention).
OUTPUT_DIR = os.path.join(REPO_ROOT, 'temp', 'probes')

CAPTURED_DATE = '2026-09-01'
EMIT_COMMAND = ('source ~/Desktop/PDJ_venv/venv/bin/activate && '
                'python tools/probes/capture_v5_wire_fixtures.py '
                '--emit-fixtures')
WARNING = ('PRE-v6 capture (PROTOCOL_VERSION 5, SETPOINT_SIZE 156). '
           'Reproducible ONLY at the pre-v6 checkout it was captured at '
           '(see captured_at_commit) — that irreproducibility is the point: '
           'this file is the frozen T-R1 legacy byte-identity baseline. '
           'Never regenerate after the v6 wire change lands.')

# ── Fixed literal inputs ─────────────────────────────────────────────────────
# Production per-leg mm→rev scales, FROZEN AS LITERALS (== hardware_config.
# GEOM_MM_TO_REV at the capture checkout; asserted below). Literals so the
# fixture inputs cannot drift under a later YAML edit — the fixture must stay
# a fixed point, not track config (the capture_cup_cycle_refs.py pinning rule).
MM_TO_REV = (0.01418332, 0.01419076, 0.01408956,
             0.01418684, 0.01426801, 0.01424951)

# Realistic motor positions (rev, ODrive convention) near the activate pose
# (JB_OP_ACTIVATE_POSITION_REVS ≈ 2.19), and mm-space fields to match.
MOTOR_REV_A = (2.1907, 2.1919, 2.1762, 2.1913, 2.2038, 2.2009)
EXT_MM_A = (154.46, 154.46, 154.46, 154.46, 154.46, 154.46)
VEL_MM_S_A = (12.5, -8.25, 3.125, 0.0, -15.75, 22.0)
CMD_NEXT_MM_A = (154.77, 154.25, 154.53, 154.46, 154.07, 155.01)
CMD_NEXT2_MM_A = (155.08, 154.05, 154.61, 154.46, 153.68, 155.55)

T_ORIGIN_BASE_US = 1756684800123456   # non-trivial, > 2**32, exercises the u64


def _hexdump(b: bytes) -> str:
    return b.hex()


def _setpoint_record(cmd: dict, t_origin_us: int, seq: int,
                     sp: p.Setpoint) -> dict:
    """One accepted frame: input dict + payload hex + full wire-frame hex."""
    payload = sp.pack()
    assert len(payload) == p.SETPOINT_SIZE
    wire = p.encode_frame(int(p.MsgType.SETPOINT), seq, payload)
    return {
        'cmd': cmd,
        't_origin_us': t_origin_us,
        'wire_seq': seq,
        'flags': sp.flags,
        'setpoint_payload_hex': _hexdump(payload),
        'wire_frame_hex': _hexdump(wire),
    }


def _build_or_die(pump: SetpointPump, cmd: dict, t_origin_us: int):
    sp, reason = pump.build(cmd, t_origin_us)
    if sp is None:
        raise RuntimeError(f'pump rejected a fixture input: {reason!r}')
    return sp


# ── Family 1: pump wire fixtures ─────────────────────────────────────────────

def capture_pump_fixtures() -> dict:
    # Guard: the frozen literals must equal the live config at the capture
    # checkout, or the "production values" claim in the fixture is a lie.
    live_mm = tuple(float(x) for x in hw.GEOM_MM_TO_REV)
    assert live_mm == MM_TO_REV, (
        f'GEOM_MM_TO_REV drifted: literal {MM_TO_REV} vs live {live_mm} — '
        f'this capture is only valid at the checkout its literals were '
        f'frozen at')
    assert float(hw.JB_OP_MAX_POSITION_STEP_REV) == 0.3

    default_args = {
        'mm_to_rev': list(MM_TO_REV),
        'num_legs': 6,
        'max_step_rev': 0.3,
        'torque_ff_enabled': False,
        'torque_ff_max_nm': 0.15,
        'torque_wire_scale': 0.9672514310008596,
        'torque_ff_ramp_frames': 0,
    }
    cases = []

    # (a) full Mode-1 frame: motor_rev + vel + both lookaheads, flags 0x3.
    pump = SetpointPump(**default_args)
    cmd = {
        'type': 'mpc_cmd', 'seq': 17,
        'ext_mm': list(EXT_MM_A),
        'motor_rev': list(MOTOR_REV_A),
        'vel_mm_s': list(VEL_MM_S_A),
        'cmd_next_mm': list(CMD_NEXT_MM_A),
        'cmd_next2_mm': list(CMD_NEXT2_MM_A),
    }
    sp = _build_or_die(pump, cmd, T_ORIGIN_BASE_US)
    assert sp.flags == 0x3
    cases.append({
        'case': 'a_full_mode1',
        'description': ('motor_rev + vel_mm_s + cmd_next_mm + cmd_next2_mm '
                        '(HAS_U1|HAS_U2 = 0x3); u0 = motor_rev VERBATIM, '
                        'u1/u2 = cmd_next(_2)_mm x mm_to_rev, torque_ff = 0'),
        'pump_args': default_args,
        'frames': [_setpoint_record(cmd, T_ORIGIN_BASE_US, 17, sp)],
    })

    # (b) ext_mm fallback: no motor_rev → u0 = ext_mm × mm_to_rev.
    pump = SetpointPump(**default_args)
    cmd = {
        'type': 'mpc_cmd', 'seq': 18,
        'ext_mm': [150.0, 151.25, 149.5, 150.75, 152.0, 148.875],
        'vel_mm_s': [5.0, 5.0, -5.0, 0.5, -0.5, 0.0],
        'cmd_next_mm': [150.2, 151.35, 149.3, 150.85, 151.9, 148.975],
        'cmd_next2_mm': [150.4, 151.45, 149.1, 150.95, 151.8, 149.075],
    }
    sp = _build_or_die(pump, cmd, T_ORIGIN_BASE_US + 25000)
    assert sp.flags == 0x3
    cases.append({
        'case': 'b_ext_fallback',
        'description': ('no motor_rev → u0 = ext_mm x mm_to_rev '
                        '(extension convention, no stow offset)'),
        'pump_args': default_args,
        'frames': [_setpoint_record(cmd, T_ORIGIN_BASE_US + 25000, 18, sp)],
    })

    # (c) no-lookahead frame: motor_rev + vel only, flags 0x0 (Mode-2 Taylor).
    pump = SetpointPump(**default_args)
    cmd = {
        'type': 'mpc_cmd', 'seq': 19,
        'motor_rev': list(MOTOR_REV_A),
        'vel_mm_s': list(VEL_MM_S_A),
    }
    sp = _build_or_die(pump, cmd, T_ORIGIN_BASE_US + 50000)
    assert sp.flags == 0x0
    cases.append({
        'case': 'c_no_lookahead',
        'description': 'no cmd_next/cmd_next2 → flags 0x0, u1 = u2 = zeros',
        'pump_args': default_args,
        'frames': [_setpoint_record(cmd, T_ORIGIN_BASE_US + 50000, 19, sp)],
    })

    # (d) torque-FF ramp-in sequence: 5 frames, ramp_frames=3, so the wire
    # torque progression is 0, 1/3, 2/3, 1, 1 of clamp(torque)×wire_scale —
    # frame 3 also drives the ±0.15 true-Nm clamp on legs 2 and 4.
    ff_args = dict(default_args)
    ff_args['torque_ff_enabled'] = True
    ff_args['torque_ff_ramp_frames'] = 3
    pump = SetpointPump(**ff_args)
    tq_nominal = [0.031, -0.022, 0.04, 0.011, -0.009, 0.027]
    tq_clamping = [0.031, -0.022, 0.2, 0.011, -0.5, 0.027]  # legs 2/4 clamp
    frames = []
    for k in range(5):
        mr = [round(v + 0.05 * k, 6) for v in MOTOR_REV_A]  # < 0.3 rev steps
        nxt = [round((mr[i] + 0.0007) / MM_TO_REV[i], 6) for i in range(6)]
        nxt2 = [round((mr[i] + 0.0014) / MM_TO_REV[i], 6) for i in range(6)]
        cmd = {
            'type': 'mpc_cmd', 'seq': 100 + k,
            'motor_rev': mr,
            'vel_mm_s': [2.0, -2.0, 2.0, -2.0, 2.0, -2.0],
            'cmd_next_mm': nxt,
            'cmd_next2_mm': nxt2,
            'torque_Nm': list(tq_clamping if k == 2 else tq_nominal),
        }
        t_us = T_ORIGIN_BASE_US + 25000 * k
        sp = _build_or_die(pump, cmd, t_us)
        frames.append(_setpoint_record(cmd, t_us, 100 + k, sp))
    cases.append({
        'case': 'd_torque_ff_ramp_sequence',
        'description': ('torque_ff_enabled=True, ramp_frames=3: wire torque '
                        'ramps 0, 1/3, 2/3, 1, 1 x clamp(torque_Nm, ±0.15) x '
                        '0.9672514310008596; frame index 2 exercises the '
                        'clamp on legs 2 (+0.2→+0.15) and 4 (−0.5→−0.15)'),
        'pump_args': ff_args,
        'frames': frames,
    })

    # (e) multi-frame sequence, default pump (FF off): torque_Nm PRESENT in
    # the cmd but never read — frames byte-identical to the pre-FF feature.
    pump = SetpointPump(**default_args)
    frames = []
    for k in range(4):
        mr = [round(v - 0.04 * k, 6) for v in MOTOR_REV_A]
        nxt = [round((mr[i] - 0.0005) / MM_TO_REV[i], 6) for i in range(6)]
        cmd = {
            'type': 'mpc_cmd', 'seq': 200 + k,
            'motor_rev': mr,
            'vel_mm_s': [-1.5, 1.5, -1.5, 1.5, -1.5, 1.5],
            'cmd_next_mm': nxt,
            'torque_Nm': list(tq_nominal),   # must be ignored (FF off)
        }
        t_us = T_ORIGIN_BASE_US + 1000000 + 25000 * k
        sp = _build_or_die(pump, cmd, t_us)
        assert sp.torque_ff == (0.0,) * 6
        assert sp.flags == 0x1               # cmd_next only → HAS_U1
        frames.append(_setpoint_record(cmd, t_us, 200 + k, sp))
    cases.append({
        'case': 'e_default_pump_sequence',
        'description': ('4-frame walk on a default pump (torque FF off): '
                        'torque_Nm present in the cmd but unread → '
                        'torque_ff = zeros; cmd_next only → flags 0x1'),
        'pump_args': default_args,
        'frames': frames,
    })

    # (f) chained: real KnotEmitter Hold frame → ipc wire form → pump.
    # Pins the production emitter→pump seam end-to-end at this checkout.
    geom = StewartGeometry()
    emit = KnotEmitter(geom)
    hold_frame = emit.frame(HoldPlan(np.array([0.0, 0.0, 170.0,
                                               0.0, 0.0, 0.0])), 0.0, 0)
    _, blob = ipc._pack(ipc.TOPIC_MPC_CMD, hold_frame)
    wire_cmd = msgpack.unpackb(blob, raw=False)
    pump = SetpointPump(mm_to_rev=live_mm,
                        max_step_rev=float(hw.JB_OP_MAX_POSITION_STEP_REV))
    sp = _build_or_die(pump, wire_cmd, T_ORIGIN_BASE_US + 2000000)
    cases.append({
        'case': 'f_emitter_hold_chained',
        'description': ('KnotEmitter.frame(HoldPlan([0,0,170,0,0,0]), 0.0, 0) '
                        '→ ipc._pack → msgpack.unpackb → production-config '
                        'pump (mm_to_rev=GEOM_MM_TO_REV, max_step_rev='
                        'JB_OP_MAX_POSITION_STEP_REV). cmd here is the '
                        'decoded wire form.'),
        'pump_args': {'mm_to_rev': list(live_mm),
                      'max_step_rev': float(hw.JB_OP_MAX_POSITION_STEP_REV)},
        'frames': [_setpoint_record(wire_cmd, T_ORIGIN_BASE_US + 2000000,
                                    300, sp)],
    })

    return {
        'fixture': 'v5_pump_wire_fixtures',
        'warning': WARNING,
        'captured_at_commit': _git_head(),
        'captured_date': CAPTURED_DATE,
        'command': EMIT_COMMAND,
        'protocol_version': int(p.PROTOCOL_VERSION),
        'setpoint_size': int(p.SETPOINT_SIZE),
        'msg_type_setpoint': int(p.MsgType.SETPOINT),
        'wire_frame_note': ('wire_frame_hex = encode_frame(MsgType.SETPOINT, '
                            'wire_seq, setpoint_payload) — full UDP frame: '
                            '8-byte header (magic 0x4A42, version 5, type, '
                            'seq, length) + 156-byte payload + CRC-16/CCITT-'
                            'FALSE'),
        'environment': _environment(),
        'cases': cases,
    }


# ── Family 2: emitter output fixtures ────────────────────────────────────────

def capture_emitter_fixtures() -> dict:
    neutral = [0.0, 0.0, 170.0, 0.0, 0.0, 0.0]
    geom = StewartGeometry()
    emit = KnotEmitter(geom)

    def _emit_hex(plan, tau: float, seq: int) -> dict:
        frame = emit.frame(plan, tau, seq)
        topic, blob = ipc._pack(ipc.TOPIC_MPC_CMD, frame)
        return {
            'tau': repr(float(tau)),        # repr round-trips the exact float
            'seq': seq,
            'topic': topic.decode('ascii'),
            'msgpack_hex': _hexdump(blob),  # frames[1]: key order + float bits
        }

    cases = []

    # HoldPlan at neutral — the steady-state production frame.
    plan = HoldPlan(np.array(neutral))
    cases.append({
        'case': 'hold_neutral',
        'recipe': ("HoldPlan(np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])); "
                   "emitter = KnotEmitter(StewartGeometry()); "
                   "frame = emitter.frame(plan, tau, seq)"),
        'frames': [
            _emit_hex(plan, 0.0, 0),
            _emit_hex(plan, 0.7625, 31),    # deliberately not knot-aligned
            _emit_hex(plan, 5.0, 200),
        ],
    })

    # Profiled legacy move: build_return_to_neutral, the pattern
    # tests/motion/test_trajectory_emitter.py uses.
    state0_pose = [12.0, -8.0, 176.0, 0.0, 0.0, 0.0]   # neutral + (12,-8,6)
    state0 = (np.array(state0_pose), np.zeros(6), np.zeros(6))
    limits = TrajectoryLimits.from_config(hw)
    plan = planner.build_return_to_neutral(state0, np.array(neutral), 2.0,
                                           limits, geom)
    td = float(plan.total_duration)
    cases.append({
        'case': 'move_return_to_neutral',
        'recipe': ("state0 = (np.array([12.0, -8.0, 176.0, 0.0, 0.0, 0.0]), "
                   "np.zeros(6), np.zeros(6)); "
                   "limits = TrajectoryLimits.from_config(hw); "
                   "plan = planner.build_return_to_neutral(state0, "
                   "np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]), 2.0, "
                   "limits, StewartGeometry()); "
                   "emitter = KnotEmitter(StewartGeometry()); "
                   "frame = emitter.frame(plan, tau, seq)"),
        'total_duration': repr(td),
        'frames': [
            _emit_hex(plan, 0.0, 0),
            _emit_hex(plan, 0.025, 1),          # one knot in
            _emit_hex(plan, 1.0, 40),           # mid-plan
            _emit_hex(plan, td, int(td / 0.025)),
            _emit_hex(plan, td + 0.5, int(td / 0.025) + 20),  # terminal hold
        ],
    })

    return {
        'fixture': 'v5_emitter_frame_fixtures',
        'warning': WARNING,
        'captured_at_commit': _git_head(),
        'captured_date': CAPTURED_DATE,
        'command': EMIT_COMMAND,
        'msgpack_note': ("msgpack_hex is frames[1] of jugglebot.motion.ipc."
                         "_pack(TOPIC_MPC_CMD, KnotEmitter.frame(plan, tau, "
                         "seq)) — it pins dict key ORDER and exact float "
                         "BITS, the byte-identity T-U8/T-R1 assert. taus and "
                         "total_duration are stored as repr() so the exact "
                         "float64 round-trips."),
        'torque_ff_note': ("KnotEmitter's default LegTorqueFeedforward rides "
                           "the shipped config (DYNAMICS_TORQUE_FF_ENABLED = "
                           f"{bool(hw.DYNAMICS_TORQUE_FF_ENABLED)}), so "
                           "torque_Nm carries the gravity FF these bytes "
                           "include."),
        'environment': _environment(),
        'cases': cases,
    }


# ── Provenance helpers ───────────────────────────────────────────────────────

def _git_head() -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=REPO_ROOT).decode().strip()
    except Exception:
        return 'UNKNOWN'


def _environment() -> dict:
    return {
        'python': sys.version.split()[0],
        'numpy': np.__version__,
        'msgpack': getattr(msgpack, 'version', None) and
        '.'.join(str(x) for x in msgpack.version),
    }


def _write(path: str, doc: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
        f.write('\n')
    print(f'wrote {os.path.relpath(path, REPO_ROOT)}')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--emit-fixtures', action='store_true',
                    help='write the committed fixture files under tests/')
    ap.add_argument('--out-dir', default=None,
                    help='write both JSONs to this dir (determinism checks)')
    args = ap.parse_args()

    if int(p.PROTOCOL_VERSION) != 5:
        print(f'REFUSING: PROTOCOL_VERSION is {p.PROTOCOL_VERSION}, not 5. '
              f'This capture is only meaningful at a pre-v6 checkout; the '
              f'committed fixtures are the frozen baseline — never '
              f'regenerate them after the v6 change.', file=sys.stderr)
        return 2

    pump_doc = capture_pump_fixtures()
    emitter_doc = capture_emitter_fixtures()

    if args.emit_fixtures:
        _write(PUMP_FIXTURE_PATH, pump_doc)
        _write(EMITTER_FIXTURE_PATH, emitter_doc)
    elif args.out_dir:
        _write(os.path.join(args.out_dir, 'v5_pump_wire_fixtures.json'),
               pump_doc)
        _write(os.path.join(args.out_dir, 'v5_emitter_frame_fixtures.json'),
               emitter_doc)
    else:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        out = os.path.join(OUTPUT_DIR, f'v5_wire_fixtures_{stamp}')
        _write(os.path.join(out, 'v5_pump_wire_fixtures.json'), pump_doc)
        _write(os.path.join(out, 'v5_emitter_frame_fixtures.json'),
               emitter_doc)

    n_pump = sum(len(c['frames']) for c in pump_doc['cases'])
    n_emit = sum(len(c['frames']) for c in emitter_doc['cases'])
    print(f'captured {len(pump_doc["cases"])} pump cases ({n_pump} frames), '
          f'{len(emitter_doc["cases"])} emitter cases ({n_emit} frames) '
          f'at commit {pump_doc["captured_at_commit"][:7]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
