"""FEAT-56: pins for physical and wire constants that are typed by hand in more than one place.

Each test reads two copies from SOURCE TEXT (AST, never an import of a ROS node or a mujoco
module) and fails with a message naming both files and both values. Nothing here changes a
copy: where the copies are supposed to be one number, the test says so; where the owner has
not decided, the test pins today's value as a tripwire and says that too (the sim's and
human_catch's 9810 were moved to the config's value 2026-11-26 and are pinned equal).

Already pinned elsewhere, not repeated here:
  * ``executor.REBASE_TOL_*`` and ``schedule.REST_RETURN_ONSET_BUDGET_REV`` against the firmware
    resume tolerances: ``tests/firmware/test_sched_c2_twin.py`` (the half-relation test).
  * ``DIAG_FLAG_LEG_HELD`` / ``DIAG_FLAG_RIG_BENCH`` (``teensy_link/activate.py``) against
    ``leg_stow_gate.h``: ``tests/firmware/test_stow_gate_xref.py``.
"""
from __future__ import annotations

import ast
import math
import os
import re

import numpy as np
import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PKG = os.path.join("ros_ws", "src", "jugglebot", "jugglebot")
_FW = os.path.join("ros_ws", "src", "jugglebot", "Teensy_code_canbridge")
_SKILLS = os.path.join(_PKG, "motion", "skills")
_TRAJ = os.path.join(_PKG, "motion", "trajectory")


def _text(rel):
    with open(os.path.join(_REPO, rel), encoding="utf-8") as f:
        return f.read()


def _const(rel, name):
    """Value of the module-level assignment ``name = <expr>`` in ``rel``, evaluated from source."""
    tree = ast.parse(_text(rel))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets):
            expr = ast.Expression(node.value)
            ast.fix_missing_locations(expr)
            return eval(compile(expr, rel, "eval"), {"np": np, "math": math, "float": float, "hw": _hw()})
    raise AssertionError("%s: no module-level assignment of %s" % (rel, name))


def _same(rel_a, val_a, rel_b, val_b, what, tol=0.0):
    ok = abs(val_a - val_b) <= tol
    assert ok, "%s: %s = %r but %s = %r" % (what, rel_a, val_a, rel_b, val_b)


def _hw():
    import hardware_config as hw  # generated; tests/conftest.py puts it on sys.path
    return hw


# ── 1. Gravity ───────────────────────────────────────────────────────────────
_G_MMS2_COPIES = [
    (os.path.join("controller", "ballistics.py"), "GRAVITY_MMS2"),
    (os.path.join(_TRAJ, "ballistics_bc.py"), "GRAVITY_MMS2"),
    (os.path.join(_TRAJ, "shaping.py"), "G_MM_S2_DEFAULT"),
    (os.path.join(_PKG, "tracking", "ballistics.py"), "GRAVITY_MMPS2"),
    (os.path.join("sim", "ball_butler", "sim.py"), "_GRAVITY_MMS2"),
]


@pytest.mark.parametrize("rel,name", _G_MMS2_COPIES, ids=[c[0] for c in _G_MMS2_COPIES])
def test_gravity_copies_equal_the_generated_config(rel, name):
    hw = _hw()
    _same(rel, float(_const(rel, name)), "config/generated/hardware_config.py",
          float(hw.GRAVITY_MMPS2), "gravity (mm/s^2)")


def test_cup_cycle_gravity_equals_the_generated_config():
    rel = os.path.join(_TRAJ, "cup_cycle.py")
    g = _const(rel, "GRAVITY")                      # np.array([0, 0, -9.806]) in m/s^2
    _same(rel, -float(g[2]), "config/generated/hardware_config.py",
          float(_hw().GRAVITY_MPS2), "gravity (m/s^2), cup_cycle.GRAVITY z")


def test_generated_gravity_units_agree():
    hw = _hw()
    _same("GRAVITY_MPS2*1000", 1000.0 * hw.GRAVITY_MPS2, "GRAVITY_MMPS2", hw.GRAVITY_MMPS2,
          "generated gravity in two units", tol=1e-9)


def test_sim_skills_gate_gravity_follows_the_config():
    """Owner decision 2026-11-26: ``sim/skills_gate.py`` takes the generated gravity (it was a typed
    9810.0, 0.04 % high). The module reads ``hw.GRAVITY_MMPS2``; this checks the source says so and
    the evaluated value equals the config, so a typed literal coming back fails here."""
    rel = os.path.join("sim", "skills_gate.py")
    assert "hw.GRAVITY_MMPS2" in _text(rel), "%s: _GRAVITY_MMS2 no longer reads the config" % rel
    _same(rel + " _GRAVITY_MMS2", _const(rel, "_GRAVITY_MMS2"), "GRAVITY_MMPS2", _hw().GRAVITY_MMPS2,
          "sim skills gate gravity")


def test_sim_skills_gate_leg_limits_follow_the_config():
    """Owner decision 2026-11-28: the sim gate's Sitting Leg limits are the generated launch working
    point (``hw.JB_TRAJ_LEG_*_LIMIT_*``), not typed numbers (it carried 350 / 5000 after the launch
    moved to 600 / 8000 on 2026-11-03, so sim trials judged a stricter plant than the robot flies).
    Source says so, and the evaluated values equal the config."""
    rel = os.path.join("sim", "skills_gate.py")
    hw = _hw()
    for name, cfg in (("_SESSION_LEG_VEL_MMPS", "JB_TRAJ_LEG_VEL_LIMIT_MMPS"),
                      ("_SESSION_LEG_ACC_MMPS2", "JB_TRAJ_LEG_ACC_LIMIT_MMPS2"),
                      ("_SESSION_LEG_JERK_MMPS3", "JB_TRAJ_LEG_JERK_LIMIT_MMPS3")):
        assert "hw." + cfg in _text(rel), "%s: %s no longer reads the config" % (rel, name)
        _same(rel + " " + name, _const(rel, name), cfg, getattr(hw, cfg), "sim skills gate leg limit")


def test_probe_gravity_literals_are_pinned_as_they_are_today():
    """Probes hand-type gravity three ways (9810.0, 9806.65 = standard gravity, config 9806).
    Pinned as found; not a claim that any is right (owner has not decided). Probe-only: no
    production path reads them."""
    found = {
        ("tools/probes/ball_arrival_offset.py", r"\bg = (9810\.0)\b"): 9810.0,
        ("tools/probes/feed_lateral_miss.py", r"^G_MM_S2 = ([0-9.]+)"): 9810.0,
        ("tools/probes/human_throw_bench.py", r"^G = ([0-9.]+)"): 9810.0,
        ("tools/probes/selftoss_landing_decomposition.py", r"^G = ([0-9.]+)"): 9806.65,
        ("tools/probes/level_vs_ballfit.py", r"^G = ([0-9.]+)"): 9806.65,
    }
    for (rel, pat), want in found.items():
        m = re.search(pat, _text(rel), re.M)
        assert m, "%s: gravity literal not found (pattern %s)" % (rel, pat)
        assert float(m.group(1)) == want, "%s gravity %s, pinned %s" % (rel, m.group(1), want)


def test_human_catch_gravity_follows_the_config():
    """``motion/skills/human_catch.py`` computes the nominal arrival speed from a rise with the
    generated gravity (owner decision 2026-11-26; it was an inline 9810.0). No gravity literal may
    sit in the formula."""
    rel = os.path.join(_SKILLS, "human_catch.py")
    text = _text(rel)
    assert "math.sqrt(2.0 * hw.GRAVITY_MMPS2 * rise)" in text, (
        "%s: the rise-to-speed formula no longer reads hw.GRAVITY_MMPS2" % rel)
    assert not re.search(r"\b98(06|10)(\.\d*)?\b", text), "%s: a typed gravity literal is back" % rel


# ── 2. Capture bound ─────────────────────────────────────────────────────────
def _node_default(name):
    """The default of ``self.declare_parameter('<name>', <default>)`` in skill_node.py."""
    rel = os.path.join(_PKG, "skill_node.py")
    ms = re.findall(r"declare_parameter\(\s*'%s'\s*,\s*([-0-9.eE]+)\s*\)" % re.escape(name), _text(rel))
    assert len(ms) == 1, "%s: expected one declare_parameter(%r), found %d" % (rel, name, len(ms))
    return float(ms[0])


def test_capture_bound_matches_the_skill_node_authority_defaults():
    rel = os.path.join(_TRAJ, "cup_cycle.py")
    bound = float(_const(rel, "CAPTURE_MAX_OFFSET_M"))
    inward = _node_default("catch_inward_authority_mm") / 1000.0
    lateral = _node_default("learner_lateral_authority_mm") / 1000.0
    _same(rel + " CAPTURE_MAX_OFFSET_M", bound,
          "skill_node.py hypot(catch_inward_authority_mm, learner_lateral_authority_mm) [m]",
          math.hypot(inward, lateral),
          "capture bound (a raised authority makes the clamp emit an aim the Capture move refuses)",
          tol=1e-12)


# ── 3. Heartbeat / Diagnostic flag bits ──────────────────────────────────────
_NODE = os.path.join(_PKG, "teensy_bridge_node.py")


def test_node_t2j_flag_bits_equal_the_generated_enum():
    from teensy_link import protocol as p
    gen = "config/generated/udp_protocol.py HeartbeatT2JFlags."
    for node_name, enum_name in (("_T2J_FLAG_TIME_SYNCED", "TIME_SYNCED"),
                                 ("_T2J_FLAG_STOW_PENDING", "STOW_PENDING_ON_RECONNECT"),
                                 ("_T2J_FLAG_ALL_AXIS_HB_OK", "ALL_AXIS_HEARTBEATS_OK")):
        _same(_NODE + " " + node_name, int(_const(_NODE, node_name)),
              gen + enum_name, int(getattr(p.HeartbeatT2JFlags, enum_name)), "T2J flag bit")
    mask = int(p.HeartbeatT2JFlags.CONE_HEALTH_MASK)
    _same(_NODE + " _T2J_CONE_HEALTH_SHIFT", int(_const(_NODE, "_T2J_CONE_HEALTH_SHIFT")),
          gen + "CONE_HEALTH_MASK lowest set bit", (mask & -mask).bit_length() - 1, "cone health shift")


def test_node_j2t_mpc_active_bit_equals_the_protocol_comment():
    """Not generated as a constant: the only schema statement is the HeartbeatJ2T field comment
    'bit0: mpc_active' in the generated header."""
    hdr = os.path.join(_FW, "udp_protocol.h")
    assert re.search(r"uint32_t flags;\s*//\s*bit0: mpc_active", _text(hdr)), \
        "%s: the HeartbeatJ2T 'bit0: mpc_active' comment moved" % hdr
    _same(_NODE + " _FLAG_MPC_ACTIVE", int(_const(_NODE, "_FLAG_MPC_ACTIVE")),
          hdr + " HeartbeatJ2T.flags bit0", 1, "J2T mpc_active bit")


def test_node_bb_flag_bits_equal_the_protocol_comment():
    """The BB bits are not generated: the schema's only statement is the ``bb_flags`` field
    comment 'bit0 ball_in_hand, bit1 heartbeat_seen, bit2 heartbeat_stale'."""
    hdr = os.path.join(_FW, "udp_protocol.h")
    m = re.search(r"uint8_t bb_flags;\s*//\s*(.+)", _text(hdr))
    assert m, "%s: bb_flags field comment not found" % hdr
    bits = {name: 1 << int(b) for b, name in re.findall(r"bit(\d)\s+(\w+)", m.group(1))}
    for node_name, field in (("_T2J_BB_FLAG_BALL_IN_HAND", "ball_in_hand"),
                             ("_T2J_BB_FLAG_HEARTBEAT_SEEN", "heartbeat_seen"),
                             ("_T2J_BB_FLAG_HEARTBEAT_STALE", "heartbeat_stale")):
        _same(_NODE + " " + node_name, int(_const(_NODE, node_name)),
              hdr + " bb_flags comment '%s'" % field, bits[field], "BB flag bit")


def test_node_diag_flag_bits_equal_the_firmware():
    # bits 2 and 3: named constants in leg_activate.h.
    la = os.path.join(_FW, "leg_activate.h")
    for node_name, fw_name in (("_DIAG_FLAG_ACTIVATE_STALL", "DIAG_FLAG_ACTIVATE_STALL"),
                               ("_DIAG_FLAG_LANE_STALL", "DIAG_FLAG_LANE_STALL")):
        m = re.search(r"\b%s\s*=\s*(0x[0-9A-Fa-f]+)u;" % fw_name, _text(la))
        assert m, "%s: %s not found" % (la, fw_name)
        _same(_NODE + " " + node_name, int(_const(_NODE, node_name)),
              la + " " + fw_name, int(m.group(1), 16), "Diagnostic flag bit")
    # bits 0 and 1: bare literals in telemetry.cpp's flags expression (no named constant exists).
    tc = os.path.join(_FW, "telemetry.cpp")
    for node_name, expr, want in (("_DIAG_FLAG_HB_STALE", r"\(stale \? (0x\w+)u", 0x1),
                                  ("_DIAG_FLAG_HB_SEEN", r"\(a\.heartbeat_seen \? (0x\w+)u", 0x2)):
        m = re.search(expr, _text(tc))
        assert m, "%s: flags expression for %s not found" % (tc, node_name)
        _same(_NODE + " " + node_name, int(_const(_NODE, node_name)),
              tc + " flags literal", int(m.group(1), 16), "Diagnostic flag bit")
        assert int(m.group(1), 16) == want


def test_node_diag_flags_cover_every_firmware_bit_including_fw53():
    """Bits 0-5 are all used and distinct: HB_STALE 0x1, HB_SEEN 0x2, ACTIVATE_STALL 0x4,
    LANE_STALL 0x8 (typed in the node) and LEG_HELD 0x10, RIG_BENCH 0x20 (FW 53, imported from
    teensy_link.activate, pinned to leg_stow_gate.h by test_stow_gate_xref.py). A bit added to the
    firmware without a node name fails here."""
    from teensy_link import activate
    node = [int(_const(_NODE, n)) for n in ("_DIAG_FLAG_HB_STALE", "_DIAG_FLAG_HB_SEEN",
                                            "_DIAG_FLAG_ACTIVATE_STALL", "_DIAG_FLAG_LANE_STALL")]
    node += [int(activate.DIAG_FLAG_LEG_HELD), int(activate.DIAG_FLAG_RIG_BENCH)]
    assert sorted(node) == [0x1, 0x2, 0x4, 0x8, 0x10, 0x20], \
        "%s + teensy_link/activate.py Diagnostic flag set is %r" % (_NODE, node)
    gate = _text(os.path.join(_FW, "leg_stow_gate.h"))
    for name, want in (("DIAG_FLAG_LEG_HELD", 0x10), ("DIAG_FLAG_RIG_BENCH", 0x20)):
        m = re.search(r"\b%s\s*=\s*(0x[0-9A-Fa-f]+)u;" % name, gate)
        assert m and int(m.group(1), 16) == want, "leg_stow_gate.h %s != %#x" % (name, want)


# ── 4. REST return budget ────────────────────────────────────────────────────
def test_rest_return_budget_is_the_half_resume_tolerance_and_holds_at_the_onset():
    """The half-relation against ``SCHED_RESUME_TOL_POS_HAND_REV`` is pinned in
    ``test_sched_c2_twin.py``. What is NOT pinned there and is here: the acceleration limit that
    the budget's own docstring derives (0.45 * a * onset^2) really fits inside the budget, and
    the other resume tolerances the schedule.py docstring restates (8 rev/s, 1000 rev/s^2)."""
    sch = os.path.join(_SKILLS, "schedule.py")
    onset = float(_const(sch, "REST_RETURN_ONSET_S"))
    acc = float(_const(sch, "REST_RETURN_HAND_ACC_LIMIT_RPS2"))
    budget = float(_const(sch, "REST_RETURN_ONSET_BUDGET_REV"))
    assert 0.45 * acc * onset ** 2 <= budget, (
        "%s: onset displacement 0.45*%r*%r^2 = %r rev exceeds REST_RETURN_ONSET_BUDGET_REV %r"
        % (sch, acc, onset, 0.45 * acc * onset ** 2, budget))
    cb = _text(os.path.join(_FW, "canbridge_config.h"))
    for name, doc_val in (("SCHED_RESUME_TOL_POS_HAND_REV", 0.05), ("SCHED_RESUME_TOL_VEL_HAND_RPS", 8.0),
                          ("SCHED_RESUME_TOL_ACC_HAND_RPS2", 1000.0)):
        m = re.search(r"\b%s\s*=\s*([-0-9.eE]+)[fu]?\s*;" % name, cb)
        assert m, "canbridge_config.h: %s not found" % name
        assert float(m.group(1)) == doc_val, (
            "canbridge_config.h %s = %s but schedule.py's docstring restates %s" % (name, m.group(1), doc_val))


# ── 5. RELEASE_KINDS ─────────────────────────────────────────────────────────
def test_release_kinds_are_identical_across_schedule_plant_model_and_memory():
    # BUG-138 moved the Learner's copy into plant_model.py (the lateral law is per release kind);
    # learner.py imports it, so it holds no literal of its own to drift.
    files = [os.path.join(_SKILLS, f) for f in ("schedule.py", "plant_model.py", "memory.py")]
    vals = {rel: tuple(_const(rel, "RELEASE_KINDS")) for rel in files}
    ref = vals[files[0]]
    for rel in files[1:]:
        assert vals[rel] == ref, "RELEASE_KINDS: %s = %r but %s = %r" % (files[0], ref, rel, vals[rel])
    learner = _text(os.path.join(_SKILLS, "learner.py"))
    assert re.search(r"^from jugglebot\.motion\.skills\.plant_model import [^\n]*\bRELEASE_KINDS\b", learner, re.M), (
        "learner.py must import RELEASE_KINDS from plant_model, not define its own")
    assert not re.search(r"^RELEASE_KINDS\s*=", learner, re.M)


# ── 6. Tool and tracker literals ─────────────────────────────────────────────
def test_kt_literals_equal_the_generated_config():
    kt = float(_hw().DYNAMICS_MOTOR_KT_NM_PER_A)
    for rel, name in ((os.path.join("tests", "hardware", "single_leg_test.py"), "KT_MEASURED"),
                      (os.path.join("tests", "hardware", "friction_ff_demo.py"), "KT_NM_PER_A")):
        _same(rel + " " + name, float(_const(rel, name)),
              "config/generated/hardware_config.py DYNAMICS_MOTOR_KT_NM_PER_A", kt, "motor Kt (Nm/A)")


def test_tracking_trapped_ball_config_backed_constants():
    """``tracking/trapped_ball.py`` reads 578.2 and 32.567 from the generated config by name (so
    ``tools/trapped_ball_finder.py``, which re-exports them, has no literal left); its
    ``PLATFORM_R_MM = 230.0`` is a hand-typed 'plat_radius 219.075 + margin' and is pinned to stay
    at or above the generated radius."""
    tb = os.path.join(_PKG, "tracking", "trapped_ball.py")
    hw = _hw()
    src = _text(tb)
    assert "float(hw.GEOM_INITIAL_HEIGHT_MM)" in src, "%s: INITIAL_HEIGHT_MM no longer reads the config" % tb
    assert "float(hw.GEOM_HAND_MM_PER_REV)" in src, "%s: HAND_MM_PER_REV no longer reads the config" % tb
    assert float(_const(tb, "PLATFORM_R_MM")) >= hw.GEOM_PLAT_RADIUS_MM, (
        "%s PLATFORM_R_MM %r is below config/generated GEOM_PLAT_RADIUS_MM %r"
        % (tb, _const(tb, "PLATFORM_R_MM"), hw.GEOM_PLAT_RADIUS_MM))
    assert float(_const(tb, "PLATFORM_R_MM")) == 230.0   # the FEAT-56 table value
