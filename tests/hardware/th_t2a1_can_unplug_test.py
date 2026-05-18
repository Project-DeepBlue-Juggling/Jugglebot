#!/usr/bin/env python3
"""T-H-T2a-1 — CAN unplug for 1-2 s.

**Target date**: 2026-05-25 (per Plan 2 Phase 5 Outcome paragraph).
**Hazard**: MEDIUM-HIGH.  Platform freewheels under gravity from the
moment ODrives disarm.  Mitigated by:

* Holding at a LOW safe pose (small drop bounded by mechanics)
* Operator HAND ON physical E-stop button throughout
* Second person present
* The 1-2 s unplug duration is the operator's guaranteed minimum;
  cable can be re-plugged within that window if needed.

**Gating**: T-H-T2b-1 (encoder-publisher kill) MUST have PASSED
recently before this test runs.  The script asks the operator to
type the SHA of the latest T-H-T2b-1 PASS artefact as part of the
pre-test checklist.

**What this script does**:

1. Walks the operator through the universal pre-test setup +
   T-H-T2b-1-PASSED-recently gate.  Type-confirmation gates on every
   item.
2. Optionally starts a background ``candump can0`` capture (if
   ``--can-iface can0`` is reachable and ``candump`` is installed).
   Captures timestamped CAN frames for ODrive disarm timing.
3. Tails the latest ``temp/logs/*.log`` for the Plan 2 Phase 5
   telemetry-stale cascade patterns.
4. Counts down 5-4-3-2-1, prompts the operator with a VERY LOUD
   "UNPLUG NOW" instruction.  Records the operator's keystroke as
   t=0 (the operator presses Enter at the moment of unplug).
5. After 1.5 s, prompts the operator to "PLUG BACK IN".  Records
   that keystroke too.
6. Continues to tail logs for 5 more seconds capturing the
   cascade's tail behaviour.
7. Computes the timing chain (telemetry-stale fire / ESTOP fire,
   relative to t=0).  Compares against documented bounds.
8. Prompts the operator for free-form observations (where did the
   platform land?  did the operator E-stop manually?  any other
   anomaly?).
9. Writes paired JSON + Markdown artefacts to ``temp/reports/``.

**What this script DOES NOT do**:

* Cannot physically pull the cable — that's the operator's job.
* Does NOT command motion.  Does NOT bypass the E-stop button.
* Does NOT analyse the CAN dump payload (the ODrive disarm event
  identification requires ODrive-protocol-specific decoding which
  is out of scope; the raw candump file is left as an artefact for
  the operator to inspect manually).

Prereq: ``jugglebot_launch.py`` is running and the platform is held
at a low safe pose; T-H-T2b-1 has PASSED recently.

Usage::

    python tests/hardware/th_t2a1_can_unplug_test.py
    python tests/hardware/th_t2a1_can_unplug_test.py --can-iface can0
    python tests/hardware/th_t2a1_can_unplug_test.py --no-candump
    python tests/hardware/th_t2a1_can_unplug_test.py --operator "Harrison Low"
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _th_test_common import (
    LogTailer, TestResult, assert_log_is_live, find_latest_mpc_log,
    get_repo_sha, safety_gate, wait_gate, write_artifacts,
)


# Documented expectation: the cascade fires the ESTOP shortly after
# telem_age exceeds 20× control_dt (default 0.500 s).  The CAN bus
# loss takes a few ticks to propagate (ODrive heartbeat watchdog →
# disarm → Jetson observes no new telemetry).  The total t0→ESTOP
# window is wider than T-H-T2b-1's because of the CAN-side delay.
# Conservative bound: [400, 800] ms.  The test PASSES if ESTOP fires
# within this window AND the platform freewheels from a known low
# pose without exceeding the mechanical workspace.
EXPECTED_ESTOP_FIRE_WINDOW_MS = (400.0, 800.0)


def start_candump(can_iface: str, output_path: Path) -> subprocess.Popen | None:
    """Start a background ``candump <iface> -t a`` process writing to
    ``output_path``.  Returns the Popen handle or None if candump is
    unavailable / interface unreachable.
    """
    if shutil.which('candump') is None:
        print("  candump not installed; CAN dump disabled.")
        return None
    try:
        f = open(output_path, 'w')
        proc = subprocess.Popen(
            ['candump', can_iface, '-t', 'a'],
            stdout=f, stderr=subprocess.STDOUT,
        )
        # Give it a moment to confirm the interface is reachable
        time.sleep(0.2)
        if proc.poll() is not None:
            print(f"  candump exited immediately (interface {can_iface} "
                  "unreachable?); CAN dump disabled.")
            return None
        print(f"  candump started → {output_path}")
        return proc
    except Exception as exc:
        print(f"  candump failed to start: {exc}; CAN dump disabled.")
        return None


def stop_candump(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
    except Exception:
        pass


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--can-iface', type=str, default='can0',
                   help="CAN interface to dump (default: can0).")
    p.add_argument('--no-candump', action='store_true',
                   help="Skip the background candump capture.")
    p.add_argument('--log-path', type=Path, default=None,
                   help="Path to the MPC log file to tail.  If omitted, "
                        "the newest *.log under temp/logs/ is used.")
    p.add_argument('--operator', type=str, default=os.environ.get('USER', 'unknown'),
                   help="Operator name for the artefact.")
    p.add_argument('--unplug-duration-s', type=float, default=1.5,
                   help="How long to wait between the UNPLUG prompt and "
                        "the PLUG-BACK-IN prompt.  Default 1.5 s.")
    args = p.parse_args()
    script_start_s = time.time()

    print("=" * 72)
    print("T-H-T2a-1 — CAN unplug 1-2 s (Plan 2 Phase 5 hardware bringup)")
    print("=" * 72)
    print()
    print("Hazard profile: MEDIUM-HIGH.  Platform FREEWHEELS under gravity")
    print("from the moment ODrives disarm.  Operator MUST keep a hand on")
    print("the physical E-stop button throughout.")
    print()
    print(f"Operator: {args.operator}")
    print(f"Repo SHA: {get_repo_sha()}")
    print()

    # --- Pre-test gates (universal + cascade-specific) ---
    safety_gate(
        "Pre-test checklist (UNIVERSAL):\n"
        "  1. Platform commanded to a LOW safe pose (close to STOW).\n"
        "  2. Hand detached.  No ball.  Platform-only mode.\n"
        "  3. Operator HAND ON physical E-stop button RIGHT NOW.\n"
        "  4. Second person present in the workshop.\n"
        "  5. jugglebot_launch.py is running and stable.\n"
        "Are ALL of these true?",
        required='yes',
    )
    safety_gate(
        "Pre-test checklist (T-H-T2a-1 specific):\n"
        "  6. T-H-T2b-1 PASSED in the past week (per the gating rule\n"
        "     in plans/active/mpc-sadpath-coverage-tiers-1-3.md).\n"
        "  7. The CAN cable to unplug is identified, traced, and a\n"
        "     CLEAR PATH to plug it back in is visible.\n"
        "  8. Both operators (operator-1 on E-stop, operator-2 on\n"
        "     cable) agree on the protocol.\n"
        "Are ALL of these true?",
        required='yes',
    )
    safety_gate(
        "FINAL HAZARD CONFIRMATION:\n"
        "  The platform WILL FREEWHEEL during the cable unplug.\n"
        "  At a low pose the drop is mechanically bounded, but if the\n"
        "  drop reaches the workspace stop UNSAFELY, operator-1 MUST\n"
        "  manually press the physical E-stop button immediately.\n"
        "Do you understand and accept this hazard?",
        required='yes',
    )

    # --- Locate the MPC log file to tail ---
    if args.log_path is not None:
        log_path = args.log_path
    else:
        log_path = find_latest_mpc_log()
    if log_path is None or not log_path.exists():
        print(f"\nABORTED — no MPC log file found at {log_path}.")
        print("Pass --log-path <path> to specify explicitly.")
        return 1
    # Guard against the silent-false-FAIL trap (see _th_test_common).
    # Critical for T-H-T2a-1: a false FAIL here could prompt an
    # unnecessary repeat of a Medium-high-hazard freewheel test.
    assert_log_is_live(log_path, script_start_s,
                       explicit=args.log_path is not None)
    print(f"\nTailing MPC log: {log_path}")

    # --- Start candump (optional) ---
    candump_path: Path | None = None
    candump_proc: subprocess.Popen | None = None
    if not args.no_candump:
        repo_root = Path(__file__).resolve().parents[2]
        reports_dir = repo_root / 'temp' / 'reports'
        reports_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        candump_path = reports_dir / f"th_t2a1_candump_{stamp}.log"
        print(f"\nStarting candump on {args.can_iface}...")
        candump_proc = start_candump(args.can_iface, candump_path)
    else:
        print("\nCAN dump capture skipped (--no-candump).")

    # --- Start log tailer ---
    tailer = LogTailer(log_path)
    tailer.start()
    time.sleep(0.1)

    # --- Final ready gate ---
    wait_gate(
        "READY.\n"
        "When you press Enter:\n"
        "  1. The script will count down 5-4-3-2-1.\n"
        "  2. On 'UNPLUG NOW!', operator-2 pulls the CAN cable AT THAT MOMENT.\n"
        "     The instant of unplug = t=0 for the cascade.\n"
        f"  3. After {args.unplug_duration_s:.1f} s, the script will print\n"
        "     'PLUG BACK IN NOW!' — operator-2 reconnects the cable.\n"
        "  4. The script captures log output for 5 s afterwards.\n"
        "Operator-1: HAND ON E-STOP.  Press Enter to start countdown."
    )

    # --- Countdown ---
    print()
    for i in range(5, 0, -1):
        print(f"  {i}...")
        time.sleep(1.0)

    # --- The instant — record t0 ---
    t0_wall_clock = time.time()
    t0_iso = datetime.fromtimestamp(t0_wall_clock).isoformat(timespec='milliseconds')
    print()
    print("*" * 72)
    print(f"*  UNPLUG NOW !!!  (t=0 @ {t0_iso})")
    print("*" * 72)

    # --- Hold the unplug window ---
    time.sleep(args.unplug_duration_s)

    # --- Plug back in ---
    plug_back_wall_clock = time.time()
    plug_back_iso = datetime.fromtimestamp(plug_back_wall_clock).isoformat(timespec='milliseconds')
    print()
    print("*" * 72)
    print(f"*  PLUG BACK IN NOW !!!  (t+{(plug_back_wall_clock - t0_wall_clock) * 1000:.0f} ms "
          f"@ {plug_back_iso})")
    print("*" * 72)

    # --- Continue tailing for 5 s to capture cascade tail ---
    print()
    print("  Continuing to tail logs for 5 s...")
    time.sleep(5.0)

    # --- Stop everything ---
    tailer.stop()
    tailer.compute_offsets(t0_wall_clock)
    if candump_proc is not None:
        stop_candump(candump_proc)
        print(f"  candump stopped; output at {candump_path}")

    # --- Analyse the timing chain ---
    chain = {
        'cable_unplugged': 0.0,
        'telem_aging_warn': None,
        'telem_stale_hard': None,
        'telem_stale_estop': None,
        'estop_sent': None,
        'cable_plugged_back_in': (plug_back_wall_clock - t0_wall_clock) * 1000.0,
    }
    for key in ('telem_aging_warn', 'telem_stale_hard',
                'telem_stale_estop', 'estop_sent'):
        hit = tailer.first_hit(key)
        if hit is not None:
            chain[key] = hit.offset_from_t0_ms

    expected_bounds = {
        'cable_unplugged': (0.0, 0.0),
        'telem_aging_warn': None,  # cascade timing depends on motor_guard
        'telem_stale_hard': None,  # behaviour during CAN loss; informational
        'telem_stale_estop': EXPECTED_ESTOP_FIRE_WINDOW_MS,
        'estop_sent': None,
        'cable_plugged_back_in': None,
    }

    # --- Verdict ---
    anomalies: list[str] = []
    bytes_seen = tailer.bytes_observed()
    if bytes_seen == 0:
        # Tailed file never grew — instrumentation failure, NOT a cascade
        # failure.  Do NOT prompt a repeat of this Medium-high-hazard
        # freewheel test on the strength of a dead-tail false FAIL.
        verdict = 'INDETERMINATE'
        anomalies.append(
            f"TAIL TARGET WAS DEAD — {log_path} did not grow during the "
            f"observation window (0 bytes appended).  This is an "
            f"INSTRUMENTATION failure, NOT a cascade failure.  The cascade "
            f"was never observed.  Confirm the platform landed safely and "
            f"the system is in a clean state, then re-run with the LIVE "
            f"session log (tee the launch console; pass --log-path).  Do "
            f"NOT conclude anything about the production cascade from this "
            f"run.")
    elif chain['telem_stale_estop'] is None:
        verdict = 'FAIL'
        anomalies.append(
            f'ESTOP pattern did not fire within the observation window '
            f'though the log grew by {bytes_seen} bytes (tailer WAS live) — '
            f'cascade may be broken; operator MUST manually press physical '
            f'E-stop if not already done.')
    else:
        lo, hi = EXPECTED_ESTOP_FIRE_WINDOW_MS
        if lo <= chain['telem_stale_estop'] <= hi:
            verdict = 'PASS'
        else:
            verdict = 'FAIL'
            anomalies.append(
                f"ESTOP fire time {chain['telem_stale_estop']:.1f} ms outside "
                f"expected window [{lo}, {hi}] ms.  "
                f"Investigate cascade timing.")

    # --- Print summary ---
    print()
    print("=" * 72)
    print(f"AUTOMATED VERDICT (timing only): {verdict}")
    print("=" * 72)
    print(f"  ESTOP fired at t+{chain['telem_stale_estop']} ms "
          f"(expected window: {EXPECTED_ESTOP_FIRE_WINDOW_MS[0]}–"
          f"{EXPECTED_ESTOP_FIRE_WINDOW_MS[1]} ms)")
    print()
    print("REMINDER: the timing verdict is necessary but NOT sufficient.")
    print("  Operator must ALSO confirm:")
    print("  - Platform landed within mechanical workspace (no damage).")
    print("  - Operator did NOT need to manually press E-stop (auto-fire worked).")
    print("  - System reached a clean exit state after cable re-plug.")
    if anomalies:
        print()
        print("ANOMALIES:")
        for a in anomalies:
            print(f"  - {a}")

    # --- Operator post-test observations ---
    print()
    print("Operator observations (free-form; empty line ends each question):")
    questions = [
        "Q1: Where did the platform land?  (within workspace? hit a stop?)",
        "Q2: Did operator-1 need to manually press the physical E-stop?",
        "Q3: Did the system reach a clean exit?  (MPC stopped writing CSV?)",
        "Q4: Any unexpected log output or audible anomaly?",
        "Q5: Override the automated verdict?  (PASS / FAIL / INDETERMINATE / blank=keep automated)",
    ]
    notes_lines: list[str] = []
    override = None
    try:
        for i, q in enumerate(questions, start=1):
            print(f"\n  {q}")
            ans = input("  > ").strip()
            if i == len(questions) and ans:
                if ans.upper() in ('PASS', 'FAIL', 'INDETERMINATE'):
                    override = ans.upper()
                    notes_lines.append(f"OPERATOR VERDICT OVERRIDE: {override}")
                else:
                    notes_lines.append(f"Q{i}: {ans}")
            else:
                if ans:
                    notes_lines.append(f"Q{i}: {ans}")
    except (EOFError, KeyboardInterrupt):
        pass
    notes = "\n".join(notes_lines)

    if override is not None:
        print(f"\nOperator overrode automated verdict: {verdict} → {override}")
        verdict = override

    # --- Write artefacts ---
    log_hits_list = [
        {
            'pattern_key': h.pattern_key,
            'wall_clock_s': h.wall_clock_s,
            'offset_from_t0_ms': h.offset_from_t0_ms,
            'log_line': h.log_line,
        } for h in tailer.hits
    ]
    if candump_path is not None:
        notes += f"\n\nCAN dump captured at: {candump_path}\n" \
                 "(parse manually for ODrive disarm event timing)"

    result = TestResult(
        test_id='T-H-T2a-1',
        test_name='CAN unplug cascade',
        date_iso=datetime.now().strftime('%Y-%m-%d'),
        repo_sha=get_repo_sha(),
        operator=args.operator,
        verdict=verdict,
        t0_wall_clock_iso=t0_iso,
        t0_description='Cable unplugged (operator action)',
        pass_criteria={
            'estop_fires_within_window_ms':
                f'{EXPECTED_ESTOP_FIRE_WINDOW_MS[0]}–'
                f'{EXPECTED_ESTOP_FIRE_WINDOW_MS[1]} ms after unplug',
            'platform_lands_within_workspace':
                'observed by operator (free-form note)',
            'no_manual_estop_needed':
                'observed by operator (free-form note)',
            'clean_exit_after_replug':
                'observed by operator (free-form note)',
        },
        timing_chain_ms=chain,
        expected_bounds_ms=expected_bounds,
        log_path=str(log_path),
        log_hits=log_hits_list,
        operator_notes=notes,
        anomalies=anomalies,
    )
    json_path, md_path = write_artifacts(result)

    print()
    print(f"Artefacts written:")
    print(f"  JSON:    {json_path}")
    print(f"  MD:      {md_path}")
    if candump_path is not None:
        print(f"  candump: {candump_path}")
    print()
    print("Next step: open the MD file, review the paste-ready logbook snippet,")
    print("and create the /investigate entry (suggested filename:")
    print(f"  logbook/{result.date_iso}-th-t2a-1-can-unplug-cascade-bringup.md")
    print(")")
    print()
    print("If T-H-T2a-1 PASS: Plan 2 hardware-bringup obligations are complete.")
    print("If T-H-T2a-1 FAIL: investigate before re-attempting; the cascade")
    print("  is the last-line safety mechanism for CAN loss in production.")

    return 0 if verdict == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
