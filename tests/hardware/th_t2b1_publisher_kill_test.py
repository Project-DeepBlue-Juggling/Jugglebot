#!/usr/bin/env python3
"""T-H-T2b-1 — encoder-publisher kill via SIGSTOP for ~1 s.

**Target date**: 2026-05-18 (per Plan 2 Phase 5 Outcome paragraph).
**Hazard**: Low.  ODrives keep receiving setpoints from motor_guard
throughout; CAN bus stays live; platform stays held.  Only the
Jetson-side observation of ``/robot_state`` (via ZMQ telemetry on
port 5556) is affected.

**What this script does**:

1. Walks the operator through the universal pre-test setup checklist
   (low pose held, hand detached, operator on physical E-stop button,
   second person present).  Type-confirmation gates ("yes") on every
   safety item.
2. Auto-discovers candidate encoder-publisher PIDs via ``pgrep -af``
   on common patterns (motor_guard / can_node / mpc_bridge_node);
   operator confirms or overrides via ``--pid``.
3. Tails the latest ``temp/logs/*.log`` for the Plan 2 Phase 5
   telemetry-stale cascade patterns.
4. Sends SIGSTOP to the chosen PID at t=0.  Waits up to 5 s for the
   ESTOP record to fire.  Sends SIGCONT to revive the publisher.
5. Computes the timing chain (offsets from t=0 to each captured log
   pattern).  Compares against the documented bound (500 ms ± 25 ms).
6. Prompts the operator for a free-form note about platform-side
   observations (did the platform stay held? did ODrives stay armed?).
7. Writes paired JSON + Markdown artefacts to ``temp/reports/``.

**What this script DOES NOT do**:

* Does NOT command motion.  Does NOT arm/disarm motors.  Does NOT
  bypass the operator's E-stop button.
* Does NOT auto-resume the system post-test.  Operator must clean
  up manually (Ctrl-C the launch file; re-launch fresh).

Prereq: ``jugglebot_launch.py`` is running and the platform is held
at a low safe pose.  Confirmed by the operator at the pre-test gate.

Usage::

    python tests/hardware/th_t2b1_publisher_kill_test.py
    python tests/hardware/th_t2b1_publisher_kill_test.py --pid 12345
    python tests/hardware/th_t2b1_publisher_kill_test.py --log-path temp/logs/abc.log
    python tests/hardware/th_t2b1_publisher_kill_test.py --operator "Harrison Low"
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

# Add tests/hardware/ to path so we can import the helper module directly
# when invoked as a script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _th_test_common import (
    LogTailer, TestResult, find_latest_mpc_log, find_pid_by_pattern,
    get_repo_sha, safety_gate, wait_gate, write_artifacts,
)


# Documented bound from Plan 2 Phase 5: E-stop fires within 500 ms ± 25 ms
# of publisher pause.  At control_dt=0.025 the ESTOP threshold is
# 20 × 0.025 = 0.500 s.
EXPECTED_ESTOP_FIRE_WINDOW_MS = (475.0, 525.0)
# Also expected (informational, not gating): WARN @ 75 ms, HARD @ 125 ms.
EXPECTED_WARN_FIRE_WINDOW_MS = (60.0, 90.0)
EXPECTED_HARD_FIRE_WINDOW_MS = (110.0, 140.0)

# Candidate publisher process patterns to try in order
PUBLISHER_CANDIDATES = [
    'motor_guard',
    'can_node',
    'mpc_bridge_node',
]


def discover_publisher_pid(arg_pid: int | None) -> int:
    """Resolve the publisher PID — either from --pid, or via pgrep
    suggestions that the operator confirms interactively.
    """
    if arg_pid is not None:
        print(f"\nUsing --pid {arg_pid} (operator-supplied).")
        return arg_pid

    print("\nSearching for candidate encoder-publisher processes...")
    candidates: list[tuple[int, str, str]] = []  # (pid, cmd, source_pattern)
    for pattern in PUBLISHER_CANDIDATES:
        for pid, cmd in find_pid_by_pattern(pattern):
            candidates.append((pid, cmd, pattern))

    if not candidates:
        print("  No candidates found via pgrep.")
        print("  Find the publisher PID manually (e.g. `ros2 node list` +")
        print("  `ps aux | grep <node-name>`) and re-run with --pid <N>.")
        sys.exit(1)

    print(f"  Found {len(candidates)} candidate(s):\n")
    for idx, (pid, cmd, src) in enumerate(candidates, start=1):
        # Truncate long command lines for display
        short_cmd = cmd[:100] + ('...' if len(cmd) > 100 else '')
        print(f"    [{idx}] PID {pid} (matched '{src}'): {short_cmd}")
    print()
    print(f"  Type the number [1-{len(candidates)}] of the publisher to SIGSTOP,")
    print(f"  or 'a' to ABORT and re-run with explicit --pid:")
    try:
        response = input("> ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n\nABORTED.")
        sys.exit(1)
    if response == 'a':
        print("ABORTED — operator chose to specify --pid explicitly.")
        sys.exit(1)
    try:
        idx = int(response)
        if not (1 <= idx <= len(candidates)):
            raise ValueError
    except ValueError:
        print(f"\nABORTED — got '{response}', expected 1-{len(candidates)} or 'a'.")
        sys.exit(1)

    chosen_pid, chosen_cmd, _ = candidates[idx - 1]
    print(f"\nChosen: PID {chosen_pid}\n  cmd: {chosen_cmd}")
    safety_gate(
        f"FINAL CONFIRMATION: SIGSTOP will be sent to PID {chosen_pid}.\n"
        f"  This is the encoder publisher (e.g. motor_guard).  Are you sure?",
        required='yes',
    )
    return chosen_pid


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--pid', type=int, default=None,
                   help="Encoder-publisher PID to SIGSTOP.  If omitted, "
                        "the script discovers candidates via pgrep.")
    p.add_argument('--log-path', type=Path, default=None,
                   help="Path to the MPC log file to tail.  If omitted, "
                        "the newest *.log under temp/logs/ is used.")
    p.add_argument('--operator', type=str, default=os.environ.get('USER', 'unknown'),
                   help="Operator name for the artefact.")
    p.add_argument('--no-cont', action='store_true',
                   help="Skip SIGCONT after the test (for debugging).")
    args = p.parse_args()

    print("=" * 72)
    print("T-H-T2b-1 — encoder-publisher kill (Plan 2 Phase 5 hardware bringup)")
    print("=" * 72)
    print()
    print("Hazard profile: LOW.  CAN bus stays live; platform stays held.")
    print("Only the Jetson-side telemetry observation is affected.")
    print()
    print(f"Operator: {args.operator}")
    print(f"Repo SHA: {get_repo_sha()}")
    print()

    # --- Universal pre-test checklist (Plan 2 Testing Plan §Hardware tests) ---
    safety_gate(
        "Pre-test checklist:\n"
        "  1. Platform commanded to a LOW safe pose (close to STOW; not the\n"
        "     standard 0,0,170 mid-workspace pose).  Holding now.\n"
        "  2. Hand detached.  No ball.  Platform-only mode.\n"
        "  3. Operator hand on PHYSICAL E-STOP button RIGHT NOW.\n"
        "  4. Second person present in the workshop.\n"
        "  5. jugglebot_launch.py is running and stable.\n"
        "Are ALL of these true?",
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
    print(f"\nTailing MPC log: {log_path}")

    # --- Discover and confirm the publisher PID ---
    pid = discover_publisher_pid(args.pid)

    # --- Sanity check: the chosen PID actually exists ---
    try:
        os.kill(pid, 0)  # signal 0 = check existence
    except OSError as exc:
        print(f"\nABORTED — PID {pid} does not exist or is unreachable: {exc}")
        return 1

    # --- Final gate ---
    wait_gate(
        "READY.  About to:\n"
        f"  1. Start log tailer on {log_path.name}\n"
        f"  2. Send SIGSTOP to PID {pid}\n"
        "  3. Wait up to 5 s for the telemetry-stale ESTOP to fire\n"
        f"  4. Send SIGCONT to PID {pid}\n"
        "Press Enter to PROCEED."
    )

    # --- Start log tailer ---
    tailer = LogTailer(log_path)
    tailer.start()
    time.sleep(0.1)  # let the tailer settle on the log's tail

    # --- Inject the fault at t=0 ---
    t0_wall_clock = time.time()
    t0_iso = datetime.fromtimestamp(t0_wall_clock).isoformat(timespec='milliseconds')
    print(f"\n[t=0  @ {t0_iso}] Sending SIGSTOP to PID {pid}...")
    try:
        os.kill(pid, signal.SIGSTOP)
    except OSError as exc:
        tailer.stop()
        print(f"ABORTED — SIGSTOP failed: {exc}")
        return 1

    # --- Wait for ESTOP (with timeout) ---
    print("  Waiting for 'telem_stale_estop' pattern (up to 5 s)...")
    estop_hit = tailer.wait_for('telem_stale_estop', timeout_s=5.0)

    # --- Revive the publisher (regardless of test outcome) ---
    if not args.no_cont:
        time.sleep(0.1)  # let the cascade finish logging
        try:
            os.kill(pid, signal.SIGCONT)
            print(f"  Sent SIGCONT to PID {pid} (publisher revived).")
        except OSError as exc:
            print(f"  SIGCONT failed: {exc} — operator must revive PID manually.")

    time.sleep(0.5)  # let any trailing log lines flush
    tailer.stop()
    tailer.compute_offsets(t0_wall_clock)

    # --- Analyse the timing chain ---
    chain = {
        'fault_injection_sigstop': 0.0,
        'telem_aging_warn': None,
        'telem_stale_hard': None,
        'telem_stale_estop': None,
        'estop_sent': None,
    }
    for key in ('telem_aging_warn', 'telem_stale_hard',
                'telem_stale_estop', 'estop_sent'):
        hit = tailer.first_hit(key)
        if hit is not None:
            chain[key] = hit.offset_from_t0_ms

    expected_bounds = {
        'fault_injection_sigstop': (0.0, 0.0),
        'telem_aging_warn': EXPECTED_WARN_FIRE_WINDOW_MS,
        'telem_stale_hard': EXPECTED_HARD_FIRE_WINDOW_MS,
        'telem_stale_estop': EXPECTED_ESTOP_FIRE_WINDOW_MS,
        'estop_sent': None,  # informational
    }

    # --- Verdict ---
    anomalies: list[str] = []
    if chain['telem_stale_estop'] is None:
        verdict = 'FAIL'
        anomalies.append(
            'ESTOP pattern did not fire within the 5 s observation window — '
            'watchdog may be broken; investigate before T-H-T2a-1.')
    else:
        lo, hi = EXPECTED_ESTOP_FIRE_WINDOW_MS
        if lo <= chain['telem_stale_estop'] <= hi:
            verdict = 'PASS'
        else:
            verdict = 'FAIL'
            anomalies.append(
                f"ESTOP fire time {chain['telem_stale_estop']:.1f} ms outside "
                f"expected window [{lo}, {hi}] ms.")

    # Frozen-motor detector firing during this test would indicate a
    # collision between the WARN/HARD/ESTOP cascade and the frozen-motor
    # detector (separate cascade).  Flag if observed.
    fm_hit = tailer.first_hit('frozen_motor_warn')
    if fm_hit is not None:
        anomalies.append(
            f"frozen-motor warning fired at t+{fm_hit.offset_from_t0_ms:.1f} ms — "
            "unexpected for a publisher-kill test (telemetry should be missing, "
            "not frozen-fresh); investigate.")

    # --- Print summary ---
    print()
    print("=" * 72)
    print(f"VERDICT: {verdict}")
    print("=" * 72)
    print(f"  ESTOP fired at t+{chain['telem_stale_estop']} ms "
          f"(expected window: {EXPECTED_ESTOP_FIRE_WINDOW_MS[0]}–"
          f"{EXPECTED_ESTOP_FIRE_WINDOW_MS[1]} ms)")
    if chain['telem_aging_warn'] is not None:
        print(f"  WARN fired at  t+{chain['telem_aging_warn']:.1f} ms")
    if chain['telem_stale_hard'] is not None:
        print(f"  HARD fired at  t+{chain['telem_stale_hard']:.1f} ms")
    if anomalies:
        print()
        print("ANOMALIES:")
        for a in anomalies:
            print(f"  - {a}")

    # --- Operator post-test observations ---
    print()
    print("Operator observations (free-form; press Enter on empty line to end):")
    print("  Q1: Did the platform stay held throughout?  (yes/no/details)")
    print("  Q2: Did ODrives stay armed (no audible disarm clicks)?")
    print("  Q3: Any unexpected log output?")
    print("  Q4: Any other anomaly?")
    print()
    notes_lines: list[str] = []
    try:
        while True:
            line = input("note> ")
            if not line.strip():
                break
            notes_lines.append(line)
    except (EOFError, KeyboardInterrupt):
        pass
    notes = "\n".join(notes_lines)

    # --- Write artefacts ---
    result = TestResult(
        test_id='T-H-T2b-1',
        test_name='encoder-publisher kill (SIGSTOP)',
        date_iso=datetime.now().strftime('%Y-%m-%d'),
        repo_sha=get_repo_sha(),
        operator=args.operator,
        verdict=verdict,
        t0_wall_clock_iso=t0_iso,
        t0_description=f'SIGSTOP sent to PID {pid}',
        pass_criteria={
            'estop_fires_within_window_ms':
                f'{EXPECTED_ESTOP_FIRE_WINDOW_MS[0]}–'
                f'{EXPECTED_ESTOP_FIRE_WINDOW_MS[1]} ms after SIGSTOP',
            'platform_never_freewheels':
                'observed by operator; no auto-instrumentation in this test',
            'odrives_stay_armed':
                'observed by operator; no auto-instrumentation in this test',
        },
        timing_chain_ms=chain,
        expected_bounds_ms=expected_bounds,
        log_path=str(log_path),
        log_hits=[
            {
                'pattern_key': h.pattern_key,
                'wall_clock_s': h.wall_clock_s,
                'offset_from_t0_ms': h.offset_from_t0_ms,
                'log_line': h.log_line,
            } for h in tailer.hits
        ],
        operator_notes=notes,
        anomalies=anomalies,
    )
    json_path, md_path = write_artifacts(result)

    print()
    print(f"Artefacts written:")
    print(f"  JSON: {json_path}")
    print(f"  MD:   {md_path}")
    print()
    print("Next step: open the MD file, review the paste-ready logbook snippet,")
    print("and create the /investigate entry (suggested filename:")
    print(f"  logbook/{result.date_iso}-th-t2b-1-encoder-publisher-kill-bringup.md")
    print("on PASS, or with -fail suffix on FAIL).")
    print()
    print("If T-H-T2b-1 PASS: T-H-T2a-1 (CAN unplug) can be scheduled.")
    print("If T-H-T2b-1 FAIL: investigate the watchdog before T-H-T2a-1.")

    return 0 if verdict == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
