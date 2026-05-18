#!/usr/bin/env python3
"""Re-score an existing T-H-T2b/2a artefact JSON against the corrected
watchdog contract (telem_age at fire vs threshold), WITHOUT re-running
the hardware test.

Use this when an earlier run captured valid cascade log_hits but was
scored by the old wall-clock-from-fault-injection criterion (which
includes ZMQ/OS buffer drain and produces a false FAIL).  The captured
ESTOP log line states telem_age verbatim, so the watchdog contract can
be evaluated forensically from the recorded data.

Usage::

    python tests/hardware/rescore_artifact.py temp/reports/t_h_t2b_1_<ts>.json

Writes a corrected paired JSON + MD via the standard artefact writer,
preserving the original run's t0 / operator / log_hits and stamping
``rescored_from`` into operator_notes for provenance.  The original
artefact is left untouched.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _th_test_common import (
    TestResult, evaluate_estop_contract, write_artifacts,
)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    src = Path(sys.argv[1])
    if not src.exists():
        print(f"ABORTED — {src} does not exist.")
        return 1

    data = json.loads(src.read_text())
    hits = list(data.get('log_hits', []))
    estop_line = next(
        (h['log_line'] for h in hits
         if h.get('pattern_key') == 'telem_stale_estop'), "")

    # Fallback: the live tailer may have missed the cascade (harness
    # bug #2 — exit-flush race on T-H-T2a-1).  The cascade is still in
    # the MPC log file on disk.  Scan it deterministically — this is
    # exactly the forensic recovery the LogTailer.stop() rescan now
    # does automatically for future runs.
    log_recovery_note = ""
    if not estop_line:
        log_path = Path(data.get('log_path', ''))
        if log_path.exists():
            from _th_test_common import LOG_PATTERNS
            estop_re = LOG_PATTERNS['telem_stale_estop']
            text = log_path.read_text(errors='replace')
            for line in text.splitlines():
                if estop_re.search(line):
                    estop_line = line.strip()
                    log_recovery_note = (
                        f"  (recovered by scanning {log_path.name} — the "
                        f"live tailer missed it; harness bug #2)")
                    # also recover the WARN line for the record
                    warn_re = LOG_PATTERNS['telem_aging_warn']
                    for wl in text.splitlines():
                        if warn_re.search(wl):
                            hits.append({'pattern_key': 'telem_aging_warn',
                                         'wall_clock_s': 0.0,
                                         'offset_from_t0_ms': None,
                                         'log_line': wl.strip()})
                            break
                    hits.append({'pattern_key': 'telem_stale_estop',
                                 'wall_clock_s': 0.0,
                                 'offset_from_t0_ms': None,
                                 'log_line': estop_line})
                    break

    if not estop_line:
        print(f"ABORTED — {src} has no captured 'telem_stale_estop' log "
              f"hit AND none found in its log file.  The watchdog ESTOP "
              f"was never observed; it cannot be re-scored to PASS.  "
              f"Re-run the hardware test with correct logging.")
        return 1
    if log_recovery_note:
        print(f"ESTOP line recovered from log file:\n{log_recovery_note}")

    verdict, contract_detail, contract_anoms = evaluate_estop_contract(
        estop_line)

    orig_notes = data.get('operator_notes', '') or ''
    prov = (f"RE-SCORED from {src.name} (original verdict: "
            f"{data.get('verdict')}).  The original criterion measured "
            f"wall-clock from fault-injection, which includes ZMQ/OS "
            f"buffer drain (a fault-injection artefact, not watchdog "
            f"latency).  This re-score evaluates the watchdog's real "
            f"contract (telem_age at fire vs threshold) from the same "
            f"captured ESTOP log line — no hardware re-run.")

    # Preserve original anomalies that are still relevant (drop the
    # stale wall-clock-window complaint), prepend the new contract anoms.
    kept = [a for a in data.get('anomalies', [])
            if 'outside expected window' not in a
            and 'outside the expected window' not in a
            and 'ESTOP fire time' not in a]

    result = TestResult(
        test_id=data['test_id'],
        test_name=data['test_name'],
        date_iso=data['date_iso'],
        repo_sha=data['repo_sha'],
        operator=data['operator'],
        verdict=verdict,
        t0_wall_clock_iso=data['t0_wall_clock_iso'],
        t0_description=data['t0_description'],
        pass_criteria={
            **data.get('pass_criteria', {}),
            'authoritative_basis':
                'telem_age at ESTOP fire vs documented threshold '
                '(see Watchdog contract evaluation section)',
        },
        timing_chain_ms=data.get('timing_chain_ms', {}),
        expected_bounds_ms=data.get('expected_bounds_ms', {}),
        log_path=data.get('log_path', ''),
        log_hits=hits,
        operator_notes=f"{prov}\n\n--- original operator notes ---\n"
                       f"{orig_notes}",
        anomalies=contract_anoms + kept,
        contract_detail=contract_detail,
    )
    json_path, md_path = write_artifacts(result, name_suffix="_rescored")
    print(f"Re-scored verdict: {verdict}")
    print(f"  {contract_detail}")
    print()
    print(f"Corrected artefacts written (original LEFT UNTOUCHED):")
    print(f"  JSON: {json_path}")
    print(f"  MD:   {md_path}")
    print()
    print(f"Original preserved: {src}")
    return 0 if verdict == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
