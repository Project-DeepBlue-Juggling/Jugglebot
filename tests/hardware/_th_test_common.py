"""Shared helpers for Plan 2 Tier-2 hardware bringup scripts.

Used by:
* ``th_t2b1_publisher_kill_test.py`` — T-H-T2b-1 (encoder-publisher kill)
* ``th_t2a1_can_unplug_test.py``    — T-H-T2a-1 (CAN unplug cascade)

These scripts OBSERVE the platform; the only state-changing action they
perform is sending ``SIGSTOP``/``SIGCONT`` to a user-confirmed process
(T-H-T2b-1) or instructing the operator to unplug a CAN cable
(T-H-T2a-1).  They do NOT command motion, do NOT arm/disarm motors,
and do NOT bypass the operator's E-stop button.
"""

from __future__ import annotations

import json
import os
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable


# ---------------------------------------------------------------------------
# Log patterns from ``controller.hardware_plant`` (Plan 2 Phase 5 staleness
# cascade at hardware_plant.py:627-702).  Compiled once at module import so
# the tailer's hot path is a list of pre-compiled regexes.
# ---------------------------------------------------------------------------

LOG_PATTERNS: dict[str, re.Pattern] = {
    # WARN: telem_age > 3× control_dt (default 0.075 s at 40 Hz)
    'telem_aging_warn': re.compile(
        r'Telemetry aging \(([\d.]+)s > ([\d.]+)s\)'),
    # HARD: telem_age > 5× control_dt (default 0.125 s); zeros leg velocities
    'telem_stale_hard': re.compile(
        r'Telemetry stale \(([\d.]+)s > ([\d.]+)s\) . zeroing velocities'),
    # ESTOP: telem_age > 20× control_dt (default 0.500 s); fires e-stop
    'telem_stale_estop': re.compile(
        r'Telemetry stale \(([\d.]+)s > ([\d.]+)s\) . triggering e-stop'),
    # Confirmation: HardwarePlant.estop() completed
    'estop_sent': re.compile(
        r'HardwarePlant: sent E-STOP \(telemetry_stale\)'),
    # Optional: frozen-motor detector (separate cascade — should NOT fire
    # in these tests; capture if it does so the operator sees the anomaly).
    'frozen_motor_warn': re.compile(r'motor_pos frozen for \d+ consecutive'),
    # Optional: motor_guard E-stop confirmation, if the launch file's log
    # path is being tailed.
    'motor_guard_estop': re.compile(r'motor_guard.*[Ee][- ]?[Ss][Tt][Oo][Pp]'),
}


# ---------------------------------------------------------------------------
# Operator-confirmation gates
# ---------------------------------------------------------------------------

def safety_gate(prompt: str, required: str = "yes") -> None:
    """Force the operator to type a specific phrase, not just press Enter.

    Use for SAFETY-CRITICAL gates (pre-test setup checklist, "operator on
    physical E-stop?", final "ready to inject?").  Abort on any mismatch.

    Use ``wait_gate`` for pacing-only prompts where the operator pressing
    Enter is sufficient.
    """
    print(f"\n{prompt}")
    print(f"  Type '{required}' to confirm, anything else to ABORT:")
    try:
        response = input("> ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n\nABORTED — interrupted at safety gate.")
        sys.exit(1)
    if response != required.lower():
        print(f"\nABORTED — got '{response}', expected '{required}'.")
        sys.exit(1)


def wait_gate(prompt: str) -> None:
    """Block until the operator presses Enter.  Use for pacing only,
    NOT for safety-critical gates.

    Ctrl-C is honoured and aborts cleanly.
    """
    print(f"\n{prompt}")
    print("  (press Enter to continue, Ctrl-C to abort)")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("\n\nABORTED — interrupted at pacing gate.")
        sys.exit(1)


# ---------------------------------------------------------------------------
# MPC log discovery
# ---------------------------------------------------------------------------

def find_latest_mpc_log(repo_root: Path | None = None) -> Path | None:
    """Return the most-recently-modified ``temp/logs/*.log`` file.

    Per CLAUDE.md, MPC writes its companion ``.log`` files there.
    Returns None if no .log files exist (script will prompt operator).
    """
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[2]
    logs_dir = repo_root / 'temp' / 'logs'
    if not logs_dir.exists():
        return None
    candidates = sorted(
        logs_dir.glob('*.log'),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


# ---------------------------------------------------------------------------
# Background log tailer — records (pattern_key, wall_clock_time, line) for
# the first hit of each pattern.  Subsequent hits append to a per-pattern
# history list so the operator can see if a pattern fired multiple times.
# ---------------------------------------------------------------------------

@dataclass
class TailerHit:
    pattern_key: str
    wall_clock_s: float            # absolute time.time() at hit
    offset_from_t0_ms: float | None  # filled in post-hoc once t0 is known
    log_line: str


class LogTailer:
    """Tail an active log file in a background thread.

    Records hits against ``LOG_PATTERNS``.  Thread-safe append to a
    list of ``TailerHit`` instances.

    Usage::

        tailer = LogTailer(log_path)
        tailer.start()
        ...           # do the test
        tailer.stop()
        tailer.compute_offsets(t0=test_start_wall_clock_s)
        for hit in tailer.hits:
            print(hit)
    """

    def __init__(self, log_path: Path):
        self._log_path = Path(log_path)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.hits: list[TailerHit] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        # Open at end-of-file so we only see lines emitted after `start()`.
        self._file = open(self._log_path, 'r')
        self._file.seek(0, os.SEEK_END)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            line = self._file.readline()
            if not line:
                time.sleep(0.005)  # 200 Hz poll — bounded latency, low CPU
                continue
            now = time.time()
            for key, pattern in LOG_PATTERNS.items():
                if pattern.search(line):
                    with self._lock:
                        self.hits.append(TailerHit(
                            pattern_key=key,
                            wall_clock_s=now,
                            offset_from_t0_ms=None,
                            log_line=line.rstrip('\n'),
                        ))

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._file.close()
        except Exception:
            pass

    def compute_offsets(self, t0: float) -> None:
        with self._lock:
            for hit in self.hits:
                hit.offset_from_t0_ms = (hit.wall_clock_s - t0) * 1000.0

    def first_hit(self, pattern_key: str) -> TailerHit | None:
        with self._lock:
            for hit in self.hits:
                if hit.pattern_key == pattern_key:
                    return hit
        return None

    def wait_for(self, pattern_key: str, timeout_s: float) -> TailerHit | None:
        """Block (with timeout) until the named pattern fires."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hit = self.first_hit(pattern_key)
            if hit is not None:
                return hit
            time.sleep(0.01)
        return None


# ---------------------------------------------------------------------------
# Artifact writer — paired JSON (machine) + Markdown (human / paste-ready)
# ---------------------------------------------------------------------------

@dataclass
class TestResult:
    test_id: str                                  # 'T-H-T2b-1' / 'T-H-T2a-1'
    test_name: str
    date_iso: str
    repo_sha: str
    operator: str
    verdict: str                                  # 'PASS' / 'FAIL' / 'INDETERMINATE'
    t0_wall_clock_iso: str
    t0_description: str                           # 'SIGSTOP sent' / 'cable unplugged'
    pass_criteria: dict[str, str]                 # human-readable
    timing_chain_ms: dict[str, float | None]      # link → offset_from_t0_ms
    expected_bounds_ms: dict[str, tuple[float, float] | None]
    log_path: str
    log_hits: list[dict]                          # serialised TailerHit dicts
    operator_notes: str
    anomalies: list[str] = field(default_factory=list)


def write_artifacts(result: TestResult, output_dir: Path | None = None) -> tuple[Path, Path]:
    """Write paired JSON + Markdown artefacts to ``temp/reports/``.

    Returns ``(json_path, md_path)``.
    """
    if output_dir is None:
        repo_root = Path(__file__).resolve().parents[2]
        output_dir = repo_root / 'temp' / 'reports'
    output_dir.mkdir(parents=True, exist_ok=True)

    stamp = result.t0_wall_clock_iso.replace(':', '').replace('-', '').replace('T', '_').split('.')[0]
    base = f"{result.test_id.lower().replace('-', '_')}_{stamp}"
    json_path = output_dir / f"{base}.json"
    md_path = output_dir / f"{base}.md"

    payload = {
        'test_id': result.test_id,
        'test_name': result.test_name,
        'date_iso': result.date_iso,
        'repo_sha': result.repo_sha,
        'operator': result.operator,
        'verdict': result.verdict,
        't0_wall_clock_iso': result.t0_wall_clock_iso,
        't0_description': result.t0_description,
        'pass_criteria': result.pass_criteria,
        'timing_chain_ms': result.timing_chain_ms,
        'expected_bounds_ms': {
            k: list(v) if v is not None else None
            for k, v in result.expected_bounds_ms.items()
        },
        'log_path': result.log_path,
        'log_hits': result.log_hits,
        'operator_notes': result.operator_notes,
        'anomalies': result.anomalies,
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=False))

    md = _render_markdown(result)
    md_path.write_text(md)
    return json_path, md_path


def _render_markdown(result: TestResult) -> str:
    lines: list[str] = []
    lines.append(f"# {result.test_id} — {result.test_name}")
    lines.append("")
    lines.append(f"- **Date**: {result.date_iso}")
    lines.append(f"- **Repo SHA**: `{result.repo_sha}`")
    lines.append(f"- **Operator**: {result.operator}")
    lines.append(f"- **Verdict**: **{result.verdict}**")
    lines.append(f"- **t0 (zero-time)**: {result.t0_wall_clock_iso} — {result.t0_description}")
    lines.append("")

    lines.append("## Pass criteria")
    lines.append("")
    for k, v in result.pass_criteria.items():
        lines.append(f"- **{k}**: {v}")
    lines.append("")

    lines.append("## Timing chain")
    lines.append("")
    lines.append("| Event | Offset from t0 (ms) | Expected window (ms) | Within bound? |")
    lines.append("|-------|---------------------|----------------------|---------------|")
    for link, offset in result.timing_chain_ms.items():
        bounds = result.expected_bounds_ms.get(link)
        if offset is None:
            offset_str = "NOT OBSERVED"
            within = "n/a"
        else:
            offset_str = f"{offset:.1f}"
            if bounds is None:
                within = "—"
            else:
                lo, hi = bounds
                within = "yes" if lo <= offset <= hi else f"**NO** (outside [{lo}, {hi}])"
        bounds_str = (f"[{bounds[0]}, {bounds[1]}]" if bounds is not None else "—")
        lines.append(f"| {link} | {offset_str} | {bounds_str} | {within} |")
    lines.append("")

    if result.anomalies:
        lines.append("## Anomalies")
        lines.append("")
        for a in result.anomalies:
            lines.append(f"- {a}")
        lines.append("")

    lines.append("## Captured log hits")
    lines.append("")
    if not result.log_hits:
        lines.append("(none captured)")
    else:
        lines.append("```")
        for hit in result.log_hits:
            off = hit.get('offset_from_t0_ms')
            off_str = f"{off:+.1f} ms" if off is not None else " ?    "
            lines.append(f"[{off_str}] [{hit['pattern_key']}] {hit['log_line']}")
        lines.append("```")
    lines.append("")

    lines.append("## Operator notes")
    lines.append("")
    lines.append(result.operator_notes or "(none)")
    lines.append("")

    lines.append("## Paste-ready logbook snippet")
    lines.append("")
    lines.append("```markdown")
    lines.append(f"**{result.test_id} verdict: {result.verdict}**")
    lines.append("")
    lines.append(f"- Date: {result.date_iso} (SHA `{result.repo_sha}`)")
    lines.append(f"- t0 event: {result.t0_description}")
    for link, offset in result.timing_chain_ms.items():
        if offset is not None:
            lines.append(f"- {link}: t0 + {offset:.1f} ms")
    lines.append("```")
    return "\n".join(lines)


def get_repo_sha() -> str:
    """Read the current HEAD SHA via git.  Falls back to '<unknown>'."""
    import subprocess
    try:
        out = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=Path(__file__).resolve().parents[2],
            timeout=5,
        )
        return out.decode().strip()
    except Exception:
        return '<unknown>'


# ---------------------------------------------------------------------------
# Process discovery helpers (used by T-H-T2b-1)
# ---------------------------------------------------------------------------

def find_pid_by_pattern(pattern: str) -> list[tuple[int, str]]:
    """Return list of ``(pid, command_line)`` matching the substring.

    Uses ``pgrep -af <pattern>`` (returns full command line).  Empty list
    if no match or pgrep unavailable.
    """
    import subprocess
    try:
        out = subprocess.check_output(
            ['pgrep', '-af', pattern], timeout=5
        ).decode()
    except subprocess.CalledProcessError:
        return []  # no match
    except Exception:
        return []  # pgrep unavailable
    matches: list[tuple[int, str]] = []
    for line in out.strip().split('\n'):
        if not line.strip():
            continue
        try:
            pid_str, cmd = line.split(' ', 1)
            matches.append((int(pid_str), cmd))
        except ValueError:
            continue
    return matches
