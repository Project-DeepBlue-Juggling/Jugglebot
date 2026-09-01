#!/usr/bin/env python3
"""
profile_session.py - record Jetson system load during an MPC / robot session
and analyse the recording for hardware bottlenecks.

Subcommands:
    start [--py-spy-targets NAMES] [--candump [IFACE]]
                                   spawn recorders, return immediately
    stop  [--no-analyze]           kill recorders, finalize, run analysis
    record [--py-spy-targets NAMES] [--candump [IFACE]] [--no-analyze]
                                   start + block until Ctrl-C + stop
    analyze [SESSION_DIR]          (re-)analyse a session

Captures (in temp/probes/profile_<timestamp>/):
    - preflight.txt          nvpmodel / governors / max freqs /
                             jetson_clocks / uname / free / ps snapshot
    - tegrastats.log         200 ms CPU per-core / EMC / GR3D / thermal /
                             power
    - pidstat.log            1 s per-process %CPU / mem-fault / IO / cswitch
    - mpstat.log             1 s per-core utilisation
    - pyspy_<name>.svg        flame graph per py-spy target name
    - candump.log            full CAN trace if --candump was passed
    - state.json             recorder PIDs / start time / preflight metadata
    - report.md              bottleneck analysis (written by `analyze`)

Example multi-tool session:
    python tools/probes/profile_session.py record \\
        --py-spy-targets trajectory_node,teensy_bridge_node \\
        --candump can0

Bottleneck detection:
    * CPU saturation per core (sustained > 95 %)
    * DVFS throttling (actual freq < 95 % of max freq)
    * Thermal warnings (any zone > 80 / 90 C)
    * EMC memory-bandwidth saturation (> 70 %)
    * RAM / swap pressure
    * Top processes by CPU / context-switch rate / IO wait
    * Highlights the leg-path processes (trajectory_node, motor_guard) specifically
    * Python hotspots from py-spy flame graph if collected

Run from anywhere - paths resolve relative to the repo root.

Motivating logbook entry: logbook/2026-05-22-mpc-compute-bound-jetson-
profiling.md (built during that investigation; see also the earlier
logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md and the 2026-05-20
warm-start / fallback entries). Reusable harness per the `tools/probes/`
convention in CLAUDE.md.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean, median
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
PROBES_ROOT = REPO_ROOT / "temp" / "probes"
LATEST_LINK = PROBES_ROOT / "latest_profile_session"

TEGRASTATS_INTERVAL_MS = 200
PIDSTAT_INTERVAL_S = 1
MPSTAT_INTERVAL_S = 1

# Thresholds for bottleneck flags.
CPU_SAT_PCT = 95.0          # core %CPU considered saturated
CPU_SAT_MIN_SAMPLES = 10    # ~2 s at 200 ms tegrastats interval
DVFS_THROTTLE_RATIO = 0.95  # actual_freq / max_freq below this = throttled
DVFS_THROTTLE_FRAC = 0.10   # >10 % of samples throttled = flag
EMC_SAT_PCT = 70.0          # avg EMC %
THERMAL_CAUTION_C = 80.0
THERMAL_WARNING_C = 90.0
RAM_FREE_MIN_MB = 200       # <200 MB free = memory pressure
CSWITCH_HIGH_PER_S = 5000   # voluntary context switches per s
NVCSWITCH_HIGH_PER_S = 10000  # involuntary context switches per s

# "run_mpc.py"/"mpc_bridge" retained as hints so this tool still recognises a
# pre-2026-09-01 capture; neither process exists on this branch (tag mpc-final).
MPC_PROC_HINTS = ("run_mpc.py", "motor_guard", "mpc_bridge", "can_node",
                  "_node", "ros2", "rclpy", "ros-args")


# ---------------------------------------------------------------------------
# Session bookkeeping
# ---------------------------------------------------------------------------

def _now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def _resolve_session(arg: str | None) -> Path:
    """Resolve a session directory argument (or the latest-session symlink)."""
    if arg:
        p = Path(arg).expanduser()
        if not p.is_absolute():
            p = (Path.cwd() / p).resolve()
        if not p.is_dir():
            sys.exit(f"profile_session: session dir not found: {p}")
        return p
    if not LATEST_LINK.exists():
        sys.exit(
            "profile_session: no session specified and no latest-session "
            f"symlink at {LATEST_LINK} - pass a session dir explicitly."
        )
    return LATEST_LINK.resolve()


def _update_latest_link(target: Path) -> None:
    PROBES_ROOT.mkdir(parents=True, exist_ok=True)
    if LATEST_LINK.is_symlink() or LATEST_LINK.exists():
        LATEST_LINK.unlink()
    LATEST_LINK.symlink_to(target)


# ---------------------------------------------------------------------------
# Preflight snapshot
# ---------------------------------------------------------------------------

def _run_quiet(cmd: list[str]) -> str:
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=5
        )
        return out.stdout + ("\n[stderr] " + out.stderr if out.stderr else "")
    except FileNotFoundError:
        return f"[not installed: {cmd[0]}]\n"
    except subprocess.TimeoutExpired:
        return f"[timeout: {' '.join(cmd)}]\n"


def _read_glob(pattern: str) -> str:
    paths = sorted(Path("/").glob(pattern.lstrip("/")))
    if not paths:
        return f"[no files matching {pattern}]\n"
    lines = []
    for p in paths:
        try:
            lines.append(f"{p}: {p.read_text().strip()}")
        except OSError as e:
            lines.append(f"{p}: [{e}]")
    return "\n".join(lines) + "\n"


def _preflight() -> tuple[str, dict]:
    """Capture pre-flight system state. Returns (text_dump, metadata)."""
    parts: list[str] = []
    meta: dict = {}

    parts.append("=== uname -a ===\n" + _run_quiet(["uname", "-a"]))
    parts.append("=== nvpmodel -q ===\n" + _run_quiet(["nvpmodel", "-q"]))
    parts.append(
        "=== /etc/nv_tegra_release ===\n"
        + (Path("/etc/nv_tegra_release").read_text()
           if Path("/etc/nv_tegra_release").exists()
           else "[file not present]\n")
    )
    parts.append(
        "=== cpu scaling_governor ===\n"
        + _read_glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")
    )
    max_freq_text = _read_glob(
        "/sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_max_freq"
    )
    parts.append("=== cpu cpuinfo_max_freq (kHz) ===\n" + max_freq_text)
    meta["cpu_max_freq_khz"] = _parse_max_freqs(max_freq_text)

    parts.append(
        "=== cpu cpuinfo_cur_freq (kHz) ===\n"
        + _read_glob("/sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_cur_freq")
    )
    parts.append("=== free -h ===\n" + _run_quiet(["free", "-h"]))
    parts.append(
        "=== jetson_clocks --show ===\n"
        + _run_quiet(["jetson_clocks", "--show"])
    )
    parts.append(
        "=== ps -eo pid,pri,ni,psr,pcpu,pmem,comm (filtered) ===\n"
        + _ps_snapshot()
    )
    parts.append(
        "=== /proc/sys/kernel/yama/ptrace_scope ===\n"
        + (Path("/proc/sys/kernel/yama/ptrace_scope").read_text()
           if Path("/proc/sys/kernel/yama/ptrace_scope").exists()
           else "[not present]\n")
    )

    return "\n".join(parts), meta


def _parse_max_freqs(text: str) -> dict[int, int]:
    out: dict[int, int] = {}
    for line in text.splitlines():
        m = re.search(r"cpu(\d+)/cpufreq/cpuinfo_max_freq: (\d+)", line)
        if m:
            out[int(m.group(1))] = int(m.group(2))
    return out


def _ps_snapshot() -> str:
    """ps snapshot, with MPC / robot processes flagged."""
    out = _run_quiet(
        ["ps", "-eo", "pid,pri,ni,psr,pcpu,pmem,comm,args", "--sort=-pcpu"]
    )
    lines = out.splitlines()
    keep = [lines[0]] if lines else []
    for ln in lines[1:]:
        if any(hint in ln for hint in MPC_PROC_HINTS) or any(
            h in ln for h in ("ros2", "rclpy", "python", "tegrastats",
                              "mpstat", "pidstat", "py-spy")
        ):
            keep.append(ln)
    return "\n".join(keep) + "\n"


# ---------------------------------------------------------------------------
# Recorder spawning / stopping
# ---------------------------------------------------------------------------

@dataclass
class Recorder:
    name: str
    pid: int
    logfile: str
    cmd: list[str]


def _spawn(cmd: list[str], stdout: Path | None) -> subprocess.Popen:
    """Launch a recorder detached from the parent's process group."""
    if stdout is not None:
        stdout.parent.mkdir(parents=True, exist_ok=True)
        f = stdout.open("w")
    else:
        f = subprocess.DEVNULL
    return subprocess.Popen(
        cmd,
        stdout=f if stdout else subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _start_recorders(session: Path,
                     py_spy_targets: list[str],
                     candump_iface: str | None) -> list[Recorder]:
    recs: list[Recorder] = []

    # tegrastats - writes directly to logfile via its own --logfile arg.
    teg_log = session / "tegrastats.log"
    teg = subprocess.Popen(
        ["tegrastats",
         "--interval", str(TEGRASTATS_INTERVAL_MS),
         "--logfile", str(teg_log)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    recs.append(Recorder("tegrastats", teg.pid, str(teg_log), teg.args))

    # pidstat - all processes, 1 s, headerless single-line format.
    if shutil.which("pidstat"):
        pid_log = session / "pidstat.log"
        p = _spawn(
            ["pidstat", "-h", "-u", "-r", "-d", "-w",
             str(PIDSTAT_INTERVAL_S)],
            pid_log,
        )
        recs.append(Recorder("pidstat", p.pid, str(pid_log), p.args))
    else:
        print("profile_session: pidstat not installed - skipping "
              "per-process recording. `sudo apt install sysstat` to enable.",
              file=sys.stderr)

    # mpstat - per-core utilisation, 1 s.
    if shutil.which("mpstat"):
        mp_log = session / "mpstat.log"
        m = _spawn(
            ["mpstat", "-P", "ALL", str(MPSTAT_INTERVAL_S)],
            mp_log,
        )
        recs.append(Recorder("mpstat", m.pid, str(mp_log), m.args))
    else:
        print("profile_session: mpstat not installed - skipping per-core "
              "recording.", file=sys.stderr)

    # ps snapshots - dump the full process table every 5 s. Lets the
    # analyzer resolve generic `python` / `python3` pidstat entries to their
    # actual command lines (which process / which script). Captured on a
    # loop because the ROS2 stack usually starts AFTER the profiler.
    ps_log = session / "ps_snapshots.log"
    ps_proc = _spawn(
        ["sh", "-c",
         "while true; do printf '=== %s ===\\n' \"$(date +%s)\"; "
         "ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu --no-headers; "
         "sleep 5; done"],
        ps_log,
    )
    recs.append(Recorder("ps-snapshots", ps_proc.pid, str(ps_log),
                         ps_proc.args))

    # py-spy attach. The target processes (trajectory_node, teensy_bridge_node, ...) almost
    # always start AFTER the profiler, so we cannot attach at start time.
    # Instead spawn one waiter per target: the `_pyspy_wait` subcommand polls
    # for the named process and exec()s py-spy onto itself once it appears
    # (the PID is stable across exec, so `stop` can still SIGINT it).
    if py_spy_targets:
        if not shutil.which("py-spy"):
            print("profile_session: --py-spy-targets requested but py-spy is "
                  "not on PATH - skipping.", file=sys.stderr)
        else:
            if _ptrace_scope_blocked():
                print("profile_session: WARNING - yama ptrace_scope is "
                      "non-zero; py-spy attach will FAIL until you run "
                      "`echo 0 | sudo tee /proc/sys/kernel/yama/"
                      "ptrace_scope`. Arming waiters anyway in case you fix "
                      "it before the targets start.", file=sys.stderr)
            script = str(Path(__file__).resolve())
            for name in py_spy_targets:
                fname = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_") or "target"
                svg = session / f"pyspy_{fname}.svg"
                w = _spawn(
                    [sys.executable, script, "_pyspy_wait",
                     name, str(svg), "180"],
                    None,
                )
                recs.append(Recorder(
                    f"py-spy-waiter[{name}]", w.pid, str(svg), w.args,
                ))

    # candump - record every CAN frame on the requested interface.
    if candump_iface:
        if not shutil.which("candump"):
            print(f"profile_session: --candump requested but candump is not "
                  "on PATH - skipping. `sudo apt install can-utils` to "
                  "install.", file=sys.stderr)
        else:
            cd_log = session / "candump.log"
            c = _spawn(
                ["candump", "-t", "a", candump_iface],
                cd_log,
            )
            recs.append(Recorder(
                f"candump[{candump_iface}]", c.pid, str(cd_log), c.args,
            ))

    return recs


def _stop_recorders(recs: Iterable[Recorder]) -> None:
    """Cleanly stop each recorder. SIGINT first (py-spy + pidstat respect it
    and flush), then a shared 4 s grace window for all of them to exit, then
    SIGTERM whichever are still alive."""
    recs = list(recs)
    for r in recs:
        try:
            os.kill(r.pid, signal.SIGINT)
        except ProcessLookupError:
            continue
    # Poll all recorders against one shared deadline so every recorder gets
    # the full grace window (not just whatever the earlier ones left over).
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        if all(not _pid_alive(r.pid) for r in recs):
            break
        time.sleep(0.1)
    for r in recs:
        if _pid_alive(r.pid):
            try:
                os.kill(r.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass


def _pid_alive(pid: int) -> bool:
    """True if `pid` is still a live process."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # exists but not ours to signal


def _find_pids_by_name(name: str) -> list[int]:
    """All PIDs whose command line matches `name` (substring, via `pgrep -fa`).

    Skips the harness's own process and the pgrep invocation. Useful both for
    auto-detecting the leg-path processes and for attaching py-spy to other named
    processes (can_node, motor_guard, etc) - many of which run as `python`
    in `ps`, so we match the full command line.
    """
    out = subprocess.run(
        ["pgrep", "-fa", name],
        capture_output=True, text=True,
    )
    pids: list[int] = []
    for line in out.stdout.splitlines():
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if "profile_session" in line or "pgrep" in line:
            continue
        pids.append(pid)
    return pids


def _ptrace_scope_blocked() -> bool:
    """True if /proc/sys/kernel/yama/ptrace_scope is non-zero (blocks py-spy
    attach without sudo). Returns False when the file doesn't exist (no Yama)."""
    p = Path("/proc/sys/kernel/yama/ptrace_scope")
    if not p.exists():
        return False
    try:
        return int(p.read_text().strip()) != 0
    except (OSError, ValueError):
        return False


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------

def cmd_start(args: argparse.Namespace) -> int:
    PROBES_ROOT.mkdir(parents=True, exist_ok=True)
    session = PROBES_ROOT / f"profile_{_now_stamp()}"
    session.mkdir()
    print(f"profile_session: session = {session}")

    pre_text, pre_meta = _preflight()
    (session / "preflight.txt").write_text(pre_text)

    py_spy_targets: list[str] = []
    if args.py_spy_targets:
        py_spy_targets = [
            t.strip() for t in args.py_spy_targets.split(",") if t.strip()
        ]

    candump_iface: str | None = args.candump

    recs = _start_recorders(session, py_spy_targets, candump_iface)

    state = {
        "session_dir": str(session),
        "started_at": time.time(),
        "started_at_iso": datetime.now().isoformat(timespec="seconds"),
        "tegrastats_interval_ms": TEGRASTATS_INTERVAL_MS,
        "pidstat_interval_s": PIDSTAT_INTERVAL_S,
        "mpstat_interval_s": MPSTAT_INTERVAL_S,
        "preflight": pre_meta,
        "recorders": [
            {"name": r.name, "pid": r.pid, "logfile": r.logfile,
             "cmd": list(r.cmd)}
            for r in recs
        ],
        "py_spy_targets": py_spy_targets,
        "candump_iface": candump_iface,
    }
    (session / "state.json").write_text(json.dumps(state, indent=2))
    _update_latest_link(session)

    print("profile_session: recording started:")
    for r in recs:
        print(f"  - {r.name} (pid {r.pid}) -> {r.logfile}")
    script_rel = Path(__file__).resolve().relative_to(REPO_ROOT)
    print(f"profile_session: run `python {script_rel} stop` to finalise.")
    return 0


def cmd_stop(args: argparse.Namespace) -> int:
    session = _resolve_session(args.session)
    state_path = session / "state.json"
    if not state_path.exists():
        sys.exit(f"profile_session: no state.json in {session}")
    state = json.loads(state_path.read_text())

    recs = [
        Recorder(r["name"], r["pid"], r["logfile"], r["cmd"])
        for r in state["recorders"]
    ]
    print(f"profile_session: stopping {len(recs)} recorder(s) in {session}")
    _stop_recorders(recs)
    state["stopped_at"] = time.time()
    state["stopped_at_iso"] = datetime.now().isoformat(timespec="seconds")
    state["duration_s"] = state["stopped_at"] - state["started_at"]
    state_path.write_text(json.dumps(state, indent=2))
    print(f"profile_session: recording stopped "
          f"({state['duration_s']:.1f} s).")

    if args.no_analyze:
        print("profile_session: --no-analyze set, skipping analysis.")
        return 0
    return _analyze(session, print_summary=True)


def cmd_record(args: argparse.Namespace) -> int:
    rc = cmd_start(args)
    if rc != 0:
        return rc
    print("profile_session: recording... press Ctrl-C to stop.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()  # tidy newline after ^C
    stop_args = argparse.Namespace(session=None, no_analyze=args.no_analyze)
    return cmd_stop(stop_args)


def cmd_analyze(args: argparse.Namespace) -> int:
    session = _resolve_session(args.session)
    return _analyze(session, print_summary=True)


def cmd_pyspy_wait(args: argparse.Namespace) -> int:
    """Internal: poll for a named process, then exec py-spy onto ourselves.

    Spawned by `start` / `record` as a detached waiter. exec() keeps the PID
    stable so `stop` can SIGINT whatever is running (waiter or py-spy).
    """
    deadline = time.monotonic() + args.wait_s
    while time.monotonic() < deadline:
        pids = _find_pids_by_name(args.name)
        # Exclude other py-spy / waiter processes that mention the same name.
        pids = [p for p in pids if p != os.getpid()]
        if pids:
            target = pids[0]
            os.execvp("py-spy", [
                "py-spy", "record",
                "-o", args.svg,
                "-p", str(target),
                "--duration", "86400",
                "--rate", "100",
            ])
        time.sleep(1.0)
    print(f"profile_session: _pyspy_wait gave up on '{args.name}' after "
          f"{args.wait_s}s (process never appeared).", file=sys.stderr)
    return 0


# ---------------------------------------------------------------------------
# tegrastats parser
# ---------------------------------------------------------------------------

RE_CPU_BLOCK = re.compile(r"CPU \[([^\]]+)\]")
# EMC / GR3D may print as "EMC_FREQ 0%" (no @MHz) on some L4T releases.
RE_EMC = re.compile(r"EMC_FREQ (\d+)%(?:@(\d+))?")
RE_GR3D = re.compile(r"GR3D_FREQ (\d+)%(?:@(\d+))?")
RE_RAM = re.compile(r"RAM (\d+)/(\d+)MB")
RE_SWAP = re.compile(r"SWAP (\d+)/(\d+)MB")
RE_TEMP = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)@([\-\d.]+)C")
RE_POWER = re.compile(r"(VDD_[A-Z_]+) (\d+)/(\d+)")
# Sensors report -256C when offline; ignore those readings.
SENTINEL_TEMP_C = -200.0


@dataclass
class TegraSample:
    cpu_pct: list[float]      # per-core %, NaN if offline
    cpu_freq_mhz: list[float]  # per-core MHz, NaN if offline
    emc_pct: float | None
    emc_mhz: float | None
    gr3d_pct: float | None
    gr3d_mhz: float | None
    ram_used_mb: float | None
    ram_total_mb: float | None
    swap_used_mb: float | None
    swap_total_mb: float | None
    temps: dict[str, float] = field(default_factory=dict)
    power_now_mw: dict[str, float] = field(default_factory=dict)
    power_avg_mw: dict[str, float] = field(default_factory=dict)


def _parse_tegrastats_line(line: str) -> TegraSample | None:
    cpu_pct: list[float] = []
    cpu_freq: list[float] = []
    m = RE_CPU_BLOCK.search(line)
    if not m:
        return None
    for tok in m.group(1).split(","):
        tok = tok.strip()
        if tok == "off":
            cpu_pct.append(float("nan"))
            cpu_freq.append(float("nan"))
            continue
        cm = re.match(r"(\d+)%@(\d+)", tok)
        if not cm:
            cpu_pct.append(float("nan"))
            cpu_freq.append(float("nan"))
            continue
        cpu_pct.append(float(cm.group(1)))
        cpu_freq.append(float(cm.group(2)))

    emc = RE_EMC.search(line)
    gr3d = RE_GR3D.search(line)
    ram = RE_RAM.search(line)
    swap = RE_SWAP.search(line)

    temps = {
        k: float(v) for k, v in RE_TEMP.findall(line)
        if float(v) > SENTINEL_TEMP_C
    }
    power_now: dict[str, float] = {}
    power_avg: dict[str, float] = {}
    for rail, now, avg in RE_POWER.findall(line):
        power_now[rail] = float(now)
        power_avg[rail] = float(avg)

    return TegraSample(
        cpu_pct=cpu_pct,
        cpu_freq_mhz=cpu_freq,
        emc_pct=float(emc.group(1)) if emc else None,
        emc_mhz=float(emc.group(2)) if emc and emc.group(2) else None,
        gr3d_pct=float(gr3d.group(1)) if gr3d else None,
        gr3d_mhz=float(gr3d.group(2)) if gr3d and gr3d.group(2) else None,
        ram_used_mb=float(ram.group(1)) if ram else None,
        ram_total_mb=float(ram.group(2)) if ram else None,
        swap_used_mb=float(swap.group(1)) if swap else None,
        swap_total_mb=float(swap.group(2)) if swap else None,
        temps=temps,
        power_now_mw=power_now,
        power_avg_mw=power_avg,
    )


def _read_tegrastats(path: Path) -> list[TegraSample]:
    if not path.exists():
        return []
    samples: list[TegraSample] = []
    for line in path.read_text(errors="replace").splitlines():
        s = _parse_tegrastats_line(line)
        if s is not None:
            samples.append(s)
    return samples


# ---------------------------------------------------------------------------
# pidstat parser
# ---------------------------------------------------------------------------

@dataclass
class PidStatRow:
    time_str: str        # 'HH:MM:SS' as printed - not numeric
    uid: int
    pid: int
    pct_cpu: float
    rss_kb: float
    cswch_s: float
    nvcswch_s: float
    iodelay: float
    command: str


def _parse_pidstat(path: Path) -> list[PidStatRow]:
    """pidstat -h -u -r -d -w 1 produces a single-line-per-sample format
    where columns vary by sysstat version. We pick fields by column header,
    discovered from the `#` header lines emitted at startup. The Time
    column is HH:MM:SS, not numeric - keep it as a string."""
    if not path.exists():
        return []
    rows: list[PidStatRow] = []
    headers: list[str] | None = None
    for line in path.read_text(errors="replace").splitlines():
        line = line.rstrip()
        if not line:
            continue
        if line.startswith("#"):
            # Header line: '#      Time   UID       PID    %usr ...'
            headers = line.lstrip("#").split()
            continue
        if headers is None:
            continue
        first = line.split(None, 1)[0]
        # Data lines start with HH:MM:SS (or unix epoch). The 'Linux 5.x'
        # banner starts with 'Linux' (alpha) - drop those.
        if first and first[0].isalpha():
            continue
        parts = line.split(None, len(headers) - 1)
        if len(parts) != len(headers):
            continue
        row = dict(zip(headers, parts))
        try:
            rows.append(PidStatRow(
                time_str=row.get("Time", ""),
                uid=int(row.get("UID", -1)),
                pid=int(row.get("PID", -1)),
                pct_cpu=float(row.get("%CPU", "nan")),
                rss_kb=float(row.get("RSS", "nan")),
                cswch_s=float(row.get("cswch/s", "nan")),
                nvcswch_s=float(row.get("nvcswch/s", "nan")),
                iodelay=float(row.get("iodelay", "nan")),
                command=row.get("Command", ""),
            ))
        except (TypeError, ValueError):
            continue
    return rows


# ---------------------------------------------------------------------------
# mpstat parser (per-core utilisation summary)
# ---------------------------------------------------------------------------

@dataclass
class MpStatRow:
    cpu: str         # 'all' or core number as string
    pct_usr: float
    pct_sys: float
    pct_iowait: float
    pct_idle: float


def _parse_mpstat(path: Path) -> list[MpStatRow]:
    if not path.exists():
        return []
    rows: list[MpStatRow] = []
    headers: list[str] | None = None
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("Linux") or line.startswith("Average"):
            continue
        toks = line.split()
        # Header lines contain '%usr' as a token.
        if "%usr" in toks:
            headers = toks
            continue
        if headers is None:
            continue
        # Sample lines start with a time stamp (HH:MM:SS or epoch); CPU column
        # is the one labelled 'CPU' in the header.
        try:
            row = dict(zip(headers, toks))
            rows.append(MpStatRow(
                cpu=row.get("CPU", ""),
                pct_usr=float(row.get("%usr", "nan")),
                pct_sys=float(row.get("%sys", "nan")),
                pct_iowait=float(row.get("%iowait", "nan")),
                pct_idle=float(row.get("%idle", "nan")),
            ))
        except (ValueError, KeyError):
            continue
    return rows


# ---------------------------------------------------------------------------
# candump parser (CAN traffic)
# ---------------------------------------------------------------------------

# `candump -t a can0` prints lines like:
#   ' (1747757432.123456)  can0  0A9   [8]  40 05 A3 3B 00 00 00 00'
# Tokens: 0=(timestamp), 1=iface, 2=arbitration_id_hex, 3=[dlc], 4..=bytes.
# Standard IDs are 3 hex chars; extended IDs are up to 8.
RE_CANDUMP_TS = re.compile(r"\(([\d.]+)\)")
RE_HEX_ID = re.compile(r"^[0-9A-Fa-f]{3,8}$")


def _parse_candump(path: Path) -> dict | None:
    """Summarise a candump.log: total frames, duration, frame rate, top IDs.

    Returns None if the file is missing or empty.  Duration is computed from
    candump's own absolute timestamps so the rate matches what was actually
    recorded (no clock-skew worries vs tegrastats).
    """
    if not path.exists() or path.stat().st_size == 0:
        return None
    counts: dict[str, int] = {}
    total = 0
    first_ts: float | None = None
    last_ts: float | None = None
    for line in path.read_text(errors="replace").splitlines():
        toks = line.split()
        if len(toks) < 3:
            continue
        m_ts = RE_CANDUMP_TS.match(toks[0])
        if not m_ts:
            continue
        if not RE_HEX_ID.match(toks[2]):
            continue
        try:
            ts = float(m_ts.group(1))
        except ValueError:
            continue
        if first_ts is None:
            first_ts = ts
        last_ts = ts
        aid = toks[2].upper().lstrip("0") or "0"
        counts[aid] = counts.get(aid, 0) + 1
        total += 1
    if total == 0:
        return None
    duration = (last_ts - first_ts) if (first_ts and last_ts) else 0.0
    rate = total / duration if duration > 0 else 0.0
    top = sorted(counts.items(), key=lambda kv: -kv[1])[:10]
    top_with_pct = [(aid, c, c / total) for aid, c in top]
    return {
        "total_frames": total,
        "duration_s": duration,
        "rate_per_s": rate,
        "top_ids": top_with_pct,
        "unique_ids": len(counts),
    }


# ---------------------------------------------------------------------------
# ps-snapshot parser (PID -> command line)
# ---------------------------------------------------------------------------

def _parse_ps_snapshots(path: Path) -> dict[int, str]:
    """Map PID -> full command line from ps_snapshots.log.

    The file is '=== <epoch> ===' delimited blocks of
    `pid pcpu pmem comm args...` lines (ps --no-headers). Last sighting of
    each PID wins, so a process that changed state mid-session resolves to
    its latest command line.
    """
    if not path.exists():
        return {}
    pid_args: dict[int, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        line = line.rstrip()
        if not line or line.startswith("==="):
            continue
        # pid pcpu pmem comm args... - split into exactly 5 fields.
        parts = line.split(None, 4)
        if len(parts) < 4:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        # args column is parts[4]; fall back to comm (parts[3]) if absent.
        pid_args[pid] = parts[4] if len(parts) == 5 else parts[3]
    return pid_args


# ---------------------------------------------------------------------------
# Analysis + report rendering
# ---------------------------------------------------------------------------

def _pct(values: list[float], p: float) -> float:
    vals = [v for v in values if v == v]  # drop NaN
    if not vals:
        return float("nan")
    vals = sorted(vals)
    k = max(0, min(len(vals) - 1, int(round(p / 100.0 * (len(vals) - 1)))))
    return vals[k]


def _sustained_runs(values: list[float], threshold: float) -> list[int]:
    """Return run lengths (sample counts) where values stayed above threshold."""
    runs: list[int] = []
    cur = 0
    for v in values:
        if v == v and v >= threshold:
            cur += 1
        else:
            if cur:
                runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    return runs


@dataclass
class Bottleneck:
    severity: str  # "warning" | "caution" | "info"
    title: str
    detail: str


def _analyze(session: Path, print_summary: bool) -> int:
    state_path = session / "state.json"
    if not state_path.exists():
        sys.exit(f"profile_session: no state.json in {session}")
    state = json.loads(state_path.read_text())

    teg = _read_tegrastats(session / "tegrastats.log")
    pid_rows = _parse_pidstat(session / "pidstat.log")
    mp_rows = _parse_mpstat(session / "mpstat.log")
    pre_meta = state.get("preflight", {})
    # JSON round-trips integer dict keys as strings; coerce back so int
    # lookups (`max_freqs.get(i)` where i is a core index) work.
    raw_freqs = pre_meta.get("cpu_max_freq_khz", {})
    max_freqs: dict[int, int] = {}
    for k, v in raw_freqs.items():
        try:
            max_freqs[int(k)] = int(v)
        except (TypeError, ValueError):
            continue

    bottlenecks: list[Bottleneck] = []
    sections: list[str] = []

    # --- Header ---
    duration = state.get("duration_s")
    sections.append(f"# Profile session report\n")
    sections.append(
        f"- session: `{session}`\n"
        f"- started: {state.get('started_at_iso', '?')}\n"
        f"- stopped: {state.get('stopped_at_iso', '?')}\n"
        f"- duration: "
        f"{f'{duration:.1f} s' if duration else '?'}\n"
        f"- tegrastats samples: {len(teg)}\n"
        f"- pidstat rows: {len(pid_rows)}\n"
        f"- mpstat rows: {len(mp_rows)}\n"
    )

    # --- Pre-flight summary ---
    sections.append("## Pre-flight state\n")
    pre_text = (session / "preflight.txt").read_text(errors="replace")
    governors = set(re.findall(r"scaling_governor: (\w+)", pre_text))
    nvpm_match = re.search(r"NV Power Mode:\s+(\S+)", pre_text)
    sections.append(
        f"- nvpmodel: `{nvpm_match.group(1) if nvpm_match else 'unknown'}`\n"
        f"- governors in use: "
        f"{', '.join(sorted(governors)) if governors else 'unknown'}\n"
        f"- CPU max freq (kHz): {dict(sorted(max_freqs.items())) if max_freqs else 'unknown'}\n"
    )
    # DVFS detection from ground truth, not the governor string.
    # `jetson_clocks` pins frequency by clamping scaling_min_freq ==
    # scaling_max_freq; the governor stays 'schedutil'. So the governor name
    # alone is NOT a reliable DVFS indicator - the observed per-core
    # frequency is. If every core stayed within 2 % of its max all session,
    # DVFS had no effect (jetson_clocks is doing its job).
    freq_pinned = False
    if teg and max_freqs:
        freq_pinned = True
        for i, max_khz in max_freqs.items():
            max_mhz = max_khz / 1000.0
            observed = [
                s.cpu_freq_mhz[i] for s in teg
                if i < len(s.cpu_freq_mhz)
                and s.cpu_freq_mhz[i] == s.cpu_freq_mhz[i]
            ]
            if observed and min(observed) < 0.98 * max_mhz:
                freq_pinned = False
                break
    dvfs_governor = "schedutil" in governors or "ondemand" in governors
    if dvfs_governor and freq_pinned:
        sections.append(
            "- DVFS: governor is schedutil, but observed frequencies stayed "
            "pinned at max for the whole session - `jetson_clocks` is active "
            "and DVFS had no effect. Good.\n"
        )
    elif dvfs_governor:
        bottlenecks.append(Bottleneck(
            "caution",
            "CPU governor is DVFS-enabled and frequencies varied",
            f"governors: {sorted(governors)}, and observed CPU frequency "
            "dropped below 98 % of max during the session. Run `sudo "
            "jetson_clocks` to pin frequencies. DVFS dips can cause "
            "Maximum_CpuTime_Exceeded events in the MPC.",
        ))

    # --- tegrastats analysis ---
    if teg:
        num_cores = max(len(s.cpu_pct) for s in teg)
        per_core_pct = [
            [s.cpu_pct[i] for s in teg if i < len(s.cpu_pct)]
            for i in range(num_cores)
        ]
        per_core_freq = [
            [s.cpu_freq_mhz[i] for s in teg if i < len(s.cpu_freq_mhz)]
            for i in range(num_cores)
        ]

        sections.append("## CPU per-core (tegrastats)\n")
        sections.append(
            "| core | mean % | p95 % | max % | mean MHz | min MHz | "
            "samples online |\n"
            "|---:|---:|---:|---:|---:|---:|---:|\n"
        )
        for i in range(num_cores):
            online_pct = [v for v in per_core_pct[i] if v == v]
            online_freq = [v for v in per_core_freq[i] if v == v]
            if not online_pct:
                sections.append(
                    f"| {i} | - | - | - | - | - | 0 (offline) |\n"
                )
                continue
            sections.append(
                f"| {i} | {mean(online_pct):.1f} | "
                f"{_pct(online_pct, 95):.1f} | {max(online_pct):.1f} | "
                f"{mean(online_freq):.0f} | {min(online_freq):.0f} | "
                f"{len(online_pct)} |\n"
            )

            # Bottleneck: sustained CPU saturation.
            runs = _sustained_runs(per_core_pct[i], CPU_SAT_PCT)
            longest = max(runs) if runs else 0
            if longest >= CPU_SAT_MIN_SAMPLES:
                dur_s = longest * TEGRASTATS_INTERVAL_MS / 1000.0
                bottlenecks.append(Bottleneck(
                    "warning",
                    f"Core {i} CPU-saturated",
                    f"sustained >{CPU_SAT_PCT:.0f} % for {dur_s:.1f} s "
                    f"(longest run of {longest} samples). Cross-reference the "
                    "'Top processes' table to find what was running on this "
                    "core during the burst. If it's a single process (e.g. "
                    "the MPC solver), only higher per-core clock will help - "
                    "more cores doesn't speed up serial work. If it's many "
                    "small processes, look for redundant ROS2 nodes / "
                    "publishers / loggers.",
                ))

            # Bottleneck: DVFS throttling.
            max_khz = max_freqs.get(i)
            if max_khz and online_freq:
                max_mhz = max_khz / 1000.0
                throttled = sum(
                    1 for f in online_freq if f < DVFS_THROTTLE_RATIO * max_mhz
                )
                frac = throttled / len(online_freq)
                if frac > DVFS_THROTTLE_FRAC:
                    bottlenecks.append(Bottleneck(
                        "warning" if frac > 0.5 else "caution",
                        f"Core {i} DVFS-throttled",
                        f"{frac*100:.0f} % of samples ran at <"
                        f"{DVFS_THROTTLE_RATIO*100:.0f} % of max freq "
                        f"({max_mhz:.0f} MHz). Either the governor is letting "
                        "the core scale down under intermittent load, or "
                        "thermal/power capping is engaging - check the "
                        "Thermal section.",
                    ))

        # EMC / GR3D / RAM / temps / power
        emc_vals = [s.emc_pct for s in teg if s.emc_pct is not None]
        gr3d_vals = [s.gr3d_pct for s in teg if s.gr3d_pct is not None]
        sections.append("## EMC / GPU\n")
        if emc_vals:
            sections.append(
                f"- EMC: mean {mean(emc_vals):.1f} %, p95 "
                f"{_pct(emc_vals, 95):.1f} %, max {max(emc_vals):.1f} %\n"
            )
            if mean(emc_vals) > EMC_SAT_PCT:
                bottlenecks.append(Bottleneck(
                    "warning",
                    "EMC memory bandwidth saturated",
                    f"mean EMC {mean(emc_vals):.1f} % (> {EMC_SAT_PCT:.0f} % "
                    "threshold). Sustained memory traffic is the bottleneck; "
                    "look at large allocations / image processing / numpy "
                    "copies.",
                ))
        if gr3d_vals:
            sections.append(
                f"- GPU (GR3D): mean {mean(gr3d_vals):.1f} %, "
                f"max {max(gr3d_vals):.1f} %\n"
            )

        # RAM / swap
        if teg[0].ram_total_mb:
            ram_used = [s.ram_used_mb for s in teg if s.ram_used_mb is not None]
            ram_total = teg[-1].ram_total_mb or 1.0
            min_free = ram_total - max(ram_used) if ram_used else 0
            sections.append(
                f"\n## Memory\n"
                f"- RAM: peak used {max(ram_used):.0f} / {ram_total:.0f} MB "
                f"(min free {min_free:.0f} MB)\n"
            )
            if min_free < RAM_FREE_MIN_MB:
                bottlenecks.append(Bottleneck(
                    "warning",
                    "RAM nearly exhausted",
                    f"min free RAM {min_free:.0f} MB (< {RAM_FREE_MIN_MB} MB "
                    "threshold). Swap activity, OOM kills, or major page "
                    "faults are likely.",
                ))
            swap_used = [
                s.swap_used_mb for s in teg if s.swap_used_mb is not None
            ]
            if swap_used:
                swap_baseline = swap_used[0]
                swap_peak = max(swap_used)
                swap_delta = swap_peak - swap_baseline
                if swap_delta > 1.0:
                    # Real paging activity during the session.
                    sections.append(
                        f"- SWAP: {swap_baseline:.0f} -> peak {swap_peak:.0f} "
                        f"MB (grew by {swap_delta:.0f} MB during session)\n"
                    )
                    bottlenecks.append(Bottleneck(
                        "warning",
                        "Active swap paging during session",
                        f"swap grew by {swap_delta:.0f} MB ({swap_baseline:.0f}"
                        f" -> {swap_peak:.0f} MB). Paging in a 40 Hz control "
                        "loop introduces unbounded page-fault latency. Check "
                        "for a memory leak or trim the working set.",
                    ))
                elif swap_peak > 0:
                    # Preexisting allocation, flat during session - not a
                    # bottleneck. Mention for context, no warning.
                    sections.append(
                        f"- SWAP: {swap_peak:.0f} MB allocated (flat - "
                        "preexisting, no active paging during session)\n"
                    )

        # Temperatures
        all_temp_keys = {k for s in teg for k in s.temps}
        if all_temp_keys:
            sections.append("\n## Thermal\n")
            sections.append("| zone | mean C | max C |\n|---|---:|---:|\n")
            for k in sorted(all_temp_keys):
                vals = [s.temps[k] for s in teg if k in s.temps]
                if not vals:
                    continue
                sections.append(
                    f"| {k} | {mean(vals):.1f} | {max(vals):.1f} |\n"
                )
                if max(vals) >= THERMAL_WARNING_C:
                    bottlenecks.append(Bottleneck(
                        "warning",
                        f"Thermal zone {k} hit {max(vals):.1f} C",
                        f">= {THERMAL_WARNING_C:.0f} C. Hardware throttling "
                        "trip points are typically only a few C above this; "
                        "DVFS dips and Maximum_CpuTime_Exceeded events "
                        "correlate with thermal events. Check airflow / "
                        "ambient.",
                    ))
                elif max(vals) >= THERMAL_CAUTION_C:
                    bottlenecks.append(Bottleneck(
                        "caution",
                        f"Thermal zone {k} reached {max(vals):.1f} C",
                        f">= {THERMAL_CAUTION_C:.0f} C - approaching warning "
                        "threshold but not throttling yet.",
                    ))

        # Power rails
        rails = sorted({k for s in teg for k in s.power_now_mw})
        if rails:
            sections.append("\n## Power rails (mW)\n")
            sections.append("| rail | mean now | peak now |\n|---|---:|---:|\n")
            for rail in rails:
                vals = [s.power_now_mw[rail] for s in teg if rail in s.power_now_mw]
                if not vals:
                    continue
                sections.append(
                    f"| {rail} | {mean(vals):.0f} | {max(vals):.0f} |\n"
                )

    # --- mpstat per-core summary ---
    if mp_rows:
        sections.append("\n## Per-core utilisation (mpstat)\n")
        sections.append(
            "| cpu | mean %busy | p95 %busy | mean %iowait |\n"
            "|---|---:|---:|---:|\n"
        )
        cores = sorted(
            {r.cpu for r in mp_rows},
            key=lambda c: (-1 if c == "all" else int(c)) if c.isdigit() or c == "all" else 999,
        )
        for c in cores:
            rs = [r for r in mp_rows if r.cpu == c]
            if not rs:
                continue
            busy = [100.0 - r.pct_idle for r in rs if r.pct_idle == r.pct_idle]
            iow = [r.pct_iowait for r in rs if r.pct_iowait == r.pct_iowait]
            if not busy:
                continue
            sections.append(
                f"| {c} | {mean(busy):.1f} | {_pct(busy, 95):.1f} | "
                f"{mean(iow):.1f} |\n"
            )
            if iow and mean(iow) > 5.0:
                bottlenecks.append(Bottleneck(
                    "caution",
                    f"Core {c} has sustained IO wait",
                    f"mean %iowait {mean(iow):.1f} % - something on this core "
                    "is blocking on disk / network IO.",
                ))

    # --- pidstat top processes ---
    if pid_rows:
        # Aggregate by PID + command.
        agg: dict[tuple[int, str], list[PidStatRow]] = {}
        for r in pid_rows:
            agg.setdefault((r.pid, r.command), []).append(r)
        per_proc = []
        for (pid, cmd), rs in agg.items():
            cpu_vals = [r.pct_cpu for r in rs if r.pct_cpu == r.pct_cpu]
            csw = [r.cswch_s for r in rs if r.cswch_s == r.cswch_s]
            nvcsw = [r.nvcswch_s for r in rs if r.nvcswch_s == r.nvcswch_s]
            iod = [r.iodelay for r in rs if r.iodelay == r.iodelay]
            if not cpu_vals:
                continue
            per_proc.append({
                "pid": pid, "cmd": cmd,
                "mean_cpu": mean(cpu_vals),
                "max_cpu": max(cpu_vals),
                "mean_cswch": mean(csw) if csw else 0.0,
                "max_cswch": max(csw) if csw else 0.0,
                "mean_nvcswch": mean(nvcsw) if nvcsw else 0.0,
                "mean_iodelay": mean(iod) if iod else 0.0,
            })

        per_proc.sort(key=lambda d: d["mean_cpu"], reverse=True)
        sections.append("\n## Top processes by CPU (pidstat)\n")
        sections.append(
            "| pid | command | mean %CPU | max %CPU | mean cswch/s | "
            "mean nvcswch/s | mean iodelay |\n"
            "|---:|---|---:|---:|---:|---:|---:|\n"
        )
        for p in per_proc[:15]:
            sections.append(
                f"| {p['pid']} | `{p['cmd']}` | {p['mean_cpu']:.1f} | "
                f"{p['max_cpu']:.1f} | {p['mean_cswch']:.0f} | "
                f"{p['mean_nvcswch']:.0f} | {p['mean_iodelay']:.2f} |\n"
            )

        # Robot-process callouts.
        sections.append("\n### Robot processes of interest\n")
        flagged = [
            p for p in per_proc
            if any(h in p["cmd"] for h in MPC_PROC_HINTS)
        ]
        if flagged:
            sections.append(
                "| pid | command | mean %CPU | max %CPU | mean nvcswch/s |\n"
                "|---:|---|---:|---:|---:|\n"
            )
            for p in flagged:
                sections.append(
                    f"| {p['pid']} | `{p['cmd']}` | {p['mean_cpu']:.1f} | "
                    f"{p['max_cpu']:.1f} | {p['mean_nvcswch']:.0f} |\n"
                )
        else:
            sections.append(
                "_No trajectory_node / motor_guard / ROS2 processes detected "
                "during recording. Was the session actually running?_\n"
            )

        # Resolve the top processes to their full command lines so generic
        # `python` / `python3` pidstat entries can be identified.
        ps_map = _parse_ps_snapshots(session / "ps_snapshots.log")
        if ps_map:
            sections.append("\n### Process command lines (from ps)\n")
            sections.append("| pid | command | full command line |\n"
                            "|---:|---|---|\n")
            for p in per_proc[:15]:
                args = ps_map.get(p["pid"], "(not captured by ps)")
                if len(args) > 130:
                    args = args[:127] + "..."
                sections.append(
                    f"| {p['pid']} | `{p['cmd']}` | `{args}` |\n"
                )
        else:
            sections.append(
                "\n_No ps_snapshots.log found - re-record to get full "
                "command-line identification of the `python` processes._\n"
            )

        # Bottleneck: context-switch storms.
        for p in per_proc[:30]:
            if p["mean_nvcswch"] > NVCSWITCH_HIGH_PER_S:
                bottlenecks.append(Bottleneck(
                    "caution",
                    f"PID {p['pid']} preempted heavily",
                    f"`{p['cmd']}` averaged {p['mean_nvcswch']:.0f} involuntary "
                    "context switches/s - kernel is interrupting it often, "
                    "suggesting CPU contention with a same-priority process.",
                ))
            elif p["mean_cswch"] > CSWITCH_HIGH_PER_S:
                bottlenecks.append(Bottleneck(
                    "info",
                    f"PID {p['pid']} switches voluntarily often",
                    f"`{p['cmd']}` averaged {p['mean_cswch']:.0f} voluntary "
                    "context switches/s - usually fine, but if this is the "
                    "MPC process consider whether IPC / blocking IO is the "
                    "real bottleneck.",
                ))

    # --- candump (CAN traffic) ---
    candump_log = session / "candump.log"
    if candump_log.exists():
        stats = _parse_candump(candump_log)
        if stats:
            sections.append("\n## CAN traffic (candump)\n")
            sections.append(
                f"- total frames: {stats['total_frames']}\n"
                f"- duration covered by candump: "
                f"{stats['duration_s']:.1f} s\n"
                f"- mean rate: {stats['rate_per_s']:.0f} frames/s\n"
            )
            if stats["top_ids"]:
                sections.append(
                    "\n| arbitration ID | frames | share % | rate/s |\n"
                    "|---|---:|---:|---:|\n"
                )
                for aid, count, pct in stats["top_ids"]:
                    rate = count / stats["duration_s"] if stats["duration_s"] > 0 else 0.0
                    sections.append(
                        f"| `{aid}` | {count} | {pct*100:.1f} | "
                        f"{rate:.0f} |\n"
                    )
            # Cross-reference: if can_node was the top CPU consumer AND CAN
            # traffic is high, the cost likely scales with frame rate (per-
            # message dispatch). If traffic is low but can_node CPU is high,
            # the cost is timer-machinery overhead, not per-frame work.
            if pid_rows and stats["rate_per_s"] > 0:
                can_node_proc = next(
                    (p for p in per_proc if "can_node" in p["cmd"]), None
                )
                if can_node_proc:
                    per_frame_us = (
                        can_node_proc["mean_cpu"] / 100.0 * 1_000_000
                        / stats["rate_per_s"]
                    )
                    sections.append(
                        f"\n_can_node averaged {can_node_proc['mean_cpu']:.1f}"
                        f" % CPU against {stats['rate_per_s']:.0f} frames/s = "
                        f"~{per_frame_us:.0f} us of CPU work per CAN frame. "
                        "Compare to the node's known timer-callback rate "
                        "(sum of its ROS2 timer frequencies x ~30-100 us "
                        "Python dispatch each) to judge whether per-frame "
                        "dispatch or timer machinery is the dominant cost._\n"
                    )

    # --- py-spy ---
    pyspy_svgs = sorted(session.glob("pyspy_*.svg"))
    if pyspy_svgs:
        sections.append("\n## py-spy flame graphs\n")
        sections.append(
            "Open in a browser (Firefox / Chrome) - they're interactive "
            "SVGs.  Width = fraction of samples spent in that call.\n\n"
        )
        for svg in pyspy_svgs:
            sections.append(f"- `{svg.name}`\n")

    # --- Bottlenecks summary (at top of report, computed last) ---
    summary_parts = ["## Bottlenecks detected\n"]
    if not bottlenecks:
        summary_parts.append(
            "_No bottlenecks crossed the configured thresholds. "
            "(That doesn't mean none exist - check the per-section tables "
            "for subtler signals.)_\n"
        )
    else:
        order = {"warning": 0, "caution": 1, "info": 2}
        bottlenecks.sort(key=lambda b: order.get(b.severity, 9))
        for b in bottlenecks:
            badge = {
                "warning": "[WARNING]",
                "caution": "[CAUTION]",
                "info":    "[INFO]   ",
            }.get(b.severity, b.severity)
            summary_parts.append(f"- **{badge}** {b.title}\n  - {b.detail}\n")

    # Compose final report: header, bottlenecks summary, then detailed sections.
    report = sections[0] + sections[1] + "\n" + "".join(summary_parts) + "\n" + \
        "".join(sections[2:])
    (session / "report.md").write_text(report)
    print(f"profile_session: report written to {session / 'report.md'}")

    if print_summary:
        print()
        print("=" * 70)
        print(f"BOTTLENECK SUMMARY  ({session.name})")
        print("=" * 70)
        if not bottlenecks:
            print("No bottlenecks above threshold. See full report for detail.")
        else:
            for b in bottlenecks:
                badge = {
                    "warning": "[WARNING]",
                    "caution": "[CAUTION]",
                    "info":    "[INFO]   ",
                }.get(b.severity, b.severity)
                print(f"{badge} {b.title}")
                print(f"         {b.detail}")
                print()
        print("=" * 70)
        print(f"Full report: {session / 'report.md'}")
    return 0


# ---------------------------------------------------------------------------
# argparse wiring
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Profile Jetson hardware load during an MPC session.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    def _add_capture_args(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--py-spy-targets", default=None, metavar="NAME[,NAME...]",
            help="comma-separated process name patterns to attach py-spy to "
                 "(matched against full cmdline via pgrep -f). Each target "
                 "name becomes its own pyspy_<name>.svg.",
        )
        p.add_argument(
            "--candump", nargs="?", const="can0", default=None, metavar="IFACE",
            help="record candump on IFACE (default 'can0' when flag is "
                 "bare). Captures every CAN frame so the analyzer can report "
                 "frame rate / top arbitration IDs and cross-reference "
                 "against can_node CPU%%.",
        )

    start = sub.add_parser("start", help="spawn recorders and return")
    _add_capture_args(start)
    start.set_defaults(func=cmd_start)

    stop = sub.add_parser("stop", help="stop recorders and auto-analyze")
    stop.add_argument("session", nargs="?", default=None,
                      help="session dir (default: latest)")
    stop.add_argument("--no-analyze", action="store_true",
                      help="skip analysis after stopping")
    stop.set_defaults(func=cmd_stop)

    rec = sub.add_parser(
        "record", help="start, wait for Ctrl-C, then stop + analyze")
    _add_capture_args(rec)
    rec.add_argument("--no-analyze", action="store_true")
    rec.set_defaults(func=cmd_record)

    an = sub.add_parser("analyze", help="(re-)analyze a recorded session")
    an.add_argument("session", nargs="?", default=None,
                    help="session dir (default: latest)")
    an.set_defaults(func=cmd_analyze)

    # Internal: py-spy waiter, spawned by start/record. Not for direct use.
    pw = sub.add_parser("_pyspy_wait", help=argparse.SUPPRESS)
    pw.add_argument("name")
    pw.add_argument("svg")
    pw.add_argument("wait_s", type=float)
    pw.set_defaults(func=cmd_pyspy_wait)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
