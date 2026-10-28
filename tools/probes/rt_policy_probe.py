"""rt_policy_probe.py - can this process become SCHED_FIFO? (JB-95)

Why it exists: the solver worker asks for SCHED_FIFO 10 and fails open to CFS.
On this box (kernel 5.10-tegra, CONFIG_RT_GROUP_SCHED=y) two independent things
refuse it with the same EPERM: the rtprio rlimit (0 by default) and the cpu
cgroup's cpu.rt_runtime_us (0 for /user.slice by default). This probe tells
them apart, with no sudo and no jugglebot imports (works on the bare system
python3 too), so the install receipt in tools/systemd/README.md is one command.

Run (from a fresh tmux pane after the reboot; the shell must have the new limits):
    python3 tools/probes/rt_policy_probe.py [--prio 10]

Prints RLIMIT_RTPRIO, the process's cpu cgroup and its cpu.rt_runtime_us, the
global sched_rt_runtime_us / period, then tries SCHED_FIFO on itself and
restores SCHED_OTHER. Exit 0 when FIFO succeeded, 1 otherwise.

PASS: "RESULT: FIFO 10 ok" and exit 0.
Today (before the recipe): RLIMIT (0, 0), user.slice rt_runtime_us 0, and
"RESULT: CFS EPERM: RLIMIT_RTPRIO=0 < 10", exit 1. Once the rlimit is raised but
the budget is not, the reason reads "EPERM: cpu cgroup /user.slice rt_runtime_us=0".
"""
from __future__ import annotations

import argparse
import errno
import os
import resource
import sys


def cpu_cgroup_rt_runtime():
    """(cgroup path, cpu.rt_runtime_us) of this process, or (None, None)."""
    try:
        with open("/proc/self/cgroup") as f:
            for line in f:
                _id, ctrls, path = line.rstrip("\n").split(":", 2)
                if "cpu" in ctrls.split(","):
                    fn = "/sys/fs/cgroup/cpu,cpuacct%s/cpu.rt_runtime_us" % path.rstrip("/")
                    with open(fn) as g:
                        return path, int(g.read())
    except (OSError, ValueError):
        pass
    return None, None


def _read(fn):
    try:
        with open(fn) as f:
            return f.read().strip()
    except OSError as e:
        return "unreadable (%s)" % e.strerror


def try_fifo(prio):
    """Attempt SCHED_FIFO `prio` on this process; return (ok, reason)."""
    reset = getattr(os, "SCHED_RESET_ON_FORK", 0)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO | reset, os.sched_param(prio))
    except OSError as e:
        soft, _hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
        if e.errno == errno.EPERM and soft < prio:
            return False, "EPERM: RLIMIT_RTPRIO=%d < %d" % (soft, prio)
        if e.errno == errno.EPERM:
            path, rt = cpu_cgroup_rt_runtime()
            return False, "EPERM: cpu cgroup %s rt_runtime_us=%s" % (path, rt)
        return False, "%s: %s" % (errno.errorcode.get(e.errno, e.errno), e)
    try:
        # Keep RESET_ON_FORK: an unprivileged task cannot clear it, and a
        # plain SCHED_OTHER is refused EPERM (Jetson, 2026-10-28).
        os.sched_setscheduler(0, os.SCHED_OTHER | getattr(os, 'SCHED_RESET_ON_FORK', 0),
                              os.sched_param(0))
    except OSError as e:
        print("warning: could not restore SCHED_OTHER: %s" % e)
    return True, "ok"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--prio", type=int, default=10)
    args = ap.parse_args(argv)
    print("python", sys.version.split()[0])
    print("RLIMIT_RTPRIO", resource.getrlimit(resource.RLIMIT_RTPRIO))
    path, rt = cpu_cgroup_rt_runtime()
    print("cpu cgroup", path, "cpu.rt_runtime_us", rt)
    print("sched_rt_runtime_us", _read("/proc/sys/kernel/sched_rt_runtime_us"),
          "sched_rt_period_us", _read("/proc/sys/kernel/sched_rt_period_us"))
    ok, why = try_fifo(args.prio)
    print("RESULT: %s %s" % ("FIFO %d" % args.prio if ok else "CFS", why))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
