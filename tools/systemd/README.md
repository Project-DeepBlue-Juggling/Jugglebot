# User systemd units

Reference copies of the `systemctl --user` units that arm
[`tools/nightly_suite.sh`](../nightly_suite.sh). The **live** copies live at
`~/.config/systemd/user/` — systemd does not read this directory. These are here
so a reimaged or replacement Jetson can be re-armed from the repo, and so a
reviewer can see what is scheduled without shelling into the box.

`nightly` is only an honest test tier while something runs it. If the timer is
lost, every `@pytest.mark.nightly` test silently stops running while
`./run_tests.sh` keeps printing PASS. Treat re-arming as part of any Jetson
rebuild.

## Install / re-arm

```bash
install -Dm644 tools/systemd/jugglebot-nightly.service \
  ~/.config/systemd/user/jugglebot-nightly.service
install -Dm644 tools/systemd/jugglebot-nightly.timer \
  ~/.config/systemd/user/jugglebot-nightly.timer
systemctl --user daemon-reload
systemctl --user enable --now jugglebot-nightly.timer

# Linger must be on, or the user manager (and the timer) stops at logout:
loginctl show-user "$USER" -p Linger      # want Linger=yes
sudo loginctl enable-linger "$USER"       # if it is not
```

## Verify

```bash
systemctl --user list-timers jugglebot-nightly.timer --all
systemctl --user status jugglebot-nightly.service     # last run's exit
tools/nightly_ticker.sh check --who "<session>"        # CLAIMED|LOWERED|STALE|UNRAISED|NEVER — the once-per-day claim; read status only on CLAIMED
cat temp/reports/nightly/status                       # GREEN|RED|DEFERRED <counts> <date>
```

## Keeping these in sync

The install is a **copy**, not a symlink, so the two can drift. `systemctl
--user enable` records the unit by name and a symlinked unit file confuses its
alias handling, which is why this is not symlinked. If you edit a unit, edit it
here, re-run the install block above, and say so in the logbook entry.

Armed 2026-08-01 (`logbook/2026-08-01-nightly-tier-and-mpc-dormancy.md`).

## System unit: `jugglebot-gui.service` (GUI server + replay API)

Unlike the nightly units above, this is a **system** unit
(`/etc/systemd/system/`, `User=jetson`), serving the GUI on :8081 and the
`/api/replay/` rosbag-replay routes. The server itself is stdlib-only and runs
under `/usr/bin/python3`; it lists recordings and serves the `.mcap` bytes over
HTTP Range (the browser decodes them itself). Only the overview worker it spawns
on demand (niced, one at a time) needs the venv, via `--worker-python`; its
results land in `temp/replay_overview/` (gitignored, tens of kB each, `--overview-dir`).
The rosbags root is passed on the `ExecStart` line.

**Migration (Phase 4):** the converter cache is gone. After installing the new
unit, remove the old cache: `rm -rf temp/replay_cache/` (up to 10 GB).

```bash
sudo cp tools/systemd/jugglebot-gui.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart jugglebot-gui
systemctl status jugglebot-gui
```

**Do not restart while someone is using the GUI**: the restart drops every open
page's connection and kills a running overview pass (it is re-run on the next request). Edit the unit here, then re-run the block above;
the install is a copy, so the two can drift.

## Real-time budget for the solver worker (JB-95)

The CATCH solver worker asks for `SCHED_FIFO` priority 10 (fail-open: if the
kernel refuses it stays on CFS and says why in the READY log line). Two things
must be true or the request gets EPERM, and the second is easy to miss:

1. the `rtprio` rlimit of the launching process is at least 10
   (`99-jugglebot-rt.conf`: `jetson - rtprio 40`; 40 stays below the IRQ
   threads and watchdogd at FIFO 50, the worker asks for 10);
2. the process's cpu cgroup has an RT budget. This kernel has
   `CONFIG_RT_GROUP_SCHED=y` and `/user.slice` ships with `cpu.rt_runtime_us = 0`,
   which makes `sched_setscheduler` return EPERM even for root.
   `jugglebot-rt-budget.service` writes 950000 (the global net's value; never
   lower, a throttled group stalls until the next period) at every boot.
   `kernel.sched_rt_runtime_us` stays 950000 as the runaway net; never set it -1.

### Install (once, needs sudo, then reboot)

```bash
cd ~/Desktop/Jugglebot
sudo cp tools/systemd/99-jugglebot-rt.conf /etc/security/limits.d/
sudo cp tools/systemd/jugglebot-rt-budget.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now jugglebot-rt-budget.service
sudo reboot
```

The reboot is needed because the tmux server (and so every launch pane) is a
child of the lingering systemd user manager, which takes its rlimits from PAM
only when it starts. Restarting tmux alone re-inherits the stale limits. It
kills tmux, the GUI and the map services, so do it between Sittings.

### Check after the reboot

```bash
ulimit -r                                                        # in a FRESH tmux pane: 40
cat /sys/fs/cgroup/cpu,cpuacct/user.slice/cpu.rt_runtime_us      # 950000
grep -i rtprio /proc/$(pgrep -f 'tmux: server')/limits           # Max realtime priority 40 40
python3 tools/probes/rt_policy_probe.py                          # exit 0, "RESULT: FIFO 10 ok"
```

If the probe says `EPERM: cpu cgroup /user.slice rt_runtime_us=0`, the budget
unit did not land (ordering): run its `ExecStart` echo by hand with sudo and
check `systemctl status jugglebot-rt-budget`.

**Fallback if the tmux server still reads 0:** set the user manager's own
hard limit, independent of PAM.

```bash
sudo mkdir -p /etc/systemd/system/user@1000.service.d
printf '[Service]\nLimitRTPRIO=40\n' | sudo tee /etc/systemd/system/user@1000.service.d/rtprio.conf
sudo systemctl daemon-reload && sudo reboot
```

### Revert

```bash
sudo systemctl disable --now jugglebot-rt-budget.service
sudo rm /etc/systemd/system/jugglebot-rt-budget.service /etc/security/limits.d/99-jugglebot-rt.conf
sudo rm -rf /etc/systemd/system/user@1000.service.d/rtprio.conf   # only if the fallback was used
sudo systemctl daemon-reload && sudo reboot
```

Until that reboot the unit is already disabled and removed, but the running
slice keeps the budget it was given, so a worker that is FIFO now stays FIFO
until the next launch. To turn RT off without touching the system at all, start
the launch with the trajectory node's parameter at 0 (`trajectory_node`
declares `solver_rt_priority`, default 10):

```bash
ros2 launch jugglebot jugglebot_launch.py   # then, or in the launch file's trajectory_node parameters:
#   --ros-args -p solver_rt_priority:=0     (on a bare `ros2 run jugglebot trajectory_node`)
```

The worker then reports `sched=CFS (disabled)` in its READY line.
