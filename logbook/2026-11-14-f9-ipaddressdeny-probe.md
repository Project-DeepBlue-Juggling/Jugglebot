---
title: "F9 probe: systemd IPAddressDeny= on a user scope does not block on this kernel"
type: investigation
date: 2026-11-14
status: complete
phase: "Night Shift close-out C15"
subsystem: night-shift
files_changed: []
---

# F9 probe: cgroup-level address deny is not feasible here

## What and why

Finding F9 asked whether `systemd-run --user --scope -p IPAddressDeny=<addr>` could replace the spelling-based denies in `tools/night_shift_settings.json` as the guard that keeps the Night Shift from sending to the can-bridge. Pre-registered rule: ADOPT only if 3/3 sends (direct, `setsid` child, re-exec) fail EPERM for the denied address, the control send is unblocked, and cgroup is unified v2. Otherwise CLOSE F9 and keep `tools/night_shift_nonet.sh`. No production code changed. The bridge address was never used; test addresses were 192.0.2.1 (TEST-NET-1) and the LAN gateway 192.168.20.1 (this box: eth1 192.168.20.34/24).

## Environment

- `stat -fc %T /sys/fs/cgroup` -> `tmpfs` (hybrid, not unified v2); `systemctl --user --version` -> systemd 245 (245.4-4ubuntu3.24), `default-hierarchy=hybrid`, kernel 5.10.104-tegra.
- `systemd-run` accepted the property silently (no warning, no prompt, no root). `systemctl --user show c15probe.scope -p IPAddressDeny` -> `IPAddressDeny=192.0.2.1/32`; `IPIngressFilterPath=` and `IPEgressFilterPath=` empty. Journal: no entries.

## Results (probe `/tmp/probe_c15_send.py`, UDP `sendto(addr, 9)`, run 2026-11-14 as `systemd-run --user --scope -p IPAddressDeny=<addr> python3 probe.py <addr> <mode>`)

| denied/sent addr | direct | setsid child | re-exec |
|---|---|---|---|
| 192.0.2.1 | OK (no error) | OK | OK |
| 192.168.20.1 | OK | OK | OK |

Control (deny 192.0.2.1, send 192.168.20.1): OK. Unscoped baseline to both: OK. So 0/6 denied sends failed; the property is recorded but never enforced (no BPF egress filter attached; the scope sits under `user@1000.service` in a hybrid hierarchy).

## Verdict

Rule not met (0/3 EPERM; cgroup is hybrid). CLOSE F9 as not feasible on this kernel/systemd; keep the network-namespace wrapper and the deny-list spellings. No Item filed. Reopen only on a unified-cgroup2 systemd with working IP accounting for user units.

## Verification

- 2026-11-14, the commands above on this Jetson: 0/6 denied sends blocked, control and baseline unaffected. Docs tests: see commit message.
