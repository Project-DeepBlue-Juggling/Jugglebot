---
title: "Backup: nightly home and weekly OS snapshots of the Jetson to the Pi server jeeves (tools/backup_to_jeeves.sh), the first off-box copy of the bags"
type: feature
date: 2026-11-26
status: in-progress
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - tools/backup_to_jeeves.sh
  - tools/systemd/jugglebot-backup-home.service
  - tools/systemd/jugglebot-backup-home.timer
  - tools/systemd/jugglebot-backup-system.service
  - tools/systemd/jugglebot-backup-system.timer
  - tools/systemd/README.md
subsystem: [config]
tags: [docs]
---

## Summary

Until today nothing on this Jetson had a second copy: the bags, `temp/`, the calibration sessions and the Claude memory existed only on one SSD. `tools/backup_to_jeeves.sh` now copies the box to the Raspberry Pi server `jeeves` (192.168.20.7, a 917 GB ext4 disk) over SSH with rsync, into the single directory `/home/jeeves/jetson-backup/` so the Jetson's files are separate from the Pi's own content.

- `home` set, nightly 04:15, user timer: `/home/jetson` without caches and rebuildable trees. First run: 36.11 GB, 151,617 files.
- `system` set, Sundays 05:00, system timer (root): the OS root without `/home`, `/tmp`, caches and snaps, stored with `--fake-super` so the unprivileged `jeeves` account keeps root ownership and modes in xattrs.
- Snapshots are `--link-dest` hard-link trees, seven dailies and four Sundays kept: a file deleted or corrupted here survives in the older snapshots, and an unchanged compressed bag is stored once. This is why the bag compression of 2026-11-25 went first (20 GB of bags to copy, not 69 GB, and files that no longer change).
- A run writes to `.incomplete` and renames on success, waits for a live Sitting to end, and leaves one status line per set in `temp/reports/backup/`.

A Jetson cannot boot from a file copy (its bootloader partitions come from the flashing tool), so the restore is a fresh JetPack flash plus two rsyncs back; the steps are in `tools/systemd/README.md`.

## Verification

- 2026-11-26, `tools/backup_to_jeeves.sh home` (first run, from the scratch worktree): rsync exit 0, 36.11 GB in 19 min (31.7 MB/s), snapshot `home/2026-11-26` published, status GREEN.
- 2026-11-26, `rsync -an --checksum --itemize-changes --files-from=<sample>` of 25 random bags plus 400 memory and logbook files against `home/latest/`: no differences.
- The system set's first run and the gate result are in the commit message.
