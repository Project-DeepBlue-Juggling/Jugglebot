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
under `/usr/bin/python3`; only the conversion worker it spawns on demand (niced,
one at a time) needs the venv, via `--worker-python`. The rosbags root and the
cache directory (`temp/replay_cache`, gitignored, LRU-capped at 10 GB) are
passed on the `ExecStart` line.

```bash
sudo cp tools/systemd/jugglebot-gui.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart jugglebot-gui
systemctl status jugglebot-gui
```

**Do not restart while someone is using the GUI**: the restart drops every open
page's connection and kills a running conversion (its partial cache is discarded
and re-queued on the next open). Edit the unit here, then re-run the block above;
the install is a copy, so the two can drift.
