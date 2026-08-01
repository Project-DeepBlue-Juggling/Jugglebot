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
cat temp/reports/nightly/status                       # GREEN|RED <counts> <date>
```

## Keeping these in sync

The install is a **copy**, not a symlink, so the two can drift. `systemctl
--user enable` records the unit by name and a symlinked unit file confuses its
alias handling, which is why this is not symlinked. If you edit a unit, edit it
here, re-run the install block above, and say so in the logbook entry.

Armed 2026-08-01 (`logbook/2026-08-01-nightly-tier-and-mpc-dormancy.md`).
