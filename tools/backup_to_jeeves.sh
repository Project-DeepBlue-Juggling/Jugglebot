#!/usr/bin/env bash
# Off-box backup of this Jetson to the Raspberry Pi server `jeeves`.
#
#   tools/backup_to_jeeves.sh home      # user data: bags, repo, temp/, calibration, config (no sudo)
#   sudo tools/backup_to_jeeves.sh system  # the OS: /usr /etc /opt /var /boot (root, to read every file)
#   add --dry-run to list what would be sent; add --now to skip the live-Sitting wait
#
# Everything lands under ONE directory on the Pi, /home/jeeves/jetson-backup/ (fixed by the
# Pi-side gate, tools/backup_jeeves_gate.sh, which is all the backup key may run), so the Jetson's files never mix with the
# Pi's own content:
#
#   jetson-backup/README.txt
#   jetson-backup/home/2026-11-26/      one dated snapshot per run
#   jetson-backup/home/latest -> 2026-11-26
#   jetson-backup/system/2026-11-26/
#
# Snapshots are rsync --link-dest hard-link trees: an unchanged file (a compressed
# bag) is stored once and shared by every snapshot that has it, while a file that
# was deleted or corrupted here still exists in the older snapshots. Kept: the
# newest $KEEP_DAILY dailies plus the newest $KEEP_WEEKLY Sunday snapshots.
#
# A run writes into <set>/.incomplete and renames it to the date only after rsync
# succeeds, so `latest` never points at a half-written snapshot and a killed run
# resumes where it stopped.
#
# The system set is stored with --fake-super: owners, modes and xattrs go into
# user xattrs on the Pi, so the unprivileged jeeves account can hold root-owned
# files and a restore (rsync --fake-super back, as root here) gets them right.
# Restore steps: tools/systemd/README.md § Backup.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKUP_HOST="${BACKUP_HOST:-jeeves@jeeves.local}"
BACKUP_KEY="${BACKUP_KEY:-/home/jetson/.ssh/id_backup_jeeves}"
DEST_ROOT="${DEST_ROOT:-jetson-backup}"   # display only: the Pi-side gate fixes the real directory
KEEP_DAILY="${KEEP_DAILY:-7}"
KEEP_WEEKLY="${KEEP_WEEKLY:-4}"
LIVE_WAIT_S="${LIVE_WAIT_S:-14400}"
STATUS_DIR="${STATUS_DIR:-$REPO/temp/reports/backup}"

usage() { sed -n '2,8p' "$0"; exit 2; }
SET="${1:-}"; shift || true
DRY=0; NOW=0
for a in "$@"; do
  case "$a" in --dry-run) DRY=1 ;; --now) NOW=1 ;; *) usage ;; esac
done

# Excludes are rsync patterns anchored at the transfer root.
case "$SET" in
  home)
    SRC=/home/jetson/
    EXCLUDES=(
      /.cache/ /.npm/ /.nvm/ /.vscode-server/ /.vscode/ /.platformio/ /snap/
      /.local/share/Trash/ /.config/Code/Cache*/ /.config/Code/CachedData/
      /.config/Code/CachedExtensionVSIXs/ /.mozilla/firefox/*/cache2/
      /night-shift/ /map-live/                       # clones of GitHub
      /Desktop/Jugglebot/ros_ws/build/ /Desktop/Jugglebot/ros_ws/install/
      /Desktop/Jugglebot/ros_ws/log/ '.pio/' '__pycache__/' 'node_modules/'
      '*.zst-partial'
    )
    FAKE_SUPER=0 ;;
  system)
    [[ $EUID -eq 0 ]] || { echo "the system set must run as root (sudo)"; exit 2; }
    SRC=/
    EXCLUDES=(
      /home/ /tmp/ /proc/ /sys/ /dev/ /run/ /mnt/ /media/ /lost+found/ /snap/
      /var/tmp/ /var/cache/ /var/lib/snapd/ /var/lib/docker/ /var/lib/containerd/
      /var/log/journal/ /swapfile
    )
    FAKE_SUPER=1 ;;
  *) usage ;;
esac

SSH=(ssh -i "$BACKUP_KEY" -o BatchMode=yes -o ConnectTimeout=15 -o ServerAliveInterval=30
     -o UserKnownHostsFile=/home/jetson/.ssh/known_hosts)   # root (the system set) has no known_hosts of its own
remote() { "${SSH[@]}" "$BACKUP_HOST" "$@"; }
say() { printf '%s %s\n' "$(date +%FT%T)" "$*"; }
status() {  # one line, the same shape as temp/reports/nightly/status
  mkdir -p "$STATUS_DIR" && printf '%s %s %s\n' "$1" "$2" "$(date -Iseconds)" > "$STATUS_DIR/status-$SET"
  [[ $EUID -eq 0 ]] && chown -R jetson:jetson "$STATUS_DIR" 2>/dev/null || true
}

# Never compete with a powered robot for IO and RAM: wait, then defer loudly.
if (( ! NOW && ! DRY )); then
  # shellcheck source=tools/lib/live_session.sh
  source "$REPO/tools/lib/live_session.sh"
  waited=0
  while live_session; do
    (( waited >= LIVE_WAIT_S )) && { say "DEFERRED: a Sitting is still live"; status DEFERRED live-session; exit 0; }
    sleep 300; waited=$(( waited + 300 ))
  done
fi

exec 9>"/tmp/jugglebot-backup-$SET.lock"
flock -n 9 || { say "another $SET backup is running"; exit 0; }

TODAY="$(date +%F)"
# The backup key is confined on the Pi by tools/backup_jeeves_gate.sh: it may run rsync
# inside the backup directory (paths below are relative to that directory) and the three
# backup-* verbs, nothing else. So no shell commands are sent, only verbs.
remote backup-prepare "$SET"

OPTS=(-aHAX --numeric-ids --delete --delete-excluded --partial --one-file-system
      --info=stats2 --human-readable -e "${SSH[*]}")
for e in "${EXCLUDES[@]}"; do OPTS+=(--exclude "$e"); done
(( FAKE_SUPER )) && OPTS+=(-M--fake-super)
# An absolute --link-dest is resolved by the gate (rrsync) inside the backup directory.
remote backup-has-latest "$SET" && OPTS+=(--link-dest "/$SET/latest/")
(( DRY )) && OPTS+=(--dry-run)

say "backup $SET -> $BACKUP_HOST:$DEST_ROOT/$SET/$TODAY$( (( DRY )) && echo " [DRY RUN]")"
rc=0
nice -n 15 ionice -c3 rsync "${OPTS[@]}" "$SRC" "$BACKUP_HOST:$SET/.incomplete/" || rc=$?
# 24 = files vanished while copying (a live system always has some): a good snapshot.
if (( rc != 0 && rc != 24 )); then
  say "FAILED: rsync exit $rc; the previous snapshot is untouched"; status RED "rsync-exit-$rc"; exit 1
fi
(( DRY )) && { say "dry run complete"; exit 0; }

# Publish: .incomplete becomes today (replacing a same-day snapshot), then prune.
remote backup-publish "$SET" "$TODAY" "$KEEP_DAILY" "$KEEP_WEEKLY"
say "OK: $SET snapshot $TODAY published"
status GREEN "$TODAY"
