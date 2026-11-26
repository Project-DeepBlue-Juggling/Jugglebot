#!/usr/bin/env bash
# Off-box backup of this Jetson to the Raspberry Pi server `jeeves`.
#
#   tools/backup_to_jeeves.sh home      # user data: bags, repo, temp/, calibration, config (no sudo)
#   sudo tools/backup_to_jeeves.sh system  # the OS: /usr /etc /opt /var /boot (root, to read every file)
#   add --dry-run to list what would be sent; add --now to skip the live-Sitting wait
#
# Everything lands under ONE directory on the Pi, $DEST_ROOT (default
# jetson-backup/ in the jeeves home), so the Jetson's files never mix with the
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
DEST_ROOT="${DEST_ROOT:-jetson-backup}"
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
BASE="$DEST_ROOT/$SET"
remote "mkdir -p '$BASE/.incomplete' && cd '$DEST_ROOT' && { [ -e README.txt ] || printf '%s\n' \
  'Backups of the Jugglebot Jetson (hostname jetson), written by tools/backup_to_jeeves.sh in the Jugglebot repo.' \
  'home/<date>/   = /home/jetson (user data, bags, repo).   system/<date>/ = the OS root, stored with rsync --fake-super.' \
  'Each dated directory is a full snapshot; unchanged files are hard links shared between snapshots.' \
  'Nothing outside this directory belongs to the Jetson. Do not edit files in place: snapshots share them.' > README.txt; }"

OPTS=(-aHAX --numeric-ids --delete --delete-excluded --partial --one-file-system
      --info=stats2 --human-readable -e "${SSH[*]}")
for e in "${EXCLUDES[@]}"; do OPTS+=(--exclude "$e"); done
(( FAKE_SUPER )) && OPTS+=(--rsync-path "rsync --fake-super")
remote "[ -d '$BASE/latest/' ]" && OPTS+=(--link-dest "../latest/")
(( DRY )) && OPTS+=(--dry-run)

say "backup $SET -> $BACKUP_HOST:$BASE/$TODAY$( (( DRY )) && echo ' [DRY RUN]')"
rc=0
nice -n 15 ionice -c3 rsync "${OPTS[@]}" "$SRC" "$BACKUP_HOST:$BASE/.incomplete/" || rc=$?
# 24 = files vanished while copying (a live system always has some): a good snapshot.
if (( rc != 0 && rc != 24 )); then
  say "FAILED: rsync exit $rc; the previous snapshot is untouched"; status RED "rsync-exit-$rc"; exit 1
fi
(( DRY )) && { say "dry run complete"; exit 0; }

# Publish: today's run replaces a same-day snapshot, then prune.
remote "set -e; cd '$BASE'; if [ -d '$TODAY' ]; then mv '$TODAY' '.old-$TODAY'; fi; mv .incomplete '$TODAY'; \
  ln -sfn '$TODAY' latest; rm -rf '.old-$TODAY'; \
  keep=\$( { ls -d 20??-??-?? | sort -r | head -n $KEEP_DAILY; \
            for d in \$(ls -d 20??-??-?? | sort -r); do [ \"\$(date -d \"\$d\" +%u)\" = 7 ] && echo \"\$d\"; done | head -n $KEEP_WEEKLY; } | sort -u ); \
  for d in \$(ls -d 20??-??-??); do echo \"\$keep\" | grep -qx \"\$d\" || rm -rf \"\$d\"; done; \
  du -sh . | cut -f1"
say "OK: $SET snapshot $TODAY published"
status GREEN "$TODAY"
