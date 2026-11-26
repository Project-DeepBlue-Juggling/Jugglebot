#!/usr/bin/env bash
# Pi-side forced command for the Jetson's backup key. Installed ON JEEVES at
# ~/.local/bin/jetson-backup-gate and named in ~/.ssh/authorized_keys:
#
#   command="/home/jeeves/.local/bin/jetson-backup-gate",restrict ssh-ed25519 AAAA... jetson-backup-to-jeeves
#
# With that line the key can do exactly two things, both confined to $ROOT:
#   1. rsync, through rrsync (which rejects any path outside $ROOT);
#   2. the three maintenance verbs below, whose arguments are pattern-checked.
# It cannot open a shell, forward a port, or touch anything else on the Pi.
# The gate lives OUTSIDE $ROOT on purpose: the key can write inside $ROOT, so a
# gate stored there could be rewritten by the key it restricts.
#
# Verbs (sent by tools/backup_to_jeeves.sh):
#   backup-prepare <set>                       mkdir <set>/.incomplete, write README.txt once
#   backup-has-latest <set>                    exit 0 if <set>/latest/ exists
#   backup-publish <set> <date> <daily> <weekly>   .incomplete -> <date>, repoint latest, prune
set -euo pipefail
ROOT="${JETSON_BACKUP_ROOT:-/home/jeeves/jetson-backup}"
cmd="${SSH_ORIGINAL_COMMAND:-}"

refuse() { echo "jetson-backup-gate: refused: $1" >&2; exit 97; }

case "$cmd" in
  "rsync --server "*) exec /usr/bin/rrsync -no-lock "$ROOT" ;;
esac

read -r verb set a b c extra <<<"$cmd" || true
[[ -z "${extra:-}" ]] || refuse "too many arguments"
[[ "${set:-}" == home || "${set:-}" == system ]] || refuse "unknown verb or set"
mkdir -p "$ROOT"
cd "$ROOT"

case "$verb" in
  backup-prepare)
    [[ -z "${a:-}" ]] || refuse "backup-prepare takes one argument"
    mkdir -p "$set/.incomplete"
    [[ -e README.txt ]] || printf '%s\n' \
      'Backups of the Jugglebot Jetson (hostname jetson), written by tools/backup_to_jeeves.sh in the Jugglebot repo.' \
      'home/<date>/   = /home/jetson (user data, bags, repo).   system/<date>/ = the OS root, stored with rsync --fake-super.' \
      'Each dated directory is a full snapshot; unchanged files are hard links shared between snapshots.' \
      'Nothing outside this directory belongs to the Jetson. Do not edit files in place: snapshots share them.' > README.txt
    ;;
  backup-has-latest)
    [[ -z "${a:-}" ]] || refuse "backup-has-latest takes one argument"
    [[ -d "$set/latest/" ]]
    ;;
  backup-publish)
    [[ "${a:-}" =~ ^20[0-9]{2}-[0-9]{2}-[0-9]{2}$ ]] || refuse "bad date"
    [[ "${b:-}" =~ ^[0-9]{1,2}$ && "${c:-}" =~ ^[0-9]{1,2}$ ]] || refuse "bad keep counts"
    (( b >= 1 )) || refuse "must keep at least one daily"
    cd "$set"
    [[ -d .incomplete ]] || refuse "nothing to publish"
    # A same-day rerun replaces that day's snapshot; the old one goes only after the swap.
    if [[ -d "$a" ]]; then mv "$a" ".old-$a"; fi
    mv .incomplete "$a"
    ln -sfn "$a" latest
    rm -rf ".old-$a"
    # Keep the newest <daily> snapshots plus the newest <weekly> Sunday snapshots.
    mapfile -t all < <(ls -d 20??-??-?? 2>/dev/null | sort -r)
    keep=("${all[@]:0:b}")
    n=0
    for d in "${all[@]}"; do
      (( n >= c )) && break
      if [[ "$(date -d "$d" +%u)" == 7 ]]; then keep+=("$d"); n=$(( n + 1 )); fi
    done
    for d in "${all[@]}"; do
      [[ " ${keep[*]} " == *" $d "* ]] || rm -rf -- "$d"
    done
    du -sh . | cut -f1
    ;;
  *) refuse "unknown verb" ;;
esac
