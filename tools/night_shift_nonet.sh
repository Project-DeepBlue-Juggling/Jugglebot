#!/usr/bin/env bash
# tools/night_shift_nonet.sh: run a command with loopback and no other network, the
# evidence read-only and the serial devices masked.
#
#   tools/night_shift_nonet.sh ./run_tests.sh --full
#   tools/night_shift_nonet.sh python -m pytest tests/ros/test_teensy_bridge_node_read.py -q
#   tools/night_shift_nonet.sh python tools/probes/<probe>.py --bag ~/Desktop/rosbags/<id>
#
# Why: the can-bridge stays powered overnight at 192.168.42.2 and its UDP link has
# one owner. The bridge and client tests all run on loopback with ephemeral ports,
# so they need no network, but nothing except this namespace makes it IMPOSSIBLE
# for a test a unit just wrote to reach the board. The Bash deny list only reads
# the spelling of a command; this holds whatever the command turns out to do.
#
# Probes and bag reads (closeout C7a, 2026-11-14) run here too. Inside the namespace:
#   - ~/Desktop/rosbags and ~/.ros/log are bound read-only: a write fails EROFS, so a
#     probe cannot reindex, convert or append to the evidence;
#   - every /dev/ttyACM* and /dev/ttyUSB* is /dev/null and /dev/serial is an empty
#     read-only tmpfs, so a probe's default serial port opens nothing;
#   - ROS_LOG_DIR, when unset, points at a fresh temp dir (rcl and launch would
#     otherwise create their log files in the read-only ~/.ros/log and fail).
# The mounts are made in an outer user namespace and the command runs in a NESTED one:
# the kernel locks mounts inherited by a less privileged namespace, so the command
# cannot umount or remount them rw. Without the nesting it can (probed 2026-11-14).
# NIGHT_SHIFT_NONET_EXTRA_RO / _EXTRA_MASK / _EXTRA_EMPTY (colon lists; MASK takes
# globs) ADD paths for tests and never remove a default. A default path that does not
# exist is skipped (nothing there to protect).
#
# Fails closed: if the namespace cannot be made, loopback cannot come up or any mount
# fails, the command does NOT run and the exit code is non-zero (125), which the merge
# gate reads as a failed suite. Needs no root (user namespaces); the network part was
# probed on this box 2026-11-07, the mounts 2026-11-14.
set -uo pipefail

if (( $# == 0 )); then
  echo "usage: $0 COMMAND [ARGS...]" >&2
  exit 2
fi

# Both the passwd home and $HOME, so a changed HOME cannot move the protection.
pw_home=$(getent passwd "$(id -un)" 2>/dev/null | cut -d: -f6)
RO="${pw_home:+$pw_home/Desktop/rosbags:$pw_home/.ros/log:}$HOME/Desktop/rosbags:$HOME/.ros/log"
RO+="${NIGHT_SHIFT_NONET_EXTRA_RO:+:$NIGHT_SHIFT_NONET_EXTRA_RO}"
MASK="/dev/ttyACM*:/dev/ttyUSB*${NIGHT_SHIFT_NONET_EXTRA_MASK:+:$NIGHT_SHIFT_NONET_EXTRA_MASK}"
EMPTY="/dev/serial${NIGHT_SHIFT_NONET_EXTRA_EMPTY:+:$NIGHT_SHIFT_NONET_EXTRA_EMPTY}"

if [[ -z ${ROS_LOG_DIR:-} ]]; then
  ROS_LOG_DIR=$(mktemp -d "${TMPDIR:-/tmp}/night_shift_nonet_roslog.XXXXXX") \
    || { echo "night_shift_nonet: cannot make ROS_LOG_DIR" >&2; exit 125; }
  export ROS_LOG_DIR
fi

exec unshare --user --map-root-user --net --mount bash -c '
  fail() { echo "night_shift_nonet: $*" >&2; exit 125; }
  ip link set lo up || fail "cannot bring loopback up"
  IFS=: read -ra ro <<< "$1"
  IFS=: read -ra mask <<< "$2"
  IFS=: read -ra empty <<< "$3"
  shift 3
  declare -A seen=()
  for d in "${ro[@]}"; do
    [[ -n $d && -e $d && -z ${seen[$d]:-} ]] || continue
    seen[$d]=1
    { mount --bind "$d" "$d" && mount -o remount,bind,ro "$d" "$d"; } \
      || fail "cannot bind $d read-only"
  done
  shopt -s nullglob
  for g in "${mask[@]}"; do
    [[ -n $g ]] || continue
    for f in $g; do
      mount --bind /dev/null "$f" || fail "cannot mask $f"
    done
  done
  for d in "${empty[@]}"; do
    [[ -n $d && -d $d ]] || continue
    mount -t tmpfs -o ro,mode=755,size=4k night_shift_nonet "$d" || fail "cannot empty $d"
  done
  exec unshare --user --map-root-user --mount "$@"
' night_shift_nonet "$RO" "$MASK" "$EMPTY" "$@"
