#!/usr/bin/env bash
# tools/night_shift_nonet.sh: run a command with loopback and no other network.
#
#   tools/night_shift_nonet.sh ./run_tests.sh --full
#   tools/night_shift_nonet.sh python -m pytest tests/ros/test_teensy_bridge_node_read.py -q
#
# Why: the can-bridge stays powered overnight at 192.168.42.2 and its UDP link has
# one owner. The bridge and client tests all run on loopback with ephemeral ports,
# so they need no network, but nothing except this namespace makes it IMPOSSIBLE
# for a test a unit just wrote to reach the board. The Bash deny list only reads
# the spelling of a command; this holds whatever the command turns out to do.
#
# Fails closed: if the namespace cannot be made or loopback cannot come up, the
# command does NOT run and the exit code is non-zero, which the merge gate reads as
# a failed suite. Needs no root (a user namespace); probed on this box 2026-11-07.
set -uo pipefail

if (( $# == 0 )); then
  echo "usage: $0 COMMAND [ARGS...]" >&2
  exit 2
fi

exec unshare --user --map-root-user --net bash -c '
  ip link set lo up || { echo "night_shift_nonet: cannot bring loopback up" >&2; exit 125; }
  exec "$@"
' night_shift_nonet "$@"
