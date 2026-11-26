#!/usr/bin/env bash
# Start and stop a process GROUP the way a terminal does (BUG-129 series runner; sourced by
# bug129_launch_series.sh and by tests/sim/test_bug129_series_report.py).
#
# Why: in a non-interactive shell `cmd &` makes the job IGNORE SIGINT, and an ignored signal
# survives exec, so `ros2 launch` (python) never installed its KeyboardInterrupt handler and
# every SIGINT the old runner sent was a no-op: 24 of 24 launches of the 2026-11-25 series
# ended with "bridge SIGKILLed after 30 s; launch group SIGKILLed". `set -m` (job control)
# starts the job with default signal dispositions in its OWN process group (pgid == pid).
#
# Trap (measured 2026-11-26, real launch): SIGINT to the whole GROUP reaches every node directly AND
# again via the launch's own forwarding ("sending signal 'SIGINT' to process[...]"): a double SIGINT,
# KeyboardInterrupt tracebacks in trajectory/skill/ball_butler/bridge shutdown (the bridge's
# on_shutdown disarm-settle sleep was interrupted). So SIGINT goes to the LEADER (the launch) only,
# and it forwards one each; the group gets TERM/KILL only as escalation. Likewise never
# `timeout -s INT` (child AND group). A terminal Ctrl-C is the group form; do not copy it here.
#
# group_start <logfile> <command...>   sets GROUP_PID (the leader, also the pgid); the command's
#                                      stdout+stderr go to <logfile>. Pass `exec x` so the leader
#                                      is the program itself.
# group_stop [int_wait_s=30] [term_wait_s=10] [kill_wait_s=5]
#                                      SIGINT the leader; wait for the group; SIGTERM the group; wait; SIGKILL. Sets
#                                      GROUP_STOP_SIGNAL to INT|TERM|KILL|none (none = already gone)
#                                      and clears GROUP_PID. Never blocks beyond the sum of waits.
GROUP_PID=""; GROUP_STOP_SIGNAL=""

group_alive() { [[ -n "${GROUP_PID:-}" ]] && kill -0 -- "-$GROUP_PID" 2>/dev/null; }

group_start() {
  local log="$1"; shift
  set -m
  # The reset is the load-bearing part: a runner that was itself started with SIGINT ignored (a
  # background job, nohup, a CI shell) cannot un-ignore it in bash (ignored-at-entry signals stay
  # ignored) and `set -m` does not change that; the exec'd launch inherited SIG_IGN (measured:
  # SigIgn 0x1001006 on `ros2 launch`, 2026-11-26). python does the reset, then execs the command.
  python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL); signal.signal(signal.SIGQUIT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' "$@" > "$log" 2>&1 &
  GROUP_PID=$!
  set +m
}

_group_wait_gone() {   # _group_wait_gone <seconds>: 0 once no member of the group remains
  local i; for (( i = 0; i < $1; i++ )); do group_alive || return 0; sleep 1; done
  group_alive && return 1 || return 0
}

group_stop() {
  local w_int="${1:-30}" w_term="${2:-10}" w_kill="${3:-5}"
  GROUP_STOP_SIGNAL=none
  if group_alive; then
    GROUP_STOP_SIGNAL=INT
    kill -INT "$GROUP_PID" 2>/dev/null   # the LEADER only: see the header
    if ! _group_wait_gone "$w_int"; then
      GROUP_STOP_SIGNAL=TERM; kill -TERM -- "-$GROUP_PID" 2>/dev/null
      if ! _group_wait_gone "$w_term"; then
        GROUP_STOP_SIGNAL=KILL; kill -KILL -- "-$GROUP_PID" 2>/dev/null
        _group_wait_gone "$w_kill" || true
      fi
    fi
  fi
  wait "$GROUP_PID" 2>/dev/null
  GROUP_PID=""
}
