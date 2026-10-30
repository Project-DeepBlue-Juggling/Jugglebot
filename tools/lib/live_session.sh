# shellcheck shell=bash
# Live-session guard: never run unattended work alongside a powered robot.
#
# SOURCED, never executed. Sourced by tools/nightly_suite.sh and
# tools/night_shift.sh, so both runners share ONE definition of "a robot
# Sitting is live" (the regex below) and one wait loop.
#
# Why the guard exists: the 04:00 timers have Persistent=true, so a night the
# Jetson was off fires at the next boot, which is often the minute an operator
# powers up for a Sitting. Nice/IO classes bound CPU and IO but NOT memory, and
# this 7.3 GB box degrades hard when over-committed. An OOM-killed pytest
# worker is a confusing failure; an OOM-killed ROS node mid-Sitting is a robot
# event. So unattended runners WAIT for the box rather than competing for it.
# It is a wait, not a skip: on budget expiry the caller records DEFERRED, never
# GREEN and never silence.
#
# LIVE_SESSION_RE can be overridden from the environment. That exists so a test
# can point the guard at a sentinel process it spawns itself; production never
# sets it.

LIVE_SESSION_RE="${LIVE_SESSION_RE:-(ros2 launch jugglebot|jugglebot_launch\.py|teensy_bridge_node|trajectory_node|orchestrator_node)}"

live_session() {
  # `pgrep -f` matches full command lines, so anything whose ARGV contains one of
  # these names matches — including our own process group when this script is
  # pasted into a `bash -c` (the pattern is then literally in the command line;
  # harmless in production, where the process is the script PATH). Drop our own
  # process group, PID and parent so the guard can never trip on itself.
  local mypgid p pg
  mypgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
  for p in $(pgrep -f "$LIVE_SESSION_RE" 2>/dev/null); do
    [[ "$p" == "$$" || "$p" == "$PPID" ]] && continue
    pg="$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ')"
    # Empty pgid = the process exited between pgrep and ps (typically one of our
    # own transient subshells). Gone is not live — only defer for a process we
    # can positively confirm is still there.
    [[ -z "$pg" ]] && continue
    [[ -n "$mypgid" && "$pg" == "$mypgid" ]] && continue
    return 0
  done
  return 1
}

# wait_for_live_session_to_clear <budget_s> <poll_s> [prefix]
# Returns 0 if the box is clear (immediately, or after waiting), 1 if a live
# session still holds it after the budget. Sets LIVE_SESSION_WAITED_S to the
# seconds waited. Progress lines go to stderr, prefixed with $3.
wait_for_live_session_to_clear() {
  local budget="$1" poll="$2" prefix="${3:-live_session}"
  LIVE_SESSION_WAITED_S=0
  live_session || return 0
  echo "$prefix: live robot session detected — waiting (budget ${budget}s)" >&2
  while live_session && (( LIVE_SESSION_WAITED_S < budget )); do
    sleep "$poll"
    LIVE_SESSION_WAITED_S=$((LIVE_SESSION_WAITED_S + poll))
  done
  if live_session; then
    return 1
  fi
  echo "$prefix: session cleared after ${LIVE_SESSION_WAITED_S}s — proceeding" >&2
  return 0
}
