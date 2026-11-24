# shellcheck shell=bash
# Live-session guard: never run unattended work alongside a powered robot.
#
# SOURCED, never executed. Sourced by tools/nightly_suite.sh and
# tools/night_shift.sh, so both runners share ONE definition of "a robot
# Sitting is live" (the regex below) and one wait loop.
#
# Why the guard exists: the 02:00 timers have Persistent=true, so a night the
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

LIVE_SESSION_DEFAULT_RE='(ros2 launch jugglebot|jugglebot_launch\.py|teensy_bridge_node|trajectory_node|orchestrator_node)'
LIVE_SESSION_RE="${LIVE_SESSION_RE:-$LIVE_SESSION_DEFAULT_RE}"

# BUG-85. With the DEFAULT regex a `pgrep -f` hit is only a candidate: the process
# counts as live only when its argv has the SHAPE of a running node or launch, so a
# sed/grep/editor/shell command that merely names a node file no longer reads as a
# Sitting. Shapes matched (argv from /proc/<pid>/cmdline, each basename compared exactly):
#   1. a node entry point:  argv[0] is, or  python*'s argv[1]/argv[2] is,  a path whose
#      basename is teensy_bridge_node | trajectory_node | orchestrator_node
#      (the installed console-script, `.../lib/jugglebot/<node> --ros-args ...`);
#      or `python* -m jugglebot.<node>`;
#   2. the launch file run directly: same positions, basename jugglebot_launch.py;
#   3. `ros2 launch jugglebot ...` or `ros2 launch <path>/jugglebot_launch.py`:
#      `ros2` (argv[0], or python*'s argv[1]/argv[2]) followed by `launch` and then
#      `jugglebot` or a jugglebot_launch.py path.
# An overridden LIVE_SESSION_RE (tests, a sentinel) keeps the old any-argv match.
_live_argv_is_robot() {
  local -a a=()
  mapfile -d '' -t a < "/proc/$1/cmdline" 2>/dev/null || return 1
  (( ${#a[@]} )) || return 1
  local base="${a[0]##*/}" i=0 j k t
  if [[ "$base" == python* ]]; then
    i=1
    [[ "${a[1]:-}" == -u || "${a[1]:-}" == -B ]] && i=2
    if [[ "${a[1]:-}" == -m ]]; then
      [[ "${a[2]:-}" =~ ^jugglebot\.(teensy_bridge_node|trajectory_node|orchestrator_node)$ ]] && return 0
      return 1
    fi
  fi
  for j in 0 $i; do
    t="${a[$j]:-}"; t="${t##*/}"
    case "$t" in
      teensy_bridge_node|trajectory_node|orchestrator_node|jugglebot_launch.py) return 0 ;;
      ros2)
        [[ "${a[$((j+1))]:-}" == launch ]] || continue
        for (( k = j + 2; k < ${#a[@]}; k++ )); do
          t="${a[$k]}"
          [[ "$t" == jugglebot || "${t##*/}" == jugglebot_launch.py ]] && return 0
        done ;;
    esac
  done
  return 1
}

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
    if [[ "$LIVE_SESSION_RE" == "$LIVE_SESSION_DEFAULT_RE" ]]; then
      # Unreadable cmdline (exited, or not ours to read) is not confirmed live; a
      # readable one must have a robot shape (see _live_argv_is_robot).
      _live_argv_is_robot "$p" || continue
    fi
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
