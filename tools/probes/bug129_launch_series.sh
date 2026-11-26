#!/usr/bin/env bash
# BUG-129 unpowered launch series: arm A (default launch) vs arm B (bridge_shm:=false).
#
# Question: does teensy_bridge_node's Fast DDS shared-memory transport cause the ~340 ms
# /robot_state publish freeze at a discovery event? Spec: map/bug-teensy-bridge-node-freezes-*
# and the series spec (2026-11-24). Parser: tools/probes/bug129_series_report.py.
#
# SAFETY FACTS (read before running)
#   * The robot's MOTOR SUPPLY IS OFF (owner-confirmed). The can-bridge board is powered from
#     the Jetson, so the launch talks to the REAL bridge; the drives are unpowered (every launch
#     logs ODrive DC_BUS_UNDER_VOLTAGE ERRORs and a latched ODRIVE_FATAL: expected, not a finding).
#   * auto_arm:=false in BOTH arms. Nothing is homed, armed or commanded: the churn steps are
#     read-only ros2 CLI calls (node list, topic hz/echo, service list). No `kill -STOP` anywhere.
#   * The runner REFUSES to start if any other jugglebot launch / node is running (live_session),
#     and watches each launch's own log for an unexpected arm or motion command; on a match it
#     SIGINTs the launch at once and aborts the series (exit 3).
#
# Per launch (~7 min incl. cool-down): t0-90 s hands off; t=60 manipulation check (fastrtps
# maps in the bridge: >0 in A, exactly 0 in B, else discard and rerun the slot); t=90 churn
# 10 s apart (node list, topic hz, echo+kill -9, service list, 30 s wait, echo+kill -9, echo
# --once /link_status); idle to t=270; ONE SIGINT to the launch (its own process group, see bug129_stop_group.sh), <=30 s, then
# SIGTERM/SIGKILL of the group; the ending signal goes to the .meta (stop_signal=) and the .log; 20 s cool-down. The 30 s mark of the FIRST launch also asserts
# that tools/lib/live_session.sh reports LIVE (first meeting of the BUG-85 argv-shape guard
# with a real launch); the result is written to the meta file and the series log, and a
# NOT-LIVE result stops the series after that launch (exit 4).
#
# Guards: flock /tmp/jugglebot-tests.lock held per launch until the launch's processes have
# exited (the fd is inherited by them); 1-min load > 4 waits up to 10 min (then runs and
# records the load); a launch only starts if its projected end is before --stop-at AND clear
# of 01:30-04:00 (nightly 02:00, Night Shift 02:45); HEAD of the checkout and the mtimes of
# the installed teensy_bridge_node.py / dds_shm.py are re-checked before each launch and the
# series aborts (exit 5) if they change.
#
# Usage:
#   tools/probes/bug129_launch_series.sh --stop-at 01:15 [--launches-per-arm 12] [--dry-run]
#                                        [--date YYYY-MM-DD] [--main DIR] [--arms-order ABBA]
# Outputs: $MAIN/temp/logs/bug129_series_<date>_<A|B>_<nn>[r<k>].{log,meta,churn},
#          $MAIN/temp/logs/bug129_series_<date>_series.log, then (non-dry) the report
#          $MAIN/temp/reports/bug129_series_<date>.md via the parser. ~5 GB of bags.
set -uo pipefail

MAIN="${HOME}/Desktop/Jugglebot"
N_PER_ARM=12
FIRST_SLOT=0               # resume: 0-based slot to start at (earlier slots keep their logs)
STOP_AT=""
DRY=0
DATE="$(date +%F)"
LAUNCH_BUDGET_S=480        # projected wall time of one launch incl. cool-down and slack
LOCK=/tmp/jugglebot-tests.lock
MAX_RETRIES=3
# What an unexpected arm / motion command looks like in the launch's own log (ANSI stripped).
FORBIDDEN_RE='setpoint output ARMED|homing (started|sequence)|Homing:|ARM_CATCH|arm_catch requested|move_to_pose|ACTIVATE requested'

while (( $# )); do
  case "$1" in
    --stop-at) STOP_AT="$2"; shift 2 ;;
    --launches-per-arm) N_PER_ARM="$2"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --date) DATE="$2"; shift 2 ;;
    --main) MAIN="$2"; shift 2 ;;
    --first-slot) FIRST_SLOT="$2"; shift 2 ;;
    -h|--help) sed -n 2,45p "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 64 ;;
  esac
done
[[ -n "$STOP_AT" ]] || { echo "--stop-at HH:MM is required (hard stop; must be <= 01:30 or >= 04:00)" >&2; exit 64; }
[[ "$STOP_AT" =~ ^([01][0-9]|2[0-3]):[0-5][0-9]$ ]] || { echo "--stop-at must be HH:MM" >&2; exit 64; }
[[ "$N_PER_ARM" =~ ^[0-9]+$ && "$N_PER_ARM" -ge 1 ]] || { echo "--launches-per-arm must be a positive integer" >&2; exit 64; }

LOGS="$MAIN/temp/logs"
SERIES_LOG="$LOGS/bug129_series_${DATE}_series.log"
INST="$MAIN/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot"
ENVSRC="source /opt/ros/foxy/setup.bash && source $MAIN/ros_ws/install/setup.bash"
LAUNCH_BASE="ros2 launch jugglebot jugglebot_launch.py auto_arm:=false"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say() { local m; m="$(date +%H:%M:%S) $*"; echo "$m"; (( DRY )) || echo "$m" >> "$SERIES_LOG"; }
mins_of_day() { local h m; h=${1%%:*}; m=${1##*:}; echo $(( 10#$h * 60 + 10#$m )); }

# --stop-at must itself be outside the quiet window (01:30, 04:00).
sm=$(mins_of_day "$STOP_AT")
if (( sm > 90 && sm < 240 )); then
  echo "--stop-at $STOP_AT is inside 01:30-04:00 (nightly 02:00, Night Shift 02:45): refused" >&2; exit 64
fi
STOP_EPOCH=$(date -d "today $STOP_AT" +%s)
(( STOP_EPOCH <= $(date +%s) )) && STOP_EPOCH=$(date -d "tomorrow $STOP_AT" +%s)

# shellcheck disable=SC1091
source "$MAIN/tools/lib/live_session.sh" || { echo "cannot source live_session.sh" >&2; exit 2; }

# ── helpers ────────────────────────────────────────────────────────────────────────────────
head_sha() { git -C "$MAIN" rev-parse HEAD 2>/dev/null; }
install_stamp() { stat -c '%Y' "$INST/teensy_bridge_node.py" "$INST/dds_shm.py" 2>/dev/null | tr '\n' ','; }
bridge_pid() {   # the installed console script: python3 .../lib/jugglebot/teensy_bridge_node ...
  local p a
  for p in $(pgrep -f 'lib/jugglebot/teensy_bridge_node' 2>/dev/null); do
    a="$(tr '\0' '\n' < "/proc/$p/cmdline" 2>/dev/null | sed -n 2p)"
    [[ "$a" == */lib/jugglebot/teensy_bridge_node ]] && { echo "$p"; return 0; }
  done
  return 1
}
in_quiet_window() {   # would [now, now+LAUNCH_BUDGET_S] touch 01:30-04:00?
  local s0; s0=$(( 10#$(date +%H) * 3600 + 10#$(date +%M) * 60 + 10#$(date +%S) ))
  (( s0 + LAUNCH_BUDGET_S > 5400 && s0 < 14400 ))
}
load1() { cut -d' ' -f1 /proc/loadavg; }
load_high() { awk -v l="$(load1)" 'BEGIN{exit !(l>4)}'; }

# Slot sequence: blocks A B B A A B B A ...
slot_arm() { case $(( $1 % 4 )) in 0|3) echo A ;; *) echo B ;; esac; }

LOCKFD=""
# shellcheck disable=SC1091
source "$HERE/bug129_stop_group.sh" || { echo "cannot source bug129_stop_group.sh" >&2; exit 2; }
stop_launch() { group_stop 30 10 5; }   # one SIGINT to the launch's process group, then TERM, KILL
trap 'say "runner interrupted: stopping the launch"; stop_launch; exit 130' INT TERM

abort_series() { say "ABORT: $*"; stop_launch; exit "${2:-1}"; }   # $2 not used by say

# ── dry run: print the plan and every command, launch nothing ──────────────────────────────
print_launch_plan() {   # $1 arm $2 slot name
  local arm="$1" nm="$2" extra=""
  [[ "$arm" == B ]] && extra=" bridge_shm:=false"
  local log="$LOGS/bug129_series_${DATE}_${nm}.log"
  cat <<EOF
  --- launch ${nm} (arm ${arm}) ---
  [pre]  live_session must be clear; load1 <= 4 (else wait <= 10 min); HEAD + install mtimes unchanged;
         exec {fd}>$LOCK ; flock -w 900 \$fd ; free -m / uptime appended to ${log%.log}.meta
  [t=0]  (set -m) bash -c '$ENVSRC && exec $LAUNCH_BASE$extra' > $log 2>&1 &   # own process group
  [t=2s..]  every 2 s: grep -aE '$FORBIDDEN_RE' on the log (ANSI stripped) -> SIGINT launch, abort series
  [t=30] (first launch only) source $MAIN/tools/lib/live_session.sh; live_session  -> must be LIVE, recorded
  [t=60] bridge pid via /proc argv; grep -c /dev/shm/fastrtps_ /proc/<pid>/maps  (A: >0, B: ==0); ls /dev/shm | wc -l
  [t=90] bash -c '$ENVSRC && timeout 30 ros2 node list --no-daemon'          ; sleep 10
         bash -c '$ENVSRC && timeout -s INT 5 ros2 topic hz /robot_state'    ; sleep 10
         bash -c '$ENVSRC && ros2 topic echo /robot_state >/dev/null & sleep 4; kill -9 \$!' ; sleep 10
         bash -c '$ENVSRC && timeout 30 ros2 service list --no-daemon'       ; sleep 30
         bash -c '$ENVSRC && ros2 topic echo /robot_state >/dev/null & sleep 4; kill -9 \$!' ; sleep 10
         bash -c '$ENVSRC && timeout 30 ros2 topic echo --once /link_status >/dev/null'
  [t=270] one SIGINT to the launch (leader of its own process group, SIGINT reset to default); wait <= 30 s, else SIGTERM the group,
          <= 10 s, else SIGKILL (the signal that ended it goes to the meta and the log); live_session must be clear; sleep 20
  [post] parser input = ${log%.log}.{log,meta}; bag path is in the log
EOF
}

if (( DRY )); then
  echo "DRY RUN: nothing is launched, no lock taken, no file written."
  echo "main=$MAIN date=$DATE launches-per-arm=$N_PER_ARM stop-at=$STOP_AT (epoch $STOP_EPOCH) budget/launch=${LAUNCH_BUDGET_S}s"
  echo "HEAD=$(head_sha)  install mtimes=$(install_stamp)"
  echo "live_session now: $(live_session && echo LIVE || echo clear)   load1: $(load1)   /dev/shm files: $(ls /dev/shm | wc -l)"
  echo "lock: $LOCK (one per launch, held until the launch's processes exit)"
  echo "slot order (A B B A ...):"
  t=$(date +%s)
  for ((i = 0; i < 2 * N_PER_ARM; i++)); do
    arm=$(slot_arm "$i"); nm=$(printf '%s_%02d' "$arm" $((i + 1)))
    verdict="starts $(date -d @"$t" +%H:%M)"
    (( t + LAUNCH_BUDGET_S > STOP_EPOCH )) && verdict="WOULD NOT START (past --stop-at)"
    printf '  %2d  %s  %s\n' $((i + 1)) "$nm" "$verdict"
    t=$((t + LAUNCH_BUDGET_S))
  done
  first_a=$(slot_arm 0); first_b=$(slot_arm 1)
  print_launch_plan "$first_a" "$(printf '%s_%02d' "$first_a" 1)"
  print_launch_plan "$first_b" "$(printf '%s_%02d' "$first_b" 2)"
  echo "  (every other launch repeats its arm's plan; a failed manipulation check reruns the slot as <arm>_<nn>r<k>, <= $MAX_RETRIES times)"
  echo "end: python tools/probes/bug129_series_report.py --date $DATE --main $MAIN   (nice -n 10)"
  exit 0
fi

# ── real run ───────────────────────────────────────────────────────────────────────────────
mkdir -p "$LOGS" "$MAIN/temp/reports"
[[ -d "$INST" ]] || { echo "installed package not found: $INST" >&2; exit 2; }
if live_session; then
  echo "REFUSED: a jugglebot launch or node is already running (live_session reports LIVE)." >&2
  pgrep -af 'ros2 launch jugglebot|jugglebot_launch\.py|teensy_bridge_node|trajectory_node|orchestrator_node' | cut -c1-160 >&2
  exit 2
fi
START_HEAD="$(head_sha)"; START_STAMP="$(install_stamp)"
say "series start: date=$DATE per-arm=$N_PER_ARM stop-at=$STOP_AT HEAD=$START_HEAD install_mtimes=$START_STAMP"
say "motor supply OFF (owner-confirmed); auto_arm:=false; runner pid $$"

LIVE_ASSERTED=0
slot=$FIRST_SLOT; attempts=0
total=$(( 2 * N_PER_ARM ))
while (( slot < total )); do
  arm=$(slot_arm "$slot")
  nm=$(printf '%s_%02d' "$arm" $((slot + 1)))
  (( attempts > 0 )) && nm="${nm}r${attempts}"
  base="$LOGS/bug129_series_${DATE}_${nm}"
  now=$(date +%s)
  (( now + LAUNCH_BUDGET_S > STOP_EPOCH )) && { say "stop: next launch would end past --stop-at $STOP_AT"; break; }
  in_quiet_window && { say "stop: next launch would touch the 01:30-04:00 quiet window"; break; }

  # Only a change to what the live nodes load counts: the auto-pull timer fast-forwards main every
# 2 min with map/ and docs/ commits (second attempt, 2026-11-25: HEAD moved by four map commits).
git -C "$MAIN" diff --quiet "$START_HEAD" HEAD -- ros_ws/src teensy_link controller config sim 2>/dev/null \
    || abort_series "code under ros_ws/src, teensy_link, controller, config or sim changed mid-series ($START_HEAD -> $(head_sha))" 5
  [[ "$(install_stamp)" == "$START_STAMP" ]] || abort_series "installed teensy_bridge_node.py / dds_shm.py changed mid-series" 5
  if ! wait_for_live_session_to_clear 600 10 "bug129_series"; then
    abort_series "another launch is running and did not clear in 10 min" 2
  fi
  waited=0
  while load_high && (( waited < 600 )); do sleep 15; waited=$((waited + 15)); done

  exec {LOCKFD}>"$LOCK"
  flock -w 900 "$LOCKFD" || abort_series "could not take $LOCK in 15 min" 2

  {
    echo "arm=$arm"; echo "slot=$nm"; echo "head=$(head_sha)"; echo "install_mtimes=$(install_stamp)"
    echo "load1_before=$(load1)"; echo "load_wait_s=$waited"
    echo "uptime=$(uptime | sed 's/^ *//')"
    echo "free_mb=$(free -m | awk '/^Mem:/{print "used="$3" avail="$7}')"
    echo "shm_files_before=$(ls /dev/shm | wc -l)"
  } > "${base}.meta"

  extra=""; [[ "$arm" == B ]] && extra=" bridge_shm:=false"
  say "launch $nm: load1=$(load1) (waited ${waited}s)"
  T0=$(date +%s); echo "start_epoch=$T0" >> "${base}.meta"
  # Own process group with default signal dispositions (see bug129_stop_group.sh): a plain
  # background job ignores SIGINT and the old runner's SIGINT never reached the launch.
  group_start "${base}.log" bash -c "$ENVSRC && exec $LAUNCH_BASE$extra"
  LAUNCH_PID=$GROUP_PID

  # wait_until <offset_s>: sleep to T0+offset, scanning the log for forbidden commands every 2 s.
  # Returns 1 if the launch died.
  wait_until() {
    local tgt=$((T0 + $1))
    while (( $(date +%s) < tgt )); do
      group_alive || return 1
      if sed 's/\x1b\[[0-9;]*m//g' "${base}.log" | grep -aEq "$FORBIDDEN_RE"; then
        say "FORBIDDEN arm/motion line in ${base}.log: $(sed 's/\x1b\[[0-9;]*m//g' "${base}.log" | grep -aE "$FORBIDDEN_RE" | head -1)"
        stop_launch; exec {LOCKFD}>&-
        say "ABORT: unexpected arm or motion command with the motor supply off"; exit 3
      fi
      sleep 2
    done
    return 0
  }
  churn() {   # churn <label> <command...> : run in the ROS env, bounded, rc to the churn log
    local lbl="$1"; shift
    local t; t=$(date +%H:%M:%S.%N | cut -c1-12)
    # The lock descriptor is closed for the churn commands: a ros2 CLI call that starts the
    # ros2 daemon would otherwise hand it the lock, and the daemon outlives the launch
    # (first series, 2026-11-25: the next launch waited 15 min on the lock and aborted).
    # `$ENVSRC || exit; cmd`, not `$ENVSRC && cmd`: with `&&`, a command of the form
    # `ros2 ... & sleep 4; kill -9 $!` backgrounds the whole `&&` list in a subshell, `$!` is
    # that subshell, and the kill leaves the ros2 process alive (first series, 2026-11-25:
    # two `ros2 topic echo` survived, held the lock, and no participant was ever killed).
    bash -c "$ENVSRC || exit 97; $*" {LOCKFD}>&- > /dev/null 2>&1; echo "$t $lbl rc=$?" >> "${base}.churn"
  }

  ok=1
  wait_until 30 || ok=0
  if (( ok && LIVE_ASSERTED == 0 )); then
    if live_session; then r=LIVE; else r=NOT_LIVE; fi
    shapes="$(for p in $(pgrep -f "$LIVE_SESSION_RE"); do _live_argv_is_robot "$p" && echo -n "$p:robot "; done)"
    echo "live_session_30s=$r" >> "${base}.meta"; echo "live_session_pids=$shapes" >> "${base}.meta"
    say "live_session at t=30 s of the first launch: $r (robot-shaped pids: ${shapes:-none})"
    LIVE_ASSERTED=1; [[ "$r" == LIVE ]] || LIVE_FAIL=1
  fi
  (( ok )) && wait_until 60 || ok=0
  manip="fail"
  if (( ok )); then
    bp="$(bridge_pid || true)"
    if [[ -n "$bp" ]]; then
      # Count only the Fast DDS shared-memory SEGMENTS (/dev/shm/fastrtps_<hex>, fastrtps_port<N>), never
      # a bare "fastrtps": every rclpy process maps ~16 libfastrtps / *_typesupport_fastrtps_c.so libraries
      # (64 maps), which made arm B read 64 for a bridge with NO shm transport (probe 2026-11-25).
      maps=$(grep -c '/dev/shm/fastrtps_' "/proc/$bp/maps" 2>/dev/null); maps=${maps:--1}   # grep -c prints 0 AND exits 1 on no match
      echo "bridge_pid=$bp" >> "${base}.meta"; echo "fastrtps_maps=$maps" >> "${base}.meta"
      echo "shm_files_t60=$(ls /dev/shm | wc -l)" >> "${base}.meta"
      if [[ "$arm" == A && "$maps" -gt 0 ]] || [[ "$arm" == B && "$maps" -eq 0 ]]; then manip=ok; fi
    else
      echo "fastrtps_maps=-1" >> "${base}.meta"; echo "bridge_pid=none" >> "${base}.meta"
    fi
  fi
  echo "manipulation_check=$manip" >> "${base}.meta"
  if (( ok )) && [[ "$manip" == ok ]]; then
    wait_until 90 || ok=0
    if (( ok )); then
      churn node_list      "timeout 30 ros2 node list --no-daemon";                                   wait_until 100 || ok=0
      (( ok )) && churn hz "timeout -s INT 5 ros2 topic hz /robot_state";                              (( ok )) && { wait_until 110 || ok=0; }
      (( ok )) && churn echo_kill9_1 'ros2 topic echo /robot_state >/dev/null & sleep 4; kill -9 $!'; (( ok )) && { wait_until 120 || ok=0; }
      (( ok )) && churn service_list "timeout 30 ros2 service list --no-daemon";                       (( ok )) && { wait_until 150 || ok=0; }
      (( ok )) && churn echo_kill9_2 'ros2 topic echo /robot_state >/dev/null & sleep 4; kill -9 $!'; (( ok )) && { wait_until 160 || ok=0; }
      (( ok )) && churn echo_once_link "timeout 30 ros2 topic echo --once /link_status";
      (( ok )) && { wait_until 270 || ok=0; }
    fi
  fi

  # ── clean shutdown, bounded ──
  exit_note="clean"
  group_alive || exit_note="launch died early"
  stop_launch
  case "$GROUP_STOP_SIGNAL" in
    INT) ;;
    none) ;;
    *) exit_note="launch group needed SIG$GROUP_STOP_SIGNAL (SIGINT did not end it)" ;;
  esac
  echo "bridge_exit=$exit_note" >> "${base}.meta"; echo "stop_signal=$GROUP_STOP_SIGNAL" >> "${base}.meta"
  echo "end_epoch=$(date +%s)" >> "${base}.meta"
  echo "bug129_runner: launch ended by SIG$GROUP_STOP_SIGNAL (${exit_note})" >> "${base}.log"
  LAUNCH_PID=""
  # No churn subscriber may outlive its launch (it would hold shm and count as a participant).
  pkill -9 -f 'bin/ros2 topic ech[o] /robot_state' 2>/dev/null || true
  # Stop the ros2 daemon the churn started, so every launch meets the same discovery state.
  bash -c "$ENVSRC && timeout 20 ros2 daemon stop" {LOCKFD}>&- > /dev/null 2>&1 || true
  exec {LOCKFD}>&-
  sleep 20
  if live_session; then abort_series "launch processes left behind after $nm" 2; fi

  if [[ "$manip" != ok ]]; then
    attempts=$((attempts + 1)); say "launch $nm: manipulation check FAILED or launch died: discarded, rerun slot ($attempts/$MAX_RETRIES)"
    (( attempts > MAX_RETRIES )) && abort_series "slot $((slot + 1)) failed its manipulation check $MAX_RETRIES times" 6
  else
    say "launch $nm done ($exit_note)"
    slot=$((slot + 1)); attempts=0
  fi
  if [[ "${LIVE_FAIL:-0}" == 1 ]]; then
    say "STOP: live_session did NOT report LIVE during a real launch (BUG-85 guard misses the real argv shape); fix the guard before more launches"
    break
  fi
done

say "series loop finished: slots completed=$slot of $total"
nice -n 10 "$HOME/Desktop/PDJ_venv/venv/bin/python" -I "$HERE/bug129_series_report.py" --date "$DATE" --main "$MAIN" 2>&1 | tee -a "$SERIES_LOG" | tail -5
[[ "${LIVE_FAIL:-0}" == 1 ]] && exit 4
exit 0
