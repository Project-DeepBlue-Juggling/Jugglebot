"""test_launch_nodes.py — String-level tripwire on jugglebot_launch.py's node set
and on the topics THE ONE rosbag record list must keep.

HONESTY NOTE — this is a string-level tripwire in the regex style of
test_gui_geometry.py, NOT a behavioural launch test.  It reads the launch file
as text and asserts a node is both DEFINED (``executable='...'``) and MEMBERED
in the returned ``LaunchDescription([...])`` list.  It cannot prove the node
actually starts under ROS2 (no launch runtime here, and tests/ros mocks ROS2);
it catches exactly one regression class:

    a node that is deleted from the production launch (or its last launcher is
    removed) while its downstream consumers stay wired.

This is the class that bit us on 2026-07-06: Phase 13 (7c7f61b) deleted
catching_cone_test.launch.py — the ONLY launcher of catch_correlation_node —
without adding the node to jugglebot_launch.py.  Its consumers (the rosbag
/cone/timing_result entry and the GUI Catching Cone panel) stayed wired, so
cone/timing_result had zero publishers and a piezo hit surfaced nothing in the
GUI even though teensy_bridge_node kept publishing cone/catch_event +
cone/heartbeat.  See logbook/2026-05-23-throw-director-and-cone-live-
integration.md for the validated-working reference wiring.

The record-list half (added 2026-08-11, bridge-temporal-trustworthiness P0)
catches the mirror-image regression: a topic silently dropped from — or never
added to — the single ``ros2 bag record`` list, which makes a session bag unable
to answer the question the session was run to answer.  A missing topic is
unrecoverable after the fact; a recorded quiet one costs nothing, which is why
that list's standing rule is add-never-trim.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent.parent
LAUNCH_PY = (ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'launch'
             / 'jugglebot_launch.py')


@pytest.fixture(scope='module')
def launch_src():
    """Read jugglebot_launch.py as text."""
    return LAUNCH_PY.read_text()


@pytest.fixture(scope='module')
def launch_description_body(launch_src):
    """Slice the returned ``LaunchDescription([...])`` list body out of the
    launch file so membership checks see only what is actually launched (not a
    node variable that was defined but silently dropped from the list)."""
    m = re.search(r'return\s+LaunchDescription\(\s*\[(.*?)\]\s*\)',
                  launch_src, re.S)
    assert m, 'Could not find the return LaunchDescription([...]) block'
    return m.group(1)


# Nodes that MUST stay in the production launch because live consumers (rosbag
# topics, GUI panels) depend on their output.  Keyed by the setup.py console
# entry-point / launch executable name.
CONSUMER_WIRED_NODES = [
    # cone/timing_result → GUI Catching Cone panel + rosbag; sole owner of the
    # per-catch predicted-vs-actual delta (the 2026-07-06 regression).
    'catch_correlation_node',
]


@pytest.fixture(scope='module')
def rosbag_record_body(launch_src):
    """Slice the ``ros2 bag record`` argument list out of the launch file.

    Bounded at the trailing ``'-s', 'mcap'`` so a topic string appearing
    elsewhere in the file (a comment, a remap) cannot satisfy a membership
    check for THE ONE LIST.
    """
    m = re.search(r"'ros2',\s*'bag',\s*'record',(.*?)'-s',\s*'mcap'",
                  launch_src, re.S)
    assert m, "Could not find the 'ros2 bag record' command list"
    return m.group(1)


# Topics whose ABSENCE from the record list is unrecoverable after the fact —
# the standing add-never-trim rule for THE ONE LIST (toss-selftuning D18).
RECORDED_TOPICS = [
    # The 1 Hz firmware instrumentation (udp_rtt_us, udp_jitter_us, interp
    # deadline misses/jitter). It was PUBLISHED but not bagged, which is why
    # logbook/2026-07-18-teensy-uptime-tracking-degradation.md could only tell
    # the operator to "watch it live" and its seven-session lag-vs-uptime table
    # can never be joined against RTT retrospectively.
    '/profile',
    # The Teensy's post-clamp executed leg command at 100 Hz — the middle
    # timeline between /leg_setpoint_echo (what the Jetson asked for) and
    # robot_state (what the encoders did). Without it a degraded session's bag
    # cannot attribute the lag to transport vs interp vs ODrive.
    '/leg_cmd_executed',
    # The per-anchor clock-discipline series (FW 11 CLOCK_DIAG 0x8F). Its value
    # is a fit over HOURS of samples — the crystal's ppm and thermal coefficient
    # — so a session that publishes it without recording it produces nothing at
    # all. It records EMPTY until the bridge is flashed to FW 11 (deliberately
    # held until after the S1 aged-bridge experiment), and a silent topic is
    # exactly what this list's add-never-trim rule is for.
    '/clock_diag',
    # The encoder-cache freshness census (FW 12 CACHE_DIAG 0x91) — the
    # instrument that decides what S1 left open: a stale encoder cache under the
    # lead clamp, or a leg that genuinely trails. /robot_state and
    # /leg_cmd_executed cannot separate those (they read the same cache), so
    # this topic is the only place the answer exists, and the answer is a trend
    # across an hours-long soak. It records EMPTY until the bridge is flashed to
    # FW 12, and a silent topic is exactly what this list's add-never-trim rule
    # is for.
    '/cache_diag',
    # The CAN RX-ring TRUE-occupancy census (FW 13 RING_DIAG 0x92) — the
    # conviction instrument for the FlexCAN_T4 `_available` leak that S2 left as
    # the surviving candidate mechanism. /cache_diag's ring fields CANNOT stand
    # in: they are computed from getRXQueueCount(), i.e. from the very counter
    # the race corrupts, so they read healthy through a fully-leaked ring. The
    # verdict is a ratchet across an hours-long soak, so a session that publishes
    # this without recording it produces nothing. It records EMPTY until the
    # bridge is flashed to FW 13, and a silent topic is exactly what this list's
    # add-never-trim rule is for.
    '/ring_diag',
]


@pytest.mark.parametrize('topic', RECORDED_TOPICS)
def test_topic_is_in_the_rosbag_record_list(rosbag_record_body, topic):
    """The topic is an argument of the launch file's single ``ros2 bag record``."""
    assert f"'{topic}'," in rosbag_record_body, (
        f'{topic} is not in the rosbag record list — a session bag will be '
        f'missing it, and a missing topic is unrecoverable after the fact.')


@pytest.mark.parametrize('executable', CONSUMER_WIRED_NODES)
def test_consumer_wired_node_is_defined(launch_src, executable):
    """The node is declared as a Node(...) in the launch file."""
    assert f"executable='{executable}'" in launch_src, (
        f'{executable} is no longer declared in jugglebot_launch.py — a live '
        f'consumer (GUI panel / rosbag topic) depends on its output.')


@pytest.mark.parametrize('executable', CONSUMER_WIRED_NODES)
def test_consumer_wired_node_is_launched(launch_src, launch_description_body,
                                         executable):
    """The node variable is a member of the returned LaunchDescription list.

    Defining a Node but leaving it out of the LaunchDescription is the same
    silent failure as deleting it — the node never starts.  We map the
    executable name back to its assigned variable, then require that variable
    inside the assembled list.
    """
    var_m = re.search(
        rf'(\w+)\s*=\s*Node\(\s*[^)]*?executable=\'{executable}\'',
        launch_src, re.S)
    assert var_m, f'Could not locate the Node(...) assignment for {executable}'
    var = var_m.group(1)
    assert re.search(rf'^\s*{var},\s*$', launch_description_body, re.M), (
        f"{var} (executable='{executable}') is defined but not a member of "
        f'the returned LaunchDescription([...]) — the node will never start.')


# ═══════════════════════════════════════════════════════════════════════
# The BLAS thread cap on the planner-calling nodes
# ═══════════════════════════════════════════════════════════════════════
#
# MEASURED 2026-09-06 (60+ reps, dose-response) —
# logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md § Diagnosis,
# "Cause pinned (2026-09-06 evening)".
#
# A unified `plan_cycle` is thousands of SMALL numpy calls, each fanning out to
# OpenBLAS's DEFAULT six-worker pool whose workers BUSY-SPIN between calls. Idle,
# that is free (194-223 ms default vs 195-207 ms capped). At THREE busy cores of
# six the same solve takes 1350-2314 ms and gaps trajectory_node's 40 Hz emitter
# 225-942 ms — past the can-bridge's 250 ms MPC_STALE watchdog, which latches and
# E-STOPs the machine mid-rung. Capped to one thread: 214-217 ms at ANY load.
#
# The variables must be set BEFORE numpy is imported, so the launch file is the
# ONLY place the cap can live — which makes its silent removal exactly the
# regression class this file exists for. It is also invisible at runtime except
# as a slow solve under load, i.e. only on the robot, only under load, only as an
# E-STOP.
#
# NOT launch-wide, deliberately: mocap_node and ball_tracker_node do genuinely
# large-matrix work where the pool earns its keep and neither is on the setpoint
# stream's thread, so this list is exactly the three planner callers.
PLANNER_NODES = ['trajectory_node', 'reload_coordinator_node',
                 'catch_coordinator_node']

#: Both spellings, because numpy 1.24.4 on this Jetson links OpenBLAS
#: (``libopenblas64_p-r0-cecebdce.3.21.so``, pthreads/armv8) but an OpenMP-
#: threaded build of any transitive dependency reads the other one.
BLAS_CAP_VARS = ['OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS']


def _node_block(launch_src, executable):
    """The source text of the ONE ``Node(...)`` call for ``executable``.

    Blocks are cut at the ``    var = Node(`` … ``    )`` indentation pair FIRST
    and then filtered by executable, rather than searched for by executable —
    the reverse order lets a non-greedy ``.*?`` span from the file's first
    ``Node(`` all the way past the wanted one, which silently returns the whole
    middle of the launch file and makes every assertion below it vacuous. (It
    did, on the first draft of this test: every executable resolved to the same
    200-line span, so the negative test passed by accident.)
    """
    blocks = re.findall(r'^    \w+ = Node\($\n(.*?)^    \)$',
                        launch_src, re.S | re.M)
    hits = [b for b in blocks if f"executable='{executable}'" in b]
    assert len(hits) == 1, (
        f'Expected exactly one Node(...) block for {executable}, found '
        f'{len(hits)} (of {len(blocks)} blocks parsed).')
    return hits[0]


@pytest.mark.parametrize('executable', PLANNER_NODES)
@pytest.mark.parametrize('var', BLAS_CAP_VARS)
def test_planner_node_caps_its_blas_thread_pool(launch_src, executable, var):
    """Each planner-calling node carries ``<var>=1`` in its ``additional_env``.

    Resolves the launch file's ``_planner_blas_env`` indirection: the node block
    must reference it (or spell the variable inline), and the dict it references
    must map the variable to ``'1'``.
    """
    block = _node_block(launch_src, executable)
    assert 'additional_env' in block, (
        f'{executable} has no additional_env — it calls the unified planner, so '
        f'an uncapped BLAS pool here can gap the 40 Hz emitter past the '
        f'can-bridge 250 ms MPC_STALE watchdog under box load.')

    inline = re.search(rf"'{var}'\s*:\s*'1'", block)
    if inline:
        return

    ref = re.search(r'additional_env\s*=\s*(?:dict\(\s*)?(\w+)', block)
    assert ref, (
        f"{executable}'s additional_env is neither an inline "
        f"'{var}': '1' nor a reference to a shared dict.")
    env_name = ref.group(1)
    env_m = re.search(rf'{env_name}\s*=\s*\{{(.*?)\}}', launch_src, re.S)
    assert env_m, (
        f"{executable} references additional_env dict `{env_name}`, which is "
        f'not defined as a literal in the launch file.')
    assert re.search(rf"'{var}'\s*:\s*'1'", env_m.group(1)), (
        f'`{env_name}` does not set {var}=1, so {executable} inherits the '
        f'DEFAULT 6-thread OpenBLAS pool. Measured 2026-09-06: at three busy '
        f'cores that takes plan_cycle from ~200 ms to 1350-2314 ms and gaps the '
        f'emitter 225-942 ms, latching MPC_STALE. See '
        f'logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md.')


@pytest.mark.parametrize('executable', ['mocap_node', 'ball_tracker_node'])
def test_the_blas_cap_is_not_applied_launch_wide(launch_src, executable):
    """The cap stops at the planner callers — this is a deliberate boundary.

    mocap_node and ball_tracker_node do LARGE-matrix work (where a real thread
    pool earns its keep) and neither is on the setpoint stream's thread, so
    capping them would be a pure loss. If a future change makes the cap global,
    this test is where that decision gets re-argued rather than absorbed.
    """
    block = _node_block(launch_src, executable)
    for var in BLAS_CAP_VARS:
        assert var not in block, (
            f'{executable} now carries {var} — the cap was scoped to the three '
            f'planner-calling nodes on purpose (PLANNER_NODES above). If that '
            f'is intended, update this test and say why in the launch file.')
