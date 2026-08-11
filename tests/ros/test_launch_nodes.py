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
