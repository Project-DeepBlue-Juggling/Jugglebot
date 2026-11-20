"""event-store.js (ros_ws/gui/js): the chart-only ring and the replay trim, the REAL module under node
(``tests/ros/js/event_store_harness.js``).

Throw / Catch markers live in their own ring so a minute of juggling cannot evict commands and faults from the Event
Log or the charts; the replay trim must not assume the stored events are time-ordered (a Catch is stamped 0.2-0.5 s
before its emission).
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.ros._node import find_node as _find_node  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
JS = ROOT / "ros_ws" / "gui" / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "event_store_harness.js"
NODE = _find_node()


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("event_store")
    for name in ("event-store.js", "clock.js"):
        shutil.copy(JS / name, sb / name)
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_600_throw_catch_events_do_not_evict_earlier_commands_or_faults(out):
    assert out["listTypes"] == ["fault", "command"]          # the Event Log list never sees chart-only types
    assert out["chartHasCommand"] is True
    assert out["chartOnlyKept"] == 500                        # own cap
    assert out["chartCount"] == 502
    assert out["chartNewest"] == "B 599"                      # newest chart-only events survive
    assert out["chartSorted"] is True                         # the range query is time-ordered across both rings


def test_trim_after_has_no_time_order_early_exit_and_covers_both_rings(out):
    assert out["afterBegin"] == 0
    assert out["trimmed"] == ["Early", "Early catch"]         # 'Mid' and 'Late' go although the last-pushed 'Early' is old
    assert out["replayedAgain"] == ["Early", "Early catch", "Late"]   # dedup keys were released with the trim
    assert out["restored"] == 502 and out["restoredListLen"] == 2


def test_subscribers_get_the_type_and_clear_is_null(out):
    assert out["seen"] == ["throw", None]
    assert out["afterClear"] == 0
