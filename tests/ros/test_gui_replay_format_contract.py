"""chunk.js CHUNK_FORMAT must equal replay/schema.py FORMAT_VERSION."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros_ws" / "gui"))

from replay import schema  # noqa: E402


def test_chunk_format_matches_schema():
    src = (ROOT / "ros_ws/gui/js/replay/chunk.js").read_text()
    m = re.search(r"CHUNK_FORMAT\s*=\s*(\d+)", src)
    assert m, "chunk.js must define CHUNK_FORMAT = <int>"
    assert int(m.group(1)) == schema.FORMAT_VERSION, (
        f"chunk.js CHUNK_FORMAT={m.group(1)} != replay/schema.py FORMAT_VERSION="
        f"{schema.FORMAT_VERSION}: both files must change together")


def test_overview_tick_kinds_drawn_by_ui_are_in_schema():
    """overview.js TICK_KINDS (the kinds the UI draws) must be a subset of schema.OVERVIEW_TICK_KINDS."""
    src = (ROOT / "ros_ws/gui/js/replay/ui/overview.js").read_text()
    block = re.search(r"export const TICK_KINDS\s*=\s*\{(.*?)\n\};", src, re.S)
    assert block, "overview.js must define TICK_KINDS"
    drawn = set(re.findall(r"^\s{4}(\w+):\s*\{", block.group(1), re.M))
    assert drawn == {"fault", "skill_attempt", "catch_event", "bb_calibration"}
    assert drawn <= set(schema.OVERVIEW_TICK_KINDS), drawn - set(schema.OVERVIEW_TICK_KINDS)


def test_picker_key_topics_are_in_allowlist():
    src = (ROOT / "ros_ws/gui/js/replay/ui/format.js").read_text()
    keys = re.search(r"KEY_TOPICS[^=]*=\s*\[(.*?)\]", src, re.S)
    assert keys, "format.js must define KEY_TOPICS"
    topics = re.findall(r"topic:\s*'(/[^']+)'", keys.group(1))
    assert len(topics) == 4 and set(topics) <= set(schema.ALLOWLIST), set(topics) - set(schema.ALLOWLIST)
