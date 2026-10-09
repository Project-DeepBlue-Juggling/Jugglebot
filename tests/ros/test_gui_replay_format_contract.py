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
