"""Generate the C++ header of the shared Wave camera wire vectors (FEAT-13).

The adapter's native tests (PlatformIO ``-e native``, Unity) cannot read JSON without a
parser dependency or a platformio.ini change, and ``src/``, ``include/`` and
``platformio.ini`` are frozen. So the JSON in this directory is the single source and
this script renders it to a checked-in header next to the native test;
``tests/firmware/test_wave_vectors_header.py`` fails when the two disagree.

    python tests/fixtures/wave_camera/gen_wave_vectors_header.py          # rewrite the header
    python tests/fixtures/wave_camera/gen_wave_vectors_header.py --check  # exit 1 if stale
"""
from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "wire_vectors.json")
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
HEADER_PATH = os.path.join(ROOT, "ros_ws", "src", "jugglebot", "WaveCamera_ESP_code", "test",
                           "wire_vectors", "wave_wire_vectors.h")

_INFO_FIELDS = {
    0: ("fw_version", "wire_version", "uptime_s"),
    1: ("tick_period_ns", "tick_jitter_us", "ticks_missed"),
    2: ("offset_spread_us", "track_err_us", "accepted_per_s", "rejected_per_s", "steps"),
    3: ("last_tick_to_clip_us", "can_tx_errors", "bus_off_events", "intent_rx_per_s"),
}


def _bytes(v):
    assert len(v["bytes"]) == 8, v["name"]
    return "{" + ", ".join("0x%02X" % b for b in v["bytes"]) + "}"


def _u(n):
    return "%dULL" % n if n > 0xFFFFFFFF else "%du" % n


def render(data: dict) -> str:
    o = ["// GENERATED from tests/fixtures/wave_camera/wire_vectors.json by",
         "// tests/fixtures/wave_camera/gen_wave_vectors_header.py. Do not hand-edit;",
         "// tests/firmware/test_wave_vectors_header.py fails when this is stale.",
         "#pragma once",
         "#include <cstdint>",
         "",
         "namespace wave_vectors {",
         ""]
    o.append("struct CmdVec { const char* name; uint8_t bytes[8]; uint8_t desired; uint8_t cmd_seq; "
             "uint8_t flags; uint32_t session_id; };")
    o.append("static const CmdVec CMD[] = {")
    for v in data["cmd"]:
        o.append('    {"%s", %s, %d, %d, %d, %s},' % (v["name"], _bytes(v), v["desired"], v["cmd_seq"],
                                                    v["flags"], _u(v["session_id"])))
    o += ["};", ""]
    o.append("struct HeartbeatVec { const char* name; uint8_t bytes[8]; uint8_t state; uint8_t status; "
             "uint8_t faults; uint16_t elapsed_100ms; uint8_t sync_age_100ms; uint8_t recordings; "
             "uint8_t cmd_seq_echo; };")
    o.append("static const HeartbeatVec HEARTBEAT[] = {")
    for v in data["heartbeat"]:
        o.append('    {"%s", %s, %d, %d, %d, %d, %d, %d, %d},' % (
            v["name"], _bytes(v), v["state"], v["status"], v["faults"], v["elapsed_100ms"],
            v["sync_age_100ms"], v["recordings"], v["cmd_seq_echo"]))
    o += ["};", ""]
    o.append("struct RecEventVec { const char* name; uint8_t bytes[8]; uint64_t t40; uint8_t kind; "
             "uint8_t reason; bool synced; bool tick_stamped; uint8_t cmd_seq_echo; "
             "uint8_t recording_seq; };")
    o.append("static const RecEventVec REC_EVENT[] = {")
    for v in data["rec_event"]:
        o.append('    {"%s", %s, %s, %d, %d, %s, %s, %d, %d},' % (
            v["name"], _bytes(v), _u(v["t40"]), v["kind"], v["reason"],
            "true" if v["synced"] else "false", "true" if v["tick_stamped"] else "false",
            v["cmd_seq_echo"], v["recording_seq"]))
    o += ["};", ""]
    o.append("// Info pages: f[] holds the page's fields in wire order (see the JSON `fields` keys).")
    o.append("struct InfoVec { const char* name; uint8_t bytes[8]; uint8_t page; uint32_t f[5]; };")
    o.append("static const InfoVec INFO[] = {")
    for v in data["info"]:
        names = _INFO_FIELDS[v["page"]]
        assert tuple(v["fields"]) == names, (v["name"], tuple(v["fields"]))
        vals = [v["fields"][n] for n in names] + [0] * (5 - len(names))
        o.append('    {"%s", %s, %d, {%s}},' % (v["name"], _bytes(v), v["page"],
                                                ", ".join(_u(x) for x in vals)))
    o += ["};", "", "}  // namespace wave_vectors", ""]
    return "\n".join(o)


def main(argv):
    with open(JSON_PATH) as fh:
        text = render(json.load(fh))
    if "--check" in argv:
        have = open(HEADER_PATH).read() if os.path.exists(HEADER_PATH) else None
        if have != text:
            print("stale: %s" % HEADER_PATH)
            return 1
        return 0
    os.makedirs(os.path.dirname(HEADER_PATH), exist_ok=True)
    with open(HEADER_PATH, "w") as fh:
        fh.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
