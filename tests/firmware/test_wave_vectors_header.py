"""FEAT-13: the generated C++ header of the Wave wire vectors must match the JSON.

The adapter's native test includes ``test/wire_vectors/wave_wire_vectors.h``; the node's
Python tests read ``tests/fixtures/wave_camera/wire_vectors.json``. The header is generated
from the JSON (the native build cannot read JSON without a dependency or a platformio.ini
change, both frozen), so a JSON edit without regenerating would leave the firmware test
checking the OLD vectors while the node test checks the new ones. Pure text: no pio, no
network, xdist-safe.
"""
from __future__ import annotations

import importlib.util
import json
import os

_GEN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "fixtures", "wave_camera", "gen_wave_vectors_header.py")
_spec = importlib.util.spec_from_file_location("gen_wave_vectors_header", _GEN)
gen = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gen)


def test_header_is_current_against_the_json():
    with open(gen.JSON_PATH) as fh:
        want = gen.render(json.load(fh))
    with open(gen.HEADER_PATH) as fh:
        have = fh.read()
    assert have == want, ("wave_wire_vectors.h is stale: run "
                          "python tests/fixtures/wave_camera/gen_wave_vectors_header.py")


def test_native_test_includes_the_generated_header():
    path = os.path.join(os.path.dirname(gen.HEADER_PATH), "..", "test_wave_wire_vectors",
                        "test_wave_wire_vectors.cpp")
    with open(path) as fh:
        assert '#include "../wire_vectors/wave_wire_vectors.h"' in fh.read()
