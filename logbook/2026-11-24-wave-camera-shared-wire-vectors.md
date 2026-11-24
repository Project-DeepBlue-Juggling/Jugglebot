---
title: "FEAT-13: one shared JSON fixture of Wave camera wire vectors, consumed by the node tests and the adapter's native tests"
type: feature
date: 2026-11-24
status: complete
phase: "FEAT-13"
related_plan: ""
related_issues: []
files_changed:
  - tests/fixtures/wave_camera/wire_vectors.json
  - tests/fixtures/wave_camera/gen_wave_vectors_header.py
  - tests/ros/test_wave_camera.py
  - tests/firmware/test_wave_vectors_header.py
  - ros_ws/src/jugglebot/WaveCamera_ESP_code/test/wire_vectors/wave_wire_vectors.h
  - ros_ws/src/jugglebot/WaveCamera_ESP_code/test/test_wave_wire_vectors/test_wave_wire_vectors.cpp
subsystem: [tests-tooling, tracking]
---

## Summary

The CAN adapter design (s7) asked for one JSON fixture of frame vectors that the ESP32 firmware tests and the node tests both consume. Now both read `tests/fixtures/wave_camera/wire_vectors.json`: 3 WAVE_CMD, 6 heartbeat, 8 REC_EVENT and 5 INFO-page vectors, each as fields plus the exact 8 bytes (laid out from the s4 tables with plain `struct` packing, not from either implementation). Between them the vectors hit every `WaveState`, `WaveEventKind` and `WaveReason` member and all four info pages, the saturation ends (elapsed 0xFFFE, sync age 255, t40 2^40-1, u32 max) and the reserved-bit masking.

- Node side: `tests/ros/test_wave_camera.py` gains parametrised `test_shared_vector_*` (decode bytes to fields, re-encode to the same bytes) and a coverage check that the set still exercises every enum member. The hand-built vectors stay (they document the tables field by field).
- Firmware side: a new native suite `test/test_wave_wire_vectors` encodes every vector's fields and compares bytes (and `decode_cmd` for the one frame the adapter decodes). `src/`, `include/` and `platformio.ini` are untouched.

Chosen mechanism for the native side: a header generated from the JSON and checked in beside the test (`test/wire_vectors/wave_wire_vectors.h`, rendered by `tests/fixtures/wave_camera/gen_wave_vectors_header.py`), with `tests/firmware/test_wave_vectors_header.py` failing when the header is stale. Why not the alternatives: reading JSON natively needs a parser library (a new dependency, and `platformio.ini` is frozen); a generate-at-build-time step needs an `extra_scripts` entry in `platformio.ini`, also frozen. The currency test is pure text, so it runs in every gate without pio, and a JSON edit that is not regenerated fails there instead of letting the firmware test silently check the old vectors.

## Verification

- 2026-11-24, `python -m pytest tests/ros/test_wave_camera.py tests/firmware/test_wave_vectors_header.py -q`: 66 passed.
- 2026-11-24, `flock /tmp/jugglebot-tests.lock pio test -e native -f test_wave_wire_vectors` (workspace and libdeps dirs as in `tests/firmware/test_wave_native.py`): 4/4 cases passed in 2.06 s.
- The full `tests/firmware/test_wave_native.py` run (all 84 native cases) and the native-side mutation were queued behind other sessions on the box lock when this entry was written; the orchestrator's gate covers them.
- Fails without the change (mutate once, restore): flipping one byte of `clear_faults_session` in the JSON fails `test_shared_vector_cmd[clear_faults_session]` and `test_header_is_current_against_the_json`; the native-side mutation (flip a byte, regenerate the header, run the suite) was queued behind the box lock and not observed by the time of commit.
