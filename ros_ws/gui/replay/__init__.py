"""Replay backend for the browser GUI: a recording's MCAP file is converted
once into a per-recording cache (manifest + overview + 10 s chunks) that the
GUI server serves as plain files.

Two halves with different interpreters:

- ``schema``, ``cache``, ``api`` are **stdlib only**: ``gui_server.py`` imports
  them under ``/usr/bin/python3`` (the systemd unit's premise is that boot never
  depends on the project venv).
- ``convert`` is the **worker**: it imports ``mcap``, ``rosbags`` and
  ``msgpack`` and only ever runs as a subprocess under the venv interpreter
  (``python -m replay.convert``), one conversion at a time.

Design record: ``plans/active/gui-rosbag-replay.md`` (Phase 1) and the
wayfinder map's ticket 05 under ``.scratch/gui-replay/`` (gitignored).
"""
