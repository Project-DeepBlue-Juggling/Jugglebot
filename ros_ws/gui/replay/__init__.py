"""Replay backend for the browser GUI: the server lists recordings and serves
their MCAP files over HTTP Range (the browser decodes them in a Web Worker),
plus a cached per-recording overview pass for the timeline strip.

Two halves with different interpreters:

- ``schema``, ``recordings``, ``api`` are **stdlib only**: ``gui_server.py`` imports
  them under ``/usr/bin/python3`` (the systemd unit's premise is that boot never
  depends on the project venv).
- ``overview`` (with ``decode``) is the **worker**: it imports ``mcap`` and
  ``rosbags`` and only ever runs as a subprocess under the venv interpreter
  (``python -m replay.overview``), one pass at a time. ``convert`` is the test
  oracle (the same venv imports, plus ``msgpack``); nothing at runtime calls it.

Design record: ``plans/active/gui-rosbag-replay.md`` (Phase 1) and the
wayfinder map's ticket 05 under ``.scratch/gui-replay/`` (gitignored).
"""
