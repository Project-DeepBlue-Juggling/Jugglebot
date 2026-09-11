"""Locate the source repo a colcon install came from — for the launch files.

Why this exists (2026-09-11, skill-stack R1 sitting): both launch files prepend a
repo root to ``teensy_bridge_node``'s ``PYTHONPATH`` so the bridge runs the LIVE
``teensy_link`` + ``config/generated/udp_protocol.py`` (deliberately not installed
into the ROS package — see ``teensy_bridge_launch.py``).  The default was the
literal ``/home/jetson/Desktop/Jugglebot``.  A launch built and sourced from a
git worktree (``~/Desktop/Jugglebot-skills``) therefore ran the MAIN checkout's
transport: PROTOCOL_VERSION 6 against a freshly flashed FW 21 / protocol-7 board,
which is the designed link darkness — and it looked exactly like a dead board.
The bench driver was fine because it fixes its own ``sys.path``; only the launch
lied.  Parallel sessions on this project isolate with worktrees (memory:
feedback_parallel_session_worktrees), so the default must be the tree that
produced THIS install, never a fixed path.

Resolution order: ``JUGGLEBOT_REPO`` (authoritative when set) → walk up from the
anchor file until a directory holding ``config/generate_config.py`` appears (the
installed launch script lives under ``<repo>/ros_ws/install/...``, the installed
package under ``<repo>/ros_ws/install/.../site-packages/jugglebot``) → the
canonical path, for a deploy whose install tree lives outside the repo.
"""
from __future__ import annotations

import os

CANONICAL_REPO = '/home/jetson/Desktop/Jugglebot'
_ANCHOR = ('config', 'generate_config.py')


def resolve_repo_root(anchor_file: str | None = None) -> str:
    """The repo root the launch/bridge should treat as the live tree.

    ``anchor_file`` defaults to this module's own installed path, which resolves
    to the same repo as the launch script that imported it.
    """
    override = os.environ.get('JUGGLEBOT_REPO')
    if override:
        return override
    here = os.path.dirname(os.path.abspath(anchor_file or __file__))
    while True:
        if os.path.isfile(os.path.join(here, *_ANCHOR)):
            return here
        parent = os.path.dirname(here)
        if parent == here:
            return CANONICAL_REPO
        here = parent
