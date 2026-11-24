"""The skill-stack suites at ``trajectory_op.emit_lead_knots`` = 1 (nightly).

The planner's wire horizon, every splice lead, freeze and cliff fence derive
from the emit lead (``schedule.EMIT_LEAD_KNOTS``, INVARIANTS I-WIRE-7), and the
constants are fixed at import. The configured lead is 0, so the per-commit gate
exercises lead 0 only; this tier re-runs the suites that read those numbers in
ONE subprocess pytest with the ``tests/conftest.py`` hook
(``JUGGLEBOT_TEST_EMIT_LEAD_KNOTS=1``) overriding the generated constant on both
import spellings before any test module imports ``schedule``. A test that
hard-codes a lead-0 number, or a call site that reads a literal 3, fails here.

``nightly`` on CONTENT: it duplicates suites that are already per-commit at
lead 0 (nothing is demoted); its runtime (minutes, ``-n 3`` inside) is the
reason it is not per-commit. When lead 1 becomes the configured lead, invert
this file to run lead 0 or retire it (design 2(c), emit-lead P1b).
"""
from __future__ import annotations

import os
import subprocess
import sys

import pytest

pytestmark = pytest.mark.nightly

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

#: The suites that read the derived numbers (design section 2(c)).
LEAD_ONE_FILES = [
    'tests/motion/test_skills_schedule.py',
    'tests/motion/test_skills_executor.py',
    'tests/motion/test_skills_install_origin.py',
    'tests/motion/test_skills_rebase_continuity.py',
    'tests/motion/test_skills_bump.py',
    'tests/motion/test_unified_cycle_splice.py',
    'tests/motion/test_cup_capture.py',
    'tests/motion/test_skills_admissible_online.py',
    'tests/motion/test_emit_lead_derivation.py',
    'tests/ros/test_install_segment.py',
    'tests/ros/test_trajectory_node.py',
    'tests/ros/test_skills_plan_bench.py',
]


def test_the_skill_stack_suites_pass_at_emit_lead_one():
    env = dict(os.environ)
    env['JUGGLEBOT_TEST_EMIT_LEAD_KNOTS'] = '1'
    # the nested run is itself a plain, parallel pytest: no nightly tier inside
    proc = subprocess.run(
        [sys.executable, '-m', 'pytest', '-q', '-n', '3', '-m', 'not nightly',
         '-p', 'no:cacheprovider'] + LEAD_ONE_FILES,
        capture_output=True, text=True, env=env, cwd=_REPO, timeout=7200)
    tail = proc.stdout[-3000:] + proc.stderr[-1000:]
    assert proc.returncode == 0, 'emit lead 1 suites failed:\n' + tail
