#!/usr/bin/env bash
# Run the Jugglebot test suite with the correct interpreter.
#
# The system python lacks hypothesis/mcap/scipy and the ROS2 message stubs, so
# tests must run under the PDJ venv. This wrapper exists so the right interpreter
# is discoverable in-repo rather than tribal knowledge.
#
# Usage:
#   ./run_tests.sh                                       # full suite minus tests/hardware (the pre-commit gate)
#   ./run_tests.sh tests/ros                             # a subset
#   ./run_tests.sh tests/ros/test_throw_ballistics.py -v # any pytest args, passed straight through
#
# tests/hardware/ needs a real robot; it is excluded unless you name it explicitly.
set -euo pipefail

VENV_PY="${PDJ_VENV_PY:-$HOME/Desktop/PDJ_venv/venv/bin/python}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -x "$VENV_PY" ]]; then
  echo "error: test interpreter not found at $VENV_PY" >&2
  echo "       set PDJ_VENV_PY to the venv python, or fix the path." >&2
  exit 1
fi

cd "$REPO_ROOT"

if [[ $# -gt 0 ]]; then
  exec "$VENV_PY" -m pytest "$@"
fi

exec "$VENV_PY" -m pytest tests/ -q --ignore=tests/hardware
