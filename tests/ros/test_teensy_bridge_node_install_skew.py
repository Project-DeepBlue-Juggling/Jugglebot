"""Install-skew self-check on teensy_bridge_node (2026-08-14).

WHY THIS EXISTS. On 2026-08-14 the operator flashed can-bridge FW 13 and ran the
S3 conviction soak. The bag recorded no ``/ring_diag`` at all: ``colcon build``
had not taken effect, so the node ran from a two-day-old ``install/`` tree that
predates the RING_DIAG subscribe. The whole time, ``BRIDGE_FW_CHECK`` logged OK
— correctly, and that is the trap. It compares the board's reported
``FW_VERSION`` against ``rpc_args.EXPECTED_BRIDGE_FW_VERSION``, which is read
from the LIVE repo-root ``teensy_link/`` tree (deliberately NOT colcon-installed;
the launch injects the repo root on PYTHONPATH). Firmware currency and HOST
currency are two independent halves of "is what is running what the repo says?",
and only the firmware half had a detector.

``jugglebot_launch.py`` already carries the sibling check for the two GENERATED
CONFIG modules (``_install_drift``). Its scope is exactly ``hardware_config.py``
and ``protocol_config.py``, so a stale node source with unchanged config passes
it green — which is precisely what happened.

What this module pins:

1. **the verdict is three-valued** — clean / stale / inconclusive, never
   collapsed. A check that could not run must not read like one that ran and
   agreed;
2. **a stale colcon layout is convicted**, and the message names BOTH paths and
   BOTH mtimes (which side is behind is the difference between "run colcon
   build" and "your checkout is stale");
3. **an absent source tree is inconclusive, not clean** — a deploy without a
   checkout must not manufacture reassurance;
4. **it never raises** — it runs inside the node constructor;
5. **the verdict is persistent on /link_status**, not only in a startup log:
   ``/rosout`` recording races node startup, so a bag can otherwise carry no
   trace of which build produced it — the 2026-08-14 failure exactly;
6. **BRIDGE_FW_CHECK carries the host half**, on the OK line too (a green
   firmware verdict beside a silent host verdict is the original trap's shape);
7. **it gates nothing** — advisory, like BRIDGE_FW_CHECK and the launch banner.

ROS 2 is mocked by tests/ros/conftest.py. All filesystem work is under
``tmp_path`` and no fixed port is bound, so this file is xdist-parallel-safe.
"""

from __future__ import annotations

import os
import re
from unittest.mock import MagicMock

from rclpy.node import Node as _RclpyNode   # the conftest mock, not real Foxy

import jugglebot.teensy_bridge_node as tbn
from jugglebot.teensy_bridge_node import (
    INSTALL_SKEW_CLEAN,
    INSTALL_SKEW_STALE,
    INSTALL_SKEW_UNKNOWN,
    install_skew_verdict,
)

from teensy_link import BridgeIdentity, MsgType
from teensy_link import rpc_args

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _messages,
    _teardown,
    _wait_until,
)

_INSTALL_REL = ('ros_ws', 'install', 'jugglebot', 'lib', 'python3.8',
                'site-packages', 'jugglebot', 'teensy_bridge_node.py')
_SRC_REL = ('ros_ws', 'src', 'jugglebot', 'jugglebot', 'teensy_bridge_node.py')


def _write(path, data: bytes, mtime: float | None = None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(data)
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def _fake_repo(tmp_path, *, running=b'installed\n', source=b'installed\n',
               with_source=True, running_mtime=None, source_mtime=None):
    """Build a repo-shaped tree: a colcon install copy plus (maybe) the source.

    The layout is the real one — ``<repo>/ros_ws/install/.../jugglebot/`` beside
    ``<repo>/ros_ws/src/jugglebot/jugglebot/`` — because the resolution under
    test is a walk UP from the running file, and a flat fixture would prove
    nothing about it.
    """
    repo = os.path.join(str(tmp_path), 'repo')
    run_path = _write(os.path.join(repo, *_INSTALL_REL), running, running_mtime)
    src_path = (_write(os.path.join(repo, *_SRC_REL), source, source_mtime)
                if with_source else None)
    return repo, run_path, src_path


def _node():
    return _build_paired_node(boot_state_read=False)


def _logging_node(monkeypatch):
    """A node whose CONSTRUCTOR logs are captured.

    The harness's ``_logging_node`` swaps the logger AFTER construction, which
    is too late for a verdict logged inside ``__init__``. Patching the mock
    Node's ``get_logger`` is reverted by monkeypatch, and xdist workers are
    separate processes running their tests sequentially, so it cannot leak.
    """
    rec = MagicMock()
    monkeypatch.setattr(_RclpyNode, 'get_logger', lambda self: rec)
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node, rec


def _send_identity(teensy, node, fw_version, protocol_version=5):
    bi = BridgeIdentity(fw_version=fw_version, protocol_version=protocol_version)
    teensy.send_to_jetson(int(MsgType.BRIDGE_IDENTITY), bi.pack())
    assert _wait_until(
        lambda: node._latest_bridge_identity is not None
        and int(node._latest_bridge_identity.fw_version) == int(fw_version))
    return bi


# ══════════════════════════════════════════════════════════════════════════════
#  The verdict function
# ══════════════════════════════════════════════════════════════════════════════

def test_a_matching_install_copy_reads_clean(tmp_path, monkeypatch):
    """Same bytes at both paths ⇒ '0', with the hash named.

    Byte equality — not mtime — is the criterion: colcon copies the .py verbatim
    and every build restamps the mtime, so an mtime comparison would cry wolf on
    every rebuild and be ignored within a week.
    """
    monkeypatch.delenv('JUGGLEBOT_REPO', raising=False)
    _repo, run_path, src_path = _fake_repo(tmp_path)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_CLEAN, detail
    assert run_path in detail and src_path in detail
    assert re.search(r'sha256=[0-9a-f]{16}', detail), detail


def test_a_stale_install_is_convicted_and_names_both_sides(tmp_path, monkeypatch):
    """The incident, reproduced in a fixture: install ≠ source ⇒ '1'.

    Both paths AND both mtimes are asserted because the verdict alone does not
    tell the operator what to DO. Running-newer means the checkout is behind;
    source-newer (the 2026-08-14 case) means colcon build did not take.
    """
    monkeypatch.delenv('JUGGLEBOT_REPO', raising=False)
    _repo, run_path, src_path = _fake_repo(
        tmp_path, running=b'two days old\n', source=b'todays edit\n',
        running_mtime=1_755_000_000.0, source_mtime=1_755_170_000.0)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_STALE, detail
    assert 'DIFFERS' in detail
    assert run_path in detail and src_path in detail
    stamps = re.findall(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[-+]\d{4}', detail)
    assert len(stamps) == 2, detail
    assert stamps[0] != stamps[1], (
        'both mtimes must be reported so the operator can tell which side is '
        'behind')


def test_an_absent_source_tree_is_inconclusive_never_clean(tmp_path, monkeypatch):
    """A deploy with no checkout ⇒ 'unknown', and it says so.

    Returning '0' here would be the worst possible answer: the row exists to
    prove currency was CHECKED, and an unrunnable check that renders clean is
    exactly the false reassurance BRIDGE_FW_CHECK gave on 2026-08-14.
    """
    monkeypatch.delenv('JUGGLEBOT_REPO', raising=False)
    _repo, run_path, _src = _fake_repo(tmp_path, with_source=False)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_UNKNOWN, detail
    assert 'NOT checked' in detail
    assert run_path in detail


def test_the_verdict_never_raises_on_an_unreadable_running_file(tmp_path,
                                                                monkeypatch):
    """It runs inside the constructor — a vanished file must not stop boot."""
    monkeypatch.delenv('JUGGLEBOT_REPO', raising=False)
    repo, run_path, _src = _fake_repo(tmp_path)
    os.remove(run_path)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_UNKNOWN, detail
    assert 'FileNotFoundError' in detail, detail
    assert repo  # the source tree resolved; only the read failed


def test_the_repo_override_pins_the_comparison_tree(tmp_path, monkeypatch):
    """``JUGGLEBOT_REPO`` wins over the walk-up, as in jugglebot_launch.

    Parallel sessions here isolate with git worktrees; without an override a
    deploy whose install sits outside its repo has no way to name its source.
    """
    other = os.path.join(str(tmp_path), 'other')
    _write(os.path.join(other, *_SRC_REL), b'a different tree\n')
    _repo, run_path, _src = _fake_repo(tmp_path)          # would read clean
    monkeypatch.setenv('JUGGLEBOT_REPO', other)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_STALE, detail
    assert other in detail


def test_an_invalid_override_yields_unknown_never_a_fallthrough(tmp_path, monkeypatch):
    """A typo'd ``JUGGLEBOT_REPO`` must NOT silently verdict a different tree.

    The override is authoritative: if the pinned tree lacks the source copy,
    the honest answer is ``unknown`` naming the override — not a
    plausible-looking CLEAN computed against whatever the walk-up finds. A
    CLEAN the operator reads as "my pinned tree was checked" is the exact
    manufactured-reassurance class this feature exists to close.
    """
    _repo, run_path, _src = _fake_repo(tmp_path)          # walk-up WOULD succeed
    bogus = os.path.join(str(tmp_path), 'no-such-checkout')
    monkeypatch.setenv('JUGGLEBOT_REPO', bogus)
    verdict, detail = install_skew_verdict(run_path)
    assert verdict == INSTALL_SKEW_UNKNOWN, detail
    assert 'NOT checked' in detail or 'not checked' in detail.lower()


def test_running_live_from_the_source_tree_reads_clean(monkeypatch):
    """The default call, against this repo: the tests import from src.

    Not a tautology — it pins that a live-tree run (the developer's and the
    test suite's normal case, and what the launch's PYTHONPATH gives
    ``teensy_link``) resolves to itself and reports CLEAN rather than falling
    through to 'unknown'.
    """
    monkeypatch.delenv('JUGGLEBOT_REPO', raising=False)
    verdict, detail = install_skew_verdict()
    assert verdict == INSTALL_SKEW_CLEAN, detail
    assert os.path.abspath(tbn.__file__) in detail


# ══════════════════════════════════════════════════════════════════════════════
#  The node — startup log, /link_status rows, BRIDGE_FW_CHECK
# ══════════════════════════════════════════════════════════════════════════════

def test_a_stale_install_logs_loudly_at_startup(monkeypatch):
    """ERROR, greppable token, and the fix spelled out."""
    monkeypatch.setattr(tbn, 'install_skew_verdict',
                        lambda *a, **k: (INSTALL_SKEW_STALE, 'DETAIL-HERE'))
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        errs = [m for m in _messages(rec.error) if 'INSTALL_SKEW' in m]
        assert len(errs) == 1, _messages(rec.error)
        assert 'INSTALL_SKEW: FAIL' in errs[0]
        assert 'DETAIL-HERE' in errs[0]
        assert 'colcon build' in errs[0]
        # The point of the line: it names the detector that stayed green.
        assert 'BRIDGE_FW_CHECK' in errs[0]
        assert 'advisory' in errs[0].lower()
    finally:
        _teardown(teensy, client, node)


def test_an_inconclusive_check_warns_and_does_not_claim_freshness(monkeypatch):
    monkeypatch.setattr(tbn, 'install_skew_verdict',
                        lambda *a, **k: (INSTALL_SKEW_UNKNOWN, 'no source tree'))
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        warns = [m for m in _messages(rec.warning) if 'INSTALL_SKEW' in m]
        assert len(warns) == 1, _messages(rec.warning)
        assert 'INCONCLUSIVE' in warns[0]
        assert 'UNVERIFIED' in warns[0]
        # Never an ERROR: a check that could not run is a note, not drift.
        assert not [m for m in _messages(rec.error) if 'INSTALL_SKEW' in m]
    finally:
        _teardown(teensy, client, node)


def test_a_clean_check_announces_itself_at_info(monkeypatch):
    """Silence on the clean path would leave 'no line' and 'not checked'
    indistinguishable in a log."""
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        oks = [m for m in _messages(rec.info) if 'INSTALL_SKEW' in m]
        assert len(oks) == 1, _messages(rec.info)
        assert 'INSTALL_SKEW: OK' in oks[0]
        assert not [m for m in _messages(rec.error) if 'INSTALL_SKEW' in m]
    finally:
        _teardown(teensy, client, node)


def test_link_status_carries_the_token_and_the_detail():
    """The bag's copy of the verdict — a bare token plus its explanation."""
    teensy, client, node = _node()
    try:
        kv = _link_kv(node)
        assert kv['install_skew'] in (INSTALL_SKEW_CLEAN, INSTALL_SKEW_STALE,
                                      INSTALL_SKEW_UNKNOWN)
        assert kv['install_skew'] == node._install_skew
        assert kv['install_skew_detail'] == node._install_skew_detail
        # Beside the firmware half, not somewhere else in the row list: the two
        # answer one question and an operator reads them together.
        keys = [v.key for v in node.link_status_pub.published[-1].values]
        assert abs(keys.index('install_skew')
                   - keys.index('bridge_fw_version')) == 1
    finally:
        _teardown(teensy, client, node)


def test_a_stale_verdict_reaches_the_bag_as_a_bare_one():
    """'1' exactly — a bag consumer must be able to compare, not parse."""
    teensy, client, node = _node()
    try:
        node._install_skew = INSTALL_SKEW_STALE
        node._install_skew_detail = 'running X DIFFERS from repo source Y'
        kv = _link_kv(node)
        assert kv['install_skew'] == '1'
        assert 'DIFFERS' in kv['install_skew_detail']
    finally:
        _teardown(teensy, client, node)


def test_the_row_is_published_even_with_no_heartbeat():
    """Never heartbeat-gated: a bridge that never answers is exactly when the
    operator most needs to know whether this node is the build they think."""
    teensy, client, node = _node()
    try:
        assert node._latest_heartbeat is None
        assert 'install_skew' in _link_kv(node)
    finally:
        _teardown(teensy, client, node)


def test_the_fw_check_ok_line_names_the_host_half(monkeypatch):
    """The two halves of host currency appear TOGETHER, on the OK line.

    The 2026-08-14 miss was an operator reading a green firmware verdict as an
    answer to both questions; a line that answers one silently invites it again.
    """
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION)
        ok = [m for m in _messages(rec.info) if 'BRIDGE_FW_CHECK: OK' in m]
        assert len(ok) == 1, _messages(rec.info)
        assert 'install_skew=0' in ok[0], ok[0]
    finally:
        _teardown(teensy, client, node)


def test_the_fw_check_fail_line_also_names_the_host_half(monkeypatch):
    """A skewed board and an unverified host build are separate diagnoses and
    both belong on the line the operator greps for."""
    monkeypatch.setattr(tbn, 'install_skew_verdict',
                        lambda *a, **k: (INSTALL_SKEW_UNKNOWN, 'no source tree'))
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION - 1)
        errs = [m for m in _messages(rec.error) if 'BRIDGE_FW_CHECK: FAIL' in m]
        assert len(errs) == 1, _messages(rec.error)
        assert 'install_skew=unknown' in errs[0], errs[0]
        # The firmware verdict's own content is untouched by the addition.
        assert 'OLDER than' in errs[0] and 'NOT refused' in errs[0]
    finally:
        _teardown(teensy, client, node)


def test_a_stale_install_is_called_out_on_an_otherwise_green_fw_line(monkeypatch):
    """The exact 2026-08-14 shape: right firmware, wrong node build."""
    monkeypatch.setattr(tbn, 'install_skew_verdict',
                        lambda *a, **k: (INSTALL_SKEW_STALE, 'running X DIFFERS'))
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION)
        ok = [m for m in _messages(rec.info) if 'BRIDGE_FW_CHECK: OK' in m]
        assert len(ok) == 1
        assert 'install_skew=1' in ok[0], ok[0]
        assert 'stale colcon install' in ok[0]
    finally:
        _teardown(teensy, client, node)


def test_the_verdict_gates_nothing(monkeypatch):
    """Advisory, exactly like BRIDGE_FW_CHECK and the launch's drift banner.

    A stale install is a CONFUSION failure, not a dangerous one — build-frozen
    is the safe direction — and a hash comparison in front of the leg/hand
    command path would convert a reporting gap into an outage. So: same
    DiagnosticStatus level, no fault, no deferred stow.
    """
    monkeypatch.setattr(tbn, 'install_skew_verdict',
                        lambda *a, **k: (INSTALL_SKEW_STALE, 'running X DIFFERS'))
    teensy, client, node, rec = _logging_node(monkeypatch)
    try:
        _link_kv(node)
        stale_level = node.link_status_pub.published[-1].level
        assert node._link_latch.stow_pending is False
        node._install_skew = INSTALL_SKEW_CLEAN
        _link_kv(node)
        assert node.link_status_pub.published[-1].level == stale_level, (
            'the install verdict must not move the link level')
        # Nothing latched a fault or armed anything off the back of it.
        assert node._last_fault_state is None
    finally:
        _teardown(teensy, client, node)
