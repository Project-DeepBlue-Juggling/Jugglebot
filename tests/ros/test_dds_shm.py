"""BUG-129 experiment: the launch-time Fast DDS shared-memory clean
(``jugglebot/dds_shm.py``) and its two launch switches.

The cleaner runs against a fake ``/dev/shm`` and a fake ``/proc`` under
``tmp_path`` (``locks``, ``<pid>/maps``, ``<pid>/fd``), so nothing here touches
the real shared memory. The file names and the /proc/locks line format are the
ones seen on the Jetson on 2026-11-23 (a live ``ros2 daemon`` held its own
segment ``fastrtps_<16 hex>`` + ``_el`` under an flock and mapped the stale
ports ``fastrtps_port74NN`` of dead participants).

The launch half is a string-level tripwire in the style of
``test_launch_nodes.py``: it cannot run a launch, it pins the wiring.
"""
from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from jugglebot.dds_shm import clean_stale_shm, summary_line

ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PY = ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'launch' / 'jugglebot_launch.py'
PROFILE = ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'resources' / 'fastdds_bridge_udp_only.xml'

LIVE_SEG = 'fastrtps_a4f2ad271dd76bc3'     # locked + open: a live participant's segment
DEAD_SEG = 'fastrtps_08f5a12181f31fbe'     # nobody holds it
MAPPED_PORT = '7412'                       # dead owner, but a live daemon maps it
DEAD_PORT = '7440'


def _fake(tmp_path, age_s=3600.0):
    shm = tmp_path / 'shm'
    proc = tmp_path / 'proc'
    shm.mkdir()
    (proc / '4242' / 'fd').mkdir(parents=True)
    names = [LIVE_SEG, LIVE_SEG + '_el', DEAD_SEG, DEAD_SEG + '_el',
             'fastrtps_port' + MAPPED_PORT, 'sem.fastrtps_port%s_mutex' % MAPPED_PORT,
             'fastrtps_port' + DEAD_PORT, 'fastrtps_port%s_el' % DEAD_PORT,
             'sem.fastrtps_port%s_mutex' % DEAD_PORT,
             'not_fastdds_at_all', 'cyclonedds_x']
    for n in names:
        (shm / n).write_bytes(b'')
        t = (shm / n).stat().st_mtime - age_s
        os.utime(shm / n, (t, t))
    ino = (shm / (LIVE_SEG + '_el')).stat().st_ino
    (proc / 'locks').write_text(
        '15: FLOCK  ADVISORY  WRITE 4242 00:19:%d 0 EOF\n'
        '16: POSIX  ADVISORY  WRITE 4242 b3:01:99999999 0 EOF\n' % ino)
    (proc / '4242' / 'maps').write_text(
        'ffff8000-ffff9000 rw-s 00000000 00:19 1 %s/%s\n'
        'ffff9000-ffffa000 rw-s 00000000 00:19 2 %s/fastrtps_port%s\n'
        'ffffa000-ffffb000 rw-s 00000000 00:19 3 %s/fastrtps_port7437 (deleted)\n'
        % (shm, LIVE_SEG, shm, MAPPED_PORT, shm))
    os.symlink(str(shm / (LIVE_SEG + '_el')), str(proc / '4242' / 'fd' / '7'))
    return shm, proc


def _left(shm):
    return sorted(p.name for p in shm.iterdir())


def test_removes_only_groups_no_live_process_holds(tmp_path):
    shm, proc = _fake(tmp_path)
    r = clean_stale_shm(shm_dir=str(shm), proc=str(proc))
    assert r.note == '' and r.errors == 0
    assert r.removed == 5                        # dead segment (2) + dead port group (3)
    assert r.kept_in_use == 4                    # live segment (2) + mapped port group (2)
    assert _left(shm) == sorted([
        LIVE_SEG, LIVE_SEG + '_el',
        'fastrtps_port' + MAPPED_PORT, 'sem.fastrtps_port%s_mutex' % MAPPED_PORT,
        'not_fastdds_at_all', 'cyclonedds_x'])


def test_a_port_mapped_by_a_live_process_keeps_its_whole_group(tmp_path):
    # Unlinking a port a live daemon still maps would leave it writing into an
    # orphan once a new participant recreates that port number.
    shm, proc = _fake(tmp_path)
    clean_stale_shm(shm_dir=str(shm), proc=str(proc))
    assert (shm / ('sem.fastrtps_port%s_mutex' % MAPPED_PORT)).exists()


def test_recent_files_are_kept(tmp_path):
    shm, proc = _fake(tmp_path, age_s=0.0)
    r = clean_stale_shm(shm_dir=str(shm), proc=str(proc), min_age_s=30.0)
    assert r.removed == 0 and r.kept_other == 5


def test_dry_run_counts_but_removes_nothing(tmp_path):
    shm, proc = _fake(tmp_path)
    before = _left(shm)
    r = clean_stale_shm(shm_dir=str(shm), proc=str(proc), dry_run=True)
    assert r.removed == 5 and _left(shm) == before
    assert summary_line(r, dry_run=True).startswith('shm clean: would remove 5 ')


def test_unreadable_proc_locks_removes_nothing(tmp_path):
    shm, proc = _fake(tmp_path)
    (proc / 'locks').unlink()
    before = _left(shm)
    r = clean_stale_shm(shm_dir=str(shm), proc=str(proc))
    assert r.removed == 0 and 'nothing removed' in r.note and _left(shm) == before


def test_scan_over_its_deadline_removes_nothing(tmp_path):
    shm, proc = _fake(tmp_path)
    before = _left(shm)
    r = clean_stale_shm(shm_dir=str(shm), proc=str(proc), deadline_s=-1.0)
    assert r.removed == 0 and 'nothing removed' in r.note and _left(shm) == before


def test_never_raises_on_a_missing_directory(tmp_path):
    r = clean_stale_shm(shm_dir=str(tmp_path / 'absent'), proc=str(tmp_path))
    assert r.removed == 0 and r.note
    assert summary_line(r).startswith('shm clean: removed 0 ')


# ── launch wiring (string-level) ─────────────────────────────────

@pytest.fixture(scope='module')
def launch_src():
    return LAUNCH_PY.read_text()


@pytest.mark.parametrize('arg', ['shm_clean', 'bridge_shm'])
def test_switch_is_declared_default_true(launch_src, arg):
    assert re.search(r"DeclareLaunchArgument\(\s*'%s',\s*default_value='true'" % arg,
                     launch_src), arg


def test_clean_runs_after_the_arguments_and_before_the_first_node(launch_src):
    body = re.search(r'return\s+LaunchDescription\(\s*\[(.*?)\]\s*\)', launch_src, re.S).group(1)
    names = re.findall(r'^\s*\*?(\w+)', body, re.M)
    assert names.index('shm_actions') > names.index('bridge_shm_arg')
    assert names.index('shm_actions') > names.index('shm_clean_arg')
    assert names.index('shm_actions') < names.index('rosbridge_websocket_node')


def test_only_the_bridge_can_get_the_udp_profile(launch_src):
    # One code occurrence of the variable name, inside the bridge_shm == 'false'
    # branch of the bridge's env; never a launch-wide SetEnvironmentVariable.
    code = [ln for ln in launch_src.splitlines()
            if 'FASTRTPS_DEFAULT_PROFILES_FILE' in ln and not ln.lstrip().startswith('#')]
    assert len(code) == 1 and "'FASTRTPS_DEFAULT_PROFILES_FILE' if '" in code[0], code
    assert 'SetEnvironmentVariable' not in launch_src
    assert "\"' == 'false' else 'JUGGLEBOT_BRIDGE_UDP_PROFILE_UNUSED'\"" in launch_src
    assert 'additional_env=_bridge_env' in launch_src


def test_udp_profile_disables_builtin_transports_and_uses_udpv4_only():
    ns = {'p': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(str(PROFILE)).getroot()
    types = [t.text for t in root.findall('.//p:transport_descriptor/p:type', ns)]
    assert types == ['UDPv4']
    part = root.find('p:participant', ns)
    assert part.get('is_default_profile') == 'true'
    assert part.find('p:rtps/p:useBuiltinTransports', ns).text == 'false'
    ids = [t.text for t in part.findall('p:rtps/p:userTransports/p:transport_id', ns)]
    assert ids == [root.find('.//p:transport_descriptor/p:transport_id', ns).text]
