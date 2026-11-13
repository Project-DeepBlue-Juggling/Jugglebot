"""The Night Shift deny list, the no-network wrapper and the app suite's log.

A deny rule beats every allow, so the list has to be right in both directions: a rule
too narrow lets a flash through (``platformio run`` was not matched until 2026-11-07),
and a rule too broad blocks reading and testing (``*teensy_link*`` refused greps and the
bridge's own loopback tests while the night's gate ran those same tests anyway).
``_denied`` is a model of the real matcher (fnmatch over the whole line; compound commands
are not split), so these tests pin the rules' intent. The real matcher was probed against
this list's shape on 2026-10-31 (the plan's T-I2) and every night's ``permission_denials``
shows it working. Running tests without the wrapper stays possible and is not refused: no
pattern can demand a prefix, which is why the protocol, not this list, requires it.
"""
from __future__ import annotations

import fnmatch
import json
import os
import shutil
import socket
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SETTINGS = os.path.join(ROOT, "tools", "night_shift_settings.json")
NONET = os.path.join(ROOT, "tools", "night_shift_nonet.sh")
SUITE = os.path.join(ROOT, "tools", "night_shift_app_suite.sh")


def _denied(command):
    deny = json.load(open(SETTINGS))["permissions"]["deny"]
    for rule in deny:
        if rule.startswith("Bash(") and fnmatch.fnmatchcase(command, rule[5:-1]):
            return rule
    return None


@pytest.mark.parametrize("command", [
    "pio run -e teensy41 -t upload",
    "cd x; pio run",
    "platformio run -t upload",
    "python -m platformio run",
    "pio test",
    "pio test -e esp32dev",
    "platformio test",
    "ros2 run jugglebot teensy_bridge_node",
    "ros2 launch jugglebot jugglebot_launch.py",
    "python ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py",
    "python -m jugglebot.teensy_bridge_node",
    "python -m teensy_link",
    'python -c "import teensy_link"',
    'python3 -c "from teensy_link import TeensyLinkClient; TeensyLinkClient()"',
    "python tools/teensy_link_bridge.py",
    "cd tools && python teensy_link_bridge.py",
    "cd ros_ws/src/jugglebot/jugglebot && python teensy_bridge_node.py",
    "cd ros_ws/src/jugglebot/jugglebot && python3 teensy_bridge_node.py",
    "python tools/night_shift_deploy.py --date x",
    "tools/night_shift_deploy.py --date x",
    "./tools/night_shift_deploy.py --date x",
    "git -C ~/night-shift/jugglebot-map push --force origin main",
    "git -C ~/night-shift/jugglebot-map push -f origin main",
    "git -C ~/night-shift/jugglebot-map push origin +main",
    "git -C ~/night-shift/jugglebot-map push origin HEAD:main",
    "git -C ~/night-shift/jugglebot-map push origin --delete main",
    "git -C ~/night-shift/jugglebot-map push -d origin main",
    "bash deploy-jetson.sh",
    "systemctl --user restart jugglebot-map",
    "git push origin --delete night-shift/x",
    "git push --force origin main",
    "ls ~/map-live/jugglebot-map",
    "cat ~/Desktop/Jugglebot/x",
    "python tests/hardware/free_platform_test.py",
    "colcon build",
    # Probes and bag reads outside the wrapper (closeout C7a): the unwrapped spellings.
    "python tools/probes/feed_catch_bag_probe.py --bag x",
    "python3 tools/probes/feed_catch_bag_probe.py",
    "python -m tools.probes.feed_catch_bag_probe",
    "tools/probes/x.sh",
    "./tools/probes/x.py",
    "bash tools/probes/x.sh",
    "cd tools/probes && python feed_catch_bag_probe.py",
    "timeout 600 python tools/probes/x.py",
    "env PYTHONPATH=. python tools/probes/x.py",
    "nice -n 10 python tools/probes/x.py",
    "cd ~/night-shift/Jugglebot && python tools/probes/x.py",
    "cd x; python tools/probes/x.py",
    "true | python tools/probes/x.py",
    "ros2 bag info ~/Desktop/rosbags/x",
    "cd x && ros2 bag info y",
    # ...and the bag verbs that write or publish, refused even wrapped.
    "tools/night_shift_nonet.sh ros2 bag play x",
    "tools/night_shift_nonet.sh ros2 bag record -a",
    "tools/night_shift_nonet.sh ros2 bag reindex x",
    "tools/night_shift_nonet.sh ros2 bag convert -i x -o y",
    # A cd line that names Desktop is refused (pre-existing rule): run from the clone root.
    "cd ~/night-shift/Jugglebot && tools/night_shift_nonet.sh python tools/probes/x.py"
    " --bag ~/Desktop/rosbags/x",
])
def test_refused(command):
    assert _denied(command), command


@pytest.mark.parametrize("command", [
    "python -m pytest tests/ros/test_teensy_bridge_node_read.py -q",
    "python -m pytest --collect-only tests/teensy_link/ -q",
    "tools/night_shift_nonet.sh python -m pytest tests/teensy_link/test_client.py -q",
    "tools/night_shift_nonet.sh ./run_tests.sh --full",
    "tools/night_shift_nonet.sh python -m pytest tests/firmware/test_wave_native.py -q",
    "grep -rn teensy_link teensy_link/ ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py",
    "git diff origin/main...HEAD -- ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py",
    "git add ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py",
    "grep -n env ros_ws/src/jugglebot/WaveCamera_ESP_code/platformio.ini",
    "cd ~/night-shift/jugglebot-map && npm run test -- --run",
    "tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app A --mapkit B --log C",
    "git -C ~/night-shift/jugglebot-map push -q origin main",
    "git -C ~/night-shift/jugglebot-map push -q -u origin night-shift/2026-11-08-BUG-7",
    "python -m pytest tests/sim/test_night_shift_deploy.py -q",
    "grep -n check tools/night_shift_deploy.py",
    "git archive origin/main tools/night_shift_gate.py mapkit | tar -x -C D/gate-base",
    "python D/gate-base/tools/night_shift_gate.py --repo . --base origin/main",
    # Probes and bag reads under the wrapper, and reading the probes (closeout C7a).
    "tools/night_shift_nonet.sh python tools/probes/feed_catch_bag_probe.py"
    " --bag ~/Desktop/rosbags/x --out temp/probes/y.json",
    "D/gate-base/tools/night_shift_nonet.sh timeout 900 python tools/probes/x.py",
    "/home/jetson/night-shift/r/gate-base/tools/night_shift_nonet.sh python3 tools/probes/x.py",
    "cd ~/night-shift/Jugglebot && tools/night_shift_nonet.sh python tools/probes/x.py",
    "tools/night_shift_nonet.sh env PYTHONPATH=. python tools/probes/x.py",
    "tools/night_shift_nonet.sh ros2 bag info ~/Desktop/rosbags/x",
    "tools/night_shift_nonet.sh python -m pytest tests/motion/test_levelling_probe.py -q",
    "grep -rln socket tools/probes/",
    "ls ~/Desktop/rosbags",
    "git diff origin/main...HEAD -- tools/probes/x.py",
    "git add tools/probes/x.py temp/probes",
])
def test_allowed(command):
    assert _denied(command) is None, _denied(command)


def _nonet_works():
    return shutil.which("unshare") and subprocess.run(
        [NONET, "true"], capture_output=True).returncode == 0


needs_nonet = pytest.mark.skipif(not _nonet_works(), reason="no user namespaces on this box")


@needs_nonet
def test_nonet_cannot_reach_the_network_but_has_loopback():
    code = (
        "import socket,sys\n"
        "a=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);a.bind(('127.0.0.1',0))\n"
        "b=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);b.sendto(b'x',a.getsockname())\n"
        "a.settimeout(5);assert a.recv(4)==b'x'\n"
        "c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)\n"
        "try:\n"
        "    c.sendto(b'x',('192.168.42.2',5005))\n"
        "except OSError as e:\n"
        "    print('unreachable',e.errno);sys.exit(0)\n"
        "sys.exit(9)\n")
    r = subprocess.run([NONET, sys.executable, "-c", code], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.startswith("unreachable")


@needs_nonet
def test_nonet_passes_the_exit_code_and_arguments():
    r = subprocess.run([NONET, "bash", "-c", 'echo "$1"; exit 7', "_", "a b"],
                       capture_output=True, text=True)
    assert r.returncode == 7 and r.stdout == "a b\n"


def test_nonet_fails_closed_when_no_namespace_can_be_made(tmp_path):
    fake = tmp_path / "unshare"
    fake.write_text("#!/usr/bin/env bash\nexit 1\n")
    fake.chmod(0o755)
    marker = tmp_path / "ran"
    env = dict(os.environ, PATH="%s%s%s" % (tmp_path, os.pathsep, os.environ["PATH"]))
    r = subprocess.run([NONET, "touch", str(marker)], env=env, capture_output=True)
    assert r.returncode != 0 and not marker.exists()


def test_nonet_needs_a_command():
    assert subprocess.run([NONET], capture_output=True).returncode == 2


# --- evidence read-only, serial masked (closeout C7a) -----------------------
# Stand-ins under tmp_path go in through the EXTRA_* variables, which only ADD paths:
# the defaults (~/Desktop/rosbags, ~/.ros/log, /dev/ttyACM*, /dev/ttyUSB*, /dev/serial)
# apply on every run, so these tests also check the real ones without writing to them.
_INSIDE = r'''
set -u
bags=$1 log=$2 tty=$3 serial=$4
r=0
no() { echo "FAIL $*"; r=1; }
erofs() { [[ $("$@" 2>&1) == *"Read-only file system"* ]] || no "not EROFS: $*"; }
erofs touch "$bags/run1/new"
erofs sh -c "echo y >> '$bags/run1/x.mcap'"
erofs mkdir "$log/new"
[[ $(cat "$bags/run1/x.mcap") == evidence ]] || no "bag unreadable"
umount "$bags" 2>/dev/null && no "umount of the read-only bind succeeded"
mount -o remount,bind,rw "$bags" 2>/dev/null && no "remount rw succeeded"
erofs touch "$bags/run1/new"
[[ -z $(cat "$tty") ]] || no "tty stand-in readable"
echo poke > "$tty" || no "write to masked tty failed"
umount "$tty" 2>/dev/null && no "umount of the tty mask succeeded"
[[ -z $(ls -A "$serial") ]] || no "serial dir not empty"
erofs touch "$serial/x"
touch "$ROS_LOG_DIR/x" || no "ROS_LOG_DIR not writable"
for dev in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e $dev ]] || continue
  [[ $(stat -Lc %t:%T "$dev") == 1:3 ]] || no "$dev not masked"
done
[[ ! -e /dev/serial ]] || [[ -z $(ls -A /dev/serial) ]] || no "/dev/serial not empty"
python3 - <<'PY' || r=1
import os
for d in ("~/Desktop/rosbags", "~/.ros/log"):
    d = os.path.expanduser(d)
    if os.path.exists(d) and os.access(d, os.W_OK):
        print("FAIL writable", d)
        raise SystemExit(1)
PY
exit $r
'''


@pytest.fixture
def evidence(tmp_path):
    bags, log, serial = tmp_path / "rosbags", tmp_path / "roslog", tmp_path / "serial"
    (bags / "run1").mkdir(parents=True)
    (bags / "run1" / "x.mcap").write_text("evidence")
    log.mkdir()
    (serial / "by-id").mkdir(parents=True)
    tty = tmp_path / "ttyFAKE0"
    tty.write_text("SECRET")
    env = dict(os.environ,
               NIGHT_SHIFT_NONET_EXTRA_RO="%s:%s" % (bags, log),
               NIGHT_SHIFT_NONET_EXTRA_MASK=str(tmp_path / "ttyFAKE*"),
               NIGHT_SHIFT_NONET_EXTRA_EMPTY=str(serial))
    env.pop("ROS_LOG_DIR", None)
    return env, bags, log, tty, serial


@needs_nonet
def test_nonet_evidence_is_read_only_and_serial_is_masked(evidence):
    env, bags, log, tty, serial = evidence
    r = subprocess.run([NONET, "bash", "-c", _INSIDE, "_", str(bags), str(log), str(tty),
                        str(serial)], env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    # Nothing leaked to the host side.
    assert sorted(os.listdir(str(bags / "run1"))) == ["x.mcap"]
    assert (bags / "run1" / "x.mcap").read_text() == "evidence"
    assert os.listdir(str(log)) == [] and tty.read_text() == "SECRET"
    assert os.listdir(str(serial)) == ["by-id"]


def test_nonet_fails_closed_when_a_mount_fails(evidence, tmp_path):
    if not _nonet_works():
        pytest.skip("no user namespaces on this box")
    env, _, _, _, _ = evidence
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    fake = bin_dir / "mount"
    fake.write_text("#!/usr/bin/env bash\nexit 32\n")
    fake.chmod(0o755)
    env["PATH"] = "%s%s%s" % (bin_dir, os.pathsep, env["PATH"])
    marker = tmp_path / "ran"
    r = subprocess.run([NONET, "touch", str(marker)], env=env, capture_output=True, text=True)
    assert r.returncode == 125 and not marker.exists(), r.stderr
    assert "night_shift_nonet: cannot bind" in r.stderr


# --- the app suite's log ----------------------------------------------------
def _git(cwd, *args):
    subprocess.run(["git"] + list(args), cwd=str(cwd), check=True, capture_output=True)


@pytest.fixture
def app(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for k in ("GIT_AUTHOR_NAME", "GIT_COMMITTER_NAME"):
        monkeypatch.setenv(k, "t")
    for k in ("GIT_AUTHOR_EMAIL", "GIT_COMMITTER_EMAIL"):
        monkeypatch.setenv(k, "t@example.com")
    root = tmp_path / "app"
    (root / "frontend").mkdir(parents=True)
    (root / "scripts").mkdir()
    for s in ("check_no_upstream_name.sh", "contract_check.sh"):
        (root / "scripts" / s).write_text('echo "%s MAPKIT_SRC=$MAPKIT_SRC"\n' % s)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for tool in ("python", "npm"):
        p = bin_dir / tool
        p.write_text('#!/usr/bin/env bash\necho "%s $*"\n'
                     '[[ "$*" == "${FAIL_ON:-none}" ]] && exit 4\nexit 0\n' % tool)
        p.chmod(0o755)
    monkeypatch.setenv("PATH", "%s%s%s" % (bin_dir, os.pathsep, os.environ["PATH"]))
    _git(root, "init", "-q")
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "-m", "seed")
    mapkit = tmp_path / "mapkit_clone"
    mapkit.mkdir()
    _git(mapkit, "init", "-q")
    _git(mapkit, "commit", "-q", "--allow-empty", "-m", "seed")
    return root, mapkit, tmp_path / "app.log"


def _head(repo):
    return subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"], check=True,
                          capture_output=True, text=True).stdout.strip()


def test_app_suite_log_has_what_the_gate_reads(app):
    root, mapkit, log = app
    r = subprocess.run([SUITE, "--app", str(root), "--mapkit", str(mapkit), "--log", str(log)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    lines = log.read_text().splitlines()
    assert lines[0] == "HEAD=%s" % _head(root)
    assert lines[1].startswith("BRANCH=")
    assert lines[2] == "MAPKIT_HEAD=%s" % _head(mapkit)
    assert lines[-1] == "EXIT=0"
    text = log.read_text()
    for step in ("python -m pytest -q", "npm run lint", "npm run typecheck", "npm run test -- --run",
                 "npm run build", "check_no_upstream_name", "contract_check"):
        assert step in text
    assert "MAPKIT_SRC=%s" % mapkit in text


def test_app_suite_runs_every_step_and_reports_the_first_failure(app, monkeypatch):
    root, mapkit, log = app
    monkeypatch.setenv("FAIL_ON", "run lint")
    r = subprocess.run([SUITE, "--app", str(root), "--mapkit", str(mapkit), "--log", str(log)],
                       capture_output=True, text=True)
    assert r.returncode == 4
    text = log.read_text()
    assert text.splitlines()[-1] == "EXIT=4"
    assert "npm run build" in text and "contract_check" in text   # the later steps still ran


def test_app_suite_usage():
    assert subprocess.run([SUITE], capture_output=True).returncode == 2
