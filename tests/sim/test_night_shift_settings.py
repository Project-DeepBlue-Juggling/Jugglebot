"""The Night Shift deny list, the Bash guard hook, the no-network wrapper and the app suite's log.

A deny rule beats every allow, so the list has to be right in both directions: a rule
too narrow lets a flash through (``platformio run`` was not matched until 2026-11-07),
and a rule too broad blocks reading and testing (``*teensy_link*`` refused greps and the
bridge's own loopback tests while the night's gate ran those same tests anyway).
``_denied`` is a model of the real matcher (fnmatch over the whole line; compound commands
are not split), so these tests pin the rules' intent. The real matcher was probed against
this list's shape on 2026-10-31 (the plan's T-I2) and every night's ``permission_denials``
shows it working. Running tests without the wrapper stays possible and is not refused: no
pattern can demand a prefix, which is why the protocol, not this list, requires it.

Probes, bag reads and the evidence directories are different: no pattern can demand the
wrapper prefix, so tools/night_shift_bash_guard.py, a PreToolUse hook, is the one
enforcement point (closeout C7a). Its tests below run the real script over a table of
bypass spellings, accepted shapes and malformed input, and run the settings file's own
hook command line against a throwaway repo with an ``origin/main``.
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
GUARD = os.path.join(ROOT, "tools", "night_shift_bash_guard.py")


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
    # Probes and bag reads outside the wrapper (closeout C7a): defence in depth only, the
    # few line-start spellings. The enforcement point is the hook (tests further down).
    "python tools/probes/feed_catch_bag_probe.py --bag x",
    "python3 tools/probes/feed_catch_bag_probe.py",
    "tools/probes/x.sh",
    "./tools/probes/x.py",
    "cd tools/probes && python feed_catch_bag_probe.py",
    "ros2 bag info ~/Desktop/rosbags/x",
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
    "tools/night_shift_nonet.sh grep -rln socket tools/probes/",
    "tools/night_shift_nonet.sh git add tools/probes/x.py temp/probes",
])
def test_allowed(command):
    assert _denied(command) is None, _denied(command)


# --- the Bash guard hook: the one enforcement point for probes and bags (C7a) -----------
_W = "tools/night_shift_nonet.sh"


@pytest.fixture(scope="module")
def project(tmp_path_factory):
    """A repo whose origin/main holds the real wrapper and guard, plus a D/gate-base copy
    of the wrapper (trusted: same bytes) and a branch-edited one (not trusted)."""
    base = tmp_path_factory.mktemp("guard")
    repo = base / "clone"
    (repo / "tools" / "probes").mkdir(parents=True)
    for src in (NONET, GUARD):
        shutil.copy(src, str(repo / "tools" / os.path.basename(src)))
    env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1",
               GIT_AUTHOR_NAME="t", GIT_COMMITTER_NAME="t",
               GIT_AUTHOR_EMAIL="t@example.com", GIT_COMMITTER_EMAIL="t@example.com")
    for args in (["init", "-q"], ["add", "-A"], ["commit", "-q", "-m", "seed"],
                 ["update-ref", "refs/remotes/origin/main", "HEAD"]):
        subprocess.run(["git"] + args, cwd=str(repo), env=env, check=True, capture_output=True)
    gate = base / "D" / "gate-base" / "tools"
    gate.mkdir(parents=True)
    shutil.copy(NONET, str(gate / "night_shift_nonet.sh"))
    fake = base / "branch" / "tools"
    fake.mkdir(parents=True)
    (fake / "night_shift_nonet.sh").write_text('#!/usr/bin/env bash\nexec "$@"\n')
    return repo, base


def _hook(project, payload, argv=None, raw=None):
    repo, _ = project
    stdin = raw if raw is not None else json.dumps(payload)
    env = dict(os.environ, CLAUDE_PROJECT_DIR=str(repo))
    return subprocess.run(argv or [sys.executable, GUARD], input=stdin, env=env,
                          capture_output=True, text=True)


def _bash(project, command):
    repo, base = project
    command = command.replace("{D}", str(base / "D")).replace("{B}", str(base / "branch"))
    return _hook(project, {"tool_name": "Bash", "cwd": str(repo),
                           "tool_input": {"command": command, "description": "x"}})


@pytest.mark.parametrize("command", [
    # The residual list of the spelling-based deny list, every one unwrapped.
    "FOO=1 python tools/probes/x.py",
    "/usr/bin/python3 tools/probes/x.py",
    'bash -c "python tools/probes/x.py"',
    "cd tools && python probes/x.py",
    "cd tools/probes && python x.py",
    "ls tools/probes | xargs -n1 python",
    "echo tools/probes/x.py | xargs python",
    "true && python tools/probes/x.py",
    "python -m tools.probes.x",
    "timeout 600 python tools/probes/x.py",
    "env PYTHONPATH=. python tools/probes/x.py",
    "nohup python tools/probes/x.py",
    "python tools/pro''bes/x.py",
    'ros2 "bag" info x',
    "ros2  bag info x",
    "ls ~/Desktop/rosbags",
    "rm -rf ~/Desktop/rosbags/x",
    "cat ~/.ros/log/latest/launch.log",
    "grep -rln socket tools/probes/",
    "git add tools/probes/x.py",
    # Wrapped, then a second command that is not.
    _W + " true && python tools/probes/y.py",
    _W + " python tools/probes/x.py; python tools/probes/y.py",
    _W + " python tools/probes/x.py || python tools/probes/y.py",
    _W + " python tools/probes/x.py | tee temp/probes/out",
    _W + " python tools/probes/x.py &",
    _W + " python tools/probes/x.py > ~/Desktop/rosbags/x/out",
    _W + " python tools/probes/x.py 2>&1",
    _W + " python $(echo tools/probes/x.py)",
    _W + " python `echo tools/probes/x.py`",
    _W + " python tools/probes/x.py\npython tools/probes/y.py",
    "cd . && " + _W + " python tools/probes/x.py",
    "FOO=1 " + _W + " python tools/probes/x.py",
    # A wrapper that is not main's: a branch's fake, or a name that only resembles it.
    "{B}/tools/night_shift_nonet.sh python tools/probes/x.py",
    "tools/night_shift_nonet.sh.bak python tools/probes/x.py",
    "bash " + _W + " python tools/probes/x.py",
    # Never, even wrapped.
    _W + " ros2 bag play x",
    _W + " ros2 bag record -a",
    _W + " ros2 bag reindex x",
    _W + " ros2 bag convert -i x -o y",
    "ros2 bag record -a",
])
def test_guard_refuses(project, command):
    r = _bash(project, command)
    assert r.returncode == 2, (command, r.stderr)
    assert "night_shift_nonet.sh" in r.stderr   # the message names the accepted shape


@pytest.mark.parametrize("command", [
    _W + " python tools/probes/x.py --bag ~/Desktop/rosbags/x --out temp/probes/y.json",
    "./" + _W + " python tools/probes/x.py",
    "  " + _W + " timeout 900 env PYTHONPATH=. python tools/probes/x.py",
    "{D}/gate-base/tools/night_shift_nonet.sh python3 tools/probes/x.py",
    _W + " ros2 bag info ~/Desktop/rosbags/x",
    _W + " ls ~/Desktop/rosbags",
    _W + " git add tools/probes/x.py",
    # Not guarded: no probes, bags or ~/.ros on the line.
    "python -m pytest tests/sim/test_night_shift_settings.py -q",
    "git status --porcelain && git log -1",
    "grep -rn teensy_link teensy_link/ | head",
    _W + " ./run_tests.sh --full >> D/gate-X.log 2>&1; echo EXIT=$? >> D/gate-X.log",
])
def test_guard_allows(project, command):
    r = _bash(project, command)
    assert r.returncode == 0, (command, r.stderr)


@pytest.mark.parametrize("raw", [
    "", "not json", "{", '{"tool_input": {}}', '{"tool_input": {"command": 3}}',
    '{"tool_input": "ls"}', '{"cwd": "/"}', '[1, 2]',
])
def test_guard_malformed_input_fails_closed(project, raw):
    guarded = _hook(project, None, raw=raw + " tools/probes/x.py ros2 bag")
    assert guarded.returncode == 2, guarded.stderr
    assert _hook(project, None, raw=raw).returncode == 0   # nothing guarded to protect


def test_guard_refuses_when_main_has_no_wrapper(project, tmp_path):
    repo, _ = project
    (tmp_path / "tools").mkdir()
    shutil.copy(NONET, str(tmp_path / _W))
    env = dict(os.environ, CLAUDE_PROJECT_DIR=str(tmp_path))   # not a git repo at all
    r = subprocess.run([sys.executable, GUARD], env=env, capture_output=True, text=True,
                       input=json.dumps({"cwd": str(tmp_path), "tool_input": {
                           "command": _W + " python tools/probes/x.py"}}))
    assert r.returncode == 2


def _wired_command():
    s = json.load(open(SETTINGS))
    (entry,) = [e for e in s["hooks"]["PreToolUse"] if e["matcher"] == "Bash"]
    (hook,) = entry["hooks"]
    assert hook["type"] == "command"
    return hook["command"]


def test_settings_wire_the_guard_for_bash():
    command = _wired_command()
    assert "origin/main:tools/night_shift_bash_guard.py" in command
    assert command.rstrip().endswith("|| exit 2")   # any failure to run is a block
    # A settings file edited mid-session is re-read: disableAllHooks would end the guard.
    deny = json.load(open(SETTINGS))["permissions"]["deny"]
    for tool in ("Edit", "Write"):
        assert "%s(**/.claude/settings*.json)" % tool in deny
    assert os.path.isfile(GUARD)
    src = open(GUARD).read()
    compile(src, GUARD, "exec")
    assert "from __future__ import annotations" in src


needs_sys_python = pytest.mark.skipif(not os.path.exists("/usr/bin/python3"),
                                      reason="the hook runs /usr/bin/python3")


@needs_sys_python
def test_wired_hook_runs_mains_guard_not_the_branch_copy(project):
    """The settings' own command line, through sh as Claude Code runs it."""
    repo, _ = project
    argv = ["sh", "-c", _wired_command()]
    ev = lambda c: {"tool_name": "Bash", "cwd": str(repo), "tool_input": {"command": c}}
    assert _hook(project, ev("git status"), argv=argv).returncode == 0
    assert _hook(project, ev("FOO=1 python tools/probes/x.py"), argv=argv).returncode == 2
    assert _hook(project, ev(_W + " python tools/probes/x.py"), argv=argv).returncode == 0
    guard = repo / "tools" / "night_shift_bash_guard.py"
    keep = guard.read_text()
    guard.write_text("import sys\nsys.exit(0)\n")   # a branch's edit to the guard
    try:
        assert _hook(project, ev("FOO=1 python tools/probes/x.py"), argv=argv).returncode == 2
    finally:
        guard.write_text(keep)


@needs_sys_python
def test_wired_hook_refuses_everything_when_main_has_no_guard(tmp_path):
    argv = ["sh", "-c", _wired_command()]
    env = dict(os.environ, CLAUDE_PROJECT_DIR=str(tmp_path))   # no repo, no origin/main
    r = subprocess.run(argv, input=json.dumps({"tool_input": {"command": "git status"}}),
                       env=env, capture_output=True, text=True)
    assert r.returncode == 2


def _nonet_works():
    return shutil.which("unshare") and subprocess.run(
        [NONET, "true"], capture_output=True).returncode == 0


needs_nonet = pytest.mark.skipif(not _nonet_works(), reason="no user namespaces on this box")


@needs_nonet
def test_nonet_cannot_reach_the_network_but_has_loopback():
    # TEST-NET-1 (RFC 5737), never the bridge's 192.168.42.2: a broken wrapper must not
    # turn this test into a packet at the robot.
    code = (
        "import socket,sys\n"
        "a=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);a.bind(('127.0.0.1',0))\n"
        "b=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);b.sendto(b'x',a.getsockname())\n"
        "a.settimeout(5);assert a.recv(4)==b'x'\n"
        "c=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)\n"
        "try:\n"
        "    c.sendto(b'x',('192.0.2.1',5005))\n"
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
# the defaults (~/Desktop/rosbags, ~/.ros/log, /dev/ttyACM*, /dev/ttyUSB*, /dev/ttyTHS*,
# /dev/hidraw*, /dev/serial, /dev/bus/usb)
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
for dev in /dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS* /dev/hidraw*; do
  [[ -e $dev ]] || continue
  [[ $(stat -Lc %t:%T "$dev") == 1:3 ]] || no "$dev not masked"
done
for d in /dev/serial /dev/bus/usb; do
  [[ ! -e $d ]] || [[ -z $(ls -A "$d") ]] || no "$d not empty"
done
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
