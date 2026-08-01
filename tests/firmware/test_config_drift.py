"""Generic config-codegen drift gate.

`config/generate_config.py` renders five artifacts from two YAML files and then
DELIVERS copies of them into six consumer directories (four firmware trees, the
ROS2 package, the GUI).  Every one of those copies is a place where the repo can
silently disagree with itself: hand-edit a generated header, or edit the YAML and
forget to re-run the generator, and the build keeps working — it just builds the
old numbers.  Nothing caught that class before this file; the existing checks are
bespoke per-constant assertions that only cover the handful of values somebody
already got burnt by.

This is the generic version: render every artifact in memory, compare against
what is on disk.  Mirrors the pattern of
`test_udp_protocol_xlang.py::test_delivered_copies_match_canonical` (which does
the same job for the UDP protocol's two delivered copies) but drives the
comparison off the generator's OWN copy lists, so a newly added consumer
directory is covered the moment it is added to `build_artifacts`.

The bespoke per-constant stale checks stay in place for now — this generic gate
has to prove itself over a few config edits before anything is deleted on its
authority.  See logbook/2026-08-01-config-drift-gate.md.

Read-only and xdist-safe: no writes, no fixed paths, no ports.
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_GEN_SCRIPT = _REPO_ROOT / "config" / "generate_config.py"


def _load_generator():
    spec = importlib.util.spec_from_file_location("gen_config_drift", _GEN_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# Rendered once at collection so the destinations can be parametrised (one test
# id per delivered copy — a failure names the file, not "the config is stale").
# A build failure is captured rather than raised so it degrades to one red test
# instead of a whole-suite collection error.
try:
    _GEN = _load_generator()
    _PLAN, _SKIPPED, _NOTES, _BB_AVAILABLE = _GEN.build_artifacts(_GEN.DEFAULT_YAML)
    _BUILD_ERROR = None
except Exception as exc:  # noqa: BLE001 — re-raised inside a test below
    _GEN, _PLAN, _SKIPPED, _NOTES, _BB_AVAILABLE = None, [], [], [], False
    _BUILD_ERROR = exc


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(_REPO_ROOT))
    except ValueError:
        return str(path)  # external (../BallButler)


_FIX = ("Fix: python config/generate_config.py && "
        "(cd ros_ws && colcon build --packages-select jugglebot)")


def _launch_env():
    """os.environ with the venv scrubbed — a faithful `ros2 launch` environment.

    On this Jetson the venv is built on the same 3.8.10 binary as
    /usr/bin/python3, so the interpreter path alone would not distinguish them.
    """
    return {k: v for k, v in os.environ.items()
            if k not in ("VIRTUAL_ENV", "PYTHONPATH", "PYTHONHOME")}

# In-repo destinations that MUST be in the generator's copy lists. Hard-coded on
# purpose: the parametrised test below reads its cases FROM those lists, so
# deleting a copy-list entry would silently shrink the gate to nothing. This list
# is the independent witness. ../BallButler is deliberately absent — it is an
# external checkout the generator legitimately skips when missing.
def _in_repo(plan):
    """The half of the plan this repo's gate is allowed to fail on.

    `../BallButler` is a SEPARATE git checkout, on its own branch, at its own
    commit.  Letting it fail `./run_tests.sh` would mean this repo's pre-commit
    gate goes red because of the working-tree state of a repository that is not
    this repository and not under this repository's version control — and the
    remediation the failure prints (`python config/generate_config.py`) WRITES
    into that external repo.  Its drift is still reported by `--check`, as an
    `EXTERNAL DRIFT:` line that does not touch the exit code.
    """
    return [(d, c) for d, c, _l in plan if not _GEN.is_external(d)]


_EXPECTED_IN_REPO_DESTINATIONS = {
    "config/generated/protocol_config.h",
    "config/generated/protocol_config.py",
    "config/generated/hardware_config.h",
    "config/generated/hardware_config.py",
    "config/generated/geometry-config.js",
    "ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h",
    "ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h",
    "ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h",
    "ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h",
    "ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h",
    "ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h",
    "ros_ws/src/jugglebot/jugglebot/protocol_config.py",
    "ros_ws/src/jugglebot/jugglebot/hardware_config.py",
    "ros_ws/gui/js/geometry-config.js",
}


def test_generator_renders():
    """`build_artifacts` runs clean — everything below depends on it."""
    if _BUILD_ERROR is not None:
        raise AssertionError(
            f"config/generate_config.py failed to render: "
            f"{type(_BUILD_ERROR).__name__}: {_BUILD_ERROR}")
    assert _PLAN, "generator produced an empty artifact plan"


def test_copy_lists_cover_every_known_consumer():
    """Nobody quietly dropped a consumer from the delivery lists."""
    planned = {_rel(dest) for dest, _c, _l in _PLAN}
    skipped = {_rel(dest) for dest in _SKIPPED}
    covered = planned | skipped
    missing = _EXPECTED_IN_REPO_DESTINATIONS - covered
    assert not missing, (
        "generate_config.py no longer delivers to: " + ", ".join(sorted(missing)) +
        " — a consumer stopped receiving regenerated config. If the removal was "
        "deliberate, drop it from _EXPECTED_IN_REPO_DESTINATIONS too.")


@pytest.mark.parametrize(
    "dest,content",
    [pytest.param(d, c, id=_rel(d)) for d, c in _in_repo(_PLAN)],
)
def test_artifact_matches_generator(dest: Path, content: str):
    """Every in-repo generated artifact and delivered copy matches a fresh render.

    Compares TEXT, not bytes: the write path's `Path.write_text(...)` translates
    newlines to `os.linesep`, so a byte compare would report every artifact
    drifted on a fresh Windows checkout (this repo is run from a Win10 clone for
    sim work).  The sibling `test_udp_protocol_xlang.py` gate compares text for
    the same reason.
    """
    assert dest.exists(), f"{_rel(dest)} is missing. {_FIX}"
    assert dest.read_text(encoding="utf-8") == content, (
        f"{_rel(dest)} differs from the generator output — either it was "
        f"hand-edited or a YAML edit was never regenerated. {_FIX}")


def test_external_deliveries_are_outside_the_exit_code_gate():
    """An external checkout can never fail this repo's gate — and is still reported.

    Pinned in memory (no writes, no dependence on ../BallButler being present):
    feed `check_artifacts` a plan whose external entry is guaranteed-missing and
    assert it lands in the external bucket, which `main()` prints but does not
    count.
    """
    assert _GEN is not None
    external = _GEN.BB_FIRMWARE_DIR / "hardware_config.h"
    internal = _GEN.OUTPUT_DIR / "protocol_config.h"
    assert _GEN.is_external(external), "../BallButler must classify as external"
    assert not _GEN.is_external(internal), "config/generated must classify in-repo"

    fake = [(external / "definitely-absent", "x", "Copied"),
            (internal.parent / "definitely-absent", "x", "Generated")]
    drift, ext_drift = _GEN.check_artifacts(fake)
    assert [d for d, _r in drift] == [fake[1][0]]
    assert [d for d, _r in ext_drift] == [fake[0][0]]


def test_check_mode_announces_what_it_did_not_check():
    """A partial run must never masquerade as a clean one.

    With the hardware YAML absent, 9 of the 16 planned artifacts — the entire
    tuning surface — drop out of the plan.  Before this, `--check` printed a
    green `CONFIG FRESH: 7` and `jugglebot_launch.py` printed `OK`, which is
    precisely the false-green the launch's own design forbids.  Driven in
    memory against a redirected HW_YAML: no writes, no dependence on the real
    file being movable.
    """
    assert _GEN is not None
    import io
    import contextlib

    mod = _load_generator()
    mod.HW_YAML = Path("/nonexistent/hardware_config.yaml")
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = mod.main(["--check"])

    # Deliberately says nothing about the exit code: whether the REMAINING
    # artifacts are fresh is a different test's business, and coupling to it
    # would turn one dirty file into an extra red here.
    assert "NOT CHECKED:" in err.getvalue(), (
        "a skipped half of the surface must be announced, not swallowed "
        f"(exit {rc}):\n" + err.getvalue() + out.getvalue())


def test_check_mode_reports_fresh_and_writes_nothing(tmp_path):
    """`--check` is a pure read: exit 0 when fresh, and zero bytes touched.

    The no-write guarantee is what makes it safe to run from
    `jugglebot_launch.py` at bring-up time. Proven by snapshotting mtime_ns +
    sha256 of every destination across the call, not by reading the code.

    (If this ever fails on mtime alone with identical hashes, suspect a
    concurrent `generate_config.py` in another session rather than a real
    regression.)
    """
    def snapshot():
        out = {}
        for dest, _c, _l in _PLAN:
            if not dest.exists():
                # A missing artifact is a real failure, but it belongs to
                # test_artifact_matches_generator, which names the file. Do not
                # let it surface here as a bare FileNotFoundError from a fixture.
                out[str(dest)] = "ABSENT"
                continue
            st = dest.stat()
            out[str(dest)] = (st.st_mtime_ns, st.st_size,
                              hashlib.sha256(dest.read_bytes()).hexdigest())
        return out

    before = snapshot()
    proc = subprocess.run(
        [sys.executable, str(_GEN_SCRIPT), "--check"],
        cwd=str(_REPO_ROOT), capture_output=True, text=True, timeout=120)
    after = snapshot()

    assert after == before, (
        "config/generate_config.py --check MODIFIED files on disk — it must be "
        "read-only (jugglebot_launch.py runs it at bring-up).")
    assert proc.returncode == 0, (
        f"--check reported drift (exit {proc.returncode}):\n{proc.stderr}")
    assert "CONFIG FRESH" in proc.stdout, proc.stdout


def test_check_mode_runs_under_the_launch_interpreter():
    """The launch's system python3 can run `--check` (it is NOT the project venv).

    `jugglebot_launch.py` shells the drift check out to whatever interpreter is
    running the launch — on the Jetson that is /usr/bin/python3 (3.8.10) with
    SYSTEM site-packages only: PyYAML is there, the venv-only scientific stack is
    not. If `--check` ever grows a heavy import, the launch check goes
    permanently silent (it is swallowed to a warning by design), so pin the
    dependency budget here. VIRTUAL_ENV/PYTHONPATH are scrubbed so this is a
    faithful launch environment even when pytest itself runs inside the venv
    (on this Jetson the venv is built on the same 3.8.10 binary, so the
    interpreter path alone would not distinguish them).
    """
    sys_py = Path("/usr/bin/python3")
    if not sys_py.exists():
        pytest.skip("no /usr/bin/python3 (not the Jetson/Ubuntu layout)")

    proc = subprocess.run(
        [str(sys_py), str(_GEN_SCRIPT), "--check"], env=_launch_env(),
        cwd=str(_REPO_ROOT), capture_output=True, text=True, timeout=120)
    combined = proc.stdout + proc.stderr
    assert "Traceback" not in combined, (
        f"--check crashed under {sys_py}:\n{combined}")
    assert ("CONFIG FRESH" in proc.stdout) or ("CONFIG DRIFT" in proc.stderr), (
        f"--check produced no verdict under {sys_py}:\n{combined}")

    # The verdict must MATCH the venv's — an assertion that holds whether or not
    # the tree happens to be fresh right now. A tolerant "either verdict is
    # fine" check would stay green through exactly the failure this test exists
    # to catch: the launch interpreter disagreeing with the one everybody
    # develops under, which shows up as a permanent, un-actionable CONFIG DRIFT
    # banner on every single bring-up.
    venv_proc = subprocess.run(
        [sys.executable, str(_GEN_SCRIPT), "--check"],
        cwd=str(_REPO_ROOT), capture_output=True, text=True, timeout=120)
    assert proc.returncode == venv_proc.returncode, (
        f"the launch interpreter ({sys_py}, exit {proc.returncode}) and the "
        f"project venv ({sys.executable}, exit {venv_proc.returncode}) "
        f"disagree about config freshness — the bring-up banner would be "
        f"un-fixable from the venv.\nlaunch:\n{combined}\n"
        f"venv:\n{venv_proc.stdout}{venv_proc.stderr}")


def test_both_interpreters_render_identical_bytes():
    """The launch interpreter and the venv must render the SAME artifacts.

    The failure this pins is silent and permanent: if the two stacks ever
    diverge (they are already a PyYAML MAJOR version apart — system 5.3.1 vs
    venv 6.0.2), `--check` goes red on every `ros2 launch` while the whole test
    suite stays green, because the suite only ever renders under the venv. The
    banner would then be un-actionable — regenerating under the venv cannot fix
    a difference the venv cannot see — and an un-actionable banner is one the
    operator learns to ignore.

    Compares render hashes directly, so it holds even when the tree is mid-edit.
    """
    sys_py = Path("/usr/bin/python3")
    if not sys_py.exists():
        pytest.skip("no /usr/bin/python3 (not the Jetson/Ubuntu layout)")
    assert _GEN is not None

    code = (
        "import hashlib,importlib.util,json,sys\n"
        "spec=importlib.util.spec_from_file_location('g',sys.argv[1])\n"
        "m=importlib.util.module_from_spec(spec)\n"
        "sys.modules['g']=m\n"
        "spec.loader.exec_module(m)\n"
        "plan,_s,_n,_bb=m.build_artifacts(m.DEFAULT_YAML)\n"
        "print(json.dumps({str(d):hashlib.sha256(c.encode('utf-8')).hexdigest()"
        " for d,c,_l in plan}))\n"
    )
    proc = subprocess.run(
        [str(sys_py), "-c", code, str(_GEN_SCRIPT)], env=_launch_env(),
        cwd=str(_REPO_ROOT), capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, (
        f"could not render under {sys_py}:\n{proc.stdout}\n{proc.stderr}")

    import json
    theirs = json.loads(proc.stdout.strip().splitlines()[-1])
    ours = {str(d): hashlib.sha256(c.encode("utf-8")).hexdigest()
            for d, c, _l in _PLAN}
    assert theirs == ours, (
        "the launch interpreter and the project venv render DIFFERENT config "
        "artifacts — the launch-time drift check would report permanent, "
        "un-fixable drift. Differing: "
        + ", ".join(sorted(k for k in set(ours) | set(theirs)
                           if ours.get(k) != theirs.get(k))))
