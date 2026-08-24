"""MJCF codegen drift gate: `sim/model/jugglebot.xml` vs its generator.

`sim/model/generate_mjcf.py` renders the MuJoCo model from
`config/hardware_config.yaml`.  Nothing checked that the rendered model and the
committed one agreed, and on 2026-03-30 they stopped agreeing: four substantive
hand-edit commits landed in the XML (two-ball support, catch tuning, renderer
perf) while the generator stood still.  Five months later, running the generator
would have silently deleted a live capability — the second ball — and quietly
halved the ball's radius and mass.  Nobody would have known until a juggle demo
behaved oddly, because *no test read the generator at all*: `test_model.py`
loads the XML and never mentions `generate_mjcf`.

This is the same failure class `tests/firmware/test_config_drift.py` closes for
`config/generated/*`, and this file is modelled on it: render the artifact, then
compare it against what is on disk.  The reconciliation is written up in
`logbook/2026-08-21-mjcf-generator-reconciled.md`.

Read-only against the repo and xdist-safe: the one write goes to `tmp_path`.
"""

from __future__ import annotations

import copy
import importlib.util
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_GEN_SCRIPT = _REPO_ROOT / "sim" / "model" / "generate_mjcf.py"
_MODEL_XML = _REPO_ROOT / "sim" / "model" / "jugglebot.xml"


def _load_generator():
    """Import generate_mjcf.py by path — it is a script, not an installed module."""
    spec = importlib.util.spec_from_file_location("gen_mjcf_drift", _GEN_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# Rendered once at collection. A render failure is captured rather than raised so
# it degrades to one red test instead of a whole-file collection error.
try:
    _GEN = _load_generator()
    _CONFIG = _GEN.load_hardware_config()
    _RENDERED = _GEN.render_xml(_CONFIG)
    _BUILD_ERROR = None
except Exception as exc:  # noqa: BLE001 — re-raised inside a test below
    _GEN, _CONFIG, _RENDERED, _BUILD_ERROR = None, None, None, exc


_FIX = (
    "Fix: run `python sim/model/generate_mjcf.py` and commit the result.\n"
    "Do NOT hand-edit sim/model/jugglebot.xml — it is a GENERATED artifact and "
    "the next regeneration reverts you. Change sim/model/generate_mjcf.py (for "
    "structure) or config/hardware_config.yaml (for numbers) instead."
)


def _first_difference(committed: str, rendered: str) -> str:
    """Line-level locator so a failure names the drift instead of dumping 200 lines."""
    a, b = committed.splitlines(), rendered.splitlines()
    for i, (x, y) in enumerate(zip(a, b), start=1):
        if x != y:
            return (f"first difference at line {i}:\n"
                    f"  committed: {x!r}\n"
                    f"  generator: {y!r}")
    if len(a) != len(b):
        longer, who = (a, "committed") if len(a) > len(b) else (b, "generator")
        return (f"identical for {min(len(a), len(b))} lines, then {who} continues "
                f"with {longer[min(len(a), len(b))]!r}")
    return "files differ only in trailing whitespace or line endings"


def _assert_matches(committed: str, rendered: str) -> None:
    """The ONE comparison, called by the real gate and by its self-check alike.

    Factored out so the self-check below exercises the assertion that actually
    guards the repo, rather than a hand-rolled restatement of it that could
    drift from the real one — which would be this file committing the very sin
    it exists to catch.
    """
    if committed == rendered:
        return
    raise AssertionError(
        "sim/model/jugglebot.xml has DRIFTED from sim/model/generate_mjcf.py — "
        "either the XML was hand-edited or a generator/YAML edit was never "
        f"regenerated.\n{_first_difference(committed, rendered)}\n{_FIX}")


def test_generator_renders():
    """`render_xml` runs clean — everything below depends on it."""
    if _BUILD_ERROR is not None:
        raise AssertionError(
            f"sim/model/generate_mjcf.py failed to render: "
            f"{type(_BUILD_ERROR).__name__}: {_BUILD_ERROR}")
    assert _RENDERED and _RENDERED.startswith('<?xml')


def test_committed_model_matches_generator(tmp_path, monkeypatch, capsys):
    """The committed XML is exactly what `main()` writes.

    Drives the real entry point with `OUTPUT_PATH` redirected into `tmp_path`,
    rather than re-implementing the render here: a gate that assembled the file
    its own way would be testing its own renderer, not the one that produced the
    file on disk.

    Compares TEXT, not bytes. `open(path, 'w')` translates '\\n' to os.linesep on
    write, so on the Win10 clone this repo is also run from, a byte compare would
    report drift on a pristine checkout. The sibling gates
    (`test_config_drift.py`, `test_udp_protocol_xlang.py`) compare text for the
    same reason. There is no semantic tolerance here — every character of the
    model must match.
    """
    assert _GEN is not None, _BUILD_ERROR
    out = tmp_path / "jugglebot.xml"
    monkeypatch.setattr(_GEN, "OUTPUT_PATH", str(out))
    _GEN.main()
    capsys.readouterr()  # swallow the generator's summary print

    assert out.exists(), "generate_mjcf.main() wrote nothing"
    _assert_matches(_MODEL_XML.read_text(encoding="utf-8"),
                    out.read_text(encoding="utf-8"))


def test_the_gate_catches_a_hand_edit(tmp_path):
    """The comparison above is not vacuous.

    Without this, a refactor that made `_RENDERED` empty, or a comparison that
    normalised too hard, would leave the gate permanently green — which is
    exactly the state the repo was in for five months, just spelled differently.
    Mutates a COPY (the real XML is never touched) three ways, spanning the
    edits that actually happened: one attribute value (the hand-stroke number
    that went stale for three days), a whole body (the second ball, which went
    unnoticed for five months), and a single trailing character.
    """
    assert _RENDERED is not None
    _assert_matches(_RENDERED, _RENDERED)  # the identical case must NOT raise

    for label, mutation in (
            ("one attribute value",
             lambda x: x.replace('range="0 0.344750"', 'range="0 0.355000"', 1)),
            ("the whole ball2 body",
             lambda x: x[:x.index('<body name="ball2"')]
                       + x[x.index('</body>', x.index('<body name="ball2"')) + 8:]),
            ("a trailing character",
             lambda x: x + "\n"),
    ):
        hand_edited = mutation(_RENDERED)
        assert hand_edited != _RENDERED, (
            f"the {label} mutation did not apply — update this test")
        scratch = tmp_path / "hand_edited.xml"
        scratch.write_text(hand_edited, encoding="utf-8")
        with pytest.raises(AssertionError, match="DRIFTED"):
            _assert_matches(scratch.read_text(encoding="utf-8"), _RENDERED)


def test_committed_model_declares_itself_generated():
    """The XML says what it is, in the file a reader actually opens.

    The absent header is half of why the drift went unseen: four people edited a
    generated file that gave them no reason to think it was one.
    """
    head = _MODEL_XML.read_text(encoding="utf-8")[:600]
    assert "GENERATED FILE" in head and "DO NOT EDIT" in head, (
        "sim/model/jugglebot.xml lost its generated-file header. " + _FIX)
    assert "generate_mjcf.py" in head, (
        "the header must name the generator that produced the file. " + _FIX)


def test_two_ball_capability_is_in_the_generator():
    """Regenerating must not delete the second ball.

    This is the capability the 2026-03-30 divergence put at risk, and the reason
    this file exists rather than a bare 'the generator runs' smoke test.
    `sim/ball/manager.py` discovers balls by scanning 'ball', 'ball2', … upward
    until one is absent, so losing the `ball2` body silently drops the model to
    one ball instead of failing loudly.
    """
    assert _RENDERED is not None
    for token in ('<body name="ball2"', 'name="ball2_joint"', 'name="ball2_geom"',
                  'name="ball2_site"', 'name="ball2_mat"',
                  'name="ball2_pos"', 'name="ball2_vel"',
                  '<exclude body1="ball" body2="ball2"/>'):
        assert token in _RENDERED, (
            f"the generator no longer emits {token!r} — two-ball simulation "
            "(sim/ball/manager.py, tests/sim/test_multiball.py) would be lost "
            "at the next regeneration.")


def test_ball_spec_comes_from_hardware_config():
    """The ball geom is driven by the YAML, not re-hardcoded in the generator.

    The pre-2026-08-21 state had the ball's size and mass written down twice, in
    two files that disagreed by 75 % on radius. Rendering with a sentinel config
    proves the numbers still flow from `config/hardware_config.yaml:physics`, so
    a future edit there cannot be silently ignored.
    """
    assert _GEN is not None
    cfg = copy.deepcopy(_CONFIG)
    cfg['physics']['juggling_ball_radius_mm'] = 12.5
    cfg['physics']['juggling_ball_mass_kg'] = 0.0999
    rendered = _GEN.render_xml(cfg)
    assert 'size="0.0125" mass="0.0999"' in rendered, (
        "generate_mjcf.py stopped reading the ball spec from hardware_config.yaml")
    # ...and the committed model carries the real owner-MEASURED values.
    #
    # 37 mm, not 35: the 35 mm this pin carried until 2026-08-21 was an ASSUMED
    # "70 mm ball" that the repo had been repeating, and the owner's caliper
    # re-measurement makes it 74 mm across. The pin was therefore guarding the
    # wrong value — which is the failure mode a drift test is least able to
    # notice about itself, since both sides agreed. The mass was always measured
    # and is unchanged. Do NOT reconcile this with the 35 mm CAPTURE RADIUS that
    # appears in the aim/catch docs: different quantity, same number by accident.
    assert 'size="0.037" mass="0.071"' in _RENDERED, (
        "the ball spec drifted from the owner-measured 71 g / 74 mm diameter "
        "(caliper, 2026-08-21)")
