"""mapkit.steps: the single step-heading rule, id normalisation and the mirror rule."""
from __future__ import annotations

import os

import pytest

from mapkit.plans import PlanError, load_plan, plan_paths
from mapkit.steps import (check_mirror, check_step_vocab, extract_step_headings,
                          normalise_step_id, parse_step_heading_text)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# (heading text, expected norm_id, expected stripped title) — taken from the real corpus.
POSITIVES = [
    ("P1 — The oval pattern dispatchable — COMPLETE 2026-10-18", "P1",
     "The oval pattern dispatchable"),
    ("Phase 8: Tier 3b — Time pathologies, resource exhaustion, hooks, races — COMPLETE (2026-05-12)",
     "P8", "Tier 3b — Time pathologies, resource exhaustion, hooks, races"),
    ("Phase W4a: `mpc.solve` pre-allocation — NOT STARTED", "PW4A", "`mpc.solve` pre-allocation"),
    ("R0 — Board and substrate", "R0", "Board and substrate"),
    ("Stage 1a — system-ID (do this first, on Path BRIDGE)", "STAGE1A",
     "system-ID (do this first, on Path BRIDGE)"),
    ("Rung 4+ (future, not yet scoped)", "R4+", "(future, not yet scoped)"),
    ("P2 — Banking leak fix — RETIRED 2026-10-23", "P2", "Banking leak fix"),
    ("Phase 0: Preparation — DONE (2026-02-19)", "P0", "Preparation"),
    ("Phase 0: MuJoCo Stewart Platform Model — ✅ COMPLETE (2026-03-16)", "P0",
     "MuJoCo Stewart Platform Model"),
    ("Phase 2: Offline trajectory optimiser & player core — IN PROGRESS (started 2026-05-22)",
     "P2", "Offline trajectory optimiser & player core"),
    ("P1 - cup-keyed accept + lob retirement (node only)", "P1",
     "cup-keyed accept + lob retirement (node only)"),
    ("Phase 10b-cone — Catching cone uplink (DONE 2026-06-10)", "P10B-CONE",
     "Catching cone uplink (DONE 2026-06-10)"),
    ("**Phase 3** — Bold id", "P3", "Bold id"),
]

NEGATIVES = [
    "Implementation Phase Summary",
    "Implementation Phases",
    "Implementation Phases (detailed)",
    "Phase Summary",
    "3. Phases",
    "Appendix A: Phase 0 Completion Notes (2026-02-19)",
    "0.4 Phase 0 outcome, 2026-10-15",
    "Items for investigation during Phase 2+",
    "2.4 C-CUP-4, the seated carry",
    "S2 — FW 12 confirmation soak (brief-launch protocol)",
    "1. Problem",
    "Prompt: something",
    "Rationale",
]


def test_positive_headings():
    """Each hand-picked real heading is a step with the expected id and suffix-stripped title."""
    bad = []
    for text, nid, title in POSITIVES:
        got = parse_step_heading_text(text)
        if got is None or got["norm_id"] != nid or got["title"] != title:
            bad.append("%r -> %r" % (text, got))
    assert not bad, "\n".join(bad)


def test_negative_headings():
    """Container headings, mid-line phase mentions, numbered sections and S-soaks are not steps."""
    bad = [t for t in NEGATIVES if parse_step_heading_text(t) is not None]
    assert not bad, bad


def test_id_normalisation():
    """Phase N == PN, Rung N == RN, Phase W4a == PW4a; Stage stays distinct; non-ids raise."""
    assert normalise_step_id("Phase 1") == normalise_step_id("P1") == "P1"
    assert normalise_step_id("Rung 2") == normalise_step_id("R2") == "R2"
    assert normalise_step_id("Phase W4a") == normalise_step_id("PW4a") == "PW4A"
    assert normalise_step_id("Stage 1a") == "STAGE1A" != normalise_step_id("P1a")
    assert normalise_step_id("Rung 4+") == "R4+"
    for bad in ("S2", "Summary", "Phase", "1"):
        with pytest.raises(ValueError):
            normalise_step_id(bad)


def test_heading_levels_and_fences():
    """Only ## to #### count; headings inside fenced code are ignored; line numbers are 1-based."""
    body = "# P9 top\n##### P8 deep\n## P1 — a\n```\n## P2 — fenced\n```\n#### Rung 3 — b\n"
    hs = extract_step_headings(body)
    assert [(h.norm_id, h.level, h.line_no) for h in hs] == [("P1", 2, 3), ("R3", 4, 7)]


def test_real_corpus_extraction_is_clean():
    """Over every real plan, every extracted id normalises and no title keeps a status suffix."""
    import re
    suffix = re.compile(r"\s[—–-]\s+(✅\s*)?(NOT STARTED|COMPLETE|DONE|RETIRED|IN PROGRESS)\b")
    bad = []
    count = 0
    for path in plan_paths(REPO_ROOT):
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        for h in extract_step_headings(text):
            count += 1
            if suffix.search(h.title):
                bad.append("%s:%d %r" % (os.path.basename(path), h.line_no, h.title))
    assert count > 100
    assert not bad, "\n".join(bad)


def _write(tmp_path, steps_yaml, body):
    f = tmp_path / "plan.md"
    f.write_text("---\ntitle: T\n" + steps_yaml + "---\n" + body, encoding="utf-8")
    return load_plan(str(f))


_STEPS = "steps:\n  - {id: P1, title: One, kind: phase, status: complete}\n" \
         "  - {id: Rung 2, title: Two., kind: rung, status: not_started}\n"


def test_mirror_pass(tmp_path):
    """Listed ids each match one heading, every heading is listed, titles mirror modulo case/space/punct."""
    plan = _write(tmp_path, _STEPS, "## Phase 1 — one — COMPLETE\n\n### R2 —  TWO\n")
    assert check_mirror(plan) == []
    assert check_step_vocab(plan) == []


def test_mirror_missing_heading(tmp_path):
    """A listed step with no heading is an offender."""
    plan = _write(tmp_path, _STEPS, "## P1 — One\n")
    msgs = check_mirror(plan)
    assert len(msgs) == 1 and "has no heading" in msgs[0]


def test_mirror_unlisted_heading(tmp_path):
    """A step heading not listed in steps: is an offender."""
    plan = _write(tmp_path, _STEPS, "## P1 — One\n## R2 — Two\n## P3 — Three\n")
    msgs = check_mirror(plan)
    assert len(msgs) == 1 and "not listed" in msgs[0]


def test_mirror_duplicate_heading(tmp_path):
    """A listed id matched by two headings is an offender."""
    plan = _write(tmp_path, _STEPS, "## P1 — One\n## Phase 1 — One\n## R2 — Two\n")
    msgs = check_mirror(plan)
    assert len(msgs) == 1 and "matches 2 headings" in msgs[0]


def test_mirror_title_mismatch(tmp_path):
    """A listed title that differs from its heading's title is an offender."""
    plan = _write(tmp_path, _STEPS, "## P1 — Uno\n## R2 — Two\n")
    msgs = check_mirror(plan)
    assert len(msgs) == 1 and "title" in msgs[0]


def test_mirror_not_asserted_without_steps(tmp_path):
    """With no steps: nothing is asserted, however many step headings exist."""
    plan = _write(tmp_path, "", "## P1 — a\n## P1 — b\n")
    assert check_mirror(plan) == []


def test_bad_step_vocab_reported(tmp_path):
    """A step status or kind outside the vocabulary is reported by check_step_vocab."""
    plan = _write(tmp_path, "steps:\n  - {id: P1, title: One, kind: stage, status: done}\n",
                  "## P1 — One\n")
    assert len(check_step_vocab(plan)) == 2


def test_project_map_plan_mirrors():
    """plans/archived/project-map.md carries steps: and passes the mirror rule."""
    plan = load_plan(os.path.join(REPO_ROOT, "plans", "archived", "project-map.md"))
    assert plan.has_steps
    assert check_mirror(plan) == []
    assert check_step_vocab(plan) == []


def test_every_plan_with_steps_mirrors():
    """Every real plan that carries steps: passes the mirror rule and the step vocabulary."""
    bad = []
    for path in plan_paths(REPO_ROOT):
        try:
            plan = load_plan(path)
        except PlanError:
            continue  # reported by test_every_real_plan_parses
        bad.extend(check_mirror(plan))
        bad.extend(check_step_vocab(plan))
    assert not bad, "\n".join(bad)
