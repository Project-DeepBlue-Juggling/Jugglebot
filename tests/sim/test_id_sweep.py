"""tools/id_sweep.py: the per-kind ID sweeper's finder, classifier and census
(plan ``item-id-per-kind.md`` P3; tests T-U12, T-U14, T-R1).

Every test builds a throwaway git repo under ``tmp_path`` with the mapkit test
helpers, so the gate stays hermetic. The real-tree T-R1 run is opt-in
(``ID_SWEEP_REAL_TREE=1``): it reads the whole checkout and depends on the
rule table being complete for today's tree, which is a census result, not a
per-commit contract (that contract is P5's ``test_no_legacy_item_ids.py``).
"""
from __future__ import annotations

import csv
import os
import subprocess
import sys
from collections import Counter

import pytest

from tests.sim.test_mapkit_ids import _commit_push, _git, _git_env, _item, _repo  # noqa: F401

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import id_sweep  # noqa: E402


def _seed(tmp_path, files, items=(("bug", "one", "JB-1"), ("idea", "two", "JB-2"))):
    """A committed repo with Items ``items`` (kind, slug, id) and ``files``."""
    work, _ = _repo(tmp_path)
    for kind, slug, sid in items:
        _item(work, kind, slug, id=sid)
    for rel, text in files.items():
        p = work / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    _commit_push(work, "seed files")
    return work


def _census(work, capsys):
    out = work / "temp" / "id-sweep"
    rc = id_sweep.main(["--census", "--root", str(work)])
    cap = capsys.readouterr()
    return rc, out, cap


def _rows(out):
    with open(str(out / "census.csv"), newline="") as fh:
        return list(csv.DictReader(fh))


def _raw_matches(work):
    """(path, line, col) of every broad-pattern match in ``git ls-files`` minus the
    tool's own :func:`id_sweep.skip_reason` (one source for the skip rule)."""
    files = subprocess.run(["git", "ls-files", "-z"], cwd=str(work), check=True,
                           capture_output=True, text=True).stdout.split("\0")
    out = []
    skipped = dict(id_sweep.scan_files(str(work))[1])
    for rel in filter(None, files):
        if id_sweep.skip_reason(str(work), rel):
            assert skipped[rel] in ("symlink", "not-a-file", "suffix", "binary"), rel
            continue
        with open(os.path.join(str(work), rel), "rb") as fh:
            data = fh.read()
        for ln, line in enumerate(data.decode("utf-8", "replace").split("\n"), 1):
            out.extend((rel, ln, m.start() + 1) for m in id_sweep.PATTERN.finditer(line))
    listed = {r["path"]: r["reason"] for r in _skipped_csv(work)} if _skipped_csv(work) else None
    if listed is not None:   # every skipped file is listed with a reason
        assert listed == skipped
    return out


def _skipped_csv(work):
    p = work / "temp" / "id-sweep" / "skipped.csv"
    if not p.exists():
        return None
    with open(str(p), newline="") as fh:
        return list(csv.DictReader(fh))


# --------------------------------------------------------------------- T-U12

def test_unclassifiable_token_stops_the_run_naming_file_and_line(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/notes.md": "fine JB-1 here\nodd JB-7q token\n"})
    rc, out, cap = _census(work, capsys)
    assert rc != 0
    assert "docs/notes.md:2:" in cap.err and "JB-7q" in cap.err
    assert not (out / "census.csv").exists()


def test_unknown_number_stops_the_run(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/notes.md": "a\nb\nsee JB-900 for that\n"})
    rc, out, cap = _census(work, capsys)
    assert rc != 0
    assert "docs/notes.md:3:" in cap.err and "JB-900" in cap.err
    assert not (out / "census.csv").exists()


# --------------------------------------------------------------------- T-U14

ABBREV_LINES = {
    "JB_OP_X": "x = cfg.JB_OP_X  # op code\n",
    "jb_lag_fold": "y = jb_lag_fold(frames)\n",
    "jb": "counter: defer jb = 0 on FW 10\n",
    "JB-side": "| JB-side feed catch | ok |\n",
    "JB": '("MAGIC", 0x4A42, "u16", \'"JB" little-endian preamble\'),\n',
}


def test_abbreviation_class(tmp_path, capsys):
    text = "".join(ABBREV_LINES.values())
    work = _seed(tmp_path, {"config/thing.py": text})
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    rows = [r for r in _rows(out) if r["file"] == "config/thing.py"]
    assert [r["token"] for r in rows] == list(ABBREV_LINES)
    assert {r["class"] for r in rows} == {"abbreviation"}, rows


# ---------------------------------------------------------------------- T-R1

MIXED = (
    "Cite JB-1 and JB-2.\n"
    "Branch night-shift/2026-11-01-JB-1 held.\n"
    "List JB-1/2 and range JB-1..2.\n"
    "Template `JB-<n>` and JB-n.\n"
    "Entry logbook/2026-11-01-jb1-thing.md stays.\n"
    "Loose jb2 in prose.\n"
    "Robot: JB's plan frame, JB_TRAJ_MAX, tx_deferred_jb.\n"
)


def _assert_complete(work, rows):
    raw = _raw_matches(work)
    keys = Counter((r["file"], int(r["line"]), int(r["col"])) for r in rows
                   if r["class"] not in id_sweep.PER_KIND_CLASSES)
    assert sorted(keys) == sorted(raw), set(keys) ^ set(raw)
    assert all(c == 1 for c in keys.values())
    assert not [r for r in rows if not r["class"]]
    assert sum(keys.values()) == len(raw)


def test_census_is_complete_on_a_synthetic_repo(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/mixed.md": MIXED, "map/README.txt": "see BUG-13\n"})
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    rows = _rows(out)
    _assert_complete(work, rows)
    by = {(r["line"], r["token"]): r["class"] for r in rows if r["file"] == "docs/mixed.md"}
    assert by[("1", "JB-1")] == "citation"
    assert by[("2", "JB-1")] == "branch"
    assert by[("3", "JB-1/2")] == "list" and by[("3", "JB-1..2")] == "range"
    assert by[("4", "JB-<n>")] == "placeholder" and by[("4", "JB-n")] == "placeholder"
    assert by[("5", "jb1-thing")] == "frozen"
    assert by[("6", "jb2")] == "loose"
    assert [r["class"] for r in rows if r["file"] == "map/README.txt"] == ["option_text"]
    summary = (out / "summary.md").read_text()
    assert "citation" in summary and "raw" in summary.lower()


def test_census_rows_are_ordered_and_frontmatter_ids_are_citations(tmp_path, capsys):
    work = _seed(tmp_path, {"a.md": "JB-2 then JB-1\n"})
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    rows = _rows(out)
    keys = [(r["file"], int(r["line"]), int(r["col"])) for r in rows]
    assert keys == sorted(keys)
    fm = [r for r in rows if r["file"].startswith("map/bug-one")]
    assert [(r["class"], r["rule"]) for r in fm] == [("citation", "frontmatter-id")]


@pytest.mark.skipif(os.environ.get("ID_SWEEP_REAL_TREE") != "1",
                    reason="reads the whole checkout; set ID_SWEEP_REAL_TREE=1")
def test_census_is_complete_on_the_real_tree(tmp_path, monkeypatch, capsys):
    out = tmp_path / "out"
    monkeypatch.setenv("HOME", os.path.expanduser("~"))  # undo the autouse git isolation
    rc = id_sweep.main(["--census", "--root", ROOT, "--out", str(out)])
    cap = capsys.readouterr()
    assert rc == 0, cap.err[-4000:]
    from pathlib import Path
    _assert_complete(Path(ROOT), _rows(out))


# ------------------------------------------------- unit 1b: classes added at P3
# ``code_literal`` (the ID code's own literals), the synthetic-repo fixtures,
# the sweeper's own files, and the frontmatter ``id:`` line inside a frozen file.

def _classes(rows, rel):
    return [(r["line"], r["token"], r["class"]) for r in rows if r["file"] == rel]


def test_frontmatter_id_in_a_frozen_item_is_still_a_citation(tmp_path, capsys):
    slug = "map-app-allocates-jb-ids-from-its-own-clone-and-collides"  # a scheme-history file
    work = _seed(tmp_path, {}, items=(("bug", "one", "JB-1"), ("bug", slug, "JB-3")))
    rel = "map/bug-%s.md" % slug
    with open(str(work / rel), "a") as fh:
        fh.write("JB-1 was renumbered.\n")
    _commit_push(work, "body")
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    rows = [r for r in _rows(out) if r["file"] == rel]
    fm = [r for r in rows if r["token"] == "JB-3"]
    assert [(r["class"], r["rule"]) for r in fm] == [("citation", "frontmatter-id")]
    assert {r["class"] for r in rows if r["token"] == "JB-1"} == {"frozen"}
    summary = (out / "summary.md").read_text()
    sec = summary.split("## Whole-file freezes", 1)[1]
    assert rel in sec.split("\n## ", 1)[0]


CODE_X = (
    'PREFIX = "JB"\n'                                   # 1 string literal
    "# fixed in JB-1 last week\n"                       # 2 a comment citation
    '"""Parse ``JB-12`` and ``jb12`` forms."""\n'       # 3 double-backtick examples
    "for jb in items:\n"                                # 4 identifier
    '_RX = re.compile(r"^(?:JB|BUG)-\\d+$")\n'          # 5 regex
    "x = 1  # see JB-12, not \"JB\" here\n"             # 6 comment: citation, quoted word
)


def test_code_literal_is_narrow(tmp_path, capsys):
    work = _seed(tmp_path, {"mapkit/x.py": CODE_X, "docs/x.md": 'PREFIX = "JB"\n',
                            "tests/sim/test_night_shift_gate.py": 'b = "night-shift/2026-10-31-JB-404"\n',
                            "tools/id_sweep.py": "# rewrites JB-1\n"},
                 items=(("bug", "one", "JB-1"), ("idea", "two", "JB-12")))
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    rows = _rows(out)
    assert _classes(rows, "mapkit/x.py") == [
        ("1", "JB", "code_literal"), ("2", "JB-1", "citation"),
        ("3", "JB-12", "code_literal"), ("3", "jb12", "code_literal"),
        ("4", "jb", "code_literal"), ("5", "JB", "code_literal"),
        ("6", "JB-12", "citation"), ("6", "JB", "code_literal")]
    assert {r["rule"] for r in rows if r["class"] == "code_literal"} == {"code-literal"}
    assert _classes(rows, "docs/x.md") == [("1", "JB", "abbreviation")]
    assert _classes(rows, "tests/sim/test_night_shift_gate.py") == [("1", "JB-404", "fixture")]
    assert _classes(rows, "tools/id_sweep.py") == [("1", "JB-1", "frozen")]
    assert "tests/sim/test_night_shift_gate.py" in id_sweep.FIXTURE_FILES


# ------------------------------------------------------ unit 1b: --apply / --verify

ITEMS4 = (("bug", "one", "JB-1"), ("bug", "two", "JB-2"), ("idea", "three", "JB-3"),
          ("idea", "twelve", "JB-12"))
XWALK4 = {"JB-1": ("BUG-1", "bug", "bug-one"), "JB-2": ("BUG-2", "bug", "bug-two"),
          "JB-3": ("IDEA-1", "idea", "idea-three"), "JB-12": ("IDEA-2", "idea", "idea-twelve")}


def _crosswalk(tmp_path, rows=XWALK4, cols=("old_id", "new_id", "kind", "stem", "note")):
    path = tmp_path / "legacy_ids.csv"
    with open(str(path), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for old, (new, kind, stem) in rows.items():
            full = {"old_id": old, "new_id": new, "kind": kind, "stem": stem, "note": ""}
            w.writerow([full[c] for c in cols])
    return path


def _overrides(work, rows):
    path = work / "tools" / "id_sweep_overrides.tsv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(str(path), "w", newline="") as fh:
        w = csv.writer(fh, delimiter="\t", lineterminator="\n")
        w.writerow(id_sweep.OVERRIDE_COLUMNS)
        w.writerows(rows)
    return path


def _snapshot(work):
    files = _git(work, "ls-files").split()
    return {f: (work / f).read_bytes() for f in files}


def _apply(work, xw, capsys, *extra):
    rc = id_sweep.main(["--apply", "--root", str(work), "--crosswalk", str(xw)] + list(extra))
    return rc, capsys.readouterr()


T_U13 = ("JB-1 vs JB-12; JB-12-style and pre-JB-1.\n"
         "Branch night-shift/2026-11-01-JB-12 held.\n"
         "Robot JB_OP_X and JB's frame stay.\n")


def test_apply_is_full_token_single_pass_and_idempotent(tmp_path, capsys):
    """T-U13: ``JB-1`` never rewrites inside ``JB-12``; a second run changes nothing."""
    work = _seed(tmp_path, {"docs/a.md": T_U13}, items=ITEMS4)
    xw = _crosswalk(tmp_path)
    rc, cap = _apply(work, xw, capsys)
    assert rc == 0, cap.err
    assert (work / "docs/a.md").read_text() == (
        "IDEA-2 vs IDEA-2; IDEA-2-style and pre-BUG-1.\n".replace("IDEA-2 vs", "BUG-1 vs")
        + "Branch night-shift/2026-11-01-IDEA-2 held.\n"
        + "Robot JB_OP_X and JB's frame stay.\n")
    assert (work / "map/bug-one.md").read_text().startswith("---\nid: BUG-1\n")
    first = _snapshot(work)
    rc, cap = _apply(work, xw, capsys)
    assert rc == 0, cap.err
    assert _snapshot(work) == first


LISTS = ("range JB-1..2 same kind\n"
         "range JB-1 to JB-3 crosses\r\n"
         "list JB-1/3 and (JB-2, 12, 3)\n"
         "no newline at end JB-12")


def test_apply_expands_lists_and_ranges_and_keeps_line_count(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/l.md": LISTS}, items=ITEMS4)
    before = (work / "docs/l.md").read_bytes()
    rc, cap = _apply(work, _crosswalk(tmp_path), capsys)
    assert rc == 0, cap.err
    after = (work / "docs/l.md").read_bytes()
    assert after == (b"range BUG-1..2 same kind\n"
                     b"range BUG-1, BUG-2, IDEA-1 crosses\r\n"
                     b"list BUG-1/IDEA-1 and (BUG-2, IDEA-2, IDEA-1)\n"
                     b"no newline at end IDEA-2")
    assert after.count(b"\n") == before.count(b"\n") and after.count(b"\r") == before.count(b"\r")
    summary = (work / "temp/id-sweep/apply-summary.md").read_text()
    for shown in ("JB-1..2", "JB-1 to JB-3", "JB-1/3", "JB-2, 12, 3"):
        assert shown in summary


def test_apply_refuses_a_number_missing_from_the_crosswalk(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "ok JB-1\nsee JB-12 here\n"}, items=ITEMS4)
    snap = _snapshot(work)
    short = {k: v for k, v in XWALK4.items() if k != "JB-12"}
    rc, cap = _apply(work, _crosswalk(tmp_path, short), capsys)
    assert rc != 0
    assert "docs/a.md:2:" in cap.err and "JB-12" in cap.err
    assert _snapshot(work) == snap


def test_apply_refuses_a_crosswalk_missing_a_column(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "ok JB-1\n"}, items=ITEMS4)
    snap = _snapshot(work)
    rc, cap = _apply(work, _crosswalk(tmp_path, cols=("old_id", "new_id", "stem")), capsys)
    assert rc != 0 and "kind" in cap.err
    assert _snapshot(work) == snap


INBOX = "parent = `--pathway` (slug or `jb` ID); see JB-1.\n"


def test_override_applies_once_and_a_stale_key_refuses(tmp_path, capsys):
    work = _seed(tmp_path, {".claude/commands/inbox.md": INBOX}, items=ITEMS4)
    _overrides(work, [[".claude/commands/inbox.md", INBOX.rstrip("\n"), "slug or `jb` ID",
                       "slug or Item ID", "loose"]])
    _commit_push(work, "overrides")
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    assert [(r["class"], r["rule"]) for r in _rows(out)
            if r["file"] == ".claude/commands/inbox.md" and r["token"] == "jb"] == [("loose", "override")]
    xw = _crosswalk(tmp_path)
    rc, cap = _apply(work, xw, capsys)
    assert rc == 0, cap.err
    assert (work / ".claude/commands/inbox.md").read_text() == \
        "parent = `--pathway` (slug or Item ID); see BUG-1.\n"
    snap = _snapshot(work)
    rc, cap = _apply(work, xw, capsys)                 # applied, not stale: a no-op
    assert rc == 0, cap.err
    assert _snapshot(work) == snap
    _overrides(work, [[".claude/commands/inbox.md", "a line that is not there", "x", "y", "loose"]])
    snap = _snapshot(work)
    rc, cap = _apply(work, xw, capsys)
    assert rc != 0 and "stale" in cap.err and ".claude/commands/inbox.md" in cap.err
    assert _snapshot(work) == snap


def test_apply_refuses_a_placeholder_with_no_override(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "file it as `JB-<n>`\n"}, items=ITEMS4)
    snap = _snapshot(work)
    rc, cap = _apply(work, _crosswalk(tmp_path), capsys)
    assert rc != 0 and "docs/a.md:1:" in cap.err and "override" in cap.err
    assert _snapshot(work) == snap


def test_verify_fails_on_an_untouched_tree_and_passes_after_apply(tmp_path, capsys):
    fixture = "tests/sim/test_mapkit_ids.py"
    work = _seed(tmp_path, {"docs/a.md": T_U13 + LISTS, fixture: 'assert parse("JB-12") == 12\n'},
                 items=ITEMS4)
    xw = _crosswalk(tmp_path)
    rc = id_sweep.main(["--verify", "--root", str(work), "--crosswalk", str(xw)])
    cap = capsys.readouterr()
    assert rc != 0 and "docs/a.md:1:" in cap.err
    rc, cap = _apply(work, xw, capsys)
    assert rc == 0, cap.err
    snap = _snapshot(work)
    rc = id_sweep.main(["--verify", "--root", str(work), "--crosswalk", str(xw)])
    cap = capsys.readouterr()
    assert rc == 0, cap.err
    assert fixture + ":1" in cap.out          # the fixture line is listed for hand conversion
    assert _snapshot(work) == snap            # verify writes nothing to the tree


# ------------------------------------------------------ P3 audit fixes (F1-F6)

def _classes_of(work, capsys, rel):
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    return [(r["token"], r["class"], r["rule"]) for r in _rows(out) if r["file"] == rel]


def test_f1_trailer_mention_in_prose_is_not_frozen_but_a_trailer_line_is(tmp_path, capsys):
    text = ("file bug JB-1 <title>`), carries **no** `Logbook-Entry:` trailer\n"
            "Logbook-Entry: foo-jb-12\n"
            "  Logbook-Entry: bar-jb-12\n")
    work = _seed(tmp_path, {"docs/g.md": text}, items=ITEMS4)
    got = _classes_of(work, capsys, "docs/g.md")
    assert got[0][1:] == ("citation", "citation"), got
    assert [c for _, c, _ in got[1:]] == ["frozen", "frozen"], got


def _rule_for(text):
    toks, _ = id_sweep.line_tokens("docs/x.md", 1, text)
    return [(t.token, (id_sweep.match_rule(t).name if id_sweep.match_rule(t) else None)) for t in toks]


def test_a_date_suffix_is_not_a_hyphenated_name_but_a_letter_hyphen_is():
    # audit P4: ``-11-05-JB-163`` (after a digit-hyphen) was frozen as if it were a filename
    got = _rule_for("delete `-11-05-JB-163` and Higher-throws-with-JB-144-update and gate-JB-141-run1")
    by = dict(got)
    assert by["JB-163"] != "inside-hyphenated-name", got
    assert by["JB-144-update"] == "inside-hyphenated-name", got
    assert by["JB-141-run1"] == "inside-hyphenated-name", got


def test_an_identity_override_keeps_a_token_a_rewrite_rule_would_take():
    # audit P4 N1/N2: a synthetic fixture range (``JB-1..5``) and a deleted-copies range
    # are history, so --verify must honour an identity row over a range-dots/citation hit
    line = "a clone with JB-1..5; a second pushes JB-6 and JB-7"
    ovr = [id_sweep.Override("logbook/x.md", line, tok, tok, "loose") for tok in ("JB-1..5", "JB-6", "JB-7")]
    idx = id_sweep.index_overrides(ovr)
    toks, kinds = id_sweep.line_tokens("logbook/x.md", 1, line)
    rows, errs = id_sweep.classify(toks, kinds, {1, 2, 3, 4, 5, 6, 7}, idx)
    assert not errs, errs
    assert [r.cls for r in rows if r.cls in id_sweep.REWRITE_CLASSES] == [], rows
    # and without the rows the same line is still rewritable (the rules are unchanged)
    rows, _ = id_sweep.classify(toks, kinds, {1, 2, 3, 4, 5, 6, 7}, None)
    assert [r.cls for r in rows if r.cls in id_sweep.REWRITE_CLASSES], rows


def test_f2_a_list_tail_that_cannot_be_rewritten_is_an_error(tmp_path, capsys):
    snap_files = {"docs/a.md": "ok JB-1\nsee JB-1, 2, 3.\n"}
    work = _seed(tmp_path, snap_files, items=ITEMS4)
    snap = _snapshot(work)
    rc, cap = _apply(work, _crosswalk(tmp_path), capsys)
    assert rc != 0 and "docs/a.md:2:" in cap.err
    assert _snapshot(work) == snap
    (tmp_path / "b").mkdir()
    (tmp_path / "c").mkdir()
    work2 = _seed(tmp_path / "b", {"docs/a.md": "see JB-1, 2 and 3\n"}, items=ITEMS4)
    rc, cap = _apply(work2, _crosswalk(tmp_path), capsys)
    assert rc != 0 and "docs/a.md:1:" in cap.err
    # the well-formed forms still rewrite
    work3 = _seed(tmp_path / "c", {"docs/a.md": "see (JB-1, 2, 3) and JB-1, 2 and JB-3, 12\n"},
                  items=ITEMS4)
    rc, cap = _apply(work3, _crosswalk(tmp_path), capsys)
    assert rc == 0, cap.err
    assert (work3 / "docs/a.md").read_text() == (
        "see (BUG-1, BUG-2, IDEA-1) and BUG-1, BUG-2 and IDEA-1, IDEA-2\n")


def test_f3_dry_run_writes_nothing_and_reports_the_same_counts(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": T_U13 + LISTS}, items=ITEMS4)
    xw = _crosswalk(tmp_path)
    before = _snapshot(work)
    status = _git(work, "status", "--porcelain")
    rc, dry = _apply(work, xw, capsys, "--dry-run", "--out", str(tmp_path / "rep"))
    assert rc == 0, dry.err
    assert _snapshot(work) == before and _git(work, "status", "--porcelain") == status
    rc, real = _apply(work, xw, capsys)
    assert rc == 0, real.err
    nums = lambda s: [x for x in s.split() if x.isdigit()][:3]  # noqa: E731
    assert nums(dry.out) == nums(real.out), (dry.out, real.out)


def test_f3_dry_run_refuses_like_the_real_apply_and_needs_apply(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "see JB-12 here\n"}, items=ITEMS4)
    short = {k: v for k, v in XWALK4.items() if k != "JB-12"}
    rc, cap = _apply(work, _crosswalk(tmp_path, short), capsys, "--dry-run")
    assert rc != 0 and "docs/a.md:1:" in cap.err
    with pytest.raises(SystemExit):
        id_sweep.main(["--census", "--dry-run", "--root", str(work)])
    capsys.readouterr()
    rc = id_sweep.main(["--verify", "--dry-run", "--root", str(work),
                        "--crosswalk", str(_crosswalk(tmp_path))])
    assert rc != 0                       # verify accepts the flag (it never writes)
    capsys.readouterr()


def test_f4_an_explicit_missing_overrides_path_is_an_error(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "x JB-1\n"}, items=ITEMS4)
    rc = id_sweep.main(["--census", "--root", str(work), "--overrides", str(tmp_path / "nope.tsv")])
    cap = capsys.readouterr()
    assert rc != 0 and "nope.tsv" in cap.err
    rc = id_sweep.main(["--census", "--root", str(work)])      # the default path may be absent
    assert rc == 0, capsys.readouterr().err


def test_f4_an_overrides_row_with_the_wrong_column_count_names_its_row(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "x JB-1\n"}, items=ITEMS4)
    p = tmp_path / "o.tsv"
    p.write_text("path\tline\told\tnew\tclass\n"
                 "docs/a.md\tx JB-1\tJB\tItem\tloose\n"
                 "docs/a.md\tx JB-1\tJB\tItem\tloose\tEXTRA\n"
                 "docs/a.md\tshort\n")
    rc = id_sweep.main(["--census", "--root", str(work), "--overrides", str(p)])
    cap = capsys.readouterr()
    assert rc != 0 and "o.tsv:3:" in cap.err and "o.tsv:4:" in cap.err and "o.tsv:2:" not in cap.err


def test_f5_skipped_files_are_counted_by_reason_and_listed(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "x JB-1\n", "pic.png": "jb\n", "bin.dat": "jb\0\n",
                            "keep.txt": "plain\n"}, items=ITEMS4)
    (work / "link.md").symlink_to("docs/a.md")
    _git(work, "add", "link.md")
    _commit_push(work, "link")
    rc, out, cap = _census(work, capsys)
    assert rc == 0, cap.err
    skipped = {r["path"]: r["reason"] for r in csv.DictReader(open(str(out / "skipped.csv"), newline=""))}
    assert skipped["pic.png"] == "suffix" and skipped["bin.dat"] == "binary"
    assert skipped["link.md"] == "symlink" and "docs/a.md" not in skipped
    summary = (out / "summary.md").read_text()
    assert "Skipped files" in summary and "symlink: 1" in summary and "suffix: 1" in summary


def test_f5_skip_predicate_is_the_tools_and_the_census_agrees_with_it(tmp_path, capsys):
    work = _seed(tmp_path, {"docs/a.md": "x JB-1\n", "pic.png": "jb\n"}, items=ITEMS4)
    files = _git(work, "ls-files").split()
    kept = set(id_sweep.tracked_text_files(str(work)))
    assert kept == {f for f in files if id_sweep.skip_reason(str(work), f) is None}
    assert "pic.png" not in kept


def test_f6_a_letter_placeholder_in_a_filename_is_not_frozen_by_a_path_rule(tmp_path, capsys):
    text = ("see JB-n-question.md and JB-<n>-question.md here\n"
            "real logbook/2026-11-01-jb1-thing.md and jb-12-fix.md stay\n")
    work = _seed(tmp_path, {".claude/commands/x.md": text}, items=ITEMS4)
    got = _classes_of(work, capsys, ".claude/commands/x.md")
    cls = {t: c for t, c, _ in got}
    assert cls["JB-n-question"] == "placeholder" and cls["JB-<n>-question"] == "placeholder", got
    assert cls["jb1-thing"] == "frozen" and cls["jb-12-fix"] == "frozen"


def test_the_crosswalk_and_the_cutover_proofs_are_frozen_like_the_sweeper(tmp_path, capsys):
    """P4: once committed, mapkit/legacy_ids.csv (every old ID, by design) and the proof scripts
    under tools/probes/id_cutover/ (they name old IDs and old file names in their own text) are
    tracked and so scanned; without a whole-file freeze --verify reads the crosswalk as 212
    unrewritten citations and --apply refuses the proofs' placeholders."""
    files = {"mapkit/legacy_ids.csv": "old_id,new_id,kind,stem\nJB-1,BUG-1,bug,bug-one\n",
             "tools/probes/id_cutover/t_x.py": "# allows gate-JB-141-run1.log and any JB-<n> here\n"}
    work = _seed(tmp_path, files, items=ITEMS4)
    for rel in files:
        got = [(t, c, r) for t, c, r in _classes_of(work, capsys, rel) if c != "per_kind"]
        assert got and all((c, r) == ("frozen", "cutover-self") for _, c, r in got), (rel, got)
