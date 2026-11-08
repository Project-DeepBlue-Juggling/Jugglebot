#!/usr/bin/env python3
"""T-R6 zero count: no old JB-<n> Item ID survives outside the named allowances, and the map is consistent.

Plan row: plans/active/item-id-per-kind.md section 4 P4, proof T-R6. (a) one git grep -w for every
old_id of mapkit/legacy_ids.csv; each hit must sit in a whole-file freeze, a fixture residue line
listed by the unit 2a/2b reports, a code_literal line of temp/id-sweep/summary.md, an override row
that kept its token (old == new), or the crosswalk itself. (b) `python -m mapkit ids --check` is
rc 0 with empty output (before the renumber commit the only allowed lines are `changed: ... no 'map: renumber' commit`; STRICT_IDS=1 drops that allowance). (c) every map/*.md has an id: line and the counts equal the crosswalk rows.
(d) per kind the id numbers are exactly 1..N with N the crosswalk's count. (e) the id: values and
the crosswalk's new_id column are the same set. Env: ID_MIGRATION_DIR (unit reports, default the
main checkout's temp/id-migration). Evidence: temp/probes/id_cutover/t_r6_zero_count.txt.
Exit 0 = zero unallowed hits and every check holds.
"""
from __future__ import annotations

import collections
import csv
import glob
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import _common as C  # noqa: E402

EV = C.OUT / "t_r6_zero_count.txt"
MIG = os.environ.get("ID_MIGRATION_DIR", "/home/jetson/Desktop/Jugglebot/temp/id-migration")
FROZEN = {
    "plans/active/item-id-per-kind.md", "plans/archived/map-item-ids.md", "docs/adr/0015-per-kind-item-ids.md",
    "logbook/2026-10-26-map-item-ids-p1-p2-ids-module-and-backfill.md",
    "logbook/2026-10-26-map-item-ids-p3-resolution-item-new.md",
    "logbook/2026-11-07-item-id-per-kind-p0-glossary-and-adr.md",
    "logbook/2026-11-07-item-id-per-kind-p1-dual-form-mapkit-night-shift.md",
    "logbook/2026-11-07-item-id-per-kind-p2-map-app-both-forms.md",
    "logbook/2026-11-08-item-id-per-kind-p3-sweeper-and-census.md",
    "tools/id_sweep.py", "tools/id_sweep_overrides.tsv", "tests/sim/test_id_sweep.py",
    "map/bug-map-app-allocates-jb-ids-from-its-own-clone-and-collides.md",  # body frozen; its id: line is rewritten
}


def strip_parens(s):
    out, d = [], 0
    for ch in s:
        if ch == "(":
            d += 1
        elif ch == ")":
            d = max(0, d - 1)
        elif d == 0:
            out.append(ch)
    return "".join(out)


# the three real, named artifacts (an Attempt label, its debrief entry, a night-shift gate log), pinned by
# file:line so a new "letter-hyphen-JB-n-letter" shape elsewhere is NOT waved through (audit P4 P3)
HYPHENATED_SITES = {
    ("logbook/2026-11-02-hand-release-step.md", 211),
    ("logbook/2026-11-02-sitting-debrief.md", 25),
    ("logbook/2026-11-03-night-shift.md", 43),
}
PROOF_SELF = "tools/probes/id_cutover/"  # these scripts name old IDs and file names in their own text


def hyphenated_name_only(text, olds_set):
    """Every old ID on the line sits inside a hyphen-joined real file or directory name, e.g.
    `gate-JB-141-run1.log` (kind inside-hyphenated-name of the census): it names an artifact, not an Item."""
    found = [m for m in re.finditer(r"JB-\d+", text) if m.group(0) in olds_set]
    return bool(found) and all(
        re.search(r"[A-Za-z]-$", text[:m.start()]) and re.match(r"-[A-Za-z]", text[m.end():]) for m in found)


def residue_lines():
    """(path, line) pairs from the 'Residue' sections of the unit 2a/2b reports."""
    got = set()
    for rep in ("p4-unit2a-report.md", "p4-unit2b-report.md"):
        p = os.path.join(MIG, rep)
        if not os.path.exists(p):
            continue
        sect = open(p).read().split("## Residue", 1)[-1]
        for ln in sect.splitlines():
            if not ln.startswith("- "):
                continue
            cur = None
            for tok in re.findall(r"(tests/[\w./]+\.py):|:?(\d+)", strip_parens(ln)):
                if tok[0]:
                    cur = tok[0]
                elif cur:
                    got.add((cur, int(tok[1])))
    return got


def code_literal_lines():
    p = C.ROOT / "temp/id-sweep/summary.md"
    got = set()
    if p.exists():
        sect = p.read_text().split("## code_literal", 1)[1].split("\n## ", 1)[0]
        for m in re.finditer(r"^- (\S+?):(\d+) ", sect, re.M):
            got.add((m.group(1), int(m.group(2))))
    return got


def kept_override_rows():
    rows = collections.defaultdict(list)
    with open(C.ROOT / "tools/id_sweep_overrides.tsv", newline="") as f:
        for r in csv.DictReader(f, delimiter="\t"):  # CSV-style quotes, as id_sweep.read_overrides reads it
            if r["old"] == r["new"]:
                rows[r["path"]].append(r["line"].strip())
    return rows


def main():
    C.OUT.mkdir(parents=True, exist_ok=True)
    EV.write_text("")
    xw = C.crosswalk()
    olds = [r["old_id"] for r in xw]
    olds_set = set(olds)
    res, lit, kept = residue_lines(), code_literal_lines(), kept_override_rows()
    cmd = ["git", "grep", "-nI", "-w"]
    for o in olds:
        cmd += ["-e", o]
    cmd += ["--", ".", ":!temp"]
    out = subprocess.run(cmd, cwd=C.ROOT, stdout=subprocess.PIPE).stdout.decode("utf8", "surrogateescape")
    why = collections.Counter()
    bad = []
    for hit in out.splitlines():
        path, n, text = hit.split(":", 2)
        n = int(n)
        if path in FROZEN:
            why["frozen"] += 1
        elif path == "mapkit/legacy_ids.csv":
            why["crosswalk"] += 1  # every old ID, by design (tracked once staged)
        elif path.startswith(PROOF_SELF):
            why["proof-self"] += 1
        elif (path, n) in res:
            why["fixture-residue"] += 1
        elif (path, n) in lit:
            why["code_literal"] += 1
        elif text.strip() in kept.get(path, []):
            why["override-kept"] += 1
        elif (path, n) in HYPHENATED_SITES and hyphenated_name_only(text, olds_set):
            why["hyphenated-real-name"] += 1
        else:
            bad.append("%s:%d %s" % (path, n, text.strip()[:160]))
    problems = ["(a) unallowed hit " + b for b in bad]
    # (b)
    r = subprocess.run([sys.executable, "-m", "mapkit", "ids", "--check"], cwd=C.ROOT,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    lines = [x for x in r.stdout.decode().splitlines() if x.strip()]
    pre_commit = [x for x in lines if x.startswith("changed:") and "no 'map: renumber' commit" in x]
    if os.environ.get("STRICT_IDS") == "1":
        pre_commit = []  # after the renumber commit: rc 0 and empty output, no allowance
    if (r.returncode != 0 or lines) and len(pre_commit) != len(lines):
        problems.append("(b) ids --check rc=%d, %d lines not of the pre-commit kind: %r" % (
            r.returncode, len(lines) - len(pre_commit), [x for x in lines if x not in pre_commit][:3]))
    elif lines and not pre_commit:
        problems.append("(b) ids --check output %r" % lines[:3])
    # (c) (d) (e)
    files = sorted(glob.glob(str(C.ROOT / "map" / "*.md")))
    ids = []
    for f in files:
        m = re.search(r"^id: (\S+)", open(f, encoding="utf8").read(), re.M)
        if m:
            ids.append(m.group(1))
    if not (len(ids) == len(files) == len(xw)):
        problems.append("(c) id lines %d, map files %d, crosswalk rows %d" % (len(ids), len(files), len(xw)))
    per = collections.defaultdict(list)
    for i in ids:
        mm = re.fullmatch(r"([A-Z]+)-(\d+)", i)
        if not mm:
            problems.append("(d) malformed id %r" % i)
            continue
        per[mm.group(1)].append(int(mm.group(2)))
    want = collections.Counter(r["new_id"].split("-")[0] for r in xw)
    for pre in sorted(set(per) | set(want)):
        if sorted(per.get(pre, [])) != list(range(1, want[pre] + 1)):
            problems.append("(d) %s numbers are not exactly 1..%d" % (pre, want[pre]))
    if set(ids) != {r["new_id"] for r in xw} or len(set(ids)) != len(ids):
        problems.append("(e) id: values and crosswalk new_id differ")
    with open(EV, "a") as f:
        f.writelines(x + "\n" for x in problems)
    C.verdict("T-R6", not problems, "old-ID hits=%d allowed %s unallowed=%d; ids --check rc=%d (%d pre-commit 'changed:' lines); map files=%d id lines=%d "
              "crosswalk rows=%d per kind %s; problems=%d" % (
                  len(out.splitlines()), dict(why), len(bad), r.returncode, len(pre_commit), len(files), len(ids), len(xw),
                  dict(sorted(want.items())), len(problems)), EV)


if __name__ == "__main__":
    main()
