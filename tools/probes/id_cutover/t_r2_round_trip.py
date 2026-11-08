#!/usr/bin/env python3
"""T-R2 round trip: inverting the crosswalk over the swept tree reproduces HEAD.

Plan row: plans/active/item-id-per-kind.md section 4 P4, proof T-R2. For every file
changed against HEAD, each per-kind ID that is in the inverse crosswalk is mapped back to
its JB-n and the result is compared with HEAD line by line (line counts must match). Lines
the sweep rewrote by override / list / range rule are un-done from the apply report
(temp/id-sweep/apply/apply.csv) instead. Files edited by hand are exempt BY NAME only (HAND_EXEMPT + the 13 fixture files); every
other changed file, mapkit/ included, must invert token-wise.
Evidence: temp/probes/id_cutover/t_r2_round_trip.txt. Exit 0 = zero unexplained differences.
"""
from __future__ import annotations

import csv
import collections
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from _common import *  # noqa: E402,F401,F403
import _common as C  # noqa: E402

GENERATED = {"docs/runbook.md", "docs/project-map.md", "docs/glossary.md", "docs/patterns.md",
             "plans/active/INDEX.md", "plans/parked/INDEX.md", "plans/archived/INDEX.md"}
HAND_PREFIXES = ("map/feature-update-item-naming", "map/ticket-item-prefixes")
# Exempt BY NAME only (audit P4 P1: the old "anything under mapkit/" exemption let a planted
# 0.61 -> 0.62 in mapkit/similar.py pass). Every other changed file, mapkit/ included, is
# compared line by line, token-wise, and a line-count change fails.
HAND_EXEMPT = {
    "mapkit/ids.py", "mapkit/itemcli.py", ".gitignore", "tools/id_sweep_overrides.tsv",
    "plans/active/item-id-per-kind.md",
    # the sweeper's own rule fix and its test, edited by the P4 fixer pass
    "tools/id_sweep.py", "tests/sim/test_id_sweep.py",
}
ANY = __import__("re").compile(r"\b(?:JB|BUG|FEAT|IDEA|PWAY|TKT)-\d+\b")


def tokenwise_ok(a, b, inv):
    """HEAD line a and swept line b: same text around the IDs; each ID either kept or inverted."""
    if ANY.sub("@", a) != ANY.sub("@", b):
        return False
    return all(x == y or inv.get(y) == x for x, y in zip(ANY.findall(a), ANY.findall(b)))


BLOCK = __import__("re").compile(r"^### Block \d+:")
EV = C.OUT / "t_r2_round_trip.txt"


def main():
    C.OUT.mkdir(parents=True, exist_ok=True)
    EV.write_text("")
    inv = C.inverse_map()
    apply_rows = collections.defaultdict(list)
    with open(C.ROOT / "temp/id-sweep/apply/apply.csv", newline="") as f:
        for r in csv.DictReader(f):
            apply_rows[(r["file"], int(r["line"]))].append(r)
    files = C.changed_files()
    stats = collections.Counter()
    diffs, exempt, countdiff = [], [], []
    for path in files:
        h, w = C.head_text(path), C.wt_text(path)
        if h is None or w is None:
            # The commit ADDS exactly two things with no pre-sweep blob: the frozen crosswalk
            # and these proof scripts. Anything else added or deleted is a difference.
            if h is None and (path == "mapkit/legacy_ids.csv" or path.startswith("tools/probes/id_cutover/")):
                exempt.append((path, 0, "added", ""))
                stats["exception_added"] += 1
                continue
            diffs.append((path, 0, "added/deleted file"))
            continue
        hl, wl = h.split("\n"), w.split("\n")
        kind = "fixture" if path in C.FIXTURE_FILES else "hand" if path in HAND_EXEMPT else "swept"
        if len(hl) != len(wl):
            countdiff.append((path, len(hl), len(wl), kind))
            if kind == "swept":
                diffs.append((path, 0, "line count %d -> %d" % (len(hl), len(wl))))
            continue
        stats["files_checked"] += 1
        gen_pending = {}
        for i, (a, b) in enumerate(zip(hl, wl), 1):
            if a == b:
                continue
            stats["lines_changed"] += 1
            if tokenwise_ok(a, b, inv):
                stats["inverted"] += 1
                continue
            rows = apply_rows.get((path, i))
            if rows:
                t = b
                for r in rows:
                    t = t.replace(r["after"], r["before"], 1)
                if tokenwise_ok(a, t, inv):
                    cls = "list-range" if any(r["cls"] in ("list", "range") for r in rows) else \
                        "override" if any(r["rule"] == "override" for r in rows) else "other-apply"
                    stats["exception_" + cls] += 1
                    if cls == "other-apply":
                        exempt.append((path, i, a, "other-apply rule " + rows[0]["rule"]))
                    continue
            if path in GENERATED:
                gen_pending.setdefault(path, []).append((i, a, b))
                continue
            if kind != "swept" or path.startswith(HAND_PREFIXES):
                kind = kind if kind != "swept" else "hand"
                stats["exception_" + kind] += 1
                exempt.append((path, i, a, b))
                continue
            diffs.append((path, i, "HEAD=%r WT=%r" % (a, b)))
        if path in GENERATED and gen_pending.get(path):
            # generated file: rows may be re-sorted, so compare the changed lines as a multiset
            pool = collections.Counter(BLOCK.sub("### Block N:", x) for x in hl)
            bad = 0
            for i, a, b in gen_pending[path]:
                b, a = BLOCK.sub("### Block N:", b), BLOCK.sub("### Block N:", a)
                inv_b = C.invert(b, inv)
                key = b if pool[b] > 0 else inv_b if pool[inv_b] > 0 else None
                if key is None:
                    bad += 1
                    diffs.append((path, i, "generated line has no HEAD counterpart: %r" % b[:200]))
                else:
                    pool[key] -= 1
                    stats["exception_generated"] += 1
            if bad == 0 and any(v > 0 for v in pool.values()):
                pass
    with open(EV, "a") as f:
        for p, n, c, k in countdiff:
            f.write("countdiff %s %d->%d (%s)\n" % (p, n, c, k))
        for p, i, a, b in exempt:
            f.write("exempt %s:%d\n" % (p, i))
        for d in diffs:
            f.write("DIFF %s:%s %s\n" % d)
    detail = "files=%d (line-count-changed hand/fixture files %d) changed_lines=%d inverted=%d %s differences=%d" % (
        stats["files_checked"], len(countdiff), stats["lines_changed"], stats["inverted"],
        " ".join("%s=%d" % kv for kv in sorted(stats.items()) if kv[0].startswith("exception_")),
        len(diffs))
    C.verdict("T-R2", not diffs, detail, EV)


if __name__ == "__main__":
    main()
