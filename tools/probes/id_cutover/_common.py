"""Shared helpers for the per-kind Item ID cutover proofs (T-R2..T-R6).

Plan row: plans/active/item-id-per-kind.md section 4 P4. Pre-sweep tree = git HEAD,
post-sweep tree = the working tree (nothing committed yet). Run from the repo root.
"""
from __future__ import annotations

import csv
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(subprocess.check_output(["git", "rev-parse", "--show-toplevel"]).decode().strip())
OUT = ROOT / "temp" / "probes" / "id_cutover"
TOKEN = re.compile(r"\b(?:BUG|FEAT|IDEA|PWAY|TKT)-\d+\b")
OLD = re.compile(r"\bJB-(\d+)\b")

# Files the P4 units edited by hand or generated, so they are not swept-only.
FIXTURE_FILES = {
    "tests/sim/" + n for n in (
        "test_mapkit_docs.py", "test_mapkit_ids.py", "test_mapkit_item_ids_cli.py",
        "test_mapkit_similar.py", "test_night_shift_candidate.py", "test_night_shift_deploy.py",
        "test_night_shift_gate.py", "test_night_shift_preflight.py", "test_night_shift_report.py",
        "test_night_shift_runner.py", "test_night_shift_settings.py", "test_question_tickets.py",
        "test_sitting_sections.py")
}
HAND_FILES = {
    ".gitignore", "tools/id_sweep_overrides.tsv", "plans/active/item-id-per-kind.md",
    "map/feature-update-item-naming-can.md", "mapkit/itemcli.py",
}


ANY_ID = re.compile(r"\b(?:JB|BUG|FEAT|IDEA|PWAY|TKT)-\d+\b")


def tokenwise_ok(a, b, inv):
    """Old text a and swept text b: same text around the IDs; each ID kept or inverted."""
    if ANY_ID.sub("@", a) != ANY_ID.sub("@", b):
        return False
    return all(x == y or inv.get(y) == x for x, y in zip(ANY_ID.findall(a), ANY_ID.findall(b)))


def crosswalk():
    with open(ROOT / "mapkit" / "legacy_ids.csv", newline="") as f:
        return list(csv.DictReader(f))


def inverse_map():
    return {r["new_id"]: r["old_id"] for r in crosswalk()}


def invert(text, inv):
    return TOKEN.sub(lambda m: inv.get(m.group(0), m.group(0)), text)


def changed_files():
    out = subprocess.check_output(["git", "diff", "--name-only", "HEAD"], cwd=ROOT).decode()
    return [n for n in out.splitlines() if n]


def head_text(path):
    try:
        return subprocess.check_output(["git", "show", "HEAD:" + path], cwd=ROOT,
                                       stderr=subprocess.DEVNULL).decode("utf8", "surrogateescape")
    except subprocess.CalledProcessError:
        return None


def wt_text(path):
    p = ROOT / path
    return p.read_bytes().decode("utf8", "surrogateescape") if p.is_file() else None


def verdict(name, ok, detail, evidence):
    line = "%s %s: %s" % (name, "PASS" if ok else "FAIL", detail)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(evidence, "a") as f:
        f.write(line + "\n")
    print(line)
    sys.exit(0 if ok else 1)
