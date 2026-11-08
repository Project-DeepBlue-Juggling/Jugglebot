#!/usr/bin/env python3
"""T-R3 Python code unchanged: the sweep touched comments and strings only.

Plan row: plans/active/item-id-per-kind.md section 4 P4, proof T-R3. For every changed .py
outside tests/, mapkit/, tools/night_shift*, tools/id_sweep* and tools/probes/id_cutover/,
HEAD and the working tree must have (a) an identical AST with every str constant masked and
(b) an identical token stream with comments, NL/NEWLINE/ENCODING dropped and strings masked;
(c) every changed string constant must be explained by the crosswalk (old text with its JB-n
IDs swapped, or the apply report's before/after spans un-done). Evidence:
temp/probes/id_cutover/t_r3_python_ast.txt. Exit 0 = no file differs and no string unexplained.
"""
from __future__ import annotations

import ast
import collections
import csv
import io
import os
import sys
import tokenize

sys.path.insert(0, os.path.dirname(__file__))
import _common as C  # noqa: E402

EV = C.OUT / "t_r3_python_ast.txt"
SKIP = ("tests/", "mapkit/", "tools/night_shift", "tools/id_sweep", "tools/probes/id_cutover/")
STRTOK = {tokenize.STRING} | {getattr(tokenize, n) for n in ("FSTRING_START", "FSTRING_MIDDLE", "FSTRING_END")
                              if hasattr(tokenize, n)}


class Mask(ast.NodeTransformer):
    def visit_Constant(self, node):
        if isinstance(node.value, str):
            return ast.copy_location(ast.Constant(value="<S>"), node)
        return node


def ast_sig(src):
    return ast.dump(Mask().visit(ast.parse(src)))


def tok_sig(src):
    out = []
    for t in tokenize.generate_tokens(io.StringIO(src).readline):
        if t.type in (tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.ENCODING):
            continue
        out.append((t.type, "<S>" if t.type in STRTOK else t.string))
    return out


def strings(src):
    return [(n.lineno, n.value) for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Constant)
            and isinstance(n.value, str)]


def main():
    C.OUT.mkdir(parents=True, exist_ok=True)
    EV.write_text("")
    inv = C.inverse_map()
    apply_rows = collections.defaultdict(list)
    with open(C.ROOT / "temp/id-sweep/apply/apply.csv", newline="") as f:
        for r in csv.DictReader(f):
            apply_rows[r["file"]].append(r)
    per_dir = collections.Counter()
    fails, changed_strings, unexplained = [], 0, []
    for path in C.changed_files():
        if not path.endswith(".py") or path.startswith(SKIP):
            continue
        h, w = C.head_text(path), C.wt_text(path)
        top = path.split("/")[0]
        per_dir[top] += 1
        try:
            ok_a = ast_sig(h) == ast_sig(w)
            ok_b = tok_sig(h) == tok_sig(w)
        except Exception as e:  # unparsable: cannot prove
            fails.append("%s: cannot parse (%s)" % (path, e))
            continue
        if not ok_a:
            fails.append("%s: AST (strings masked) differs" % path)
        if not ok_b:
            fails.append("%s: token stream differs" % path)
        if not (ok_a and ok_b):
            continue
        for (ln, a), (_, b) in zip(strings(h), strings(w)):
            if a == b:
                continue
            changed_strings += 1
            t = b
            for r in apply_rows.get(path, []):
                t = t.replace(r["after"], r["before"])
            if not (C.tokenwise_ok(a, b, inv) or C.tokenwise_ok(a, t, inv)
                    or all(C.tokenwise_ok(x, y, inv) or C.tokenwise_ok(x, z, inv)
                           for x, y, z in zip(a.split("\n"), b.split("\n"), t.split("\n")))):
                unexplained.append("%s:%d %r -> %r" % (path, ln, a[:120], b[:120]))
    with open(EV, "a") as f:
        for x in fails + ["UNEXPLAINED " + u for u in unexplained]:
            f.write(x + "\n")
    detail = "py files=%d (%s) changed string constants=%d unexplained=%d ast/token failures=%d" % (
        sum(per_dir.values()), ", ".join("%s %d" % kv for kv in sorted(per_dir.items())),
        changed_strings, len(unexplained), len(fails))
    C.verdict("T-R3", not fails and not unexplained, detail, EV)


if __name__ == "__main__":
    main()
