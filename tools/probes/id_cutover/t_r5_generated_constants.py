#!/usr/bin/env python3
"""T-R5 wire and config unchanged: generated constants and firmware sources differ in prose only.

Plan row: plans/active/item-id-per-kind.md section 4 P4, proof T-R5. Every changed line under
config/, both firmware trees and tools/probes/teensy_link_profiling/ (git diff HEAD -U0) must be a comment line or a
line whose only change is an Item ID (token-wise, crosswalk inverse) sitting inside a string
literal or after a // or # comment marker. The generated Python modules are also loaded at
HEAD and in the working tree and every non-string constant compared, and MAGIC and
PROTOCOL_VERSION are compared in the generated header, the generated module and teensy_link/.
Evidence: temp/probes/id_cutover/t_r5_generated_constants.txt. Exit 0 = nothing else changed.
"""
from __future__ import annotations

import collections
import enum
import importlib.util
import os
import re
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
import _common as C  # noqa: E402

EV = C.OUT / "t_r5_generated_constants.txt"
PATHS = ["config", "ros_ws/src/jugglebot/Teensy_code_canbridge", "ros_ws/src/jugglebot/Teensy_code_platform",
         "tools/probes/teensy_link_profiling"]
COMMENT = re.compile(r"^\s*(#|//|/\*|\*)")
MARKS = re.compile(r"^(MAGIC|PROTOCOL_VERSION)\b|^\s*(?:constexpr\s+\w+\s+)?(MAGIC|PROTOCOL_VERSION)\s*=", re.M)


def token_inside_text(line, pat_a):
    """True when every changed ID in `line` is in a string literal or a trailing comment."""
    for m in C.ANY_ID.finditer(line):
        before = line[:m.start()]
        if "//" in before or "#" in before or before.count('"') % 2 == 1 or before.count("'") % 2 == 1:
            continue
        if "/*" in before:
            continue
        return False
    return True


def hunks():
    out = subprocess.check_output(["git", "diff", "HEAD", "-U0", "--"] + PATHS, cwd=C.ROOT).decode(
        "utf8", "surrogateescape")
    fn, minus, plus = None, [], []
    pairs = []
    for l in out.split("\n") + ["@@END"]:
        if l.startswith("+++ "):
            fn = l[6:]
        elif l.startswith("--- "):
            continue
        elif l.startswith("@@"):
            if minus or plus:
                pairs.append((fn, minus, plus))
            minus, plus = [], []
        elif l.startswith("-"):
            minus.append(l[1:])
        elif l.startswith("+"):
            plus.append(l[1:])
    return pairs


def mask(v, depth=0):
    if isinstance(v, str):
        return "<S>"
    if isinstance(v, (list, tuple)):
        return type(v).__name__ + str([mask(x, depth + 1) for x in v])
    if isinstance(v, dict):
        return "{" + ",".join("%r:%s" % (k if not isinstance(k, str) else "<S>", mask(x, depth + 1))
                              for k, x in sorted(v.items(), key=lambda kv: repr(kv[0]))) + "}"
    if isinstance(v, type):
        if issubclass(v, enum.Enum):
            return "enum %s %s" % (v.__name__, [(m.name, m.value) for m in v])
        fields = getattr(v, "__dataclass_fields__", None)
        return "class %s %s" % (v.__name__, sorted((k, str(f.type)) for k, f in fields.items()) if fields else "")
    if isinstance(v, struct.Struct):
        return "Struct " + v.format
    if callable(v) or isinstance(v, type(sys)):
        return "<callable>"
    return re.sub(r" at 0x[0-9a-f]+", "", repr(v))


def load(path, text, tag):
    d = tempfile.mkdtemp(prefix="tr5_")
    p = os.path.join(d, os.path.basename(path))
    open(p, "w").write(text)
    name = "tr5_%s_%s" % (tag, os.path.basename(path)[:-3])
    spec = importlib.util.spec_from_file_location(name, p)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return {k: mask(v) for k, v in vars(mod).items() if not k.startswith("__")}


def main():
    C.OUT.mkdir(parents=True, exist_ok=True)
    EV.write_text("")
    inv = C.inverse_map()
    cls = collections.Counter()
    bad = []
    for fn, minus, plus in hunks():
        if len(minus) != len(plus):
            bad.append("%s: hunk with %d removed / %d added lines" % (fn, len(minus), len(plus)))
            continue
        for a, b in zip(minus, plus):
            if not C.tokenwise_ok(a, b, inv):
                bad.append("%s: change beyond Item IDs: %r -> %r" % (fn, a[:150], b[:150]))
            elif COMMENT.match(a) and COMMENT.match(b):
                cls["comment-line"] += 1
            elif token_inside_text(b, a):
                cls["id-in-string-or-trailing-comment"] += 1
            else:
                bad.append("%s: ID changed outside a string or comment: %r" % (fn, b[:150]))
    # generated Python modules: non-string constants equal
    gen = subprocess.check_output(["git", "ls-files", "config/generated"], cwd=C.ROOT).decode().split()
    gen += [c for c in C.changed_files() if c.startswith("tools/probes/teensy_link_profiling/")]
    mods = 0
    for path in [g for g in gen if g.endswith(".py")]:
        try:
            a = load(path, C.head_text(path), "head")
            b = load(path, C.wt_text(path), "wt")
        except Exception as e:
            bad.append("%s: cannot load (%s)" % (path, str(e)[:120]))
            continue
        mods += 1
        for k in sorted(set(a) | set(b)):
            if a.get(k) != b.get(k):
                bad.append("%s: constant %s differs" % (path, k))
    # MAGIC / PROTOCOL_VERSION
    marks = 0
    for path in ["config/generated/udp_protocol.h", "config/generated/udp_protocol.py"] + [
            g for g in subprocess.check_output(["git", "ls-files", "teensy_link"], cwd=C.ROOT).decode().split()
            if g.endswith(".py")]:
        def vals(t):
            return [re.sub(r"\s*(//|#).*$", "", m.group(0)).strip() for m in
                    re.finditer(r"^.*\b(MAGIC|PROTOCOL_VERSION)\s*(=|:)[^\n]*$", t or "", re.M)]
        a, b = vals(C.head_text(path)), vals(C.wt_text(path))
        marks += len(b)
        if a != b:
            bad.append("%s: MAGIC/PROTOCOL_VERSION lines differ %r vs %r" % (path, a, b))
    with open(EV, "a") as f:
        f.writelines(x + "\n" for x in bad)
    C.verdict("T-R5", not bad, "hunk lines %s; python modules compared=%d; MAGIC/PROTOCOL_VERSION lines checked=%d; "
              "problems=%d" % (dict(cls), mods, marks, len(bad)), EV)


if __name__ == "__main__":
    main()
