#!/usr/bin/env python3
"""Prototype of a SEMANTIC gate hash: the eight gated files hashed by parse tree
with docstrings stripped (comments never reach the AST), beside the committed
text hash ``admissible.gate_hash()``.

Shows that a comment or docstring edit leaves the semantic hash unchanged while
the text hash flips (today that flip invalidates the whole box and discards a
running sweep), and that a code edit flips both. It is a prototype, not a
replacement: a behavioural fingerprint (the gate's outputs on the box's own
certification points) is the sound version, see the logbook entry.

usage: python tools/probes/admissible_obviation/semantic_gate_hash.py

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
"""
import ast, hashlib, os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
from jugglebot.motion.skills import admissible as ab                  # noqa: E402

PKG = os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot', 'jugglebot')


def gated_path(parts):
    if parts == ('segments.py',):
        return os.path.join(PKG, 'motion', 'skills', 'segments.py')
    if parts == ('hardware_config.py',):
        return os.path.join(PKG, 'hardware_config.py')
    return os.path.join(PKG, 'motion', *parts)


def strip_docstrings(tree):
    for node in ast.walk(tree):
        if (isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Module))
                and node.body and isinstance(node.body[0], ast.Expr)
                and isinstance(getattr(node.body[0], 'value', None), ast.Constant)
                and isinstance(node.body[0].value.value, str)):
            node.body = node.body[1:] or [ast.Pass()]
    return tree


def semantic_hash(sources):
    h = hashlib.sha256()
    for src in sources:
        h.update(ast.dump(strip_docstrings(ast.parse(src)), include_attributes=False).encode())
    return h.hexdigest()[:12]


def main():
    srcs = [open(gated_path(p)).read() for p in ab._GATED_FILES]
    print('committed text gate_hash', ab.gate_hash(), '| semantic', semantic_hash(srcs))
    edited = list(srcs)
    edited[4] = edited[4].replace('# ── Window kinds', '# -- Window kinds (comment edited)')
    assert edited[4] != srcs[4]
    print('after a comment edit to unified_cycle.py: semantic', semantic_hash(edited),
          '| text', hashlib.sha256(''.join(edited).encode()).hexdigest()[:12], '(text changes, semantic does not)')
    edited = list(srcs)
    edited[4] = edited[4].replace("LAUNCH = 'launch'", "LAUNCH = 'launchx'")
    print('after a code edit: semantic', semantic_hash(edited), '(flips)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
