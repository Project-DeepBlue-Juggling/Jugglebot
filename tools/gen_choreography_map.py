#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the ROS2 topic/service/action choreography map for the Python nodes.

Static analysis only — this parses every module under
``ros_ws/src/jugglebot/jugglebot/`` with :mod:`ast` and never imports ROS2 (so it
runs anywhere, including under the mocked-ROS conftest and on a machine with no
ROS install).  Not just ``*_node.py``: see :func:`scan_nodes`.

Why static and not introspective: importing the nodes would pull rclpy and the
custom message package, and a runtime ``get_topic_names_and_types()`` only sees
what a *live* graph happens to have wired up at that instant.  The wiring we
want pinned is the one in the source.

**Honesty rule.** A map that quietly guesses is worse than no map.  Every
topic/service/action name is resolved only through the cheap, exact paths in
:func:`_resolve_name` (string literal, module/class constant, all-literal
f-string, ROS-parameter default).  Anything else is emitted verbatim as
``UNRESOLVED(<expr>)`` so a reader can see there is a hole rather than trusting
a wrong name.

Usage::

    python tools/gen_choreography_map.py                  # write the committed doc
    python tools/gen_choreography_map.py --out /tmp/x.md  # write elsewhere
    python tools/gen_choreography_map.py --check          # exit 1 if the doc is stale
    python tools/gen_choreography_map.py --stdout         # print, write nothing
"""

from __future__ import annotations

import argparse
import ast
import os
import sys
from typing import Dict, List, Optional, Tuple

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TOOLS_DIR)
_NODE_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'jugglebot')
_DEFAULT_OUT = os.path.join(_REPO_ROOT, 'ros_ws', 'docs', 'choreography.md')

#: ``method name`` -> (endpoint kind, index of the name argument in ``args``,
#: index of the message/service/action type argument).
_METHOD_CALLS = {
    'create_publisher':    ('publisher',      1, 0),
    'create_subscription': ('subscriber',     1, 0),
    'create_service':      ('service_server', 1, 0),
    'create_client':       ('service_client', 1, 0),
}

#: ``constructor name`` -> same tuple.  ``ActionServer(self, Type, 'name', ...)``
#: takes the node as arg 0, so the type/name indices shift by one.
_CTOR_CALLS = {
    'ActionServer': ('action_server', 2, 1),
    'ActionClient': ('action_client', 2, 1),
}

#: Every spelling that can produce an endpoint — used as a cheap substring
#: pre-filter before paying for ``ast.parse``.
_CALL_KEYWORDS = tuple(_METHOD_CALLS) + tuple(_CTOR_CALLS)

#: Endpoint kinds grouped into the three report sections.
_SECTIONS = (
    ('Topics', 'publisher', 'subscriber', 'publishers', 'subscribers'),
    ('Services', 'service_server', 'service_client', 'servers', 'clients'),
    ('Actions', 'action_server', 'action_client', 'servers', 'clients'),
)

#: One-line contract notes for the wires whose semantics are NOT inferable from
#: the graph.  Source doc is named in each note so the reader can go deeper.
#: Keyed by resolved endpoint name.
_CONTRACT_NOTES = {
    'catch/dynamic_target': (
        'MULTI-PUBLISHER by design: catch_coordinator_node (reactive catch) and '
        'reload_coordinator_node (tier-8b deferred A->B reach) publish the same '
        'wire and trajectory_node consumes either identically. Installs are '
        'gated by the catch-armed latch and bounded by the declared reach '
        'envelope (ros_ws/docs/catch_reach_envelope.md, contract C-REACH-1; '
        'orientation/arrival semantics in ros_ws/docs/catch_arrival_contract.md).'
    ),
    'throw_announcements': (
        'MULTI-PUBLISHER by design: ball_butler_node announces a real BB throw '
        'and reload_coordinator_node publishes the toss self-announcement '
        '(thrower_name = target_id = this robot) so the correlation -> catch '
        'path closes unchanged (ros_ws/docs/ball_possession_contract.md).'
    ),
    'catch/armed': (
        'HAND-OWNERSHIP LATCH (1/6). Mirror of the trajectory/arm_catch latch on '
        'trajectory_node; while raised, catch/dynamic_target may actuate the '
        'platform and catch_coordinator_node may actuate the hand '
        '(ros_ws/docs/control_modes.md, ros_ws/docs/safety.md).'
    ),
    'catch/prime_hold': (
        'HAND-OWNERSHIP LATCH (2/6). Raised at PREPARE, BEFORE catch/armed rises, '
        'and released LAST at terminal: while True the reload/toss owns the hand '
        'and catch_coordinator_node must not prime it '
        '(ros_ws/docs/hand_command_continuity.md, ros_ws/docs/ball_possession_contract.md).'
    ),
    'catch/prime_dispatched': (
        'HAND-OWNERSHIP LATCH (3/6). Announces every reload-side prime dispatch so '
        'catch_coordinator_node holds its anti-stutter in-flight window instead of '
        'restarting a live ascent (ros_ws/docs/hand_command_continuity.md).'
    ),
    'catch/pretilt_hold': (
        'HAND-OWNERSHIP LATCH (4/6). Raised on the same tick as prime_hold and '
        'released with it; suppresses the pre-tilt while the toss owns the '
        'platform pose (ros_ws/docs/levelling_frame.md).'
    ),
    'catch/vel_scale': (
        'HAND-OWNERSHIP LATCH (5/6). Catch-speed knob relayed at PREPARE, before '
        'catch/armed rises, so catch_coordinator_node holds the value before any '
        'arm (ros_ws/docs/hand_command_continuity.md).'
    ),
    'catch/unified_mode': (
        'HAND-OWNERSHIP LATCH (6/6). Session-scoped declaration; while raised '
        'the cycle plan owns the hand and catch_coordinator_node must not arm '
        'a reactive stroke — TRANSIENT_LOCAL depth 1 on both ends.'
    ),
    'catch/reach_center': (
        'NOT a hand latch - platform-side. Declares the reach-envelope centre '
        '(contract C-REACH-1) one tick before trajectory/arm_catch, so every '
        'catch/dynamic_target installed under the latch is bounded relative to a '
        'centre the consumer already holds (ros_ws/docs/catch_reach_envelope.md).'
    ),
}

#: The six catch/* wires that gate hand ownership between reload_coordinator_node
#: and catch_coordinator_node.  Named here (not just in the notes) so the drift
#: test can assert the annotation set has not silently shrunk.
HAND_OWNERSHIP_LATCH_TOPICS = (
    'catch/armed',
    'catch/prime_hold',
    'catch/prime_dispatched',
    'catch/pretilt_hold',
    'catch/vel_scale',
    'catch/unified_mode',
)

#: Topics deliberately written by more than one node.
MULTI_PUBLISHER_TOPICS = (
    'catch/dynamic_target',
    'throw_announcements',
)

#: Modules that ARE scanned (source present, endpoints real) but that
#: ``jugglebot_launch.py`` does NOT start.  EMPTY since 2026-09-01: its two
#: former members, ``motion_bridge_node`` and ``mpc_bridge_node``, were deleted
#: outright with the MPC chain rather than kept dormant (git tag ``mpc-final``;
#: see ``logbook/2026-09-01-mpc-chain-removed.md``), so there is no longer a
#: scanned-but-unstarted node to tag.
#:
#: Kept as a mechanism, not vestigially: without a marker the map would answer
#: "who consumes `platform_pose_topic`?" with a node the launch file never
#: starts — lying by omission.  Rendered as ``(not launched)`` beside the node
#: name, and cross-checked against the launch file by
#: ``tests/ros/test_choreography_map.py`` so the tuple cannot drift out of date
#: in either direction.
NOT_LAUNCHED_NODES = ()

_BANNER = (
    'Python-node graph only - GUI/rosbridge consumers are NOT included.'
)

#: Blockquote paragraphs under the banner.  Wrapped at render time.
_PREAMBLE = (
    'Built by static analysis of every module under '
    '`ros_ws/src/jugglebot/jugglebot/` that wires a ROS endpoint - the '
    '`*_node.py` files plus `spacemouse_handler.py`, which is a launched node '
    'without the suffix and the only publisher of `platform_pose_topic`. '
    'A name that could not be resolved to a literal is shown as '
    '`UNRESOLVED(<expr>)` rather than guessed.',

    'Launch-time remappings are NOT applied (there are none in '
    '`ros_ws/src/jugglebot/launch/jugglebot_launch.py` today); a name resolved '
    'from a ROS parameter default is tagged `param:<key>` and can be overridden '
    'at launch.',

    'Every scanned module is started by that launch file. When one is not, it '
    'is tagged `(not launched)` wherever it appears, because a wire whose '
    'publishers (or service servers) are ALL tagged carries nothing on a '
    'running robot. No module is tagged today: the last two - '
    '`motion_bridge_node` and `mpc_bridge_node` - were deleted with the MPC '
    'chain on 2026-09-01 (git tag `mpc-final`).',

    'Call-site line numbers are omitted on purpose - they rot silently and '
    'would make the drift test fire on unrelated edits; `grep -n` the name in '
    'the node file instead.',
)


# ---------------------------------------------------------------------------
# Expression rendering (no ast.unparse — the venv is Python 3.8)
# ---------------------------------------------------------------------------

def _render(node: Optional[ast.AST]) -> str:
    """Render an AST expression back to approximate source text.

    Only used for display (type names and UNRESOLVED markers), never for
    resolution, so an approximate rendering is safe.
    """
    if node is None:
        return '<missing>'
    if isinstance(node, ast.Constant):
        return repr(node.value)
    if isinstance(node, ast.Str):          # pragma: no cover - py<3.8 shape
        return repr(node.s)
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return _render(node.value) + '.' + node.attr
    if isinstance(node, ast.Call):
        return _render(node.func) + '(...)'
    if isinstance(node, ast.Subscript):
        return _render(node.value) + '[...]'
    if isinstance(node, ast.JoinedStr):
        return 'f-string'
    return type(node).__name__


# ---------------------------------------------------------------------------
# Name resolution
# ---------------------------------------------------------------------------

class _ModuleIndex:
    """Constants and imports of one node module, indexed for cheap lookup."""

    def __init__(self, tree: ast.Module) -> None:
        self.module_consts: Dict[str, str] = {}
        self.class_consts: Dict[str, str] = {}
        self.param_defaults: Dict[str, str] = {}
        self.imports: Dict[str, str] = {}
        #: Names (and parameter keys) bound to two different string values.
        #: Unresolvable by construction — see :func:`_conflicts`.
        self.poisoned: set = set()
        self._index(tree)

    # -- indexing ---------------------------------------------------------
    def _index(self, tree: ast.Module) -> None:
        for stmt in tree.body:
            if isinstance(stmt, (ast.Import, ast.ImportFrom)):
                self._index_import(stmt)
            elif isinstance(stmt, ast.Assign):
                self._record(self.module_consts, stmt)
            elif isinstance(stmt, ast.ClassDef):
                for sub in stmt.body:
                    if isinstance(sub, ast.Assign):
                        self._record(self.class_consts, sub)
        # declare_parameter('name', 'default') anywhere in the module
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            if not (isinstance(fn, ast.Attribute) and fn.attr == 'declare_parameter'):
                continue
            if len(node.args) < 2:
                continue
            key = _const_str(node.args[0])
            default = _const_str(node.args[1])
            if key is None or default is None:
                continue
            if self.param_defaults.get(key, default) != default:
                # Two declarations of the same key with different defaults —
                # cannot say which node's declaration applies.
                self.poisoned.add(key)
                self.param_defaults.pop(key, None)
            elif key not in self.poisoned:
                self.param_defaults[key] = default

    def _index_import(self, stmt: ast.AST) -> None:
        if isinstance(stmt, ast.ImportFrom):
            mod = stmt.module or ''
            for alias in stmt.names:
                self.imports[alias.asname or alias.name] = mod + '.' + alias.name
        elif isinstance(stmt, ast.Import):
            for alias in stmt.names:
                self.imports[alias.asname or alias.name] = alias.name

    def _record(self, store: Dict[str, str], stmt: ast.Assign) -> None:
        value = _const_str(stmt.value)
        if value is None:
            return
        for target in stmt.targets:
            if not isinstance(target, ast.Name):
                continue
            name = target.id
            if store.get(name, value) != value:
                # Same constant, two different values (e.g. one per class).
                # Flow-insensitively unresolvable — refuse rather than guess.
                self.poisoned.add(name)
                store.pop(name, None)
            elif name not in self.poisoned:
                store[name] = value

    # -- lookup -----------------------------------------------------------
    def lookup_name(self, ident: str) -> Optional[str]:
        if ident in self.poisoned:
            return None
        if ident in self.module_consts:
            return self.module_consts[ident]
        return self.class_consts.get(ident)


def _const_str(node: Optional[ast.AST]) -> Optional[str]:
    """Return the string value of a literal node, else ``None``."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Str):          # pragma: no cover - py<3.8 shape
        return node.s
    if isinstance(node, ast.JoinedStr):
        parts = []
        for value in node.values:
            piece = _const_str(value)
            if piece is None:
                return None
            parts.append(piece)
        return ''.join(parts)
    return None


def _unwrap_param(node: ast.AST) -> Optional[str]:
    """``str(self.get_parameter('X').value)`` / ``self.get_parameter('X').value``
    -> ``'X'``; anything else -> ``None``."""
    # Unwrap a single str(...) / a chain of .value attribute accesses.
    while True:
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                and node.func.id == 'str' and len(node.args) == 1:
            node = node.args[0]
            continue
        if isinstance(node, ast.Attribute) and node.attr in ('value', 'string_value'):
            node = node.value
            continue
        break
    if isinstance(node, ast.Call):
        fn = node.func
        if isinstance(fn, ast.Attribute) and fn.attr == 'get_parameter' and node.args:
            return _const_str(node.args[0])
    return None


class _LocalScope:
    """String-literal (and parameter-derived) local assignments of one function."""

    def __init__(self) -> None:
        self.literals: Dict[str, str] = {}
        self.params: Dict[str, str] = {}
        self.poisoned: set = set()


def _conflicts(scope: _LocalScope, name: str,
               literal: Optional[str], param: Optional[str]) -> bool:
    """True if *name* already holds a DIFFERENT binding in *scope*."""
    if name in scope.literals:
        return scope.literals[name] != literal
    if name in scope.params:
        return scope.params[name] != param
    return False


def _own_nodes(func: ast.AST):
    """Yield the nodes of *func*'s own body, NOT descending into nested scopes.

    ``ast.walk`` would descend into a nested ``def``/``lambda``/``class``, so a
    binding that only exists inside a helper would leak into the enclosing
    function's scope and be used to resolve a call site that the helper's
    binding never reaches.  That is a *silent* wrong answer — the resolver would
    report provenance ``literal`` for a name it cannot actually see — which the
    honesty rule forbids.  Nested functions still get their own scope (the
    caller indexes every function separately and :func:`scan_file` hands each
    one its own), so nothing is lost; the enclosing scope simply stops
    absorbing bindings that are not its own.
    """
    stack = list(ast.iter_child_nodes(func))
    while stack:
        node = stack.pop()
        yield node
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.Lambda, ast.ClassDef)):
            continue
        stack.extend(ast.iter_child_nodes(node))


def _index_locals(tree: ast.Module) -> Dict[int, _LocalScope]:
    """Map every statement's enclosing function to its local string bindings.

    Returned keyed by ``id()`` of the function node; the caller walks functions
    itself so it always has the function object at hand.
    """
    scopes: Dict[int, _LocalScope] = {}
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        scope = _LocalScope()
        for sub in _own_nodes(node):
            if not isinstance(sub, ast.Assign):
                continue
            names = [t.id for t in sub.targets if isinstance(t, ast.Name)]
            if not names:
                continue
            literal = _const_str(sub.value)
            param = _unwrap_param(sub.value)
            for name in names:
                # Rebound to something different somewhere in this function?
                # This index is flow-INsensitive, so it cannot say which binding
                # reaches the call site — refuse to answer rather than pick one
                # and be confidently wrong.
                if _conflicts(scope, name, literal, param):
                    scope.poisoned.add(name)
                    scope.literals.pop(name, None)
                    scope.params.pop(name, None)
                    continue
                if name in scope.poisoned:
                    continue
                if literal is not None:
                    scope.literals[name] = literal
                elif param is not None:
                    scope.params[name] = param
        scopes[id(node)] = scope
    return scopes


def _resolve_name(node: Optional[ast.AST],
                  index: _ModuleIndex,
                  scope: Optional[_LocalScope]) -> Tuple[str, str]:
    """Resolve an endpoint-name expression.

    Returns ``(name, provenance)``.  *provenance* is ``'literal'``,
    ``'constant'``, ``'param:<key>'`` or ``'unresolved'``.  An unresolved name
    comes back as ``UNRESOLVED(<expr>)`` — deliberately un-plottable, so a
    reader cannot mistake it for a real wire.
    """
    literal = _const_str(node)
    if literal is not None:
        return literal, 'literal'

    if isinstance(node, ast.Name):
        if scope is not None and node.id in scope.literals:
            return scope.literals[node.id], 'literal'
        if scope is not None and node.id in scope.params:
            key = scope.params[node.id]
            default = index.param_defaults.get(key)
            if default is not None:
                return default, 'param:' + key
        const = index.lookup_name(node.id)
        if const is not None:
            return const, 'constant'

    # Deliberately NO ``self.X`` / ``obj.X`` -> module-or-class-constant path.
    # ``index`` holds *module* and *class-body* constants keyed by bare name, so
    # looking up ``node.attr`` would answer ``self.TOPIC`` (an instance
    # attribute assigned in ``__init__``) with an unrelated module-level
    # ``TOPIC``, confidently and with provenance ``constant``.  Instance-attribute
    # tracking is real work; until someone does it, an attribute expression falls
    # through to ``UNRESOLVED(self.TOPIC)`` — visibly a hole, per the honesty
    # rule, instead of a plausible lie.  No endpoint in the package uses this
    # shape today (all 140 resolve as ``literal`` bar one ``param:``).

    param_key = _unwrap_param(node) if node is not None else None
    if param_key is not None:
        default = index.param_defaults.get(param_key)
        if default is not None:
            return default, 'param:' + param_key

    return 'UNRESOLVED(' + _render(node) + ')', 'unresolved'


# ---------------------------------------------------------------------------
# Scanning
# ---------------------------------------------------------------------------

class Endpoint(object):
    """One create_publisher/subscription/service/client or Action{Server,Client}.

    ``lineno`` is kept for diagnostics (a failing drift test can point at the
    call site) but is deliberately NOT rendered into the map — see
    :func:`_fmt_side`.
    """

    __slots__ = ('name', 'kind', 'node', 'msg_type', 'lineno', 'provenance')

    def __init__(self, name, kind, node, msg_type, lineno, provenance):
        # type: (str, str, str, str, int, str) -> None
        self.name = name
        self.kind = kind
        self.node = node
        self.msg_type = msg_type
        self.lineno = lineno
        self.provenance = provenance

    def __repr__(self):
        # type: () -> str
        return 'Endpoint(%s %s %s:%d)' % (self.kind, self.name,
                                          self.node, self.lineno)


def scan_file(path: str, label: Optional[str] = None) -> List[Endpoint]:
    """Scan one module for ROS endpoints.

    *label* names the module in the map; it defaults to the basename without
    the extension.
    """
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()
    # Cheap pre-filter: parsing the whole package costs ~1.1 s, and 59 of its 71
    # modules contain no endpoint call at all.  A substring miss is conclusive —
    # the AST can only contain a call whose spelling appears in the source.
    if not any(keyword in source for keyword in _CALL_KEYWORDS):
        return []
    tree = ast.parse(source, filename=path)
    index = _ModuleIndex(tree)
    scopes = _index_locals(tree)
    node_name = label if label is not None else os.path.basename(path)[:-len('.py')]

    endpoints: List[Endpoint] = []

    def visit(container: ast.AST, scope: Optional[_LocalScope]) -> None:
        for child in ast.iter_child_nodes(container):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                visit(child, scopes.get(id(child)))
                continue
            if isinstance(child, ast.Call):
                spec = _call_spec(child)
                if spec is not None:
                    kind, name_idx, type_idx = spec
                    name_arg = child.args[name_idx] if len(child.args) > name_idx else None
                    type_arg = child.args[type_idx] if len(child.args) > type_idx else None
                    name, provenance = _resolve_name(name_arg, index, scope)
                    raw_type = _render(type_arg)
                    msg_type = index.imports.get(raw_type, raw_type)
                    endpoints.append(Endpoint(name, kind, node_name, msg_type,
                                              child.lineno, provenance))
            visit(child, scope)

    visit(tree, None)
    return endpoints


def _call_spec(call: ast.Call) -> Optional[Tuple[str, int, int]]:
    func = call.func
    if isinstance(func, ast.Attribute):
        return _METHOD_CALLS.get(func.attr) or _CTOR_CALLS.get(func.attr)
    if isinstance(func, ast.Name):
        return _CTOR_CALLS.get(func.id)
    return None


#: Directories under the package that never contribute live wiring.
_SKIP_DIRS = frozenset(('__pycache__', 'tests', 'archived'))


def scan_nodes(node_dir: str = _NODE_DIR) -> List[Endpoint]:
    """Scan every module under *node_dir*, sorted for determinism.

    **Not** just ``*_node.py``: ``spacemouse_handler.py`` is a launched node
    whose filename does not carry the suffix, and it is the only publisher of
    ``platform_pose_topic``.  Scanning by filename convention would have shown
    that topic as publisher-less — a map that lies by omission.  Modules with no
    ROS endpoints contribute nothing and never appear.
    """
    endpoints: List[Endpoint] = []
    for dirpath, dirnames, filenames in os.walk(node_dir):
        dirnames[:] = sorted(d for d in dirnames if d not in _SKIP_DIRS)
        for fname in sorted(filenames):
            if not fname.endswith('.py'):
                continue
            path = os.path.join(dirpath, fname)
            rel = os.path.relpath(path, node_dir)
            label = rel[:-len('.py')].replace(os.sep, '/')
            endpoints.extend(scan_file(path, label=label))
    return endpoints


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def _wrap_note(note: str, indent: str = '  ') -> List[str]:
    """Hard-wrap a contract note at ~78 columns for a readable diff."""
    words = note.split()
    lines: List[str] = []
    current = indent + '-'
    for word in words:
        if len(current) + 1 + len(word) > 78 and current.strip() != '-':
            lines.append(current)
            current = indent + ' '
        current += ' ' + word
    if current.strip():
        lines.append(current)
    return lines


def _wrap_quote(paragraph: str) -> List[str]:
    """Hard-wrap a blockquote paragraph at ~76 columns."""
    lines: List[str] = []
    current = '>'
    for word in paragraph.split():
        if len(current) + 1 + len(word) > 76 and current != '>':
            lines.append(current)
            current = '>'
        current += ' ' + word
    if current != '>':
        lines.append(current)
    return lines


def render_markdown(endpoints: List[Endpoint]) -> str:
    """Render the choreography map.  Deterministic for a given endpoint set."""
    lines: List[str] = []
    lines.append('# Topic choreography map')
    lines.append('')
    lines.append('<!-- GENERATED by tools/gen_choreography_map.py — DO NOT EDIT.')
    lines.append('     Regenerate: python tools/gen_choreography_map.py')
    lines.append('     Pinned by:  tests/ros/test_choreography_map.py -->')
    lines.append('')
    lines.append('> **' + _BANNER + '**')
    lines.append('>')
    for paragraph in _PREAMBLE:
        lines.extend(_wrap_quote(paragraph))
        lines.append('>')
    lines.pop()          # drop the trailing '>' separator
    lines.append('')

    by_kind: Dict[str, Dict[str, List[Endpoint]]] = {}
    for endpoint in endpoints:
        by_kind.setdefault(endpoint.kind, {}).setdefault(endpoint.name, []).append(endpoint)

    for title, server_kind, client_kind, server_label, client_label in _SECTIONS:
        servers = by_kind.get(server_kind, {})
        clients = by_kind.get(client_kind, {})
        names = sorted(set(servers) | set(clients))
        lines.append('## ' + title)
        lines.append('')
        if not names:
            lines.append('_None._')
            lines.append('')
            continue
        for name in names:
            lines.append('### `' + name + '`')
            lines.append('')
            lines.append('- **' + server_label + ':** ' + _fmt_side(servers.get(name)))
            lines.append('- **' + client_label + ':** ' + _fmt_side(clients.get(name)))
            types = sorted({e.msg_type for e in servers.get(name, []) + clients.get(name, [])})
            lines.append('- **type:** ' + ', '.join('`' + t + '`' for t in types))
            provenances = sorted({e.provenance for e in
                                  servers.get(name, []) + clients.get(name, [])
                                  if e.provenance != 'literal'})
            if provenances:
                lines.append('- **name source:** ' + ', '.join('`' + p + '`'
                                                               for p in provenances))
            note = _CONTRACT_NOTES.get(name)
            if note is not None:
                lines.append('- **contract:**')
                lines.extend(_wrap_note(note, '  '))
            lines.append('')

    orphans = _orphans(by_kind)
    lines.append('## Unmatched endpoints')
    lines.append('')
    lines.append('Wires with a producer but no in-repo Python consumer (or the')
    lines.append('reverse). Expected for GUI/rosbridge-facing topics, bag-only')
    lines.append('recordings and externally driven services — listed so a genuinely')
    lines.append('broken wire cannot hide among them.')
    lines.append('')
    if orphans:
        for label, name in orphans:
            lines.append('- `' + name + '` — ' + label)
    else:
        lines.append('_None._')
    lines.append('')

    unresolved = sorted({e.name for e in endpoints if e.provenance == 'unresolved'})
    lines.append('## Unresolved names')
    lines.append('')
    if unresolved:
        for name in unresolved:
            lines.append('- `' + name + '`')
    else:
        lines.append('_None — every endpoint name resolved to a literal, a '
                     'module/class constant or a declared parameter default '
                     '(the non-literal ones are tagged `name source:` above)._')
    lines.append('')

    return '\n'.join(lines) + '\n'


def _fmt_side(endpoints: Optional[List[Endpoint]]) -> str:
    """Format one side of a wire.

    Node names only — **no line numbers**, deliberately.  Line numbers churn on
    every unrelated edit above a call site, which would make the drift test fire
    on non-wiring changes (and train readers to regenerate blindly); a committed
    line number that nobody re-checks is also exactly the kind of lie this map
    exists to avoid.  ``grep -n '<name>'`` finds the call site in one step.
    """
    if not endpoints:
        return '_none_'
    names = sorted({e.node for e in endpoints})
    return ', '.join(
        '`' + n + '`' + (' (not launched)' if n in NOT_LAUNCHED_NODES else '')
        for n in names)


def _orphans(by_kind: Dict[str, Dict[str, List[Endpoint]]]) -> List[Tuple[str, str]]:
    out: List[Tuple[str, str]] = []
    for title, server_kind, client_kind, server_label, client_label in _SECTIONS:
        kind_label = title[:-1].lower()   # Topics -> topic, Services -> service
        servers = by_kind.get(server_kind, {})
        clients = by_kind.get(client_kind, {})
        for name in sorted(set(servers) - set(clients)):
            out.append((kind_label + ' with no ' + client_label, name))
        for name in sorted(set(clients) - set(servers)):
            out.append((kind_label + ' with no ' + server_label, name))
    return sorted(out, key=lambda pair: (pair[1], pair[0]))


def generate(node_dir: str = _NODE_DIR) -> str:
    return render_markdown(scan_nodes(node_dir))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--node-dir', default=_NODE_DIR,
                        help='directory of *_node.py files to scan')
    parser.add_argument('--out', default=_DEFAULT_OUT,
                        help='output markdown path')
    parser.add_argument('--check', action='store_true',
                        help='exit 1 if --out differs from freshly generated text')
    parser.add_argument('--stdout', action='store_true',
                        help='print to stdout instead of writing a file')
    args = parser.parse_args(argv)

    text = generate(args.node_dir)

    if args.stdout:
        sys.stdout.write(text)
        return 0

    if args.check:
        if not os.path.exists(args.out):
            sys.stderr.write('CHOREOGRAPHY DRIFT: %s does not exist\n' % args.out)
            return 1
        with open(args.out, 'r', encoding='utf-8') as handle:
            existing = handle.read()
        if existing != text:
            sys.stderr.write('CHOREOGRAPHY DRIFT: %s is stale — '
                             'run python tools/gen_choreography_map.py\n' % args.out)
            return 1
        sys.stdout.write('CHOREOGRAPHY FRESH: %s\n' % args.out)
        return 0

    with open(args.out, 'w', encoding='utf-8') as handle:
        handle.write(text)
    sys.stdout.write('Wrote %s\n' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
