"""homelock-race: a T->J callback must never run before the state it reads exists.

The link client's RX thread is live from ``start()`` in ``TeensyBridgeNode.__init__``,
so a callback is eligible to run the instant it is subscribed. 2026-11-25 20:55 (real
launch): a DIAGNOSTIC arrived during construction, ``_on_reference_loss`` reached
``self._home_lock`` (created ~300 lines after the subscription) and raised
AttributeError. The client swallows callback exceptions and ``ReferenceLossWatch``
had already consumed the edge, so the BUG-122 homed-flag clear was LOST.

Two pins: (a) the instance, delivered from inside the constructor; (b) the class, a
static scan of ``__init__``: every ``self.<attr>`` that any subscribed callback path
reads (following ``self.method()`` calls transitively) is assigned before the
subscription line. ROS timers/services/subscriptions cannot fire during ``__init__``
(the executor spins after construction); the one other thread live in ``__init__`` is
the client RX thread, which only reaches the node through ``subscribe``.
"""

from __future__ import annotations

import ast
import collections
import inspect

from teensy_link import TeensyLinkClient
from teensy_link.protocol import Diagnostic, MsgType

from jugglebot.can import odrive

from tests.ros import conftest  # noqa: F401  (installs the mock ROS 2 modules)
from tests.teensy_link.conftest import FakeTeensy

HAND = 6


class _FiringClient(TeensyLinkClient):
    """A real client whose ``subscribe`` delivers one frame at once, on the
    constructing thread, the way a frame landing right after subscription would.
    Exceptions are recorded, not raised: the real client swallows them too."""

    def __init__(self, *a, fire=None, **k):
        super().__init__(*a, **k)
        self._fire = fire or {}
        self.errors = []

    def subscribe(self, msg_type, callback, **kw):
        unsub = super().subscribe(msg_type, callback, **kw)
        payload = self._fire.get(int(msg_type))
        if payload is not None:
            try:
                callback(int(msg_type), 0, payload, None)
            except Exception as e:  # noqa: BLE001
                self.errors.append((int(msg_type), repr(e)))
        return unsub


def test_reference_loss_diagnostic_during_construction_is_latched():
    from jugglebot.teensy_bridge_node import TeensyBridgeNode
    teensy = FakeTeensy(bind_host="127.0.0.1", stream_port=0, rpc_port=0)
    teensy.start()
    loss = Diagnostic(axis_id=HAND, axis_state=1,
                      active_errors=odrive.ERR_INITIALIZING, disarm_reason=0).pack()
    client = _FiringClient(
        teensy_addr=("127.0.0.1", teensy.stream_port), rpc_port=teensy.rpc_port,
        local_bind_stream=0, local_bind_rpc=0, bind_host="127.0.0.1",
        fire={int(MsgType.DIAGNOSTIC): loss})
    node = None
    try:
        node = TeensyBridgeNode(client=client, boot_state_read=False,
                                stow_on_shutdown=False)
        assert client.errors == []
        # Before the boot read the cache is not authoritative, so the loss is kept
        # pending (the read-apply path publishes not-homed when the read lands:
        # test_event_before_the_boot_read_is_applied_when_it_lands).
        assert node._ref_loss.pending is not None
        assert node._ref_loss.pending.axis == HAND
    finally:
        if node is not None:
            node._stow_on_shutdown = False
            node.on_shutdown()
        client.stop()
        teensy.stop()


# ── The class: static ordering scan ────────────────────────────

def _node_class():
    import jugglebot.teensy_bridge_node as m
    tree = ast.parse(inspect.getsource(m))
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'TeensyBridgeNode')
    return m.TeensyBridgeNode, cls


def _self_attr(n):
    return (isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
            and n.value.id == 'self')


def _stores(fn):
    return {n.attr: n.lineno for n in ast.walk(fn)
            if _self_attr(n) and isinstance(n.ctx, ast.Store)}


def _scan():
    """-> (subscribe_line_to_callback, {callback: {attr: (method, line)}}, assigned_line)."""
    node_cls, cls = _node_class()
    meths = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    init = meths['__init__']

    def callees(fn):
        return {n.attr for n in ast.walk(fn)
                if _self_attr(n) and n.attr in meths}

    def closure(name):
        seen, todo = {name}, [name]
        while todo:
            for c in callees(meths[todo.pop()]):
                if c not in seen:
                    seen.add(c)
                    todo.append(c)
        return seen

    assigned = dict(_stores(init))
    for call in (n for n in ast.walk(init) if isinstance(n, ast.Call)):
        f = call.func
        if _self_attr(f) and f.attr in meths:           # helper called from __init__
            for m in closure(f.attr):
                for a in _stores(meths[m]):
                    assigned[a] = min(assigned.get(a, 10 ** 9), call.lineno)
    subs = []
    for n in ast.walk(init):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'subscribe' and isinstance(n.func.value, ast.Attribute)
                and n.func.value.attr == '_client'):
            subs.append((n.lineno, n.args[1].attr))
    reads = {}
    for _, cb in subs:
        r = collections.OrderedDict()
        for m in closure(cb):
            for n in ast.walk(meths[m]):
                if _self_attr(n) and isinstance(n.ctx, ast.Load) and n.attr not in meths \
                        and not hasattr(node_cls, n.attr):    # Node API / class constants
                    r.setdefault(n.attr, (m, n.lineno))
        reads[cb] = r
    return subs, reads, assigned


def test_scan_finds_the_subscriptions():
    subs, reads, _ = _scan()
    assert len(subs) >= 18                      # the T->J block (20 at 2026-11-25)
    assert {'_on_diagnostic', '_on_telemetry'} <= set(reads)
    assert {'_home_lock', '_home_in_progress'} <= set(reads['_on_diagnostic'])


def test_every_subscribed_callback_reads_only_attributes_assigned_before_it():
    subs, reads, assigned = _scan()
    late = []
    for line, cb in sorted(subs):
        for attr, (via, _) in reads[cb].items():
            first = assigned.get(attr)
            if first is None or first > line:
                late.append(f'subscribe@{line} {cb} -> self.{attr} (read in {via}; '
                            f'first assigned at {first})')
    assert not late, ('callbacks subscribed before the state they read exists '
                      '(move the subscribe after it, or the assignment before it):\n'
                      + '\n'.join(late))
