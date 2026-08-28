"""THE shared vocabulary for the parenthetical DETAIL a refusal carries.

Every operator-facing FSM in this package terminalises as a single outcome
STRING — ``REJECTED_<CODE>`` / ``ABORTED_<CODE>``, optionally with a
parenthesised detail — and that string is the only thing that reaches every
channel at once: the action result, the session's ``per_cycle_outcomes[]``, the
toss record's ``outcome`` column and the node's one authoritative log line. A
number that lives in a second log line is a number the operator loses the moment
they read the result instead of the console, or read a bag instead of a live
terminal (``toss_sequencer.TossSequencer._abort`` states the same doctrine for
the abort half).

So: **every limit-bearing refusal names, in the outcome itself, the offending
requested value, the nearest limit, and the knob that moves it.** This module is
where that sentence is turned into text, once, so the three FSMs
(``toss_sequencer``, ``toss_session``, ``reload_sequencer``) and the node cannot
each invent their own punctuation — and so the tests can import the same
formatter rather than restating it.

Pure Python, no ROS, no config imports: it is text formatting over numbers the
caller already has in scope, and it must stay importable from the pure FSMs
(``ros_ws/.../motion``, ``controller/`` and these sequencers share that rule).

The OTHER half of the contract is :func:`base_outcome`. Adding a detail turns
``'REJECTED_X'`` into ``'REJECTED_X(...)'``, which silently breaks every
equality test against the bare code — including, in production, the auto-reload
trigger and the zombie-move superseder. Consumers that want the CODE ask for it
here rather than slicing the string themselves, so a future enrichment cannot
quietly disarm a guard.
"""

from __future__ import annotations


def base_outcome(outcome) -> str:
    """The bare CODE of an outcome string — everything before the first ``(``.

    ``'REJECTED_DWELL(dwell 0.90 s < floor 1.04 s)'`` -> ``'REJECTED_DWELL'``;
    a bare code is returned unchanged, and so is a COMPOSED one
    (``'ABORTED_CYCLE_REJECTED_DWELL(…)'`` -> ``'ABORTED_CYCLE_REJECTED_DWELL'``,
    ``'STOPPED_RELOAD_REJECTED_NOT_CENTERED(…)'`` likewise) — the composers
    prefix, they never wrap, so splitting at the FIRST paren is correct for
    every form the machine mints.

    Non-strings are coerced rather than refused: the callers are guards on a
    terminal path, where an exception would be strictly worse than a mismatch.
    """
    return str(outcome).split('(', 1)[0]


#: What a SUBCODE may be made of. A refusal that RELAYS another layer's verdict
#: puts that layer's code first in the parenthetical — ``REJECTED_POSITION(BUSY:
#: a move is in flight …)``, ``REJECTED_THROW_ENVELOPE(END_STOP:modelled peak
#: …)`` — while a DESCRIPTIVE detail is prose and has no subcode at all. The
#: character class is what tells the two apart without a second convention: a
#: leading token of only capitals, digits and underscores is a code; anything
#: containing a space or a lowercase letter is a sentence.
_SUBCODE_CHARS = frozenset(
    'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_')


def outcome_subcode(outcome) -> str:
    """The relayed SUBCODE inside an outcome's parenthetical, or ``''``.

    ``'REJECTED_POSITION(NO_RESPONSE)'`` and
    ``'REJECTED_POSITION(BUSY: a move is in flight)'`` both give the code;
    ``'REJECTED_WORKSPACE(|B.y| = 178.0 mm > 160.0 …)'`` gives ``''``, because
    there is no subcode there to mistake for one.

    It exists so a guard can match ``(code, subcode)`` instead of a whole
    outcome string. Matching the whole string is what breaks the moment a
    refusal starts carrying its numbers — and the failure is silent, because a
    guard that stops matching simply does nothing.
    """
    text = str(outcome)
    open_i = text.find('(')
    if open_i < 0:
        return ''
    detail = text[open_i + 1:]
    if detail.endswith(')'):
        detail = detail[:-1]
    head = detail.split(':', 1)[0].strip()
    return head if head and all(c in _SUBCODE_CHARS for c in head) else ''


def _fmt(value, digits: int) -> str:
    """A number at a fixed precision, NaN/inf-safe (``'{:.1f}'`` renders those
    as ``nan``/``inf``, which is exactly what an operator should see when a
    goal field is one)."""
    try:
        return '{:.{}f}'.format(float(value), int(digits))
    except (TypeError, ValueError):                            # noqa: BLE001
        return str(value)


def _knob(name: str) -> str:
    """The bracketed knob suffix — the thing the operator turns. Empty for a
    bound with no operator-facing knob (a derived floor names its terms in the
    tail instead)."""
    return ' [{}]'.format(name) if name else ''


def bound_msg(quantity: str, value, op: str, limit, unit: str = '',
              knob: str = '', digits: int = 1, limit_label: str = '',
              tail: str = '') -> str:
    """One ``<quantity> <value> <op> [<limit_label> ]<limit> [<knob>][; <tail>]``
    clause — the shape every single-sided bound refusal uses.

    ``quantity`` is free text and may end in ``=`` where that reads better
    (``'|B-A| ='`` vs a bare field name like ``'dwell'``); the unit is repeated
    on both sides so ``0.90 s < 1.04 s`` cannot be misread as a mixed-unit
    comparison. ``tail`` carries the decomposition or the remedy — the part that
    tells the operator *which way to turn the knob*.
    """
    text = '{} {}{} {} {}{}{}'.format(
        quantity, _fmt(value, digits), ' ' + unit if unit else '',
        op, limit_label + ' ' if limit_label else '',
        _fmt(limit, digits), ' ' + unit if unit else '')
    return '{}{}{}'.format(text, _knob(knob),
                           '; ' + tail if tail else '')


def range_msg(quantity: str, value, low, high, unit: str = '',
              knob: str = '', digits: int = 1, tail: str = '') -> str:
    """``<quantity> <value> outside [<low>, <high>] [<knob>][; <tail>]`` — the
    two-sided sibling of :func:`bound_msg`, for a band whose two ends share one
    knob name (``num_throws``, the Teensy event-velocity band)."""
    text = '{} {}{} outside [{}, {}]{}'.format(
        quantity, _fmt(value, digits), ' ' + unit if unit else '',
        _fmt(low, digits), _fmt(high, digits), ' ' + unit if unit else '')
    return '{}{}{}'.format(text, _knob(knob), '; ' + tail if tail else '')
