"""The ``jugglebot/juggle_request`` / ``skills/nominate`` request grammar.

Pure Python (no ROS2).  One definition, two readers: the orchestrator's relay
(``orchestrator_node._svc_juggle_request``, which refuses every key not in
:data:`JUGGLE_NUMERIC_FIELDS`) and skill_node's ``skills/nominate`` service
(which also allows :data:`NOMINATE_EXTRA_FIELDS`).

``<pattern>[,apex_m=<f>][,separation_mm=<f>][,dwell_s=<f>][,num_cycles=<n>]``
(``skills/nominate`` only: ``[,apex_m_ball2=<f>][,columns_angle_deg=<f>]``).
"""
from __future__ import annotations

import math
from typing import Dict, Mapping, Optional, Tuple

#: goal field -> parser.  The keys ARE the Juggle.action field names, so the
#: wire string reads the same as the goal it becomes.  Each parser returns the
#: value or raises ValueError; ``0`` is legal and means "the node's default",
#: exactly as an omitted key does.
JUGGLE_NUMERIC_FIELDS = {
    'apex_m': float,
    'separation_mm': float,
    'dwell_s': float,
    'num_cycles': int,
}

#: The two extra keys only ``skills/nominate`` accepts.  Each overrides the
#: node PARAMETER of the same name for that one nomination (nothing is
#: written to the parameter); a goal still reads both from the parameters.
NOMINATE_EXTRA_FIELDS = {
    'apex_m_ball2': float,
    'columns_angle_deg': float,
}


def parse_juggle_request(data, extra_keys: Optional[Mapping] = None
                         ) -> Tuple[str, Dict[str, float]]:
    """``<pattern>[,key=value]...`` -> ``(pattern, {field: value})``, or raise
    ValueError naming the offending token.  Tokens after the pattern may come
    in any order.  ``extra_keys`` (key -> parser, e.g.
    :data:`NOMINATE_EXTRA_FIELDS`) widens the accepted keys.

    Strict on purpose: an unknown or malformed token REFUSES the request
    rather than being dropped, because dropping it would silently fly the
    node's default in place of a value the operator typed (a typo'd
    ``apex=1.2`` becoming a 0.9 m attempt).  Negative and non-finite values
    are refused here too -- they can only be a GUI bug, and 0 is already the
    spelling of "use the node's default"."""
    fields = dict(JUGGLE_NUMERIC_FIELDS)
    if extra_keys:
        fields.update(extra_keys)
    parts = [p.strip() for p in str(data).split(',')]
    pattern = parts[0]
    if not pattern:
        raise ValueError('no pattern')
    values: Dict[str, float] = {}
    for tok in parts[1:]:
        key, sep, raw = tok.partition('=')
        key = key.strip()
        parse = fields.get(key)
        if not sep or parse is None:
            raise ValueError('unknown token %r' % (tok,))
        if key in values:
            raise ValueError('%s given twice' % (key,))
        try:
            value = parse(raw.strip())
        except ValueError:
            raise ValueError('%s=%r is not a valid %s'
                             % (key, raw.strip(), parse.__name__)) from None
        if not math.isfinite(value) or value < 0:
            raise ValueError('%s=%r must be finite and >= 0'
                             % (key, raw.strip()))
        values[key] = value
    return pattern, values
