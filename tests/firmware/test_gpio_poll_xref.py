"""Cross-reference the ball-sensor poller's SOURCE against the generated headers.

WHAT THIS GUARDS — AND WHAT IT DOES NOT
---------------------------------------
This is one of **two** layers over ``gpio_poll.cpp``, and it is the one that
executes nothing.  The poller is perfectly host-compilable — it touches CAN3
only through ``can_jugglebot_send()`` and the clock only through ``time_base.h``,
both of which the native harness's fake HAL defines, and it makes no FlexCAN or
FreeRTOS call at all — so its *behaviour* is asserted as a running binary by
``tests/firmware/native/test_gpio_poll.cpp``: the three-state ``Get_Version``
gate, timeout-is-not-a-miss, the asymmetric debounce, the stale-gap reseed, the
arrival-stamped ``t_bridge_us``, the runtime toggle's drain.  None of that is
re-litigated here.

What a source scan is uniquely good for is the class of defect a *passing
binary cannot see*, because the binary would pass just as happily with the
wrong constant compiled into it:

1. **A bare endpoint literal.**  ODrive endpoint ids are firmware-build-specific,
   and the wrong one *answers plausibly* — 700 on a Pro 0.6.11 is
   ``encoder_estimator1.status``, a live-looking sensor that never changes with
   no timeout to diagnose it (plan § "the repo's existing endpoint id is wrong
   for the Pro").  A hard-coded 726 would survive a future re-qualification of
   the endpoint table that moved the symbol; a hard-coded 700/488 is the
   wrong-board bug itself.  The poller must reference
   ``EndpointId::odrive_pro_0_6_11::get_gpio_states``, and no bare id may appear.

2. **A bare tuning literal.**  ``gpio_pin`` / ``check_interval_ms`` /
   ``max_missing_samples`` / ``check_timeout_ms`` / ``expected_fw`` are config
   values that Phase 7 is expected to retune from hardware data; a transcribed
   ``2`` or ``5`` in the firmware silently ignores the YAML.

3. **The ``#if`` trap** (recorded in
   ``logbook/2026-07-29-hand-sensor-ball-detect-config.md``): ``ENABLED`` emits
   as a ``constexpr bool``, not a macro.  ``#if JB_BD_ENABLED`` evaluates an
   *undefined identifier* to 0 and silently compiles the poller out even when
   the YAML says enabled — working firmware that does nothing, discovered on the
   bench.  The compile-out must be a constexpr-gated early return.

It also pins the RX half of the round trip, in ``can_buses.cpp``: the ``TxSdo``
case that routes a reply into the poller.  That case is not compiled by the
native driver (which calls ``gpio_poll_record()`` directly), and its endpoint
match is the only thing stopping a reply to some *other* SDO read from
masquerading as a sensor sample — so it must name the same board-qualified
symbol the request used, never a transcribed id.

Pure stdlib + the generated Python constants (the same source the C++ header is
generated from), so it always runs: no compiler, no ROS 2, no hardware.
"""

from __future__ import annotations

import os
import re

import jugglebot.hardware_config as hw      # noqa: E402 (on sys.path via tests/conftest)
import jugglebot.protocol_config as proto   # noqa: E402


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot',
                       'Teensy_code_canbridge')
_CPP = os.path.join(_FW_DIR, 'gpio_poll.cpp')
_CAN_BUSES = os.path.join(_FW_DIR, 'can_buses.cpp')


def _read(path: str) -> str:
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def _src() -> str:
    return _read(_CPP)


def _code_only(text: str) -> str:
    """Strip // and /* */ comments — literals in prose are documentation."""
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', text)


# ── the endpoint symbol ───────────────────────────────────────────────────────

def test_request_uses_the_board_qualified_endpoint_symbol():
    assert 'EndpointId::odrive_pro_0_6_11::get_gpio_states' in _code_only(_src())


def test_no_bare_endpoint_id_literal_in_the_poller():
    """726 (Pro), 700 (S1 — the wrong-board id), 488 (S1 commutation mapper)."""
    code = _code_only(_src())
    for bad in (proto.ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES,
                proto.ENDPOINT_ODRIVE_S1_0_6_11_GET_GPIO_STATES,
                proto.ENDPOINT_ODRIVE_S1_0_6_11_COMMUTATION_MAPPER_POS_ABS):
        assert not re.search(r'(?<![\w.])%d(?![\w.])' % bad, code), (
            'bare endpoint literal %d in gpio_poll.cpp — use the '
            'EndpointId:: symbol' % bad)


# ── the JBBallDetect config symbols ───────────────────────────────────────────

def test_every_ball_detect_config_value_is_referenced_by_symbol():
    code = _code_only(_src())
    for name in ('ENABLED', 'GPIO_PIN', 'CHECK_INTERVAL_MS',
                 'MAX_MISSING_SAMPLES', 'CHECK_TIMEOUT_MS', 'EXPECTED_FW'):
        assert 'JBBallDetect::%s' % name in code, (
            'gpio_poll.cpp never reads JBBallDetect::%s' % name)


def test_expected_fw_is_compared_elementwise_inside_the_version_gate():
    """The gate must compare all three elements — a two-element compare would
    bless 0.6.12, whose ``get_gpio_states`` id is 764, not 726.  Scoped to the
    gate's own body: the park message prints the triple too, and a print is not
    a compare."""
    code = _code_only(_src())
    m = re.search(r'\n\s*static\s+Gate\s+version_gate\s*\([^)]*\)\s*\{(.*?)\n\}',
                  code, re.S)
    assert m is not None, 'no version_gate() in gpio_poll.cpp'
    body = m.group(1)
    for i in range(len(hw.JB_BD_EXPECTED_FW)):
        assert 'JBBallDetect::EXPECTED_FW[%d]' % i in body, (
            'version_gate() never compares element %d of the fw triple' % i)
    assert not re.search(r'EXPECTED_FW\[%d\]' % len(hw.JB_BD_EXPECTED_FW), body)


def test_no_bare_debounce_or_timing_literal():
    """The pin, the poll period and the debounce depth all come from YAML."""
    code = _code_only(_src())
    # 20 / 100 as bare millisecond literals; 5 as a bare sample count. (2 is too
    # common a small integer to screen for — GPIO_PIN is covered by the
    # symbol-reference test above.)
    for bad in (hw.JB_BD_CHECK_INTERVAL_MS, hw.JB_BD_CHECK_TIMEOUT_MS,
                hw.JB_BD_MAX_MISSING_SAMPLES):
        assert not re.search(r'(?<![\w.\[])%d(?![\w.\]])' % bad, code), (
            'bare literal %d in gpio_poll.cpp — use the JBBallDetect:: '
            'constant' % bad)


# ── the RX half: can_buses.cpp's TxSdo routing case ───────────────────────────

def _txsdo_case_body() -> str:
    """The body of ``case ODriveCmd::TxSdo:`` in can_buses.cpp, comments stripped.

    Scoped to the case rather than the whole file because can_buses.cpp is 1,000+
    lines of unrelated decode; a bare-literal screen over all of it would trip on
    unrelated arithmetic and teach the next reader to widen the exemption."""
    code = _code_only(_read(_CAN_BUSES))
    m = re.search(r'case\s+ODriveCmd::TxSdo\s*:(.*?)\n\s*(?:case\s|default\s*:)',
                  code, re.S)
    assert m is not None, (
        'no `case ODriveCmd::TxSdo:` in can_buses.cpp — the poller has no reply '
        'path, so it can only ever time out')
    return m.group(1)


def test_txsdo_decode_matches_on_the_board_qualified_endpoint_symbol():
    body = _txsdo_case_body()
    assert 'EndpointId::odrive_pro_0_6_11::get_gpio_states' in body, (
        'the TxSdo case must match the reply on the same board-qualified '
        'endpoint symbol the request used')
    assert 'gpio_poll_record' in body, (
        'the TxSdo case decodes the reply but never routes it to the poller')


def test_no_bare_endpoint_id_literal_in_the_txsdo_decode():
    """Same three ids as the request-side screen: a transcribed id here would
    accept the WRONG endpoint's reply as a ball reading."""
    body = _txsdo_case_body()
    for bad in (proto.ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES,
                proto.ENDPOINT_ODRIVE_S1_0_6_11_GET_GPIO_STATES,
                proto.ENDPOINT_ODRIVE_S1_0_6_11_COMMUTATION_MAPPER_POS_ABS):
        assert not re.search(r'(?<![\w.])%d(?![\w.])' % bad, body), (
            'bare endpoint literal %d in can_buses.cpp\'s TxSdo case — use the '
            'EndpointId:: symbol' % bad)


# ── the compile-out mechanism (the recorded Phase 3 trap) ─────────────────────

def test_compile_out_is_constexpr_gated_not_preprocessor_gated():
    src = _src()
    assert not re.search(r'^\s*#\s*if.*(JB_BD_|JBBallDetect)', src, re.M), (
        'ENABLED is a constexpr bool, not a macro: #if JB_BD_ENABLED evaluates '
        'an undefined identifier to 0 and compiles the poller out silently')
    assert re.search(r'if\s*\(\s*!\s*JBBallDetect::ENABLED\s*\)\s*return',
                     src), 'no constexpr-gated early return on !ENABLED'
