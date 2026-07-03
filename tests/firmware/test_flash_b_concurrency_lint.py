"""Flash-B (Tier-2 item 17) — concurrency/parity STRUCTURAL lint.

Items 1-3 of Flash-B are pure concurrency/timing fixes on the Teensy that the native
doctest harness CANNOT prove: the fake HAL's PRIMASK intrinsics are no-ops,
`IntervalTimer::priority()` is a stub, and doctest runs single-threaded, so there is
no preemption to tear a 64-bit read or reorder a publish. The real concurrency
behaviour is an on-hardware-replay gap (see tests/firmware/native/README.md).

So this lint pins the fixes STRUCTURALLY at the source level — a regression that
reverts any of them fails `pytest tests/ -q` even though the native harness stays
green:

  1. every PLATFORM (AxisState) `.last_heartbeat_us` read/write routes through the
     IRQ-guarded `atomic_read_u64` / `atomic_write_u64` accessors (no torn u64 feeding
     the CAN3 safety watchdog on the 32-bit Cortex-M7). The ball_butler_state.h field
     is EXEMPT — it has its own seqlock (write_bb_heartbeat / snapshot_bb);
  2. `interp_begin_stow()` publishes its payload under a `__disable_irq` /
     `__set_PRIMASK` PRIMASK barrier (so the 500 Hz ISR can't observe s_stow_active
     without the matching s_stow_pos[] — a garbage-base stow descent);
  3. the 500 Hz interp IntervalTimer runs at `priority(16)` (above the FreeRTOS
     syscall ceiling 0x20), NOT the old `priority(32)` (exactly AT the ceiling →
     masked by RTOS critical sections).
"""

from __future__ import annotations

import re
from pathlib import Path

_FW = (Path(__file__).resolve().parents[2]
       / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge")

# Firmware translation units where a platform last_heartbeat_us read/write can appear.
_HB_FILES = [
    "can_buses.cpp",
    "fault_machine.cpp",
    "telemetry.cpp",
    "Teensy_code_canbridge.ino",
    "axis_state.h",
    "ball_butler_state.h",
]

_LEG_INTERP = _FW / "leg_interp.cpp"


def _strip_line_comment(line: str) -> str:
    """Drop a trailing // comment so a mention of the field in prose isn't linted."""
    i = line.find("//")
    return line if i < 0 else line[:i]


def test_no_bare_platform_last_heartbeat_us():
    """Item 1: no bare platform `.last_heartbeat_us` read/write survives — all via
    atomic_*_u64. The bb_state (BallButlerState) field + its seqlock, the local
    BallButlerSnapshot copy, and the field DECLARATIONS are the only exemptions."""
    offenders: list[str] = []
    decl_re = re.compile(r"uint64_t\s+last_heartbeat_us")   # field declarations
    for fname in _HB_FILES:
        path = _FW / fname
        text = path.read_text().splitlines()
        for n, raw in enumerate(text, start=1):
            code = _strip_line_comment(raw)
            if "last_heartbeat_us" not in code:
                continue                                    # comment-only mention
            # ball_butler_state.h owns the BB field + its seqlock accessors.
            if fname == "ball_butler_state.h":
                continue
            # Field declaration (axis_state.h) — not an access.
            if decl_re.search(code):
                continue
            # Wrapped in the IRQ-guarded 64-bit accessor → OK.
            if "atomic_read_u64" in code or "atomic_write_u64" in code:
                continue
            # bb_state is the BallButlerState (own seqlock) — out of scope by design.
            if "bb_state.last_heartbeat_us" in code:
                continue
            # A local BallButlerSnapshot copy (`bb.last_heartbeat_us`) is already atomic.
            if re.search(r"\bbb\.last_heartbeat_us", code):
                continue
            offenders.append(f"{fname}:{n}: {raw.strip()}")
    assert not offenders, (
        "bare platform .last_heartbeat_us access(es) found — route every platform "
        "(AxisState) read/write through atomic_read_u64/atomic_write_u64 "
        "(Flash-B item 17 §1):\n  " + "\n  ".join(offenders))


def _extract_function(text: str, signature: str) -> str:
    """Return the brace-balanced body of the function whose def contains `signature`."""
    start = text.index(signature)
    # find the opening brace of the body
    brace = text.index("{", start)
    depth = 0
    i = brace
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    raise AssertionError(f"unbalanced braces after {signature!r}")


def test_interp_begin_stow_has_primask_barrier():
    """Item 2: interp_begin_stow() wraps its payload publish in a PRIMASK barrier
    (__disable_irq … __set_PRIMASK) so the 500 Hz ISR can't latch a half-written
    s_stow_pos[] behind s_stow_active."""
    body = _extract_function(_LEG_INTERP.read_text(), "void interp_begin_stow()")
    assert "__disable_irq()" in body, (
        "interp_begin_stow() lost its __disable_irq() — the s_stow_active publish is "
        "no longer ordered after the s_stow_pos[] writes (Flash-B item 17 §2)")
    assert "__set_PRIMASK(" in body, (
        "interp_begin_stow() lost its __set_PRIMASK() restore — use the "
        "save-pm/disable/restore-pm idiom (nests safely), not a bare __enable_irq "
        "(Flash-B item 17 §2)")
    # The flag store must be INSIDE the barrier (before the restore).
    assert body.index("s_stow_active = true") < body.index("__set_PRIMASK("), (
        "s_stow_active=true must be set INSIDE the PRIMASK barrier (Flash-B item 17 §2)")


def test_interp_isr_priority_is_above_the_syscall_ceiling():
    """Item 3: the interp IntervalTimer runs at priority(16) (above the 0x20 FreeRTOS
    syscall ceiling), NOT the old priority(32) (exactly AT the ceiling → masked)."""
    text = _LEG_INTERP.read_text()
    assert "s_timer.priority(16)" in text, (
        "leg_interp_init() must set s_timer.priority(16) — above the FreeRTOS syscall "
        "ceiling (0x20) so RTOS critical sections cannot delay the 500 Hz ISR "
        "(Flash-B item 17 §3)")
    assert "s_timer.priority(32)" not in text, (
        "leg_interp_init() still sets priority(32) — that is EXACTLY AT the FreeRTOS "
        "syscall ceiling and is masked by every critical section (Flash-B item 17 §3)")
