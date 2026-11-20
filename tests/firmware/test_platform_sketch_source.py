"""Source-level checks on the Platform Teensy sketch that no native binary can make (P2 audit T2-T4).

The sketch (`Teensy_code_platform.ino`) needs the Teensy core, so its glue is never compiled on
the host. These tests pin, from the source text:

  * the kill switch: with `PLATFORM_IMU_ENABLED=0` every IMU path is compiled out, i.e. every
    IMU token in the sketch sits inside an `#if PLATFORM_IMU_ENABLED` region (T4);
  * loop()'s order: the CAN dispatch, then tilt, then `FwUpdate::tick()`, then
    `imuService.pass()`, so a BEGIN or an abandoned update is seen before any I2C in the same
    pass (T3 f), and the dispatch goes through `LoopService::Can::may_dispatch()` (B2);
  * setup() starts no Wire: Wire is configured at the first probe, inside the 10 s settle's
    protected region (B3);
  * FlexCAN_T4's own source still has the shapes the native dispatch model encodes: the
    events() return packing, the `frame.mb == -1` copy-into-every-free-mailbox loop, the
    software-queue fallback in write(), and the mailboxOffset() formula (T2), in every installed
    framework-arduinoteensy (the pinned teensy@5.1.0 builds with 1.160.0). Skipped when none is.
"""
from __future__ import annotations

import glob
import os
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
SKETCH = REPO / "ros_ws" / "src" / "jugglebot" / "Teensy_code_platform" / "Teensy_code_platform.ino"

IMU_TOKEN = re.compile(r"\b(?:imu\w*|Imu\w*|Wire\b|icm42688\w*|PlatformImu\w*|IMU_I2C_HZ|IMU_(?:CMD|STATUS|DATA|REPLY)\b)")


def _strip(src: str) -> str:
    """Comments and string/char literals blanked, newlines kept (so line numbers survive)."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("\n" * src.count("\n", i, j))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append(" ")
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def _code() -> str:
    return _strip(SKETCH.read_text())


def _body(code: str, signature: str) -> str:
    start = code.index(signature)
    i = code.index("{", start)
    depth = 0
    for j in range(i, len(code)):
        depth += {"{": 1, "}": -1}.get(code[j], 0)
        if depth == 0:
            return code[i:j + 1]
    raise AssertionError(f"unbalanced body for {signature}")


def test_kill_switch_compiles_out_every_imu_path():
    stack: list[bool] = []          # per #if level: is this branch the IMU-enabled one?
    offenders = []
    for lineno, line in enumerate(_code().splitlines(), 1):
        d = line.strip()
        if d.startswith("#"):
            word = d[1:].strip().split()
            kw = word[0] if word else ""
            if kw in ("if", "ifdef", "ifndef"):
                stack.append(kw == "if" and re.fullmatch(r"#\s*if\s+PLATFORM_IMU_ENABLED", d) is not None)
            elif kw in ("else", "elif"):
                stack[-1] = False
            elif kw == "endif":
                stack.pop()
            if kw == "include" and not any(stack) and IMU_TOKEN.search(d):
                offenders.append((lineno, d))
            continue
        if not any(stack) and IMU_TOKEN.search(line):
            offenders.append((lineno, d))
    assert not stack, "unbalanced #if in the sketch"
    assert not offenders, (
        "IMU code outside #if PLATFORM_IMU_ENABLED (the -DPLATFORM_IMU_ENABLED=0 build would keep it):\n"
        + "\n".join(f"  line {n}: {t}" for n, t in offenders))


def test_loop_order_dispatch_tilt_fwupdate_then_imu():
    loop = _body(_code(), "void loop()")
    order = ["LoopService::Can::may_dispatch(", "can1.events()", "tiltService.service(",
             "FwUpdate::tick()", "imuService.pass(", "updateHandFan()"]
    pos = [loop.find(t) for t in order]
    assert all(p >= 0 for p in pos), dict(zip(order, pos))
    assert pos == sorted(pos), dict(zip(order, pos))
    assert loop.count("can1.events()") == 1
    dispatch = loop[pos[0]:pos[1]]
    assert "FwUpdate::sessionOpen()" in dispatch and "canFreeTxMailboxes()" in dispatch
    assert "can1.getTXQueueCount()" in dispatch
    assert "FwUpdate::sessionOpen()" in loop[pos[4]:]


def test_setup_starts_no_wire_and_init_start_does():
    code = _code()
    assert "Wire." not in _body(code, "void setup()")
    init_start = _body(code, "void init_start(uint16_t odr_hz)")
    assert "Wire.begin()" in init_start and "Wire.setClock(" in init_start


def _flexcan_sources():
    pats = [os.path.expanduser("~/.platformio/packages/framework-arduinoteensy*/libraries/FlexCAN_T4/FlexCAN_T4.tpp")]
    return sorted({p for pat in pats for p in glob.glob(pat)})


@pytest.mark.skipif(not _flexcan_sources(), reason="no Teensy framework (FlexCAN_T4) installed")
@pytest.mark.parametrize("tpp", _flexcan_sources())
def test_flexcan_t4_has_the_shapes_the_dispatch_model_encodes(tpp):
    src = re.sub(r"\s+", " ", Path(tpp).read_text())
    # events(): one RX frame per call, and the packing loop()'s may_dispatch() decodes.
    assert "return (uint64_t)(rxBuffer.size() << 12) | txBuffer.size();" in src
    # events(): the queued frame copied into EVERY free mailbox, one pop per copy (the defect).
    assert re.search(r"if \( frame\.mb == -1 \) \{ for \(uint8_t i = mailboxOffset\(\); i < FLEXCANb_MAXMB_SIZE\(_bus\); i\+\+\) "
                     r"\{ if \( FLEXCAN_get_code\(FLEXCANb_MBn_CS\(_bus, i\)\) == FLEXCAN_MB_CODE_TX_INACTIVE \) \{ "
                     r"(?://[^}]*?)?writeTxMailbox\(i, frame\); txBuffer\.pop_front\(\);", src)
    # write(): no free mailbox -> the software queue.
    assert "msg_copy.mb = -1; struct2queueTx(msg_copy);" in src
    # mailboxOffset(): the formula LoopService::Can::mailbox_offset() restates.
    assert ("uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_bus) - 6 /* MAXMB - FIFO */ - "
            "((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);") in src
