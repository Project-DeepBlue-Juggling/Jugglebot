"""Round-trip + edge-case tests for the wire-protocol codec."""

from __future__ import annotations

import struct

import pytest

from teensy_link import protocol as p


def test_constants_match_firmware_spec():
    # These are load-bearing — if they ever drift we've broken the protocol.
    #
    # PROTOCOL_VERSION is deliberately NOT pinned here. Its canonical pin is
    # tests/firmware/test_udp_protocol_xlang.py::test_protocol_version_frozen,
    # which checks the generator spec against BOTH the generated Python module
    # (which is where `p.PROTOCOL_VERSION` comes from) and the committed C++
    # header, and whose failure message carries the fleet-reflash procedure.
    # A second literal here duplicated that pin without being named in the
    # procedure, so a bump updated the canonical one and left this one stale —
    # which is exactly what happened on the 4→5 bump (bf1e9a5, 2026-07-31).
    # One constant, one pin.
    assert p.MAGIC == 0x4A42  # "JB" little-endian
    assert p.HEADER_SIZE == 8
    assert p.CRC_SIZE == 2
    assert p.PORT_STREAM == 5005
    assert p.PORT_RPC == 5006
    assert p.NUM_LEGS == 6
    assert p.NUM_AXES == 7
    assert p.HEARTBEAT_HZ == 10


def test_crc16_canonical_check_value():
    # CRC-16/CCITT-FALSE canonical test vector
    assert p.crc16_ccitt(b"123456789") == 0x29B1


def test_heartbeat_j2t_roundtrip():
    hb = p.HeartbeatJ2T(t_jetson_us=0x0123_4567_89AB_CDEF, flags=0xDEAD_BEEF)
    blob = hb.pack()
    assert len(blob) == p.HEARTBEAT_J2T_SIZE
    decoded = p.HeartbeatJ2T.unpack(blob)
    assert decoded.t_jetson_us == hb.t_jetson_us
    assert decoded.flags == hb.flags


def test_cmd_result_frame_roundtrip():
    # The loud command-outcome relay: a raw BB CAN1 frame (0x7D5) wrapped
    # verbatim. The decoded throw outcome lives inside `data` (cmd_type, outcome,
    # detail0/1 int16 LE) — the codec just preserves the 8 payload bytes.
    cr = p.CmdResultFrame(
        t_bridge_us=0x0011_2233_4455_6677,
        can_id=0x7D5, dlc=8,
        data=(0, 0x29, 0x01, 0x00, 0xFA, 0x00, 0, 0),  # THROW, ABORTED_NOT_SETTLED
    )
    blob = cr.pack()
    assert len(blob) == p.CMD_RESULT_FRAME_SIZE
    decoded = p.CmdResultFrame.unpack(blob)
    assert decoded.t_bridge_us == cr.t_bridge_us
    assert decoded.can_id == cr.can_id
    assert decoded.dlc == cr.dlc
    assert tuple(decoded.data) == cr.data


def test_telemetry_roundtrip():
    tm = p.Telemetry(
        t_teensy_us=12345678,
        pos_rev=tuple(0.1 * i for i in range(p.NUM_AXES)),
        vel_rps=tuple(-0.2 * i for i in range(p.NUM_AXES)),
    )
    blob = tm.pack()
    assert len(blob) == p.TELEMETRY_SIZE
    decoded = p.Telemetry.unpack(blob)
    assert decoded.t_teensy_us == tm.t_teensy_us
    assert decoded.pos_rev == pytest.approx(tm.pos_rev)
    assert decoded.vel_rps == pytest.approx(tm.vel_rps)


def test_setpoint_roundtrip():
    sp = p.Setpoint(
        u0=tuple(float(i) for i in range(6)),
        u1=tuple(float(i + 10) for i in range(6)),
        u2=tuple(float(i + 20) for i in range(6)),
        v0=tuple(float(i + 30) for i in range(6)),
        accel=tuple(float(i + 40) for i in range(6)),
        torque_ff=tuple(float(i + 50) for i in range(6)),
        flags=0b101,
        t_origin_us=987654321,
    )
    blob = sp.pack()
    assert len(blob) == p.SETPOINT_SIZE
    decoded = p.Setpoint.unpack(blob)
    assert decoded.flags == sp.flags
    assert decoded.t_origin_us == sp.t_origin_us
    assert decoded.u0 == pytest.approx(sp.u0)
    assert decoded.u1 == pytest.approx(sp.u1)
    assert decoded.u2 == pytest.approx(sp.u2)
    assert decoded.v0 == pytest.approx(sp.v0)
    assert decoded.accel == pytest.approx(sp.accel)
    assert decoded.torque_ff == pytest.approx(sp.torque_ff)


def test_encode_then_decode_full_frame():
    hb = p.HeartbeatJ2T(t_jetson_us=42, flags=0)
    frame = p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 7, hb.pack())
    assert len(frame) == p.HEADER_SIZE + p.HEARTBEAT_J2T_SIZE + p.CRC_SIZE
    mt, seq, payload = p.decode_frame(frame)
    assert mt == int(p.MsgType.HEARTBEAT_J2T)
    assert seq == 7
    assert p.HeartbeatJ2T.unpack(payload).t_jetson_us == 42


def test_decode_rejects_bad_magic():
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[0] = 0  # corrupt magic
    with pytest.raises(ValueError, match="magic"):
        p.decode_frame(bytes(frame))


def test_decode_rejects_bad_crc():
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[-1] ^= 0xFF  # flip a CRC bit
    with pytest.raises(ValueError, match="CRC"):
        p.decode_frame(bytes(frame))


def test_decode_raises_typed_crc_error():
    # CRC corruption raises the typed CrcError (a ValueError subclass, so any
    # legacy `except ValueError` still catches it). The host RX path counts this
    # as a crc_error rather than a structural decode_error.
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[-1] ^= 0xFF  # flip a CRC bit
    assert issubclass(p.CrcError, ValueError)
    with pytest.raises(p.CrcError, match="CRC"):
        p.decode_frame(bytes(frame))


def test_decode_structural_error_is_not_crc_error():
    # A structural failure (bad magic — checked before the CRC) is a plain
    # ValueError, NOT a CrcError, so the two RX-stat buckets never
    # cross-contaminate (the bug the old `"CRC" in str(e)` match risked).
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[0] ^= 0xFF  # corrupt magic
    with pytest.raises(ValueError) as exc:
        p.decode_frame(bytes(frame))
    assert not isinstance(exc.value, p.CrcError)


def test_decode_rejects_inconsistent_length():
    # Header says length=99 but frame too short to actually carry it
    blob = b"\x00" * 4
    head = struct.pack("<HBBHH", p.MAGIC, p.PROTOCOL_VERSION, int(p.MsgType.HEARTBEAT_J2T), 0, 99)
    crc = p.crc16_ccitt(head + blob)
    bogus = head + blob + struct.pack("<H", crc)
    with pytest.raises(ValueError, match="length"):
        p.decode_frame(bogus)


def test_seq_wraps_at_16_bits():
    # The encoded seq field truncates to uint16 — make sure a request for seq=70000
    # still produces a valid frame whose decoded seq is the low 16 bits.
    blob = b"\x00" * 4
    frame = p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 70_000, p.HeartbeatJ2T().pack())
    _, seq, _ = p.decode_frame(frame)
    assert seq == 70_000 & 0xFFFF


# ── HeartbeatT2J torque_clamp_mask (flags bits 8-13, 2026-07-14 ingest clamp) ──
#
# The per-leg torque_ff ingest-clamp mask rides FREE BITS of the existing u32
# flags field — deliberately NO new payload field, NO size change, NO
# PROTOCOL_VERSION bump (an old parser ignores the bits; a new parser reads 0
# from a pre-clamp firmware). These pin the generated constants and the
# extraction arithmetic the bridge node uses for /link_status.


def test_torque_clamp_flag_constants():
    # Shift + mask are generated single-source (config/generate_udp_protocol.py).
    assert p.HEARTBEAT_TORQUE_CLAMP_SHIFT == 8
    assert int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK) == 0x3F00
    # Exactly 6 legs' worth of bits, starting at the shift.
    assert int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK) == (
        ((1 << p.NUM_LEGS) - 1) << p.HEARTBEAT_TORQUE_CLAMP_SHIFT)
    # The mask must not collide with the single-bit flags (bits 0-3).
    single_bits = (int(p.HeartbeatT2JFlags.TIME_SYNCED)
                   | int(p.HeartbeatT2JFlags.STOW_PENDING_ON_RECONNECT)
                   | int(p.HeartbeatT2JFlags.ALL_AXIS_HEARTBEATS_OK)
                   | int(p.HeartbeatT2JFlags.MPC_ACTIVE))
    assert single_bits & int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK) == 0


def test_heartbeat_t2j_torque_clamp_mask_roundtrip():
    # Firmware packs: flags |= (mask << SHIFT) & TORQUE_CLAMP_MASK. Parse side
    # extracts: (flags & TORQUE_CLAMP_MASK) >> SHIFT. Round-trip through the real
    # payload codec with other flag bits set, so the extraction can't alias them.
    leg_mask = 0b000101  # legs 0 and 2 clamped
    flags = (int(p.HeartbeatT2JFlags.TIME_SYNCED)
             | int(p.HeartbeatT2JFlags.MPC_ACTIVE)
             | ((leg_mask << p.HEARTBEAT_TORQUE_CLAMP_SHIFT)
                & int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK)))
    hb = p.HeartbeatT2J(t_teensy_us=1, flags=flags, uptime_ms=1)
    blob = hb.pack()
    assert len(blob) == p.HEARTBEAT_T2J_SIZE  # still 73 B — no wire-size change
    decoded = p.HeartbeatT2J.unpack(blob)
    extracted = ((int(decoded.flags) & int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK))
                 >> p.HEARTBEAT_TORQUE_CLAMP_SHIFT)
    assert extracted == leg_mask
    # The single-bit flags survive alongside the mask.
    assert decoded.flags & int(p.HeartbeatT2JFlags.TIME_SYNCED)
    assert decoded.flags & int(p.HeartbeatT2JFlags.MPC_ACTIVE)
    assert not decoded.flags & int(p.HeartbeatT2JFlags.STOW_PENDING_ON_RECONNECT)


def test_torque_clamp_mask_zero_on_legacy_flags():
    # A pre-clamp firmware never sets bits 8-13, so extraction over any legacy
    # flags word (bits 0-3 only) must read 0 — the "dormant until reflash" story.
    for legacy in (0x0, 0x1, 0x3, 0xF):
        hb = p.HeartbeatT2J(t_teensy_us=1, flags=legacy, uptime_ms=1)
        decoded = p.HeartbeatT2J.unpack(hb.pack())
        assert ((int(decoded.flags) & int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK))
                >> p.HEARTBEAT_TORQUE_CLAMP_SHIFT) == 0


# ── CLOCK_DIAG (0x8F) — bridge-temporal-trustworthiness P1 ───────────────────
#
# The additive per-anchor clock-discipline + interp-occupancy uplink introduced
# with can-bridge FW 11. FW 11 is committed UNFLASHED (the flash reboots the
# Teensy and destroys the aged uptime state the S1 experiment interrogates), so
# for the whole pre-flash window these tests are the ONLY thing standing between
# the frame and a silent layout error: no board is emitting it and no bag can
# contradict it. That is the reason to pin the semantics here and not merely the
# byte count.


def test_clock_diag_msg_type_is_143_and_below_rpc_response():
    # 0x8F was the LAST free uplink id under RPC_RESPONSE (0x90). A regression
    # that renumbers it silently re-points every recorded frame at another
    # message type, and a bag captured across the change would decode as garbage
    # rather than fail — so this is pinned as a literal, not derived.
    assert int(p.MsgType.CLOCK_DIAG) == 143 == 0x8F
    assert int(p.MsgType.CLOCK_DIAG) < int(p.MsgType.RPC_RESPONSE)
    # Nothing else may share the id.
    others = [int(v) for k, v in vars(p.MsgType).items()
              if isinstance(v, p.MsgType) and k != 'CLOCK_DIAG']
    assert 143 not in others


def test_clock_diag_is_additive_protocol_version_unchanged():
    """FW 11 adds a message type and changes NO existing frame.

    The whole reason CLOCK_DIAG is a new ``MsgType`` rather than fields appended
    to PROFILE is that the decoders here are exact-size unpacks: an appended
    field turns into a per-frame decode error on an unaware Jetson, so the frame
    goes DARK instead of degrading. A new type is ignored cleanly. That property
    is only worth anything if ``PROTOCOL_VERSION`` also stays put — a bump makes
    ``decode_frame`` reject EVERY frame in both directions (the 24608bb
    total-darkness failure), which would take the link down over a diagnostic.
    """
    assert p.PROTOCOL_VERSION == 5
    # The frames that existed before FW 11 keep their exact sizes.
    assert p.PROFILE_SIZE == 76
    assert p.HEARTBEAT_T2J_SIZE == 73
    assert p.LEG_CMD_SIZE == 56
    assert p.BRIDGE_TX_DIAG_SIZE == 42
    assert p.BRIDGE_IDENTITY_SIZE == 3


def test_clock_diag_roundtrip():
    # Distinct, sign-varied values per field: with a uniform payload a
    # transposed pair or a truncated slice still "looks plausible". err_us and
    # freq_ppb are the two SIGNED fields and both are given negative values —
    # an accidental 'I' in the struct format would raise on pack rather than
    # round-trip, which is exactly the failure this catches.
    cd = p.ClockDiag(
        t_local_us=0x0123_4567_89AB_CDEF,
        jetson_wall_us=1_754_000_000_123_456,
        dt_local_us=30_000_017,
        rtt_us=1_234,
        err_us=-987,
        freq_ppb=-12_345,
        anchor_seq=4_294_967_295,      # u32 max — the wrap boundary
        interp_ticks=15_000,
        recover_slew_ticks=37,
        extrap_ticks=421,
        flags=int(p.ClockDiagFlags.FREQ_VALID),
    )
    blob = cd.pack()
    assert len(blob) == p.CLOCK_DIAG_SIZE == 49
    d = p.ClockDiag.unpack(blob)
    assert d.t_local_us == 0x0123_4567_89AB_CDEF
    assert d.jetson_wall_us == 1_754_000_000_123_456
    assert d.dt_local_us == 30_000_017
    assert d.rtt_us == 1_234
    assert d.err_us == -987
    assert d.freq_ppb == -12_345
    assert d.anchor_seq == 4_294_967_295
    assert d.interp_ticks == 15_000
    assert d.recover_slew_ticks == 37
    assert d.extrap_ticks == 421
    assert d.flags == int(p.ClockDiagFlags.FREQ_VALID)


def test_clock_diag_signed_fields_survive_the_extremes():
    # err_us saturates at the int32 rails in firmware (a STEP can carry an
    # arbitrary jump), so both rails must survive the codec rather than wrap into
    # a plausible small number with the wrong sign.
    for err, freq in ((-2_147_483_648, 2_147_483_647),
                      (2_147_483_647, -2_147_483_648)):
        d = p.ClockDiag.unpack(p.ClockDiag(err_us=err, freq_ppb=freq).pack())
        assert d.err_us == err
        assert d.freq_ppb == freq


def test_clock_diag_step_vs_slew_flag_semantics():
    """STEPPED / FIRST_ANCHOR / FREQ_VALID are independent bits with a contract.

    The semantics being pinned, all three of which a consumer MUST honour:

    * a SLEWED anchor (the steady-state case) carries neither STEPPED nor
      FIRST_ANCHOR, and normally carries FREQ_VALID — this is the only shape
      whose ``freq_ppb`` is a crystal measurement;
    * a STEPPED anchor never carries FREQ_VALID, because a step means the
      measured offset moved further in one interval than any crystal can explain
      (>20 ms in 30 s is >666 ppm), so the interval contains a link gap, a boot,
      or a host clock jump;
    * the FIRST anchor since boot carries STEPPED **and** FIRST_ANCHOR — it
      steps the whole epoch — and its ``err_us`` of 0 means the value does not
      EXIST, there having been no prior offset to difference against.

    The bits are three separate powers of two so a consumer can test them
    independently; a shared/overlapping encoding would make "stepped" and "first"
    indistinguishable, and the first anchor of every session would then look like
    a mid-session fault.
    """
    F = p.ClockDiagFlags
    assert (int(F.STEPPED), int(F.FIRST_ANCHOR), int(F.FREQ_VALID)) == (1, 2, 4)

    # Steady state: slewed, frequency sample usable.
    slewed = p.ClockDiag.unpack(
        p.ClockDiag(err_us=-412, freq_ppb=-8_700, dt_local_us=30_000_000,
                    flags=int(F.FREQ_VALID)).pack())
    assert not slewed.flags & int(F.STEPPED)
    assert not slewed.flags & int(F.FIRST_ANCHOR)
    assert slewed.flags & int(F.FREQ_VALID)

    # First anchor after boot: stepped AND first, no frequency sample, err_us 0
    # because there was nothing to difference against.
    first = p.ClockDiag.unpack(
        p.ClockDiag(err_us=0, freq_ppb=0, dt_local_us=0,
                    anchor_seq=1,
                    flags=int(F.STEPPED) | int(F.FIRST_ANCHOR)).pack())
    assert first.flags & int(F.STEPPED)
    assert first.flags & int(F.FIRST_ANCHOR)
    assert not first.flags & int(F.FREQ_VALID)
    assert first.err_us == 0

    # Mid-session re-acquisition after a link gap: stepped but NOT first, so
    # err_us is real (and may be saturated), and the frequency sample is refused.
    regained = p.ClockDiag.unpack(
        p.ClockDiag(err_us=2_147_483_647, freq_ppb=0,
                    dt_local_us=4_294_967_295, anchor_seq=97,
                    flags=int(F.STEPPED)).pack())
    assert regained.flags & int(F.STEPPED)
    assert not regained.flags & int(F.FIRST_ANCHOR)
    assert not regained.flags & int(F.FREQ_VALID)

    # THE misread this encoding exists to prevent: freq_ppb is 0 on every sample
    # above that lacks FREQ_VALID, and that 0 is NO SAMPLE — never "zero
    # frequency error". A consumer that reads the number without the bit cannot
    # tell a perfectly-disciplined crystal from a refused measurement.
    for sample in (first, regained):
        assert sample.freq_ppb == 0
        assert not sample.flags & int(F.FREQ_VALID)


def test_clock_diag_full_frame_roundtrip_and_unknown_type_tolerance():
    # End-to-end through the framing layer, and the standing wire-compatibility
    # property that makes an additive MsgType safe: a decoder that has never
    # heard of 0x8F still decodes the FRAME (magic/version/CRC/length all check
    # out) and hands back the type for the dispatcher to ignore. Nothing raises,
    # so an FW 11 board on an old Jetson is quiet, not broken.
    cd = p.ClockDiag(t_local_us=42, anchor_seq=7,
                     flags=int(p.ClockDiagFlags.FREQ_VALID))
    frame = p.encode_frame(int(p.MsgType.CLOCK_DIAG), 11, cd.pack())
    mt, seq, payload = p.decode_frame(frame)
    assert mt == 143
    assert seq == 11
    assert p.ClockDiag.unpack(payload).anchor_seq == 7


# ── CACHE_DIAG (0x91) — the encoder-cache freshness census, FW 12 ────────────
#
# The confirmation instrument for the one question the S1 aged-bridge experiment
# left open: with the transport, the interp deadline, the heap, CAN throughput,
# the ODrives and the Jetson-side link all cleared, is the Teensy's ENCODER
# CACHE going stale with uptime (so the lead clamp pins the command to a
# position the leg left hundreds of ms ago), or does the leg genuinely trail?
# Like CLOCK_DIAG before it, FW 12 is committed before any board runs it, so
# these tests are the only thing standing between the frame and a silent layout
# error.


def test_cache_diag_msg_type_is_145_and_opens_a_block_above_rpc_response():
    # 0x91 is the FIRST id of a new uplink block: 0x81-0x8F was full and 0x90 is
    # RPC_RESPONSE. Pinned as a literal because a renumber would silently
    # re-point every already-recorded frame at another message type — a bag
    # spanning the change would decode as garbage rather than fail.
    assert int(p.MsgType.CACHE_DIAG) == 145 == 0x91
    # Deliberately ABOVE RpcResponse, and that is safe BY CONSTRUCTION: the
    # STREAM/RPC split is which socket a frame was sent on, and both ends
    # dispatch through a msg_type table. If anyone ever adds a range test
    # (`msg_type < RPC_RESPONSE` as a stand-in for "is an uplink"), this
    # assertion is the note that it was considered and rejected.
    assert int(p.MsgType.CACHE_DIAG) > int(p.MsgType.RPC_RESPONSE)
    # Nothing else may share the id.
    others = [int(v) for k, v in vars(p.MsgType).items()
              if isinstance(v, p.MsgType) and k != 'CACHE_DIAG']
    assert 145 not in others


def test_cache_diag_is_additive_protocol_version_unchanged():
    """FW 12 adds a message type and changes NO existing frame.

    Same reasoning as CLOCK_DIAG's equivalent: these decoders are exact-size
    unpacks, so an appended field on an existing frame is a per-frame decode
    error on an unaware Jetson and the frame goes DARK instead of degrading. A
    new type is ignored cleanly. And that only holds if ``PROTOCOL_VERSION``
    stays put — a bump makes ``decode_frame`` reject EVERY frame in both
    directions (the 24608bb total-darkness failure), which would take the link
    down over a diagnostic.
    """
    assert p.PROTOCOL_VERSION == 5
    # Every frame that existed before FW 12 keeps its exact size — including
    # CLOCK_DIAG, which shipped one firmware revision earlier and is the one
    # most likely to be disturbed by an edit in the same region of the spec.
    assert p.CLOCK_DIAG_SIZE == 49
    assert p.PROFILE_SIZE == 76
    assert p.HEARTBEAT_T2J_SIZE == 73
    assert p.LEG_CMD_SIZE == 56
    assert p.BRIDGE_TX_DIAG_SIZE == 42
    assert p.BRIDGE_IDENTITY_SIZE == 3


def test_cache_diag_roundtrip():
    # Distinct values per field and per ARRAY SLOT: the two 7-wide age arrays sit
    # adjacent on the wire, so a uniform payload would let an off-by-one slice
    # (min[1:8] read as max[0:7]) round-trip while silently attributing every
    # leg's age to its neighbour. Every slot here is unique across BOTH arrays.
    cd = p.CacheDiag(
        t_local_us=226_800_000_000,          # 63 h — the S1 aged-bridge point
        age_min_us=(101, 202, 303, 404, 505, 606, 707),
        age_max_us=(3_701, 3_802, 3_903, 4_004, 4_105, 4_206, 4_307),
        enc_frames=(61_000_001, 61_000_002, 61_000_003, 61_000_004,
                    61_000_005, 61_000_006, 61_000_007),
        seq=3_600,
        window_us=1_000_013,
        rx_cap_hits_jb=0,
        rx_cap_hits_bb=7,
        rx_cap_hits_cone=13,
        decode_short=2,
        decode_bad_axis=5,
        rx_depth_hwm_jb=9,
        rx_depth_hwm_bb=3,
        rx_depth_hwm_cone=1,
        samples=100,
        seen_mask=0x7F,
    )
    blob = cd.pack()
    assert len(blob) == p.CACHE_DIAG_SIZE == 129
    d = p.CacheDiag.unpack(blob)
    assert d.t_local_us == 226_800_000_000
    assert tuple(d.age_min_us) == (101, 202, 303, 404, 505, 606, 707)
    assert tuple(d.age_max_us) == (3_701, 3_802, 3_903, 4_004, 4_105, 4_206, 4_307)
    assert tuple(d.enc_frames) == (61_000_001, 61_000_002, 61_000_003,
                                   61_000_004, 61_000_005, 61_000_006,
                                   61_000_007)
    assert d.seq == 3_600
    assert d.window_us == 1_000_013
    assert d.rx_cap_hits_jb == 0
    assert d.rx_cap_hits_bb == 7
    assert d.rx_cap_hits_cone == 13
    assert d.decode_short == 2
    assert d.decode_bad_axis == 5
    assert d.rx_depth_hwm_jb == 9
    assert d.rx_depth_hwm_bb == 3
    assert d.rx_depth_hwm_cone == 1
    assert d.samples == 100
    assert d.seen_mask == 0x7F


def test_cache_diag_per_axis_arrays_are_seven_wide_legs_then_hand():
    # Six legs AND the hand: the hand shares the same axes[] cache and its
    # dispatch shift tracked the same uptime curve (2026-07-28 anomaly-fixes
    # sitting), so a six-wide array would have excluded the very axis that first
    # made the drift visible outside the leg path.
    #
    # All THREE per-axis arrays are checked together because they are read
    # together, per axis: an age is only interpretable against that same axis's
    # frame count (a stalled value with the count advancing is a different fault
    # from a stalled value with the count paused). One array drifting in width
    # would silently misalign exactly that comparison.
    cd = p.CacheDiag()
    assert len(cd.age_min_us) == 7
    assert len(cd.age_max_us) == 7
    assert len(cd.enc_frames) == 7
    # The mask is one bit per axis and must reach axis 6.
    d = p.CacheDiag.unpack(p.CacheDiag(seen_mask=1 << 6).pack())
    assert d.seen_mask == 0x40


def test_cache_diag_enc_frames_are_cumulative_and_wrap_correct():
    """The per-axis frame counters survive the u32 boundary, and 0 is a value.

    They are CUMULATIVE SINCE BOOT and differenced by the consumer, so the only
    codec property that matters is that the whole u32 range round-trips —
    including the wrap boundary, since unsigned differencing across it is
    correct and must not be "helped" by a signed slip. At the ~272 frames/s/axis
    broadcast rate the wrap is ~182 days away, which is well beyond any uptime
    this investigation contemplates but is exactly the sort of assumption that
    goes unchecked until it bites.

    0 is pinned separately because it is the DIAGNOSTIC value, not a null: an
    axis whose count does not move across a window received nothing, which is
    the upstream half of the split this field exists to make.
    """
    d = p.CacheDiag.unpack(p.CacheDiag(
        enc_frames=(0, 1, 4_294_967_294, 4_294_967_295, 0, 2_147_483_648, 7)
    ).pack())
    assert tuple(d.enc_frames) == (0, 1, 4_294_967_294, 4_294_967_295, 0,
                                   2_147_483_648, 7)
    # Wrap-correct differencing across the u32 boundary, which is how the
    # consumer reads two adjacent samples.
    assert (d.enc_frames[3] + 3) % 2**32 == 2
    assert (2 - d.enc_frames[3]) % 2**32 == 3


def test_cache_diag_ages_saturate_at_the_u32_rail_rather_than_wrapping():
    """The saturation rail survives the codec intact.

    The firmware clamps an age to ``UINT32_MAX`` instead of letting it wrap,
    because a wrapped age reads as a small, HEALTHY number — the single failure
    this frame must never produce, given that its whole job is to detect ages
    that grew. The rail is also what an axis that has never been cached reports,
    which is why ``seen_mask`` is a separate field rather than an inference from
    a magic value.
    """
    rail = 0xFFFF_FFFF
    d = p.CacheDiag.unpack(p.CacheDiag(
        age_min_us=(rail,) * 7, age_max_us=(rail,) * 7,
        window_us=rail, seq=rail, samples=0xFFFF, seen_mask=0).pack())
    assert tuple(d.age_min_us) == (rail,) * 7
    assert tuple(d.age_max_us) == (rail,) * 7
    assert d.window_us == rail
    assert d.seq == rail
    assert d.samples == 0xFFFF
    # seen_mask 0 with every age at the rail is the legitimate "nothing has ever
    # been cached" shape (a bridge that booted with no ODrives on the bus). It
    # must decode, not raise: a pre-arm frame is the FIRST one a session sees.
    assert d.seen_mask == 0


def test_cache_diag_full_frame_roundtrip_and_unknown_type_tolerance():
    # End-to-end through the framing layer, plus the property that makes an
    # additive MsgType safe: a decoder that has never heard of 0x91 still decodes
    # the FRAME (magic/version/CRC/length all check out) and hands the type back
    # for the dispatcher to ignore. Nothing raises, so an FW 12 board talking to
    # an old Jetson is quiet, not broken — and 0x91 sitting ABOVE RPC_RESPONSE
    # changes nothing about that, which is the point of exercising it here.
    cd = p.CacheDiag(t_local_us=99, seq=5, seen_mask=0x3F)
    frame = p.encode_frame(int(p.MsgType.CACHE_DIAG), 21, cd.pack())
    mt, seq, payload = p.decode_frame(frame)
    assert mt == 145
    assert seq == 21
    assert p.CacheDiag.unpack(payload).seq == 5
