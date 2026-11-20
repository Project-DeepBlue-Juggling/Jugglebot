# Platform IMU capture protocol (IDEA-20)

Normative contract for the Platform Teensy's IMU capture on the jugglebot bus: the CAN frames, the frozen-capture
image, the state machine, and what the can-bridge and `teensy_bridge_node` must do. Every number named here lives in
`config/protocol_config.yaml` (`can_ids.platform.IMU_*` and `platform_imu:`) and reaches code only through
`python config/generate_config.py` (`PlatformImu*` in C++, `PLATFORM_IMU_*` / `PlatformImu*` in Python).
`tests/firmware/test_platform_imu_protocol.py` pins the YAML against this page. Plan:
`plans/active/platform-imu.md` (P2 builds the Platform Teensy side, P3 the can-bridge and Jetson side).

All frames are classic CAN, 11-bit, DLC 8, little-endian (the repo convention: FwUpdate, 0x7DD, Wave). Unused bytes
are 0. Raw sensor counts go on the wire; scaling and calibration happen on the Jetson.

## 1. IDs

| ID | Name | Direction | When |
| --- | --- | --- | --- |
| 0x6E3 | IMU_CMD | bridge to Platform Teensy | on an RPC (section 6), including the Jetson's ABORT at a pacifying command or Attempt start |
| 0x6E4 | IMU_REPLY | Platform Teensy to bridge | one per IMU_CMD |
| 0x6E5 | IMU_STATUS | Platform Teensy to bridge | every `status_period_ms` (1 s), in every state |
| 0x6E6 | IMU_DATA | Platform Teensy to bridge | only inside an accepted READ block |
| 0x6E7, 0x6E8 | IMU_LIVE_ACCEL, IMU_LIVE_GYRO | reserved | never sent; held for a future live stream |

The block (`can_block_first..last`) sits beside the Platform Teensy's 0x6E0-0x6E2, far above the ODrive range
(0x000-0x11F, nodes 0-8): the bridge decodes any non-whitelisted id below 0x0E0 into an ODrive axis cache, so an IMU id
there would corrupt it. Command outranks reply, reply outranks status, status outranks data.

## 2. Frames

**IMU_CMD** `[op u8][args]`:

| op | Args (bytes 1-7) |
| --- | --- |
| ENABLE 0x01, DISABLE 0x02, ABORT 0x06 | none |
| FREEZE 0x03 | `[reason u8]`: ATTEMPT_END 1, FAULT 2, SOFT_ESTOP 3, OPERATOR 4 (0 is ERR_BAD_ARGS) |
| RESUME 0x04 | `[capture_id u16]` |
| READ 0x05 | `[capture_id u16][first_seq u16][count u16][frame_gap_10us u8]`; count 1..`read_max_words` (256) |

**IMU_REPLY** `[op u8][status u8][capture_id u16][detail u32]`, the FwUpdate reply shape. `capture_id` is the frozen
capture's id (0 when none). `detail`: FREEZE gives `n_samples`; READ gives `first_seq | count << 16` on OK and
`first_seq | region_end << 16` on ERR_OUT_OF_RANGE; otherwise 0. Status codes: 0x0x accepted, 0x1x refused with
nothing changed and nothing sent. OK 0x00; ALREADY_FROZEN 0x01 (FREEZE found a capture frozen earlier, so this
Attempt was not recorded); ERR_BAD_ARGS 0x10; ERR_NOT_FROZEN 0x11; ERR_WRONG_CAPTURE 0x12; ERR_OUT_OF_RANGE 0x13;
ERR_EMPTY 0x14 (FREEZE with no samples); ERR_DISABLED 0x15; ERR_FW_UPDATE 0x16.

**IMU_STATUS** `[state u8][flags u8][temp_c i8][fifo_overflows u8][i2c_errors u8][sync_peak u8][fill_s u8][boot u8]`.
`flags`: SENSOR_OK 0x01, TIME_SYNCED 0x02 (a 0x7DD frame in the last period), RING_WRAPPED 0x04, I2C_400K 0x08,
DISCONTINUITY 0x10. `temp_c` is the chip's temperature rounded (-128 = unknown). The two counters are since boot,
saturating at 255; `i2c_errors` counts every failed I2C transfer, including those the RUNNING retry absorbs (section 4). `sync_peak` is the largest |innovation| of the 0x7DD slave in the period (wall time received minus
`get_wall_time_us()` before the IIR update), in 10 us counts, 254 saturating, 255 = no sync frame. `fill_s` is retained
samples / `nominal_odr_hz`. `boot` is the low byte of `boot_id` (section 3).

**IMU_DATA** `[seq u16][word 6 B]`. `seq` addresses the frozen-capture image (section 3), so a frame's `seq` alone says
what it carries and a gap is found from `seq` alone, across blocks and across a late tail from a replaced block.

## 3. The frozen-capture image

A capture is the ring's retained samples, oldest first, plus its anchors and a header, presented as one address space
of 6-byte words with fixed bases:

| Region | seq | Content |
| --- | --- | --- |
| samples | `0 .. 2N-1` (N <= `ring_samples` = 32000) | word 2k = `ax ay az` of retained sample k, 2k+1 = `gx gy gz`; int16 chip LSB, chip axes |
| anchors | `anchor_seq_base` 0xFA80 `+ 3a .. 3a+2` (a < `anchor_ring` = 400) | 18-byte anchor a, oldest first |
| header | `header_seq_base` 0xFFF0 `.. +10` | the 66-byte header |

A READ must lie wholly inside one region's valid part, else ERR_OUT_OF_RANGE with nothing sent. The sketch's separate
GET_HEADER and READ_ANCHORS replies are READs of these regions, so header, anchors and samples share one loss-recovery
path and one CRC.

**Anchor** (18 B): `[index u32][local_us u32][wall_us u64][flags u8][0]`. `index` counts samples appended since the
capture began, after the drain the anchor belongs to. `local_us` is the low 32 bits of `micros64()` taken immediately
before that drain's FIFO_COUNT read, and `wall_us` is the same instant on the synced wall clock (`micros64()` plus the
offset, snapshotted together). Pairing rule: sample `index - 1` was produced no later than `local_us` plus the
FIFO_COUNT read latency (tens of us). One anchor is taken at the first drain after each `anchor_period_ms` (100 ms),
and one at the first drain after any discontinuity. Before every discontinuity and at every FREEZE the board also emits a
**closing anchor**: an ordinary-flag anchor stamped with the drain that produced the samples pushed since the previous
anchor (the earlier drain's `local_us` and `wall_us`), so the pre-gap segment ends exactly on an ordinary anchor. A reader
splits segments at the DISCONTINUITY anchor: samples with index before the closing anchor's `index` are pre-gap, and
samples from the closing anchor's `index` up to the DISCONTINUITY anchor's `index` are post-gap (the first drain after
the gap); the closing anchor itself belongs to the pre-gap fit. `flags`: DISCONTINUITY 0x01 (samples lost just before this anchor,
by FIFO overflow, a bad FIFO header, a failed FIFO burst read or a re-probe: fit each side separately), WALL_INVALID 0x02 (no 0x7DD yet), SYNC_STEP 0x04 (the wall
offset was stepped since the previous anchor).

**Header** (66 B, 11 words):

| Off | Type | Field | Off | Type | Field |
| --- | --- | --- | --- | --- | --- |
| 0 | u8 | `header_format_version` (1) | 36 | u16 | gyro LSB per dps x10 (328 at +/-1000 dps) |
| 1 | u8 | freeze reason | 38 | u8 | raw GYRO_ACCEL_CONFIG0 (UI filter) |
| 2 | u16 | capture_id | 39 | u8 | raw ACCEL_CONFIG0 |
| 4 | u32 | boot_id | 40 | u8 | raw GYRO_CONFIG0 |
| 8 | u32 | n_samples (N) | 41 | u8 | I2C clock, 100 kHz units |
| 12 | u32 | first_index (the `index` of retained sample 0) | 42 | i16 | raw TEMP_DATA at capture start |
| 16 | u16 | n_anchors | 44 | i16 | raw TEMP_DATA at freeze |
| 18 | u16 | capture flags: WRAPPED 1, DISCONTINUITY 2, SENSOR_ERROR 4, NEVER_SYNCED 8, I2C_400K 0x10 | 46 | u16 | FIFO overflows in this capture |
| 20 | u64 | freeze wall_us | 48 | u16 | I2C errors in this capture |
| 28 | u32 | freeze local_us | 50 | u16 | largest sync innovation in this capture, us |
| 32 | u16 | ODR, Hz (nominal) | 52 | u16 | Platform Teensy FW_VERSION |
| 34 | u16 | accel LSB per g (2048 at +/-16 g) | 54 | u32 | `payload_crc32` |
| | | | 58 | u32 | reserved 0 |
| | | | 62 | u32 | `header_crc32` over bytes 0-61 |

Both CRCs are CRC-32/IEEE (Python `zlib.crc32`, the FwUpdate VERIFY CRC). `payload_crc32` covers the anchor words then
the sample words, in seq order, exactly as the IMU_DATA payloads carry them. The Platform Teensy computes it at FREEZE,
in slices of `CRC_SLICE_BYTES` (8192 B, expected to hold `loop()` for about 1 ms each; P2 times one slice on the board), and sends the FREEZE reply when it is done.

`boot_id` is a nonzero u32 that changes on every warm boot: it mixes a boot counter kept in RAM2 (not cleared by the
core's startup code, so it survives a warm reset) and the RTC 32 kHz count with the cycle counter and the timing of CAN
frames received during `setup()`; no hardware RNG is called. The capture_id counter starts at its low 16 bits and steps at each
FREEZE that creates a capture, skipping 0, so an id from before a reboot almost never names a capture after it.

## 4. State machine

States (IMU_STATUS byte 0): DISABLED 0, PROBING 1, RUNNING 2, FROZEN 3, UPLOADING 4, BACKOFF 5, FW_HALTED 6. The
board boots into PROBING (enabled); DISABLE is RAM-only. For the first `BOOT_SETTLE_MS` (10 s) after boot the board does
no IMU work: no I2C, no IMU_STATUS, and every IMU_CMD is dropped unanswered. The first probe and the first STATUS come at
10 s, so no STATUS for 10 s after a boot is normal. Internal transitions: PROBING to RUNNING when the chip answers
and is configured; RUNNING to BACKOFF on the third failed FIFO transfer in a row (below) or a failed WHO_AM_I re-check; BACKOFF to PROBING after a bounded
back-off of at least 1 s; any state to FW_HALTED at FwUpdate BEGIN (I2C stopped, capture discarded); FW_HALTED to
PROBING if that update is abandoned, unless the BEGIN arrived during the 10 s boot settle, in which case FW_HALTED holds
until the next reboot (see "Recovery if IMU code faults"); UPLOADING to FROZEN once the block's last frame is handed to FlexCAN. Probe failures
stay in PROBING (BACKOFF is entered only from RUNNING), re-probing after 1, 2, 4, 8, 16 and 32 s, then every 60 s; the
schedule depends only on the sensor, not on host polling, and 10 s of clean RUNNING resets it. A board with no IMU
therefore shows PROBING with SENSOR_OK clear and a rising `i2c_errors`: that is not an alarm. A FIFO
overflow alone keeps RUNNING and marks the next anchor DISCONTINUITY.

**Retry in RUNNING (FW 12).** A failed FIFO transfer while RUNNING is counted in `i2c_errors` and, if it is the
first or second in a row, absorbed: the next loop pass clears the bus (nine SCL pulses and a STOP, then an I2C
controller reset, ~0.12 ms, no device transaction) and the pass after retries. A failed FIFO_COUNT read took
nothing from the FIFO, so it is simply counted again: no DISCONTINUITY, no sample lost. A failed burst read leaves
the chip's FIFO read position unknown, so the rest of that drain is dropped, a closing anchor is written, the FIFO is
flushed and the next anchor is DISCONTINUITY. A failed flush is recovered and flushed again. A bad FIFO header (the
transfer completed, its content did not parse) is handled the same way (nothing of that burst is kept, flush,
DISCONTINUITY) and joins the run of failures, but it is not an I2C error and needs no bus clear. A successful burst ends
the run; a successful count or flush ends it only if the run holds no failed burst and no bad header (after those, the
recovery, flush and count succeed even when every burst then fails again). The third failure in a row goes to BACKOFF
as above. A closing anchor written in the middle of a drain (a failed burst, a bad header, a FREEZE) has its
`local_us` and `wall_us` moved back by the drain's counted-but-unread packets x 980 us (the nominal 1 ms period less
2 %), so it stays an upper bound on its last sample's time without the drain's lag. The board also clears the bus before every re-probe that
follows an error. A retry costs about three loop passes, far inside the 128 ms the chip FIFO holds at 1 kHz. Recovery from BACKOFF keeps appending to the same
ring behind a DISCONTINUITY anchor; only boot, ENABLE from DISABLED and RESUME start a new capture. There is no I2C
traffic in FROZEN or UPLOADING. Nothing in this table or in the Platform Teensy depends on Armed: the gating below is the bridge's and the Jetson's; RESUME flushes the chip FIFO and re-checks WHO_AM_I.

Legality (checks run in this order: op and length, FW_HALTED, DISABLED, state, capture_id, range):

| op | DISABLED | PROBING, BACKOFF | RUNNING | FROZEN | UPLOADING | FW_HALTED |
| --- | --- | --- | --- | --- | --- | --- |
| ENABLE | to PROBING | OK, no change | OK | OK | OK | ERR_FW_UPDATE |
| DISABLE | OK | to DISABLED | to DISABLED | to DISABLED, capture discarded | same | ERR_FW_UPDATE |
| FREEZE | ERR_DISABLED | to FROZEN with SENSOR_ERROR, or ERR_EMPTY | to FROZEN | ALREADY_FROZEN | ALREADY_FROZEN, block continues | ERR_FW_UPDATE |
| RESUME | ERR_DISABLED | ERR_NOT_FROZEN | ERR_NOT_FROZEN | new capture: RUNNING, or PROBING if the chip fails | same, block cut | ERR_FW_UPDATE |
| READ | ERR_DISABLED | ERR_NOT_FROZEN | ERR_NOT_FROZEN | to UPLOADING | replaces the block | ERR_FW_UPDATE |
| ABORT | OK | OK | OK | OK | to FROZEN | ERR_FW_UPDATE |

RESUME and READ carry a `capture_id`; a mismatch with the frozen one is ERR_WRONG_CAPTURE. That is what makes an RPC
retry safe: a delayed RESUME can never thaw a capture frozen after it was sent.

**Platform Teensy TX rule.** An IMU frame is handed to `write()` only when the software TX queue is empty and at least
5 FlexCAN mailboxes are free (4 stay free after it for the tilt, state, fan and firmware-update replies), so it always
lands directly in a mailbox and never in the library's software TX queue (the FlexCAN_T4 all-mailboxes-busy defect writes
a queued frame into every free mailbox and drops the rest). A frame that cannot be placed waits in the IMU module, REPLY
and STATUS ahead of DATA; the IMU keeps at most 4 frames pending. At most one IMU_DATA frame per `loop()` pass, and none
sooner than the block's gap after the previous one. The gap is `frame_gap_10us` x 10 us, raised to
`read_min_gap_10us` (400 us) when smaller.

**Compile-time kill switch.** `PLATFORM_IMU_ENABLED=0` compiles every IMU path out. That image ignores IMU_CMD (0x6E3,
it falls to the ordinary frame handling), sends no 0x6E4-0x6E6 frame, and still reports its FW version (11 or later), so the host can infer
"no IMU" only from STATUS silence: no STATUS for at least 3 s on an FW 11+ board that has been up for more than 10 s is a
no-IMU build, not a fault.

**Recovery if IMU code faults.** The mechanism: a fault in IMU code makes the core write a CrashReport, wait 8 s and
reset. The board then reboots about every 18.5 s (8 s CrashReport delay, about 0.4 s boot, 10 s settle) and is alive and
IMU-free for about 10 s of each cycle. A firmware-update BEGIN that lands in that window latches the IMU off for that
boot (FW_HALTED holds until the next reboot even if the update is abandoned), so the update completes. The Jetson uploader
today sends ONE BEGIN and waits 1.0 s (`teensy_link_bridge.py`, `_FW_REPLY_TIMEOUT_S`), about 54 % per attempt, so the
operator re-runs the upload until the log shows BEGIN OK (about 5 back-to-back attempts cover a cycle); P3 builds the
retry into the uploader. Not covered: hangs (there is no watchdog), faults before the settle check (static
initialisers, `setup()`, the per-frame hooks in `canSniff`), and faults in the non-IMU FW 11 changes (tilt service,
dispatch). Decision record: a CrashReport safe mode was considered and not built (owner decision 2026-11-18 against
safe-boot machinery); the settle window plus the BEGIN latch cover the same IMU-path crash class.

## 5. Timestamps (the Jetson's fit)

Per segment between DISCONTINUITY anchors, the Jetson fits `local_us(k) = a + b k` over the anchors (k = `index - 1 -
first_index`). The slope comes from least squares and the intercept from the lower envelope of the residuals, since
each anchor is an upper bound on its sample's time and drain jitter only adds lag. The fit takes no nominal rate on
trust, so the chip's ~1 % clock error is measured, not assumed. Local time maps to wall time through a straight-line fit
of `wall_us - local_us` against `local_us` (piecewise at SYNC_STEP). That keeps the 0x7DD slave's dispatch jitter out
of the rate fit. The residual spread is written to the capture metadata as its timing quality. No anchor is taken while
FROZEN, so upload traffic cannot bias a capture's timestamps.

## 6. can-bridge and Jetson obligations (P3)

Two typed RPCs, following HAND_FAN_SET (the bridge lays out the frame; its OK means queued on CAN, and the Platform
Teensy's verdict is the IMU_REPLY). They are defined in `config/generate_udp_protocol.py` with their `rpc.cpp` cases
(can-bridge FW 45, P3a); an FW 44 board answers both with ERR_UNKNOWN_METHOD:

- **0x0064 PLATFORM_IMU_CTRL**, `ArgPlatformImuCtrl {op u8, reason u8, capture_id u16}`, op in {ENABLE, DISABLE,
  FREEZE, RESUME, ABORT} (READ and any unknown op are ERR_BAD_ARGS, nothing sent, so a READ can only pass the clamp
  below). The bridge puts `reason` in byte 1 for FREEZE and `capture_id` in bytes 1-2 for RESUME, zeroes the rest, and
  leaves judging their values to the Platform Teensy.
- **0x0065 PLATFORM_IMU_READ**, `ArgPlatformImuRead {capture_id u16, first_seq u16, count u16, frame_gap_10us u8}`.
  ERR_BAD_ARGS for count 0 or above `read_max_words`. Both RPCs are ALLOWED while Armed (owner, 2026-11-19).

Both are gated on `jugglebot_commands_allowed()` (ERR_BUS_DOWN). It uplinks 0x6E4-0x6E6, not the reserved ids:
IMU_REPLY and IMU_STATUS join the Platform reply whitelist and ride the existing 8-slot reply ring as PLATFORM_FRAMEs
(a few frames per second: STATUS is 1 Hz and the host sends the next IMU_CMD only after the previous reply), and
IMU_DATA takes the dedicated ring of item 5.

**Boot silence.** A Platform Teensy that has just booted sends no IMU_STATUS for 10 s (section 4); the Jetson treats that
as normal, not as a fault, and sends no IMU_CMD in that window (the board drops them unanswered). A board with no IMU
shows PROBING with SENSOR_OK clear; a no-IMU build shows STATUS silence (kill switch, section 4).

**Uploads run while Armed.** Owner decisions of 2026-11-18/19, superseding the first design (READ refused while Armed,
an ABORT at every arming edge):

1. *Where the gate lives.* The can-bridge cannot know what an Attempt is, so the gate is on the Jetson: it starts an
   upload only when no Attempt is running, with the robot Armed and holding the Active pose (the STANDBY hold of
   `ARMING_CONTRACT.md`) between Attempts. The bridge does not ABORT on arming any more; the Jetson sends ABORT at a
   pacifying command or at an Attempt start. Failure prevented: a bridge-side Armed refusal would make a 50 s upload
   impossible in the one state, Active, where the Platform sits between Attempts, so the data would never leave the
   board before the next Attempt overwrote the ring.
2. *Toggle (GUI, P3).* With "Upload IMU data after each Attempt" ON (default ON) the Jetson REFUSES the Attempt-starting
   command (the Juggle request) and every other command that starts motion while an upload is in progress. Pacifying
   commands (deactivate, stow, Soft E-stop and the like) always pass and abort the upload; that capture is flagged
   `complete = false`. With the toggle OFF the ring is never frozen and nothing is refused; switching it ON applies from
   the next Attempt. The GUI shows progress and time remaining. Failure prevented: an Attempt starting with the ring
   frozen (no capture of it), and a stop command waiting behind a transfer.
3. *Target.* The full window (64000 data frames, plus 960 anchor words and the header) uploads in under 60 s: about
   1250 frames/s on the bus, 51 s. Only the Attempt's span plus a margin is requested, so a shorter Attempt uploads
   proportionally faster.
4. *Bus protection.* While Armed the bridge enforces a hard minimum inter-frame gap by clamping the READ's
   `frame_gap_10us` up to `bridge_armed_min_gap_10us` (63 x 10 us = 630 us, about 1590 frames/s; Armed is the bridge's
   `fault_mpc_active()`). Not Armed, the gap goes out as asked and the Platform Teensy's own floor applies. The operating pace is
   a Jetson config value passed as that argument, so it is tunable without a reflash. The Platform Teensy's own
   `read_min_gap_10us` (400 us) is below the bridge cap, so no YAML constant changes. Failure prevented: a bad Jetson
   config value flooding the bus the Legs and Hand share.
5. *Uplink.* One UDP datagram per CAN frame at about 1250 frames/s is too much work in the bridge loop that also emits
   the Leg and Hand setpoints. IMU_DATA frames get their own bridge ring and a new ADDITIVE batched UDP message type
   (no PROTOCOL bump, per the additive-message precedent): about 16 data frames per datagram (`seq` plus 6-byte word
   each), flushed at the 100 Hz telemetry tick. P3 defines the struct in `config/generate_udp_protocol.py`. REPLY and
   STATUS keep their existing per-frame route. As built (FW 45): MsgType PLATFORM_IMU_BATCH 0x95, 220-byte payload
   `{t_bridge_us u64, count u8, flags u8, ring_hwm u16, rx_frames u32, ring_drops u32, batches_sent u32, send_fails
   u32, frames u8[192]}`, where `frames` holds `count` (at most 24) records of the IMU_DATA frame's 8 data bytes
   verbatim, zero after. The ring holds 128 frames (80 ms at the cap), is lock-free single-producer single-consumer,
   and drops the NEWEST frame when full; the counters are cumulative since bridge boot, so a hole in `seq` with
   `ring_drops` unchanged was lost on the bus or in UDP, not in the bridge. At most 4 datagrams leave per tick.

*As built (P3c, 2026-11-19).* The refused commands are `standby`, `trajectory`, `spacemouse`, `gui`, `level`, `home` and the
Juggle request (orchestrator command topic, the Juggle relay and `skill_node`'s Juggle goal); `deactivate` always passes
and aborts, the Soft E-stop aborts through the bridge's Fault edge; `activate`, `clear_errors` and the Juggle stop are never
gated. The decision is one pure function, `jugglebot/platform_imu_gate.imu_upload_gate`, reading cached
`platform_imu/status` only: a status older than 2 s or absent allows (a dead bridge node cannot lock the operator out), and an
upload busy for more than 180 s counts as idle and is aborted. Toggle OFF mid-upload aborts the running upload (capture
flagged incomplete). The Attempt span is 2 s before the first motion to 3 s after the end, the last 32 s if longer.

`teensy_bridge_node` freezes at the ending REST of every Attempt, and on a Fault or Soft E-stop. It uploads when no
Attempt is running, re-requests holes, checks both CRCs, writes `temp/imu_capture/<stamp>/`, then RESUMEs. On start, a
status of FROZEN means an earlier capture is still waiting: a FREEZE returns ALREADY_FROZEN with its id, and the node
uploads it. A changed `boot` byte, or ERR_NOT_FROZEN or ERR_WRONG_CAPTURE mid-upload, means the Platform Teensy
rebooted: the partial capture is written with `complete = false`, as is a capture whose upload a pacifying command
aborted. A `payload_crc32` mismatch gets one full re-read, then the capture is written with `crc_ok = false` and
RESUMEd.

Host behaviour added by the P3 audit (no wire change): (a) a Fault or Soft E-stop freezes at once but the READs start
`imu_fault_upload_delay_s` later (default 5 s, at most 60 s; while still Armed up to 30 s more), and a second non-NONE
fault value in a cascade does not abort the upload the first edge started; (b) every early end of an upload RESUMEs the
capture the node froze (the one exception is a Platform that rebooted or discarded it), an abort sends RESUME alone
(legal from UPLOADING, table above), and a ring that STATUS still shows FROZEN with no upload running is healed: RESUME
by the remembered capture id, or, for a capture frozen by a process that is gone, a FREEZE probe (ALREADY_FROZEN returns
the id; STATUS carries none) then RESUME, after 3 s or 15 s, at most 3 tries per frozen episode; (c) a span-only upload
has no payload CRC (it covers the whole capture) and `complete` then means the requested span arrived; (d) the toggle
is default ON at every node start (not persisted).

## 7. Walk-through: one capture with one lost frame

N = 32000 (wrapped), 320 anchors (960 words), header 11 words; 64971 words in all.

1. Ending REST: CTRL FREEZE(ATTEMPT_END). The Platform Teensy goes RUNNING to FROZEN, computes the CRC and replies
   `[03][00][0x4A21][32000]`.
2. Once no Attempt is running: READ(0x4A21, 0xFFF0, 11, 80). It replies `[05][00][0x4A21][0xFFF0 | 11 << 16]`, then sends seq
   0xFFF0..0xFFFA. The node checks `header_crc32`, the version, capture_id against the FREEZE reply, and N.
3. READ (gap 80, 800 us) the anchors (0xFA80, four blocks: 256, 256, 256, 192), then the samples (250 blocks of 256). The node asks for
   the next block when the current block's last seq arrives, or after `count x gap + 50 ms`.
4. Frame seq 0x1235 (the gyro word of sample 2330) is dropped in the bridge ring. The bitmap shows the hole after its
   block.
5. READ(0x4A21, 0x1235, 1, 100) returns that one frame, and the bitmap is full.
6. `zlib.crc32(anchor bytes + sample bytes)` equals `payload_crc32`. The node writes the capture with `complete = true`.
7. CTRL RESUME(0x4A21): `[04][00][0x4A21][0]`. The ring resets and a new capture starts. A retried RESUME or a stale
   READ now gets ERR_NOT_FROZEN; nothing is lost.

## 8. Bus load and timing

Facts and sources (surveys of the 2026-11-18 design round, `plans/active/platform-imu.md` section 2):

- Every ODrive and setpoint frame has a lower CAN id (higher priority) than 0x6E6, so an upload frame can delay a
  control frame by at most one frame time, about 0.13 ms at 111 bit/frame. Source: the id map in
  `config/protocol_config.yaml` and the arbitration rule.
- The bridge emits setpoints from a 500 Hz timer interrupt to every Armed axis on every tick, including HOLD
  (`leg_interp.cpp`), which also feeds the ODrive watchdog. So the between-Attempts bus load is about the Attempt's:
  about 59 % with the Hand Idle and 64 % with the Hand Closed-loop (2280 versus 5277 frames/s in the 2026-11-13 and
  2026-11-18 bags' `/profile`).
- 1250 frames/s of IMU_DATA adds about 14 % (111-bit frames). The risk is bridge loop time (existing residue: 47 ms
  worst emit gap; Leg scheduler grace 30 ms), not bus arbitration.

Numbers:

- **During an Attempt:** only IMU_STATUS, 1 fps, +0.011 % on a bus at 64.2 %. Then one CMD and one REPLY at the
  FREEZE. No upload runs.
- **Upload (the target):** `frame_gap_10us` 80 gives 1250 fps; the 64000 data frames take 51 s, and with block
  turnarounds of a few ms plus the anchors and header the full capture lands under 60 s. At the bridge cap (625 us,
  1600 fps) about 40 s; at the old 1000 us default about 66 to 70 s. `read_default_gap_10us` (100, 1000 fps) stays the
  default the YAML ships; the Jetson's configured pace overrides it per READ.
- **Rate ramp (P4):** the operating pace is measured, not assumed; see the plan's P4 gate.
- **No anchors** are taken during an upload (FROZEN), so upload traffic cannot bias a capture's timestamps.
- **RAM:** the ring is 32000 x 12 B = 384000 B, plus the anchor ring (400 slots), in RAM2 clear of the CrashReport area.
