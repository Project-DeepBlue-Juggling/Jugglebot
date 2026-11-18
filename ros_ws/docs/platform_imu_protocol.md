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
| 0x6E3 | IMU_CMD | bridge to Platform Teensy | on an RPC (section 6), and one ABORT at the Armed edge |
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
saturating at 255. `sync_peak` is the largest |innovation| of the 0x7DD slave in the period (wall time received minus
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
and one at the first drain after any discontinuity. `flags`: DISCONTINUITY 0x01 (samples lost just before this anchor,
by FIFO overflow or a re-probe: fit each side separately), WALL_INVALID 0x02 (no 0x7DD yet), SYNC_STEP 0x04 (the wall
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
in slices that hold `loop()` for at most about 5 ms each, and sends the FREEZE reply when it is done.

`boot_id` is a random nonzero u32 drawn at boot. The capture_id counter starts at its low 16 bits and steps at each
FREEZE that creates a capture, skipping 0, so an id from before a reboot almost never names a capture after it.

## 4. State machine

States (IMU_STATUS byte 0): DISABLED 0, PROBING 1, RUNNING 2, FROZEN 3, UPLOADING 4, BACKOFF 5, FW_HALTED 6. The
board boots into PROBING (enabled); DISABLE is RAM-only. Internal transitions: PROBING to RUNNING when the chip answers
and is configured; RUNNING to BACKOFF on an I2C error or a failed WHO_AM_I re-check; BACKOFF to PROBING after a bounded
back-off of at least 1 s; any state to FW_HALTED at FwUpdate BEGIN (I2C stopped, capture discarded); FW_HALTED to
PROBING if that update is abandoned; UPLOADING to FROZEN once the block's last frame is handed to FlexCAN. A FIFO
overflow alone keeps RUNNING and marks the next anchor DISCONTINUITY. Recovery from BACKOFF keeps appending to the same
ring behind a DISCONTINUITY anchor; only boot, ENABLE from DISABLED and RESUME start a new capture. There is no I2C
traffic in FROZEN or UPLOADING; RESUME flushes the chip FIFO and re-checks WHO_AM_I.

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

**Platform Teensy TX rule.** An IMU frame is handed to FlexCAN only when it lands directly in a free TX mailbox, never
in the library's software TX queue (the FlexCAN_T4 all-mailboxes-busy defect writes a queued frame into every free
mailbox and drops the rest). A frame that cannot be placed waits in the IMU module, REPLY and STATUS ahead of DATA. At
most one IMU_DATA frame per `loop()` pass, and none sooner than the block's gap after the previous one. The gap is
`frame_gap_10us` x 10 us, raised to `read_min_gap_10us` (400 us) when smaller.

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
Teensy's verdict is the IMU_REPLY). They are reserved by comment in `config/generate_udp_protocol.py` and are defined
there with their `rpc.cpp` cases in P3:

- **0x0064 PLATFORM_IMU_CTRL**, `ArgPlatformImuCtrl {op u8, reason u8, capture_id u16}`, op in {ENABLE, DISABLE,
  FREEZE, RESUME, ABORT} (READ is ERR_BAD_ARGS). Allowed while Armed: a FREEZE at Attempt end or on a Fault must not
  wait for disarm.
- **0x0065 PLATFORM_IMU_READ**, `ArgPlatformImuRead {capture_id u16, first_seq u16, count u16, frame_gap_10us u8}`.
  ERR_REJECTED while Armed; ERR_BAD_ARGS for count 0 or above `read_max_words`.

Both are gated on `jugglebot_commands_allowed()` (ERR_BUS_DOWN). At every not-Armed to Armed edge the bridge sends one
IMU_CMD ABORT itself, so a block in flight stops within one loop pass (arming wins; the capture stays frozen and the
upload resumes after disarm). It uplinks 0x6E4-0x6E6, not the reserved ids. IMU_DATA rides its own ring, drained fast
enough for the requested pace when not Armed (at least 1000 fps).

`teensy_bridge_node` freezes at the ending REST of every Attempt, and on a Fault or Soft E-stop. It uploads when not
Armed, re-requests holes, checks both CRCs, writes `temp/imu_capture/<stamp>/`, then RESUMEs. On start, a status of
FROZEN means an earlier capture is still waiting: a FREEZE returns ALREADY_FROZEN with its id, and the node uploads it.
A changed `boot` byte, or ERR_NOT_FROZEN or ERR_WRONG_CAPTURE mid-upload, means the Platform Teensy rebooted: the
partial capture is written with `complete = false`. A `payload_crc32` mismatch gets one full re-read, then the capture
is written with `crc_ok = false` and RESUMEd.

## 7. Walk-through: one capture with one lost frame

N = 32000 (wrapped), 320 anchors (960 words), header 11 words; 64971 words in all.

1. Ending REST: CTRL FREEZE(ATTEMPT_END). The Platform Teensy goes RUNNING to FROZEN, computes the CRC and replies
   `[03][00][0x4A21][32000]`.
2. Once not Armed: READ(0x4A21, 0xFFF0, 11, 100). It replies `[05][00][0x4A21][0xFFF0 | 11 << 16]`, then sends seq
   0xFFF0..0xFFFA. The node checks `header_crc32`, the version, capture_id against the FREEZE reply, and N.
3. READ the anchors (0xFA80, four blocks: 256, 256, 256, 192), then the samples (250 blocks of 256). The node asks for
   the next block when the current block's last seq arrives, or after `count x gap + 50 ms`.
4. Frame seq 0x1235 (the gyro word of sample 2330) is dropped in the bridge ring. The bitmap shows the hole after its
   block.
5. READ(0x4A21, 0x1235, 1, 100) returns that one frame, and the bitmap is full.
6. `zlib.crc32(anchor bytes + sample bytes)` equals `payload_crc32`. The node writes the capture with `complete = true`.
7. CTRL RESUME(0x4A21): `[04][00][0x4A21][0]`. The ring resets and a new capture starts. A retried RESUME or a stale
   READ now gets ERR_NOT_FROZEN; nothing is lost.

## 8. Bus load and timing

- **During an Attempt:** only IMU_STATUS, 1 fps, +0.011 % on a bus at 64.2 % (111-bit accounting, `plans/active/platform-imu.md` section 2).
  Then one CMD and one REPLY at the FREEZE. A READ is refused while Armed.
- **Upload at the default gap (1000 us):** at most 1000 fps of IMU_DATA, plus one CMD and one REPLY per block (about
  4 blocks per second). An idle bus at about 2280 fps (25.3 %) rises to about 3290 fps (36.5 %; 42.7 % at 130-bit
  stuffing). The frames alone take 65 s; with block turnarounds of a few ms the full capture takes about 66 to 70 s.
  At a 500 us gap (2000 fps, about 48 %) it would take about 35 s, if the bridge ring keeps up.
- **Arbitration:** IMU ids lose to every ODrive and Leg frame. They beat 0x7DD and 0x7DE, but at 1000 fps a 0x7DD
  frame waits at most one IMU frame (about 130 us), and no anchors are taken during an upload.
- **RAM:** the ring is 32000 x 12 B = 384000 B, plus the anchor ring (400 slots), in RAM2 clear of the CrashReport area.
