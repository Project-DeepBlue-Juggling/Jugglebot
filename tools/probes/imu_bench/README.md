# IMU bench: Teensy 4.0 + ICM-42688-P

Bench proof of concept for map Item IDEA-20 (Platform accelerometer). A standalone Teensy 4.0 streams an
ICM-42688-P over USB serial; a single-file browser page (Web Serial) plots it and shows the orientation.
**Status, 2026-11-18: runs on hardware (address 0x68). Zero FIFO overflows and I2C errors at 1 and 2 kHz, sensor rate 999.97 Hz at 1 kHz, |a| = 1.004 g at rest Z up; the other five faces are not yet measured. Numbers: `logbook/2026-11-18-imu-bench-proof-of-concept.md`.**

The driver (`firmware/src/icm42688.h/.cpp`) is transport-agnostic (takes a `TwoWire&`) so it can be lifted
into the Platform Teensy firmware later; the bench sketch (`firmware/src/main.cpp`) owns the serial protocol.

## The board

Elecrow ICM-42688-P breakout, SKU PIA11001P (sold by LogicalEdges); no schematic is published. Pads as reported by the
owner: SDA, SCL, CS, A-DO, A-CS, INT2, VDD, AD0, A-SD, A-SC, INT1, VIO, GND. The `A-*` pads are presumably the
auxiliary-bus (AUX1) pins and are unused here. Its Stemma QT (Qwiic) connector carries I2C.

## Wiring (Stemma QT / Qwiic)

| Stemma QT wire | Teensy 4.0 |
|---|---|
| black, GND | GND |
| red, 3V3 | 3V3 (NOT 5 V / VIN) |
| blue, SDA | pin 18 (Wire SDA0) |
| yellow, SCL | pin 19 (Wire SCL0) |

Multimeter checks on this undocumented board, unpowered then powered:
1. **Pull-ups**: with the board powered from 3V3 and the Teensy not driving the bus, SDA and SCL should each
   read about 3.3 V to GND. 0 V or floating means no pull-ups: add 2.2-4.7 kOhm from SDA and SCL to 3V3
   (the Teensy's internal pull-ups are weak and are not enabled by `Wire.begin()` for fast modes).
2. **CS (chip select)**: on the ICM-42688-P, CS must be tied high (3V3) for I2C mode. Check CS-to-3V3 continuity
   on the breakout; a floating CS can leave the chip in SPI mode, which looks like "nothing answers".
3. **Address strap** (the AD0 pad here): AP_AD0 low = 0x68, high = 0x69. The firmware tries both and reports which one answered.
4. INT pins are not used (polled only).

## Build and flash

```
cd tools/probes/imu_bench/firmware
pio run                 # compile  (also: ~/.platformio/penv/Scripts/pio.exe run)
pio run -t upload       # flash via the Teensy micro-USB (press the program button if asked)
```
The platform is pinned (`teensy@5.1.0`) like the Platform Teensy project. Build output (`.pio/`) is gitignored.
If the IMU is not found the board prints `#E ...` diagnostics (address tried, hints) every second and keeps retrying.

## Viewer

Open `viewer/index.html` in desktop Chrome or Edge, click **Connect**, pick the Teensy's port. Close any other
program holding the port (serial monitor, `pio device monitor`). If the browser refuses Web Serial from `file://`,
serve it: `cd viewer && python -m http.server 8000`, then open `http://localhost:8000/`.
**Demo data** (button, or open the page with `?demo=1`) runs a scripted motion with no hardware.

Features: 3D board following a 6-axis Mahony filter (calibrate gyro bias while still; reset orientation; **yaw drifts,
no magnetometer**), scrolling accel (g) and gyro (dps) plots with fixed range-aware axes and a Y zoom, tiles (|a|,
roll/pitch from gravity, temperature, host and sensor-timestamp sample rates, missed sequence numbers, FIFO overflows,
per-axis std over the last 2 s), device controls, and CSV recording of every sample (no decimation).
Drawing is requestAnimationFrame-driven and decimated; the data path (filter, CSV) sees every sample.

## Serial protocol (USB CDC, text, newline-terminated)

Device to host:
- `#S {json}` status: `ready, addr, who, odr, accel_fs_g, gyro_fs_dps, lsb_per_g, lsb_per_dps, temp_lsb_per_c, temp_off_c,
  ts_tick_us, i2c_hz, overflows, serial_drops, i2c_errs, bad_packets, paused, seq`. Sent at boot, on `s`, and after any command.
- `#I text`, `#W text`, `#E text`: info, warning (FIFO overflow, flushes), error/init diagnostic.
- `D,seq,ts,ax,ay,az,gx,gy,gz,temp`: one sample. `seq` counts every sample the chip delivered (gaps mean the host link
  dropped lines), `ts` is the chip FIFO timestamp unwrapped to 32 bit (ticks of `ts_tick_us`, 32/30 us), accel/gyro raw int16
  (divide by `lsb_per_g` / `lsb_per_dps`), `temp` raw FIFO int8 (`temp/2.07+25` degC). About 45 bytes per line, about 90 kB/s at 2 kHz.

Host to device (one command per line): `o <hz>` ODR (50, 100, 200, 500, 1000, 2000, 4000, 8000), `a <g>` (2/4/8/16),
`g <dps>` (125/250/500/1000/2000), `i <400|1000>` I2C kHz, `s` status, `p` pause output (FIFO keeps draining), `r` resume.
Changing ODR or range flushes the FIFO (a few samples lost, `seq` continues).

## Register verification

Verified 2026-11-18 against the TDK InvenSense ICM-42688-P datasheet, DS-000347 rev 1.6 (read from a text mirror, not the
PDF itself). Section numbers are the datasheet's. Everything the driver writes or decodes was checked; one mismatch was found
and fixed, one unit correction made:

| Fact | Code | Datasheet | Result |
|---|---|---|---|
| FIFO_CONFIG1 needs FIFO_RESUME_PARTIAL_RD (bit 6) when the FIFO is read in several transactions | `icm42688.cpp` begin(), now 0x4F | 14.45: bit6=0 "requires re-reading of the entire FIFO" | **MISMATCH, fixed** (was 0x0F) |
| FIFO timestamp unit | `main.cpp` TS_TICK_US = 32/30 us | 14.42 TMST_RES=0 is nominally 1 us; 12.7: without CLKIN the FIFO interval reads 937.5 per 1 ms, scale by 32/30 | **MISMATCH, fixed** (was 1.0 us, 6.7 percent rate error) |
| INTF_CONFIG0 0x4C: b6 COUNT_REC=1 records, b5 COUNT_ENDIAN=1 big, b4 DATA_ENDIAN=1 big, value 0x70, b7=0 (invalid-sample code) | begin() | 14.34 | MATCH |
| FIFO_CONFIG 0x16 stream = 01 in bits 7:6 = 0x40 | begin() | 14.4 | MATCH |
| FIFO_CONFIG1 0x5F enables b0 ACCEL, b1 GYRO, b2 TEMP, b3 TMST_FSYNC | begin() | 14.45, 6.4 | MATCH |
| 16-byte packet 3 layout: hdr, accel XYZ, gyro XYZ (big-endian), temp int8 at 0x0D, timestamp at 0x0E-0F | readFifo() | 6.1 packet 3 | MATCH |
| Header: b7 FIFO empty, b6 accel, b5 gyro, b4 20-bit (0 here), b3:2 = 10 timestamp; 16 B header is 0110_10xx | readFifo() masks 0x80, 0x70 | 6.2, 6.4 table | MATCH (b3:2 not asserted) |
| Timestamp needs TMST_CONFIG 0x54 | not written (reset 0x23 has TMST_EN=1, TMST_RES=0, TMST_DELTA_EN=0, so the field is the free-running ODR timestamp) | 14.42 | MATCH by reset default |
| INT_STATUS 0x2D: b4 RESET_DONE, b1 FIFO_FULL, clear on read | begin(), fifoCount() | 14.21 | MATCH |
| FIFO_COUNTH/L 0x2E/0x2F; reading COUNTH latches both; count in records per COUNT_REC | fifoCount() burst 0x2D..0x2F | 14.22, 14.23 | MATCH |
| SIGNAL_PATH_RESET 0x4B bit1 FIFO_FLUSH = 0x02 | flushFifo() | 14.33 | MATCH |
| DEVICE_CONFIG 0x11 bit0 soft reset, wait 1 ms | begin() waits 10 ms | 14.1 | MATCH |
| PWR_MGMT0 0x4E: gyro LN (3:2=11), accel LN (1:0=11) = 0x0F; gyro on >= 45 ms, no register writes for 200 us after OFF->ON | begin() delay 50 ms | 14.36 | MATCH |
| GYRO_CONFIG0 0x4F: FS 2000/1000/500/250/125 = 0..4 in 7:5; ODR 8k=3,4k=4,2k=5,1k=6,200=7,100=8,50=9,500=0xF | odrCode(), gyroFsCode() | 14.37 | MATCH |
| ACCEL_CONFIG0 0x50: FS 16/8/4/2 g = 0..3 in 7:5, same ODR codes | accelFsCode() | 14.38 | MATCH |
| GYRO_ACCEL_CONFIG0 0x52 value 0 = BW ODR/2 for both | begin() | 14.40 | MATCH |
| INT_CONFIG1 0x64 bit4 INT_ASYNC_RESET must be cleared (reset 0x10) | begin() writes 0x00 | 12.6, 14.50 | MATCH (only matters for INT pins) |
| Temperature: 16-bit register C = raw/132.48 + 25; FIFO 8-bit C = raw/2.07 + 25 | viewer uses 2.07 | 4.13, 14.5/14.6 | MATCH |
| Only ODR/FS/mode may change while sensors run; everything else with sensors off | all config before PWR_MGMT0; setOdr/Fs only change ODR/FS | 12.9 | MATCH |
| Invalid samples are -32768 (first ODR, mismatched ODRs) with FIFO_HOLD_LAST_DATA_EN=0 | main.cpp drain() now skips them | 12.8, 14.34 | NEW handling, added |
| FIFO timestamp is 16 bit, absolute (not delta); wraps after 65536 ticks (about 70 ms) | firmware unwraps to 32 bit | 6.1, 14.42 | MATCH; ODR >= about 15 Hz needed for unambiguous unwrap |
| Datasheet-recommended FIFO read procedure (read count, then burst) | fifoCount() then readFifo() | no explicit procedure text found beyond 6.x and 14.22-14.24 | NOT FOUND (behaviour above follows the register descriptions) |
| Wire receive buffer 136 B, 8 packets per burst | `WireIMXRT.h` (Teensyduino source, not datasheet) | n/a | verified in source |
| Teensy 4 `Serial.availableForWrite()` semantics (sums free whole 2 kB tx buffers, excluding the one being filled) | main.cpp drain() | `usb_serial.c` | verified in source: the `>= len` test is conservative and cannot false-drop a healthy link |

Still not verifiable without hardware: that the chip really answers on this Stemma QT board (pull-ups, CS strap), real I2C
throughput and drain-cadence margin at 2 kHz, that Edge/Chrome Web Serial asserts DTR so the Teensy `Serial` is true (the viewer
now sets it explicitly), USB/serial behaviour on unplug, and the 32/30 timestamp factor on this particular unit.

## Bench validation checklist

1. Boot: `#I ICM-42688-P initialised` and a status line with `who:71` (0x47). If not, follow the `#E` hint.
2. |a| about 1.000 g (tile) at rest, on each of the six faces (+/-X, +/-Y, +/-Z up); expect +1.00 on the up axis.
3. Gyro near zero at rest (a constant offset of under about 1 dps is normal; use Calibrate).
4. Rotate the board 90 degrees about X (then Y) by hand: the 3D board and roll/pitch tiles follow and read about 90.
5. Tap the board: sharp spikes on accel plot.
6. Sensor-timestamp rate and host rate both match the ODR; if the sensor rate is off by a factor (16x), set the timestamp tick.
7. Missed seq = 0 and FIFO overflows = 0 for a few minutes at 1 kHz, then at 2 kHz (use I2C 1 MHz if 400 kHz overflows;
   2 kHz is about 80 percent of a 400 kHz bus). Overflow flushes the FIFO and is counted.
8. Noise at rest: per-axis std tiles; datasheet-class noise is a few mg and a few hundredths of dps (the 16 g / 2000 dps
   ranges are coarse, lower the ranges to see it).

## Known limitations

- No magnetometer: yaw drifts. Mahony gains are untuned; accel correction is gated to 0.7-1.3 g.
- Polled FIFO (no INT pin): drain cadence 2 ms (4-5 ms below 500 Hz); a stalled loop longer than about 60 ms at 2 kHz overflows the 2 kB FIFO.
- If the USB host stops reading, whole batches are dropped (`serial_drops`, visible as seq gaps) instead of blocking the loop.
- 8-bit FIFO temperature is coarse; sensor temperature is die temperature.
- Bank-0 only; no offset registers, no anti-alias/notch tuning, no interrupts, no hot re-init other than after repeated I2C errors.
- Simpler robust alternative considered: read the data registers (0x1D-0x2A) at a fixed timer. Rejected as the primary path because
  of beat between poll timer and sensor clock; the FIFO was implemented as requested.
