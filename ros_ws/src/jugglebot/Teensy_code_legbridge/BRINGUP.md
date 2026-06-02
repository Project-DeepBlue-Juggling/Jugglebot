# Leg-bridge Teensy 4.1 — bench bring-up walkthrough

End-to-end steps to get the leg-bridge firmware from a fresh clone to a
running Teensy with link lights blinking and `ping 192.168.42.2` responding.
Designed for the Jetson Orin Nano + Ubuntu 20.04 + JetPack stack this project
runs on. Each step has a verification command and a "what should happen"
note.

> **Status as of 2026-06-02:** the build is green to `firmware.hex` on this
> Jetson. Flashing requires installing `teensy-loader-cli` once (Step 2). The
> firmware has NOT yet been validated on hardware — see
> [`HANDOFF-teensy-can-offload-firmware-wip.md`](../../../../plans/active/HANDOFF-teensy-can-offload-firmware-wip.md)
> §"Needs hardware validation" for what to verify on the bench after first flash.

---

## Step 0 — Hardware checklist

Before flashing, confirm the following are present on the bench. Items marked
**REQUIRED FOR FIRST BOOT** must be wired before flashing or the firmware
will assert/hang at startup.

| Item | Status needed | Notes |
|------|---------------|-------|
| Teensy 4.1 + USB-B cable to Jetson | **REQUIRED FOR FIRST BOOT** | Confirm via `lsusb \| grep 16c0`. You should see `16c0:0486` (existing firmware) or `16c0:0478` (bootloader mode). |
| PJRC Ethernet kit soldered to Teensy 4.1 Ethernet pads | **REQUIRED FOR FIRST BOOT** | Without this, the firmware's `Ethernet.begin()` will time out and the boot LED will fast-blink (fatal path). |
| USB-Ethernet adapter on Jetson + cable to Teensy magjack | **REQUIRED** for ping verification | Per [ADR-0007](../../../../docs/adr/0007-point-to-point-static-link.md). Jetson side already configured (Step 1). |
| CAN1 transceiver (TJA1051T/3) on pins 22 (TX) / 23 (RX) | Optional for bring-up; required for time-sync master | LED-blink + ping work without it. If wired to the shared CAN1, the Teensy will start broadcasting 0x7DD at 100 Hz. |
| CAN2 transceiver (TJA1051T/3) on pins 1 (TX) / 0 (RX) | Optional for bring-up; required for leg-ODrive commands | Same — LED-blink + ping work without it. |
| 120 Ω termination on each CAN bus end | Required if either bus is wired | Standard CAN good practice. |

If the Ethernet kit isn't yet soldered, you can still validate Steps 1-3 (build
to HEX), but skip the flash until the kit is in place.

---

## Step 1 — Jetson network setup (one-time)

Already done on this Jetson. The configuration that's in place:

```bash
# Verify (no changes needed if these match):
nmcli connection show teensy-link | grep -E "ipv4\.addresses|ipv4\.method|802-3-ethernet\.mac"
# Expected:
#   802-3-ethernet.mac-address:  6C:1F:F7:C6:E4:6B
#   ipv4.method:                  manual
#   ipv4.addresses:               192.168.42.1/30

ip -c addr show dev eth1   # adapter interface name on this Jetson
# Expected: NO-CARRIER + state DOWN if no Teensy plugged in yet, or UP + 192.168.42.1/30 if linked.
```

If the connection doesn't exist, see [ADR-0011](../../../../docs/adr/0011-mac-pinned-nmcli.md)
and the parent plan's *Jetson network setup* section to recreate it.

---

## Step 2 — Install Teensy Loader CLI (one-time, requires sudo)

PlatformIO's bundled `teensy_post_compile` is built against glibc 2.34 and
won't run on Ubuntu 20.04's glibc 2.31. Install Ubuntu's `teensy-loader-cli`
package instead — `platformio.ini` is already configured to use it:

```bash
sudo apt update
sudo apt install -y teensy-loader-cli

# Verify:
teensy-loader-cli --help 2>&1 | head -3
# Expected: usage banner mentioning --mcu and --hex options
```

While we're here, set up a udev rule so non-root users can talk to the
Teensy's HID interface (no `sudo` needed on every flash):

```bash
sudo tee /etc/udev/rules.d/00-teensy.rules > /dev/null <<'EOF'
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[7-9]?", ENV{MTP_NO_PROBE}="1", MODE="0666"
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Source for this rule: the canonical [PJRC udev file](https://www.pjrc.com/teensy/00-teensy.rules).

---

## Step 3 — Activate the Python venv + regenerate generated headers

```bash
cd ~/Desktop/Jugglebot
source ~/Desktop/PDJ_venv/venv/bin/activate

# Regenerate everything codegen-driven (idempotent — safe to re-run):
python config/generate_config.py
python config/generate_udp_protocol.py
```

The first script emits `protocol_config.h` + `hardware_config.h` into the
firmware tree; the second emits `udp_protocol.h`. Both are needed before the
firmware will compile cleanly.

---

## Step 4 — Build the firmware

```bash
cd ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_legbridge
pio run
```

What you should see at the end:

```
[extra_script] installed teensy_size wrapper -> arm-none-eabi-size
[extra_script] wrote patched linker script: ... .pio/patched_ld/imxrt1062_t41.ld
[extra_script] prepended LIBPATH so -T imxrt1062_t41.ld resolves to the patched copy
[extra_script]   broadened .ARM.exidx output to also capture bare .ARM.extab
...
Linking .pio/build/teensy41/firmware.elf
Calculating size .pio/build/teensy41/firmware.elf
Checking size .pio/build/teensy41/firmware.elf
   text     data      bss      dec      hex   filename
 205120    34496    91392   331008    50d00   .pio/build/teensy41/firmware.elf
Building .pio/build/teensy41/firmware.hex
========================= [SUCCESS] Took XXs =========================
```

If the build fails:

- **`No such file ... protocol_config.h`** — re-run Step 3's codegen.
- **`R_ARM_PREL31 relocation truncated`** — `extra_script.py` didn't run.
  Check `platformio.ini` has `extra_scripts = extra_script.py` under
  `[env:teensy41]` (NOT under `[platformio]`).
- **`pio: command not found`** — venv not activated; re-run Step 3's `source`.

---

## Step 5 — Flash the firmware

```bash
pio run -t upload
```

On first flash, `teensy_loader_cli` may say:

```
Waiting for Teensy device...
 (hint: press the button on the Teensy to enter bootloader mode)
```

**Press the program button on the Teensy 4.1 (small white button next to the
USB connector).** The loader will then detect the bootloader (VID 16c0:0478),
write the HEX, and reboot the Teensy into your new firmware.

**Verification** — within a few seconds of flash completion:

```bash
# The new firmware uses USB Serial mode; should appear at /dev/ttyACM0
ls -la /dev/ttyACM*
# Expected: /dev/ttyACM0

# Teensy MUST blink the on-board LED at 1 Hz (50% duty). If it doesn't,
# the FreeRTOS scheduler didn't start — see "If something goes wrong" below.

lsusb | grep 16c0
# Expected: 16c0:0483 Teensyduino Serial  (not 0486 RawHID — confirms new firmware is running)
```

---

## Step 6 — Verify the dedicated Ethernet link

With the USB-Ethernet adapter cable to the Teensy connected:

```bash
# Link should now negotiate — adapter blinks, Jetson interface goes UP:
ip -c addr show dev eth1
# Expected: state UP, 192.168.42.1/30

# Ping the Teensy:
ping -c 5 192.168.42.2
# Expected: 5/5 responses, time < ~2 ms

# Latency/jitter baseline (Phase 2 done-when):
ping -i 0.01 -c 1000 192.168.42.2 | tail -3
# Expected: 0% loss; min/avg/max/mdev with max < ~2 ms
```

If `ip addr` still shows `<NO-CARRIER>` after the flash completes:

- Confirm the Ethernet cable is plugged in at both ends.
- Confirm the PJRC Ethernet kit's 6-pin ribbon is correctly oriented on the
  Teensy 4.1 Ethernet pads (the keying on the kit is fragile; reverse
  orientation is a common mistake).
- Open the serial monitor (`pio device monitor`) and look for QNEthernet
  diagnostic output during boot — it logs which PHY it sees.

---

## Step 7 — Open the serial monitor

```bash
pio device monitor
# Baud 115200 (the platformio.ini default)
# Exit with Ctrl-C, then Ctrl-A then K, OR Ctrl-T then X (depends on terminal)
```

What you should see streaming at ~1 Hz:

- FreeRTOS task scheduling: a `diag_task` line every second confirming the
  scheduler is alive.
- Heartbeat lines from the time-sync master if CAN1 is wired.
- Once the Jetson UDP responder for `TIME_OF_DAY_QUERY` is implemented and
  the link is up, you should see the wall-clock anchor settle.

---

## What good looks like at end of bring-up

A successful Phase 2 + Phase 3 bring-up has all of these true at the same
time:

- ✅ `lsusb | grep 16c0` shows `16c0:0483 Teensyduino Serial`
- ✅ `/dev/ttyACM0` exists and `pio device monitor` shows clean FreeRTOS task
  scheduling
- ✅ On-board LED blinks at 1 Hz (50% duty)
- ✅ USB-Ethernet adapter link lights blink on the Jetson side
- ✅ `ip -c addr show dev eth1` shows `state UP` and `192.168.42.1/30`
- ✅ `ping -c 1000 -i 0.01 192.168.42.2` shows 0% loss, max latency < 2 ms
- ✅ `ip -c route` still shows house-LAN default via `eth0` (the Teensy
  link doesn't steal the default route)

That puts the firmware at the end of plan Phase 2 and ready for Phase 3
(UDP echo + framing layer validation).

---

## If something goes wrong

| Symptom | Likely cause | First-line fix |
|---------|--------------|----------------|
| `teensy_loader_cli` says "Waiting for Teensy..." forever after pressing the button | Bootloader entry didn't happen | Try a different USB cable (some are charge-only). Tap the button briefly, not held. |
| LED stays solid OR doesn't blink at 1 Hz | FreeRTOS scheduler didn't start, or fast-blink fatal path is engaged | Open `pio device monitor` — fatal-path firmware writes the failure reason to USB CDC. Check `configTOTAL_HEAP_SIZE` if it's a heap problem. |
| `ping 192.168.42.2` times out, link lights ON | PHY up but firmware not bound | Check serial console for QNEthernet errors. Confirm the `ipv4.method manual` connection is up on the Jetson side. |
| `ping` times out, link lights OFF | Cable problem or PHY not powered | Check the PJRC Ethernet kit ribbon orientation. Verify 3.3 V at the kit's power pin. |
| `pio run -t upload` fails with `teensy_post_compile` OR `teensy_reboot: GLIBC_2.3x not found` | PlatformIO is using one of its bundled glibc-2.34 binaries | `platformio.ini` should have `upload_command = /usr/bin/teensy_loader_cli --mcu=TEENSY41 -w -v $SOURCE` (NOT `upload_protocol = teensy-cli` — that still invokes the broken `teensy_reboot`). |
| `pio run -t upload` fails with `Cannot find teensy_loader_cli` | apt package not installed | Re-run Step 2's `sudo apt install -y teensy-loader-cli`. |

For deeper issues, the firmware
[`README.md`](README.md) lists the per-module hardware-validation
expectations, and the
[`HANDOFF-teensy-can-offload-firmware-wip.md`](../../../../plans/active/HANDOFF-teensy-can-offload-firmware-wip.md)
§"Needs hardware validation" enumerates every behaviour the bench should
confirm before this firmware is trusted.

---

## What comes after Phase 2 (this walkthrough's scope)

This walkthrough takes you through plan Phases 2 (FreeRTOS skeleton +
Ethernet bring-up). The migration plan's subsequent phases — UDP echo,
protocol contract validation, CAN bus bring-up, ODrive protocol, fault state
machine replay, Jetson-side bridge, full cutover — are each documented in
[`teensy-can-offload.md`](../../../../plans/active/teensy-can-offload.md).
Work them in order; each phase has its own "Done when" criteria.

The single most important next step after Phase 2 is **Phase 5 CAN bench
validation**: wire one leg ODrive to CAN2, drive it IDLE → CLOSED_LOOP →
position commands → IDLE, and confirm telemetry decodes correctly. That
single-axis test catches the largest class of ODrive-protocol port bugs.
