# =============================================================================
# extra_script.py — patch the Teensy linker script to fix .ARM.extab capture
# =============================================================================
# Root cause of the link failure with freertos-teensy:
#
# Teensy 4.1's stock linker script (imxrt1062_t41.ld) places
#   *(.ARM.exidx* .ARM.extab.text* .gnu.linkonce.armexidx.*)
# into an output section that lives in ITCM (LMA=FLASH, VMA=ITCM ~ 0x00000000).
# But libgcc's pr-support.o (pulled in via freertos-teensy → libstdc++ →
# std::thread) emits a bare ".ARM.extab" section *without* the .text suffix.
# The bare .ARM.extab doesn't match the script's pattern and falls through to
# the FLASH catch-all (~0x60000000). The 1.6 GB gap blows out the intra-object
# R_ARM_PREL31 relocation that links .ARM.exidx → .ARM.extab inside pr-support.o.
#
# Fix: patch the section pattern to match bare .ARM.extab too. We copy the
# stock Teensy linker script into the project's .pio/ build dir, swap the
# pattern, and point PlatformIO's LDSCRIPT_PATH at the patched copy. The
# stock file under ~/.platformio/packages/ is untouched, so other Teensy
# projects on this Jetson are unaffected.
#
# The patched output section ends up in ITCM AT> FLASH (same as .ARM.exidx),
# so the PREL31 cross-section reference is now within range.
#
# Hooked into platformio.ini via `extra_scripts = extra_script.py` — NO `pre:`
# prefix, deliberately (platformio.ini:62-67): a `pre:` script runs before the
# platform builder sets LDSCRIPT_PATH and cannot override it. This file therefore
# runs as straight-line code at SConscript load, registering no SCons actions.
# =============================================================================

import hashlib
import os
import shutil

Import("env")  # noqa: F821  -- provided by SCons
env  # type: ignore[name-defined]

PROJECT_DIR = env["PROJECT_DIR"]  # type: ignore[index]
BUILD_DIR = env.subst("$BUILD_DIR")  # type: ignore[attr-defined]
HOME = os.path.expanduser("~")


# -----------------------------------------------------------------------------
# Staleness guard: force a clean build whenever udp_protocol.h changes.
#
# 24608bb (2026-07-31). config/generate_udp_protocol.py rewrites udp_protocol.h
# IN PLACE inside src_dir. After a PROTOCOL_VERSION 4 -> 5 bump the incremental
# `pio run` carried the pre-bump objects forward, and the flashed binary spoke
# protocol v4 while announcing itself as FW 8. Symptom: total CAN darkness in the
# GUI on a perfectly healthy CAN bus — the per-frame version gate rejected every
# frame in both directions. It cost a session to find, because nothing in the
# build, the flash, or the firmware banner said anything was wrong.
#
# That failure was at least loud on the wire once you looked. An ADDITIVE change
# — a new MsgType, a new payload struct — does NOT bump PROTOCOL_VERSION, so the
# identical stale-object mechanism instead produces a binary that emits a SHORT
# payload, which the Jetson's exact-size struct.unpack drops with no log line at
# all; the affected /link_status row just reads 'unknown (never seen)' forever,
# indistinguishable from an old bridge. There is no version gate to catch that
# one. Hence: unconditional guard, not one keyed on the version constant.
#
# Content hash, not mtime. A git checkout or a re-run of the generator that
# produces identical bytes perturbs mtimes without changing the wire format; a
# spurious clean costs ~30 s of rebuild, a missed one costs a silent mis-flash.
#
# The marker lives inside BUILD_DIR so each env (teensy41 and the bench_sysid
# variant that extends it) tracks its own objects independently, and
# `pio run -t clean` resets the guard along with everything else.
#
# WHY THIS WIPES $BUILD_DIR/src AND NOT $BUILD_DIR (measured 2026-08-02, not
# assumed): this script runs as straight-line code at SConscript load, which is
# AFTER PlatformIO's library-dependency finder has materialised the per-library
# build directories and cached their existence in SCons's Dir nodes. An
# `shutil.rmtree(BUILD_DIR)` here therefore kills the run it is trying to protect
# — the src objects compile, then the link stage dies with
# `*** [.pio/build/teensy41/lib1b6/SPI] ... No such file or directory`. A
# `pre:`-prefixed second script would run early enough, but `pre:` is exactly what
# platformio.ini:62-67 forbids (it breaks the LDSCRIPT_PATH override this file
# also performs), so there is no earlier hook available.
#
# Scoping the wipe to $BUILD_DIR/src is not a workaround, it is the correct
# target: `udp_protocol.h` is a project-local header, so the ONLY objects that can
# embed the stale wire format are the project's own. The vendored libraries
# (FrameworkArduino, QNEthernet, freertos-teensy, SPI) cannot include it, and
# their build directories are precisely the ones that must survive. Wiping every
# src object plus the link products forces a full recompile of everything that
# touches the protocol, which is the entire 24608bb failure class.
#
# Object FILES are removed, not the src/ directory itself, for the same
# already-materialised-Dir-node reason.
#
# If a build fails partway AFTER the marker is written, the next run sees a
# matching marker and does NOT wipe. That is correct: every object that exists was
# compiled against the current header, and SCons rebuilds the missing ones from
# its own dependency scan.
#
# Must run BEFORE _install_size_wrapper(), which writes into BUILD_DIR.
# -----------------------------------------------------------------------------
_MARKER_NAME = ".udp_protocol_h_sha256"


def _force_clean_if_udp_protocol_changed() -> None:
    header = os.path.join(PROJECT_DIR, "udp_protocol.h")
    if not os.path.isfile(header):
        print(f"[extra_script] WARNING: {header!r} not found; stale-object guard "
              "SKIPPED (run config/generate_udp_protocol.py).")
        return
    with open(header, "rb") as fh:
        digest = hashlib.sha256(fh.read()).hexdigest()

    marker = os.path.join(BUILD_DIR, _MARKER_NAME)
    previous = None
    if os.path.isfile(marker):
        try:
            with open(marker, "r") as fh:
                previous = fh.read().strip()
        except OSError:
            previous = None

    src_obj_dir = os.path.join(BUILD_DIR, "src")
    if previous != digest and os.path.isdir(src_obj_dir):
        was = ("NO MARKER (first guarded build / hand-cleaned tree)"
               if previous is None else previous[:12])
        print(f"[extra_script] *** udp_protocol.h CHANGED ({was} -> {digest[:12]}) "
              f"— wiping {src_obj_dir} + link products to force a full recompile; "
              "stale objects flashed a protocol-v4 binary as FW8 on 2026-07-31 "
              "(24608bb, total CAN darkness on a healthy bus) ***")
        for root, _dirs, files in os.walk(src_obj_dir):
            for name in files:
                if name.endswith(".o"):
                    os.remove(os.path.join(root, name))
        # Drop the link products too, so a stale firmware.hex can never be picked
        # up by a flash step that runs after a compile step failed.
        for art in ("firmware.elf", "firmware.hex", "firmware.bin"):
            path = os.path.join(BUILD_DIR, art)
            if os.path.isfile(path):
                os.remove(path)

    os.makedirs(BUILD_DIR, exist_ok=True)
    with open(marker, "w") as fh:
        fh.write(digest + "\n")


_force_clean_if_udp_protocol_changed()

# At extra-script load time, LDSCRIPT_PATH may not yet be populated by the platform
# builder. Hardcode the well-known Teensy 4.1 path; this script is
# Teensy-4.1-specific so the hardcode is unambiguous. If you ever build for
# another Teensy variant, point this at the matching .ld file.
ORIG = os.path.join(
    HOME, ".platformio/packages/framework-arduinoteensy/cores/teensy4/imxrt1062_t41.ld"
)

# -----------------------------------------------------------------------------
# Workaround: PlatformIO's bundled `teensy_size` helper is built against
# glibc 2.34, but the Jetson's Ubuntu 20.04 has glibc 2.31. The tool aborts
# the build at the post-link size-check step, even though firmware.elf has
# already been produced. Replace it with a thin wrapper around the toolchain's
# arm-none-eabi-size (which lives in the gcc-arm-none-eabi package shipped
# alongside this firmware's toolchain).
# -----------------------------------------------------------------------------
def _install_size_wrapper() -> None:
    wrapper_dir = os.path.join(BUILD_DIR, "size_wrapper")
    os.makedirs(wrapper_dir, exist_ok=True)
    wrapper = os.path.join(wrapper_dir, "teensy_size")
    real_size = os.path.join(
        HOME, ".platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-size",
    )
    if not os.path.isfile(real_size):
        return
    # Tiny shell shim that just forwards args to arm-none-eabi-size.
    with open(wrapper, "w") as fh:
        fh.write(f"#!/bin/sh\nexec {real_size} \"$@\"\n")
    os.chmod(wrapper, 0o755)
    # Prepend to PATH so PlatformIO's post-link size step picks this up.
    cur_path = env.get("ENV", {}).get("PATH", "")  # type: ignore[attr-defined]
    new_path = wrapper_dir + os.pathsep + cur_path
    env["ENV"]["PATH"] = new_path  # type: ignore[index]
    print(f"[extra_script] installed teensy_size wrapper -> arm-none-eabi-size")


_install_size_wrapper()

if not os.path.isfile(ORIG):
    print(f"[extra_script] WARNING: stock linker script not found at {ORIG!r}; not patching.")
else:
    # Write the patched script as a SAME-NAMED file in a directory that we
    # prepend to LIBPATH, so the linker finds OURS before the framework's.
    # PlatformIO computes the -T arg from LDSCRIPT_PATH early (basename only)
    # and refuses to honour later env.Replace(LDSCRIPT_PATH=...) overrides,
    # so we instead control which `imxrt1062_t41.ld` the -L search resolves.
    patched_dir = os.path.join(PROJECT_DIR, ".pio", "patched_ld")
    os.makedirs(patched_dir, exist_ok=True)
    patched = os.path.join(patched_dir, "imxrt1062_t41.ld")
    with open(ORIG, "r") as fh:
        text = fh.read()

    # The single line we need to broaden:
    #   *(.ARM.exidx* .ARM.extab.text* .gnu.linkonce.armexidx.*)
    needle = "*(.ARM.exidx* .ARM.extab.text* .gnu.linkonce.armexidx.*)"
    replacement = "*(.ARM.exidx* .ARM.extab* .gnu.linkonce.armexidx.*)"
    if needle not in text:
        print(f"[extra_script] WARNING: expected pattern not in {ORIG}; linker script unchanged.")
    else:
        text = text.replace(needle, replacement, 1)
        with open(patched, "w") as fh:
            fh.write(text)
        # Prepend our directory to LIBPATH so `-L<patched_dir>` precedes the
        # framework-cores -L. The linker's -T <basename> search then finds
        # our patched imxrt1062_t41.ld first.
        env.Prepend(LIBPATH=[patched_dir])  # type: ignore[attr-defined]
        print(f"[extra_script] wrote patched linker script: {patched}")
        print(f"[extra_script] prepended LIBPATH so -T imxrt1062_t41.ld resolves to the patched copy")
        print(f"[extra_script]   broadened .ARM.exidx output to also capture bare .ARM.extab")
