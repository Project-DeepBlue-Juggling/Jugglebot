# =============================================================================
# extra_script.py — Jetson-host build workarounds for the PLATFORM Teensy sketch
# =============================================================================
# Two independent workarounds, both copied from the sibling sketches:
#
# 1. teensy_size shim (same as CatchingCone_code/extra_script.py).
#    PlatformIO's bundled `teensy_size` helper is built against glibc 2.34, but
#    the Jetson's Ubuntu 20.04 has glibc 2.31 — the tool aborts the build at the
#    post-link size-check step even though firmware.elf was already produced.
#    Replace it with a thin wrapper around the toolchain's arm-none-eabi-size.
#
# 2. Linker-script patch (same as Teensy_code_canbridge/extra_script.py, retargeted
#    from the Teensy 4.1 script to the Teensy 4.0 one).
#    Teensy 4.0's stock linker script (imxrt1062.ld) places
#        *(.ARM.exidx* .ARM.extab.text* .gnu.linkonce.armexidx.*)
#    into an output section that lives in ITCM (LMA=FLASH, VMA~0x00000000). But
#    libgcc's pr-support.o emits a BARE ".ARM.extab" section without the .text
#    suffix, which does not match that pattern and falls through to the FLASH
#    catch-all (~0x60000000). The ~1.6 GB gap blows out the intra-object
#    R_ARM_PREL31 relocation that links .ARM.exidx -> .ARM.extab inside
#    pr-support.o, and the link fails with "relocation truncated to fit".
#
#    The can-bridge hits this via freertos-teensy -> std::thread. THIS sketch hits
#    it via std::vector: `packedMsgs`/`sendUs` growth references
#    std::__throw_length_error, which pulls libstdc++'s functexcept.o and with it
#    the exception machinery. Verified on 2026-07-27 by nm-ing the built objects:
#    Teensy_code.ino.cpp.o's only unwinder-adjacent undefined symbol is
#    _ZSt20__throw_length_errorPKc, and the -fno-exceptions family in
#    platformio.ini is NOT sufficient on its own (the reference comes from the
#    already-compiled libstdc++, not from our translation unit).
#
#    Fix: copy the stock script into .pio/patched_ld/ with the pattern broadened
#    to match bare .ARM.extab, and prepend that directory to LIBPATH so the
#    linker's `-T imxrt1062.ld` search resolves to our copy. The stock file under
#    ~/.platformio is never modified, so other Teensy projects on this Jetson are
#    unaffected.
#
# NB: this project has NO upload_command on purpose — see platformio.ini's header.
# The build products here are a compile gate, not a flash artefact.
# =============================================================================

import os

Import("env")  # noqa: F821 — provided by PlatformIO's SConscript context

PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
BUILD_DIR = env.subst("$BUILD_DIR")  # noqa: F821
HOME = os.path.expanduser("~")

# Teensy 4.0 linker script (the 4.1 board uses imxrt1062_t41.ld — this project is
# board = teensy40, so the name is unambiguous). If this sketch is ever built for
# another Teensy variant, retarget this constant.
LD_NAME = "imxrt1062.ld"
ORIG = os.path.join(
    HOME, ".platformio/packages/framework-arduinoteensy/cores/teensy4", LD_NAME
)


def _install_size_wrapper() -> None:
    wrapper_dir = os.path.join(BUILD_DIR, "size_wrapper")
    os.makedirs(wrapper_dir, exist_ok=True)
    wrapper = os.path.join(wrapper_dir, "teensy_size")
    real_size = os.path.join(
        HOME,
        ".platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-size",
    )
    if not os.path.isfile(real_size):
        return
    with open(wrapper, "w") as fh:
        fh.write('#!/bin/sh\nexec %s "$@"\n' % real_size)
    os.chmod(wrapper, 0o755)
    cur_path = env.get("ENV", {}).get("PATH", "")  # noqa: F821
    env["ENV"]["PATH"] = wrapper_dir + os.pathsep + cur_path  # noqa: F821
    print("[extra_script] installed teensy_size wrapper -> arm-none-eabi-size")


def _patch_linker_script() -> None:
    needle = "*(.ARM.exidx* .ARM.extab.text* .gnu.linkonce.armexidx.*)"
    replacement = "*(.ARM.exidx* .ARM.extab* .gnu.linkonce.armexidx.*)"
    if not os.path.isfile(ORIG):
        print("[extra_script] WARNING: stock linker script not found at %r; "
              "not patching (the link will fail on R_ARM_PREL31)." % ORIG)
        return
    with open(ORIG, "r") as fh:
        text = fh.read()
    if needle not in text:
        print("[extra_script] WARNING: expected .ARM.exidx pattern not in %s; "
              "linker script unchanged." % ORIG)
        return
    patched_dir = os.path.join(PROJECT_DIR, ".pio", "patched_ld")
    os.makedirs(patched_dir, exist_ok=True)
    patched = os.path.join(patched_dir, LD_NAME)
    with open(patched, "w") as fh:
        fh.write(text.replace(needle, replacement, 1))
    # PlatformIO computes the -T arg from LDSCRIPT_PATH early (basename only) and
    # refuses later env.Replace() overrides, so control which imxrt1062.ld the -L
    # search resolves instead.
    env.Prepend(LIBPATH=[patched_dir])  # noqa: F821
    print("[extra_script] wrote patched linker script: %s" % patched)
    print("[extra_script]   broadened .ARM.exidx output to also capture bare "
          ".ARM.extab")


_install_size_wrapper()
_patch_linker_script()
