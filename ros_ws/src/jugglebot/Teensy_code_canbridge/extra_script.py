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
# Hooked into platformio.ini via `extra_scripts = pre:extra_script.py`.
# =============================================================================

import os
import shutil

Import("env")  # noqa: F821  -- provided by SCons
env  # type: ignore[name-defined]

PROJECT_DIR = env["PROJECT_DIR"]  # type: ignore[index]
BUILD_DIR = env.subst("$BUILD_DIR")  # type: ignore[attr-defined]
HOME = os.path.expanduser("~")

# At pre: hook time, LDSCRIPT_PATH may not yet be populated by the platform
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
