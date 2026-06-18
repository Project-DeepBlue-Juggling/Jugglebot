# PlatformIO extra script for the catching cone firmware (Jetson host builds).
#
# Single workaround, copied from Teensy_code_canbridge/extra_script.py:
# PlatformIO's bundled `teensy_size` helper is built against glibc 2.34, but
# the Jetson's Ubuntu 20.04 has glibc 2.31 — the tool aborts the build at the
# post-link size-check step even though firmware.elf was already produced.
# Replace it with a thin wrapper around the toolchain's arm-none-eabi-size.
# (No linker-script patch here — the cone firmware has no FreeRTOS/std::thread,
# so the canbridge's .ARM.extab workaround does not apply.)

import os

Import("env")  # noqa: F821 — provided by PlatformIO's SConscript context

BUILD_DIR = env.subst("$BUILD_DIR")  # noqa: F821
HOME = os.path.expanduser("~")


def _install_size_wrapper() -> None:
    wrapper_dir = os.path.join(BUILD_DIR, "size_wrapper")
    os.makedirs(wrapper_dir, exist_ok=True)
    wrapper = os.path.join(wrapper_dir, "teensy_size")
    real_size = os.path.join(
        HOME, ".platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-size",
    )
    if not os.path.isfile(real_size):
        return
    with open(wrapper, "w") as fh:
        fh.write(f"#!/bin/sh\nexec {real_size} \"$@\"\n")
    os.chmod(wrapper, 0o755)
    cur_path = env.get("ENV", {}).get("PATH", "")  # noqa: F821
    new_path = wrapper_dir + os.pathsep + cur_path
    env["ENV"]["PATH"] = new_path  # noqa: F821
    print("[extra_script] installed teensy_size wrapper -> arm-none-eabi-size")


_install_size_wrapper()
