"""Firmware build-reproducibility guard.

The can-bridge firmware links two external libraries (freertos-teensy, QNEthernet)
on top of the PlatformIO ``teensy`` platform. If any of those float — a bare git
URL that tracks HEAD, a bare library name that tracks "latest", or an unpinned
``platform =`` — then the NEXT ``pio run`` can pull a different toolchain / library
and change the flashed binary with NO local edit, silently invalidating a powered
re-validation. This test pins them: the platform + every ``lib_deps`` entry must
carry an explicit version / commit ref.

The requirement being enforced: firmware library versions must be pinned. Pure
text-parse of platformio.ini — no PlatformIO, no compiler, always runs.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

_FW_DIR = (Path(__file__).resolve().parents[2]
           / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge")
_PIO_INI = _FW_DIR / "platformio.ini"
_EXTRA_SCRIPT = _FW_DIR / "extra_script.py"


@pytest.fixture(scope="module")
def env_teensy41() -> str:
    """The [env:teensy41] section body (the flashed firmware's build config)."""
    text = _PIO_INI.read_text()
    m = re.search(r"\[env:teensy41\]\n(.*?)(?=\n\[|\Z)", text, re.DOTALL)
    assert m, "no [env:teensy41] section in platformio.ini"
    return m.group(1)


def test_platform_is_version_pinned(env_teensy41):
    """`platform = teensy` must carry an explicit @version — an unpinned platform
    floats the whole ARM toolchain on the next resolve."""
    m = re.search(r"^\s*platform\s*=\s*(\S+)", env_teensy41, re.MULTILINE)
    assert m, "no `platform =` line in [env:teensy41]"
    platform = m.group(1)
    assert "@" in platform, (
        f"platform is unpinned ({platform!r}); pin it as `teensy@<version>` so the "
        "toolchain can't drift under a validated flash.")


def _lib_deps(section: str) -> list:
    """The non-blank, non-comment entries under `lib_deps = ...` (a multi-line list)."""
    m = re.search(r"^\s*lib_deps\s*=\s*\n((?:\s+\S.*\n?)+)", section, re.MULTILINE)
    assert m, "no multi-line `lib_deps =` block in [env:teensy41]"
    out = []
    for line in m.group(1).splitlines():
        s = line.strip()
        if s and not s.startswith(";"):
            out.append(s)
    return out


def test_every_lib_dep_is_pinned(env_teensy41):
    """Every lib_deps entry pins a version: a git URL must carry a #ref (commit/tag),
    a registry entry (owner/lib) must carry @version. A bare URL/name tracks HEAD /
    latest → non-reproducible builds."""
    deps = _lib_deps(env_teensy41)
    assert deps, "lib_deps is empty — expected freertos-teensy + QNEthernet"
    for dep in deps:
        if dep.endswith(".git") or (dep.startswith("http") and "#" not in dep):
            pytest.fail(f"git lib_dep is unpinned (no #<commit/tag>): {dep!r}")
        if dep.startswith("http"):
            assert "#" in dep, f"git lib_dep must pin a #ref: {dep!r}"
        else:
            assert "@" in dep, f"registry lib_dep must pin @<version>: {dep!r}"


def test_known_libs_present_and_pinned(env_teensy41):
    """The two libraries the firmware actually links are present AND pinned to the
    versions the current flash was built against (freertos-teensy commit 7e6ae6a /
    QNEthernet 0.35.0 / platform teensy@5.1.0). Bumping any of these is a deliberate
    act that must be paired with a re-flash + powered re-validation — update this
    test in the same change so the intent is recorded."""
    deps = _lib_deps(env_teensy41)
    joined = "\n".join(deps)
    assert "freertos-teensy" in joined and "#7e6ae6a" in joined, (
        "freertos-teensy must be pinned to commit 7e6ae6a…")
    assert "QNEthernet@0.35.0" in joined, "QNEthernet must be pinned to @0.35.0"
    assert re.search(r"platform\s*=\s*teensy@5\.1\.0", env_teensy41), (
        "platform must be pinned to teensy@5.1.0")


# ── Stale-object guard (24608bb class) ───────────────────────────────────────

def test_extra_script_forces_clean_when_udp_protocol_h_changes():
    """extra_script.py must wipe BUILD_DIR when udp_protocol.h content changes.

    Same family of risk as the version pins above, from the other direction: those
    stop the toolchain drifting under a validated flash, this stops a validated
    SOURCE tree producing a binary built from pre-edit objects.

    On 2026-07-31 (24608bb) the codegen rewrote udp_protocol.h in place after a
    PROTOCOL_VERSION 4 -> 5 bump; the incremental ``pio run`` carried the old
    objects forward and the flashed binary spoke v4 while announcing FW 8. The
    result was total CAN darkness on a healthy bus, and it was silent — nothing in
    the build or the banner flagged it. For an ADDITIVE protocol change (new
    MsgType, new payload struct) there is not even a version gate to make it loud:
    the bridge emits a short payload, the Jetson's exact-size struct.unpack drops
    it without a log line, and the row reads 'unknown (never seen)' forever.

    Text assertion rather than an executed build: driving extra_script.py needs a
    live SCons ``env``, and this test must run on every commit, on any box, with no
    PlatformIO installed. It pins the three things whose removal reinstates the
    failure — the content marker, the hash, and the object wipe — so a future
    simplification that quietly drops the guard trips here instead of at the next
    flash.

    The wipe is scoped to ``$BUILD_DIR/src`` rather than all of ``$BUILD_DIR``, and
    that scope is deliberate on both counts (measured 2026-08-02): an
    ``rmtree($BUILD_DIR)`` from this script kills the very run it is protecting,
    because the script loads after PlatformIO has already materialised the
    per-library build dirs; and ``udp_protocol.h`` is a project-local header, so
    the project's own objects are the only ones that can carry a stale wire format
    in the first place.
    """
    src = _EXTRA_SCRIPT.read_text()
    assert ".udp_protocol_h_sha256" in src, (
        "extra_script.py must record a CONTENT hash of udp_protocol.h in BUILD_DIR "
        "(mtime is not enough — a checkout perturbs mtimes without changing bytes)")
    assert "hashlib.sha256" in src, "the marker must be a sha256 of the header content"
    assert 'os.path.join(BUILD_DIR, "src")' in src, (
        "the guard must target the PROJECT object dir — those are the only objects "
        "that can embed a stale udp_protocol.h")
    assert 'name.endswith(".o")' in src and "os.remove(" in src, (
        "a changed udp_protocol.h must DELETE the project objects; nothing else "
        "invalidates them, because the codegen rewrites the header in place inside "
        "src_dir and the incremental build happily carries the old .o files forward")
    assert "firmware.elf" in src, (
        "the link products must go too, so a stale firmware.hex cannot be flashed "
        "after a failed recompile")
