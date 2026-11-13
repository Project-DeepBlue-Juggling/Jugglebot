"""Atomic text-file writes for Items under ``map/``.

The map app re-indexes ``map/`` on every change and several writers (Sessions,
the Night Shift, the app) share it, so ``open(path, "w")`` (truncate, then
write) lets a reader see an empty or half-written Item, and a crash mid-write
destroys it. :func:`write_text_atomic` writes a temp file in the same
directory, fsyncs it and ``os.replace``-s it over the target: a reader sees the
old file or the new one, never a mix. A watcher sees the change as a rename
onto the path (moved-to), not as a modify of it.

The temp name is ``.<basename>.<random>.tmp``: a dotfile that never ends in
``.md``, so no ``map/`` loader (they all take ``name.endswith(".md")``) can
pick it up as an Item.
"""
from __future__ import annotations

import os
import stat
import tempfile


def _new_file_mode() -> int:
    """The mode ``open(path, "w")`` would create: 0666 less the umask, read from
    /proc so the process-global umask is never changed (threads may be writing)."""
    try:
        with open("/proc/self/status", encoding="ascii") as fh:
            for line in fh:
                if line.startswith("Umask:"):
                    return 0o666 & ~int(line.split()[1], 8)
    except (OSError, ValueError, IndexError):
        pass
    return 0o644


def write_text_atomic(path: str, text: str, newline: str = "\n") -> None:
    """Replace ``path`` with ``text`` (utf-8) atomically.

    ``newline`` is passed to the text layer as in ``open()`` (``"\\n"``: write
    ``\\n`` as is; ``""``: no translation either, bytes as given). The existing
    file's permission bits are kept; a new file gets the umask default rather
    than mkstemp's 0600. On any failure the temp file is removed and the
    original is untouched.
    """
    directory = os.path.dirname(os.path.abspath(path))
    try:
        mode = stat.S_IMODE(os.stat(path).st_mode)
    except FileNotFoundError:
        mode = _new_file_mode()
    fd, tmp = tempfile.mkstemp(dir=directory, prefix="." + os.path.basename(path) + ".",
                               suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline=newline) as fh:
            os.fchmod(fh.fileno(), mode)
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        raise
