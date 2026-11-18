---
title: "mapkit atomic write works where os.fchmod does not exist (BUG-118)"
type: bugfix
date: 2026-11-18
status: complete
phase: "maintainer review"
subsystem: tooling
files_changed:
  - mapkit/atomic.py
  - tests/sim/test_mapkit_items.py
---

# mapkit atomic write works where os.fchmod does not exist (BUG-118)

## What

`mapkit/atomic.py` `write_text_atomic` copied the target's permission bits onto its temp file with `os.fchmod`, which Windows does not have. Every map write from the owner's Windows box therefore raised `AttributeError` (found 2026-11-18 on an IDEA-20 progress note). The helper was introduced on 2026-11-14 (`2026-11-14-maintainer-review-fixes`) and only ever ran on Linux before that.

## Fix

`os.fchmod` is used when the platform has it (unchanged on the Jetson). Where it is missing the mode is set with `os.chmod` on the temp path after the file is closed and before the `os.replace`. The temp file is created 0600 by `mkstemp`, so the later chmod never exposes content more widely than the target's mode.

## Verification

- Readers of the changed module (`grep -rn "atomic" mapkit tools tests --include="*.py"`, 2026-11-18): `mapkit/items.py` and `mapkit/ids.py` import it; `tests/sim/test_mapkit_items.py` tests it directly. The other hits are unrelated helpers of the same name in `tools/`.
- 2026-11-18, `python -m pytest tests/sim/test_mapkit_items.py tests/sim/test_mapkit_ids.py -q`: 64 passed. The new case `test_write_text_atomic_works_without_os_fchmod` removes `os.fchmod` and checks that the write lands, the mode bits are kept and no temp file is left.
- Not run on Windows itself: the test simulates the missing attribute on Linux. The first map write from the Windows box is the real check.
