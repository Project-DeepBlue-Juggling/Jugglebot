---
id: BUG-118
kind: bug
title: mapkit atomic write uses os.fchmod, which Windows lacks, so map writes fail on the Windows box
status: complete
created: '2026-11-18'
updated: '2026-11-18T11:37:18Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: map-app
blocked_by: []
links: []
parent: map-improvements
filed_by: claude
---

The map writes go through one atomic-write helper that copies the file mode onto its temp file with `os.fchmod`, which does not exist on Windows. Found 2026-11-18 when the owner's Windows session could not write the IDEA-20 progress note.

## Where

- `mapkit/atomic.py:53`: `os.fchmod(fh.fileno(), mode)` inside the temp-file write, the only `os.fchmod` call in `mapkit/` (checked with grep on 2026-11-18).

## Suggested fix (not implemented)

- Guard the call with `hasattr(os, "fchmod")`, or switch to `os.chmod(tmp, mode)` on the temp path before the `os.replace`.
- Add a test that monkeypatches away `os.fchmod` and confirms a write still lands.
