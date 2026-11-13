---
id: IDEA-24
kind: idea
title: Move dated history out of code and normative documents into the logbook
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:57Z'
owners: []
priority: medium
area: docs
blocked_by: []
links: []
filed_by: claude
---

Move dated history out of code and normative documents into the logbook, where it already has a home. From the 2026-11-13 maintainer review. The token-budget rule in `CLAUDE.md` already says a plan past about 400 lines has become a changelog; the same has happened to these files, and every Session that opens them pays for it.

## Measured

| File | Size | Note |
| --- | --- | --- |
| `Teensy_code_canbridge/canbridge_config.h` | line 55 is one 34,056-character comment | The firmware version history from 1 to 43 |
| `motion/skills/INVARIANTS.md` | 157 KB in 212 lines | Table cells up to 8,670 characters |
| `motion/skills/executor.py` | about 2,650 docstring and 1,480 comment lines of 8,016 | 220 dated references; one docstring is 90 lines |
| `tools/probes/README.md` | 164 KB | 81 commits in three months |
| `config/hardware_config.yaml` | 204 of 2,089 lines read as history | |
| `config/generate_udp_protocol.py` | about 5 KB comment on the protocol version line | |
| `CLAUDE.md` | 44 KB, loaded by every Session | The architecture section is mostly a record of deletions |
| `plans/active/two-ball-oval.md`, `learner-generalisation.md` | 604 and 510 lines | Past the plan rule's own 400 |

## Wanted

A rule and one pass per file: a comment or a contract row states the invariant and its reason in the present tense and names the logbook entry that holds the history. The history itself is cut, not rewritten; `git log` and the logbook keep it.

## Risks

`INVARIANTS.md` and `CLAUDE.md` are normative and frozen for the Night Shift, so this is day work with an audit.
