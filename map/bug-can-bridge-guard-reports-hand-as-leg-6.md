---
kind: bug
title: can-bridge Guard reports the Hand as "leg 6"
status: inbox
created: '2026-09-21'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/18
filed_by: harrison
---

`CONTEXT.md` fixes the vocabulary: a **Leg** is one of the six Stewart-platform actuators, indexed 0 to 5; the seventh axis is the **Hand** (wire and firmware index: axis 6). The Hand is never a Leg.

The can-bridge Guard's operator-facing messages still name the Hand "leg 6". At the 2026-09-13 sitting two Hand-axis `MAX_DEVIATION` latches were reported as "leg 6" and read as Leg faults, which sends the diagnosis to the wrong actuator.

## Wanted

Every operator-facing string for axis index 6 (firmware fault text, `teensy_bridge_node` log lines, GUI event log) says "hand", not "leg 6". Purely a naming change: no wire-format change and no PROTOCOL_VERSION bump. If the text originates in firmware it needs a flash, so bundle it with the next can-bridge firmware unit rather than flashing for this alone.
