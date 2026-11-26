---
id: BUG-147
kind: bug
title: Ball Butler flasher takes the first reply frame for BEGIN and COMMIT without checking its opcode
status: inbox
created: '2026-11-26'
updated: '2026-11-26T03:18:42Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: ball-butler
blocked_by: []
links: []
filed_by: claude
---

`tools/teensy_link_bridge.py`, the Ball Butler flashing tool, waits for the answers to BEGIN and COMMIT with `waiter.wait(reply_id)` and does not check the opcode of the frame it receives (`_send_begin` near line 641, COMMIT near 741). Found 2026-11-26 while choosing a carrier for the Ball Butler status frame (BUG-133).

## Finding

- Any unsolicited frame on that reply id arriving during those two steps would be read as the answer.
- Nothing sends such a frame today: the FW 10 status frame was put on the command-result channel for this reason.

## Wanted

- Filter both waits on the expected opcode and ignore other frames until the timeout.
