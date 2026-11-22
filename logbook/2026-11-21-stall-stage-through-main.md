---
title: "hand_stream_bench --stage stall runs through main(): the _MAX_DEV_DEFAULT KeyError is gone, and a frame shift whose reply is lost is still restored (BUG-123)"
type: bugfix
date: 2026-11-21
status: complete
phase: BUG-123
related_plan: ""
related_issues: []
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/hand_stall_stage.py
  - tests/motion/test_hand_stream_bench_stall.py
  - logbook/2026-11-21-stall-stage-through-main.md
  - logbook/INDEX.md
subsystem: [tooling, hand]
tags: [bench, hand, stall, restore]
---

## What and why

At the 2026-11-20 bench Sitting, `python tests/hardware/hand_stream_bench.py --stage stall` crashed in `main()` with `KeyError: 'stall'` at `args.max_dev = _MAX_DEV_DEFAULT[args.stage]`. That was before the link opened, so nothing was commanded. The stage had only been tested through `run_stall` against a fake IO, never through `main()`. The stall stage has no deviation belt, so the stage now skips the belt default, and an explicit `--max-dev` with `--stage stall` is refused.

Walking the rest of the `--stage stall` path found one defect that a fake run hides. `_body` set `st["shift"] = 0.0` only after `SET_ABSOLUTE_POSITION` returned, and `_finish` reads 0.0 as "never shifted". So a lost reply (an `RpcTimeout` after the shift reached the drive) or a Ctrl-C between the shift and its measurement left the encoder frame shifted by +0.65 rev, with no restore and no warning. The shift is now marked `"sent"` before the call, the same idiom as `lowered`. `_finish` then measures it as `pos - p_before`, which is valid because the Hand is not energised before that measurement, and restores it. If the shift cannot be measured, `_finish` reports `frame_ok = False` and says "re-home".

## Verification

- 2026-11-21, `python -m pytest -q -p no:cacheprovider tests/motion/test_hand_stream_bench_stall.py tests/teensy_link/test_hand_stream_bench_gap.py tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stroke_cut.py` (under the test lock): **122 passed in 4.59 s**.
- Five new tests run the real `main(['--stage','stall',...])` against fakes for the link, RPC, caches and ODrive USB: the PASS path through to the verdict line, the `--max-dev` refusal, and an exception injected after the frame shift (a lost `SET_ABSOLUTE_POSITION` reply, a `RuntimeError` at `SET_CONTROLLER_MODE`, and a Ctrl-C at `SET_POS_GAIN`), each asserting that the frame and the 70 A limit are restored. All five failed on the branch head with `KeyError('stall')`. With only the KeyError fixed, the lost-reply case still failed (one `SET_ABSOLUTE_POSITION`, no restore).
- Not run against hardware.
