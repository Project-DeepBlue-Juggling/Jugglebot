---
id: BUG-86
kind: bug
title: rosbridge service pool blames the executor when the last ready poll loses the race to its deadline
status: complete
created: '2026-11-08'
updated: '2026-11-07T18:47:02Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-08-rosbridge-pool-timeout-names-the-server.md
filed_by: claude
---

The nightly of 2026-11-08 went RED on one test: `test_pool_times_out_without_sending_when_the_server_never_appears` got `the rosbridge executor thread never ran the send` where it expects `the server never became visible`.

## Cause

Read from the code, not reproduced. `ServiceClientPool.call` polls every 20 ms. It checks `now + 0.02 < deadline`, sleeps 20 ms, then asks the executor to run the check again with whatever is left of the deadline. On a loaded box that can be nothing, so `_run_on_executor_and_wait` gives up and the error says the executor never ran, though it ran on every earlier poll.

## Effect

A `TimeoutError` is raised either way, nothing is sent, and the caller is told the same failure. Only the text is wrong, and on a real timeout it points at the wrong thing.

## Wanted

Once the executor has run the check at least once, a timeout means the server never became visible. A test that makes the last wait lose the race, with no wall-clock dependence.
