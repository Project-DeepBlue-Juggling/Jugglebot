---
title: "Cone \"only syncs after a colcon build\" — zombie + build-correlation hypotheses both REFUTED by bag ground truth; a real (but latent, never-fired) UDP co-bind hazard found; morning failures now point at cone MCU restarts"
type: investigation
date: 2026-07-12
status: resolved-with-open-items
phase: GUI + catching cone
related_entries:
  - 2026-07-12-gui-cone-catch-correlation-regression
  - 2026-07-06-phase13-socketcan-decommission
  - 2026-05-23-throw-director-and-cone-live-integration
files_changed: []            # NO code change landed by this investigation
commits:
  - pending-backfill         # placeholder — backfill the SHA of the entry+probe commit
subsystem:
  - ros
  - can
  - gui
tags:
  - IPC
  - observability
  - safety
---

# Cone "only syncs after a colcon build" — refuted; a real UDP co-bind hazard found, but the cone MCU is what keeps restarting

> **Read this first.** This entry contains two independent findings that must not
> be conflated:
> 1. **A real, latent co-bind hazard** in `TeensyLinkClient` (HIGH confidence,
>    read off the code) — two processes can silently own the can-bridge UDP link.
>    **It has never been observed to fire.**
> 2. **The morning failures were NOT caused by it** (the timing artifacts refute
>    that outright — Step 5). The leading explanation is **cone MCU restart /
>    power-wiring intermittency**, which remains **open** (Open Item (c)).
>
> The first draft of this entry ran (1) and (2) together and got (2) wrong. See
> *Withdrawn claims*.

## Summary

Follow-up to [[2026-07-12-gui-cone-catch-correlation-regression]]. After that
fix landed, the operator reported a new symptom: *"the cone syncs perfectly
immediately after a colcon build, but subsequent launches don't sync — zombie
ROS2 processes?"* **Both halves of that model are refuted by the data.** There
were no zombies (live process forensics found nothing but the systemd GUI daemon
and the `ros2` CLI daemon), and decoding `/cone/heartbeat` out of **every** bag
from the day (**17 sessions**) shows **14 of the 15 sessions from 15:57 onward
were `connected=True` and `time_synced=True` from their first published
heartbeat** — including the session immediately *before* the 19:17 colcon rebuild,
the one immediately *after*, the next one, and three further launches 45 minutes
to 3 hours later with no build in between. There is no first-launch-only signature
anywhere in the ground truth. (The single exception, `20-03-36`, is *not* a
build-correlation failure either: it is a mid-session cone reboot that heals
itself 44 s in, with no rebuild — see Step 5.)

The only genuinely broken sessions were the **two morning ones**
(10-37-19, 10-42-51), where `cone/heartbeat.connected` was `False` for the
**entire** session (0/882 and 0/1606 samples) — i.e. `teensy_bridge_node` saw
**no cone frames at all**, which is a *transport* failure, not a CAN2 time-sync
failure.

Looking at those two sessions surfaced a **real structural hazard**:
`teensy_bridge_node` and the parallel bench system-ID harness each construct a
`TeensyLinkClient` that binds the **same** can-bridge UDP ports (`0.0.0.0:5005`
stream, `:5006` RPC) with `SO_REUSEADDR`. The second bind succeeds silently, and
the kernel then delivers the **entire** inbound stream to exactly one of the two
sockets — so if the bench harness wins, every cone datagram is **silently
dropped** (it does not subscribe `CONE_FRAME`) and a healthy cone presents as a
disconnected one. **That hazard is real (HIGH confidence — it is read straight
off the code) and the operational mitigation below stands.**

**But it is NOT what broke the morning sessions.** This entry's first draft
attributed them to that co-bind, on the strength of bench runs "bracketing" the
dead sessions. The timing artifacts refute it: across all **17** bags and all
**22** `bench_sysid_*` runs of the day there is **not one overlapping (bench,
ROS-session) pair** — the bench runs are *adjacent* to the dead sessions, never
*concurrent* with them, and both morning runs are 3-second aborted runs that
could not have starved a 163-second session even if they had overlapped. The
mechanism requires two sockets bound **at the same time**; that never happened.
The **leading explanation is now cone-side power/wiring intermittency**: the same
`connected=False` signature recurs in three sessions with **no bench process
running at all** (`18-34-09`, `18-44-56`, `20-03-36` — each a dropout ending in a
**cone MCU reboot**), and it appears in the `cone_test_*` bags of **2026-05-24**,
six weeks *before* the bench harness existed. **No code change landed**: no defect
reproduced in the reported path, and the interlock that would close the (real,
latent) co-bind hazard lives in shared transport the parallel gain-tuning
workstream is actively depending on — a coordinate-first change, not a drive-by.

## Symptoms

Operator report, evolving over the day:

1. *(Prior entry, already fixed)* Cone catches never displayed in the GUI —
   root-caused to the `catch_correlation_node` launch orphan (commit `eedcbed`)
   plus a **stale installed launch copy** that needed a `colcon build`, not just
   a relaunch. Documented in
   [[2026-07-12-gui-cone-catch-correlation-regression]].
2. *(This entry)* **"Cone syncs perfectly immediately after a colcon build, but
   subsequent launches don't sync. Zombie ROS2 processes?"** — the operator's
   read of the GUI cone panel showing *Disconnected* / not-time-synced on
   launches that were not preceded by a build.

## Diagnosis

### Step 1 — the zombie hypothesis: refuted (live process forensics)

Live process forensics found **no** leftover `teensy_bridge_node`, no orphaned
`rosbridge`, no stranded `ros2 launch`, no lingering bench process. The only
long-lived things on the box were the **systemd GUI daemon**
(`jugglebot-gui.service`, which serves static files and holds no ROS/UDP state)
and the **`ros2` CLI daemon** (a discovery cache, not a node). There is no
process that could be holding the cone link across launches in the way the
hypothesis requires.

### Step 2 — the build-correlation: refuted by topic ground truth

Rather than argue from process lists, the decisive move was to stop trusting
*any* live-observation channel and decode the recorded topic itself. A new probe
(`tools/probes/cone_bag_decode.py`) walks every bag in `~/Desktop/rosbags/`,
decodes `/cone/heartbeat`, and reports, per session, the fraction of heartbeat
samples with `connected=True` and `time_synced=True` plus the state of the
**first** published heartbeat.

Per-session results for **all 17 bags of the day** (the four `20-0x` / `22-17`
sessions postdate the first decode run and were added on re-decode — they are the
strongest test of the build-correlation, being launches *hours* after the 19:17
rebuild, and they are where the cone-intermittency evidence turned up):

| Session (bag) | heartbeats | `connected=True` | `time_synced=True` | sync@1st | verdict |
|---|---|---|---|---|---|
| `2026-07-12_10-37-19` | 882 | **0 / 882** | 0 / 882 | N | **DEAD — no cone frames at all** |
| `2026-07-12_10-42-51` | 1606 | **0 / 1606** | 0 / 1606 | N | **DEAD — no cone frames at all** |
| `2026-07-12_15-57-59` | 2843 | 2843 / 2843 | 2843 / 2843 | **Y** | synced from first HB |
| `2026-07-12_16-06-23` | 280 | 280 / 280 | 280 / 280 | **Y** | synced from first HB |
| `2026-07-12_16-08-47` | 1095 | 1095 / 1095 | 1095 / 1095 | **Y** | synced from first HB |
| `2026-07-12_18-30-05` | 2106 | 2106 / 2106 | 2106 / 2106 | **Y** | synced from first HB |
| `2026-07-12_18-34-09` | 1010 | **830 / 1010** | 1009 / 1010 | **Y** | synced from first HB, then **DROPPED OUT at t+32.3 s and came back at t+50.3 s with `last_catch_seq=0`, `have_any_catch=False` → cone MCU REBOOT** |
| `2026-07-12_18-37-27` | 2793 | 2793 / 2793 | 2793 / 2793 | **Y** | synced from first HB (the prior entry's live-confirm session) |
| `2026-07-12_18-42-22` | 1400 | 1400 / 1400 | 1400 / 1400 | **Y** | synced from first HB |
| `2026-07-12_18-44-56` | 613 | **470 / 613** | 612 / 613 | **Y** | synced from first HB, then **DROPPED OUT at t+12.4 s, back at t+26.7 s with seq reset → cone MCU REBOOT** |
| `2026-07-12_19-16-18` | 424 | 424 / 424 | 424 / 424 | **Y** | **synced from first HB — the session immediately BEFORE the 19:17 colcon rebuild** |
| `2026-07-12_19-17-31` | 870 | 870 / 870 | 870 / 870 | **Y** | **synced from first HB — immediately AFTER the rebuild** |
| `2026-07-12_19-19-12` | 515 | 515 / 515 | 515 / 515 | **Y** | **synced from first HB — the NEXT launch after that, i.e. the one the hypothesis predicts should FAIL** |
| `2026-07-12_20-02-47` | 350 | 350 / 350 | 350 / 350 | **Y** | synced from first HB — 45 min after the rebuild |
| `2026-07-12_20-03-36` | 593 | **155 / 593** | 154 / 593 | **N** | **PARTIAL — cone already down at launch, came up at t+43.9 s with seq reset → cone MCU REBOOT. No bench process within ~54 min.** |
| `2026-07-12_20-04-49` | 836 | 836 / 836 | 836 / 836 | **Y** | synced from first HB (cone counter restarted at 0 — corroborates the 20:03 reboot) |
| `2026-07-12_22-17-12` | 845 | 845 / 845 | 845 / 845 | **Y** | synced from first HB — 3 hours after the rebuild |

**The `19-16-18` / `19-17-31` / `19-19-12` triple is the killer evidence.** A
colcon build boundary sits between the first and second of those, and the three
sessions are *indistinguishable* in the data: all three synced on their very
first heartbeat. If a build were what makes the cone sync, the pre-build session
would look different from the post-build one, and the *second* post-build launch
would degrade. Neither happens. **There is no first-launch-only signature in the
ground truth**, and the build-correlation is dead.

The four late sessions extend the same test further: `20-02-47`, `20-04-49` and
`22-17-12` are launches **45 minutes to 3 hours** after the rebuild, with no build
in between, and all three sync from their first heartbeat. The one exception —
`20-03-36` — fails in a way the build-correlation cannot explain either (see Step
5: it is a *mid-session cone reboot*, and it recovers **without** a rebuild, 44 s
in).

### Step 3 — what actually was broken: the two morning sessions

The only sessions that ever failed are `10-37-19` and `10-42-51`. Their
signature is not "cone connected but never time-synced on CAN2" — it is
`connected=False` for **100 %** of heartbeats (0/882 and 0/1606). `connected` in
`cone/heartbeat` reflects whether `teensy_bridge_node` is *receiving cone frames
at all*. So the failure is upstream of time-sync entirely: **the bridge never
saw a single cone frame** in either session.

### Step 4 — a real structural hazard (which did *not* fire that day)

Looking at the transport around those two dead sessions surfaced a genuine
structural hazard in the code. It is real, it is latent, and it is **not** what
broke the morning sessions — Step 5 handles the attribution. Read this section
as *"here is a loaded gun we found while looking"*, not *"here is the murder
weapon"*.

- `controller/teensy_link/client.py` — `TeensyLinkClient.start()` creates the
  stream and RPC sockets and binds them with `SO_REUSEADDR` set
  (`client.py:174-175` for the stream socket, `client.py:179-180` for the RPC
  socket). The defaults are the wildcard address on the can-bridge ports:
  **`0.0.0.0:5005`** (stream) and **`0.0.0.0:5006`** (RPC).
- **`teensy_bridge_node`** constructs a `TeensyLinkClient` — this is how the
  whole ROS stack talks to the can-bridge (legs, hand, cone, BB).
- The **bench system-ID harness** (the parallel gain-tuning workstream's driver)
  constructs its own `TeensyLinkClient` — same class, same defaults, same ports.

With **two** wildcard `SO_REUSEADDR` UDP sockets bound to the same port, the
kernel does **not** fan out: each inbound datagram is delivered to **exactly
one** socket, and the winner is stable for the lifetime of the binds. And the
bench harness **does not subscribe `CONE_FRAME`** — it wants leg telemetry — so
if the bench socket is the winner, **every** cone datagram is **silently
discarded**. From `teensy_bridge_node`'s point of view the cone has simply gone
quiet: no frames → no heartbeat from the device → it publishes
`connected=False`, `time_synced=False`. **A perfectly healthy cone would present
as a disconnected cone, and nothing anywhere would log a word about it.**

**Confidence: HIGH that this hazard is real.** It is read straight off the code —
two `TeensyLinkClient` constructions, identical default binds, `SO_REUSEADDR`, one
non-subscribing consumer. **Nothing about that requires the morning incident to be
true**, and the operational mitigation (Open Item (a)) is worth following on the
strength of the code alone.

### Step 5 — what actually broke the morning sessions (attribution)

The first draft of this entry attributed the two dead sessions to the Step-4
co-bind, on the strength of bench runs "bracketing" them. **The timing artifacts
refute that, and the refutation is not close.**

**(1) Adjacency, not overlap.** The bench manifests carry their own `created` /
`finished` stamps; the bags carry `message_start_time` / `message_end_time`. Laid
side by side:

| | window |
|---|---|
| bench run `bench_sysid_20260712_103657` | 10:36:57 → **10:37:00** (3 s) |
| dead session `10-37-19` | **10:37:23** → 10:38:52 (89 s) — starts **23 s after that bench run had already exited** |
| dead session `10-42-51` | 10:42:55 → **10:45:38** (163 s) |
| bench run `bench_sysid_20260712_104600` | **10:46:00** → 10:46:03 (3 s) — starts **22 s after that session had already ended** |

The co-bind mechanism requires two sockets bound **at the same time**. These
windows do not touch. Widening the check to the whole day: **across all 17 bags
and all 22 `bench_sysid_*` runs of 2026-07-12 there is not one overlapping
(bench, ROS-session) pair.** The co-bind event this entry blamed **never occurred
that day**.

**(2) Even a perfect overlap could not have done it.** Both morning bench runs are
**3 seconds long** and are manifest-only aborts (`stages: {}` and
`{"pos_steps": {}}`; no CSVs were written — they connected, measured the 100 Hz
telemetry rate, and exited). A 3-second link theft cannot starve the bridge for
the **89 s** and **163 s** of *total* starvation the bags record. The magnitudes
are off by two orders of magnitude.

**(3) The same signature occurs with no bench process anywhere near it.** Three
sessions show the cone dropping off the bridge and coming back:

- `18-34-09` — connected for 32.3 s, **dropout**, back at t+50.3 s
- `18-44-56` — connected for 12.4 s, **dropout**, back at t+26.7 s
- `20-03-36` — **already down at launch**, comes up 43.9 s in

In every case the cone returns with **`last_catch_seq = 0` and
`have_any_catch = False`** — it did not merely resume, it **rebooted**. (The next
session's counter confirms it independently: `20-02-47` ends at seq 3, and
`20-04-49` starts fresh with no catch history and takes its first catch as seq 1.)
The nearest bench run to `20-03-36` **finished at 19:09:30 — 54 minutes earlier**;
the next one started at 22:28. There was no second binder within an hour.

**(4) The shape is wrong for a port race.** A co-bind is **all-or-nothing per
bind**: the kernel picks one socket and that choice holds for the life of the
binds (Discussion → *The real hazard*). It cannot produce connectivity that flips
**mid-session** — but `18-34-09`, `18-44-56` and `20-03-36` all flip mid-session,
and `20-03-36` *heals itself* 44 s in with no process starting or stopping. Those
are device-side events.

**(5) The signature is older than the hazard.** The `cone_test_*` bags of
**2026-05-24** contain **three** sessions with `connected = 0/N` for their entire
duration (0/755, 0/982, 0/525) plus one partial (703/5549) — the exact morning
signature. The bench system-ID harness was first committed on **2026-07-11**
(`f218acf`), and `TeensyLinkClient` itself on **2026-06-02** (`2b605af`). **The
failure predates the second binder by six weeks and the transport class by nine
days.** It cannot be diagnostic of a co-bind.

**Confidence, restated:**

- **LOW — candidate only, NOT supported by the timing artifacts** — that the
  co-bind caused the 10:37 and 10:42 failures. It is retained only as a
  mechanism that *could* produce this signature, not as an attribution. It would
  need direct evidence of a `TeensyLinkClient` process alive during
  10:37:23–10:38:52 **and** 10:42:55–10:45:38 to be revived, and no such artifact
  exists.
- **LEADING — cone MCU restart / power-wiring intermittency** (Open Item (c)).
  Three confirmed reboots on 2026-07-12, a self-healing partial-connectivity
  session with no bench process within an hour, and a documented history of
  identical full-session dropouts back in May. The morning sessions are the same
  family, only longer: a cone that is *down* — not a cone whose datagrams are
  being stolen.

**Still unexplained:** why the morning outage lasted **at least 9 minutes**
(10:37:23 through 10:45:38, spanning both bags *and* the 4-minute gap between
them), where the evening dropouts were 14–44 s. A single sustained cone
power/connector fault explains both bags with one event and needs no second
process — but the *duration* is not yet accounted for, and no bag covers the gap.

## Discussion

### The hypothesis-refutation arc (the spine of this entry)

The operator arrived with two linked hypotheses — *zombie processes* and
*only a build fixes it* — and **both were refuted**, one by process forensics
and one by ground truth. The instructive part is not that they were wrong; it is
*how* they were killed. The temptation in a "sometimes it works, sometimes it
doesn't" report is to reason about *mechanisms* (what could a stale process
hold? what does a build change?) and to sample the live system until the story
fits. That path is a trap on this box specifically, because the live-observation
channel is itself unreliable (`ros2 topic echo` false-negatives on high-rate
RELIABLE topics here — [[memory reference_ros2_topic_echo_flaky_foxy]]), so any
"I looked and it wasn't there" is weak evidence.

What killed both hypotheses in one pass was **decoding the recorded topic out of
every bag from the day and putting the sessions in a table**. The bags are
write-once ground truth; they were recorded by the same stack, at the time, with
no observer in the loop. And once tabulated, the answer was not subtle: fourteen
of the day's fifteen post-15:57 sessions synced from their *first* heartbeat, with
a colcon build sitting right in the middle of the sequence and leaving **no
trace** (and the fifteenth failed in a way the build hypothesis cannot explain
either — it recovered mid-session, without a rebuild). The
`19-16-18` (pre-build) / `19-17-31` (post-build) / `19-19-12` (next launch)
triple is a controlled experiment the operator ran without meaning to, and it
falsifies the build-correlation outright — the "subsequent launch" that the
hypothesis says should fail is right there in the data, synced.

Generalisable lesson: **when a report is about a correlation ("X only works
after Y"), go find the boundary event in recorded data and check whether the
signature changes across it.** Don't argue about the mechanism of Y. A build
boundary with an identical signature on both sides ends the discussion in one
step, and it took less time than the process forensics did.

### Why the operator's model was reasonable, and where it came from

This is *not* an operator error, and it would be a bad lesson to file it as one.
The build-correlation was **a correct lesson, learned from real evidence, in an
adjacent domain — and then over-generalised.**

Finding (1) — the `catch_correlation_node` launch orphan
([[2026-07-12-gui-cone-catch-correlation-regression]]) — **genuinely did require
a `colcon build`**. `ros2 launch` runs the **installed** copy of the launch file,
not the source tree copy, so the restored node did not enter the roster on a
relaunch alone; it only appeared after a rebuild. The operator watched cone
catches start appearing in the GUI *immediately after a build and not before*,
and drew exactly the right inference for that problem. The memory note
`feedback_ros_ws_changes_need_colcon_guidance` exists precisely because this is
a real and recurring property of `ros_ws/`.

The failure mode is what happened next: **"a build fixes cone problems"** got
carried across from *launch-graph membership* (where it is true, and is a
statement about install-tree staleness) to *device time-sync* (where it is
meaningless — the cone's CAN2 sync has nothing to do with what Python is on
disk). A true rule in one domain became a false rule in the adjacent one, and
because the domains share the word "cone", the transfer felt free.

The defence against this class is cheap and worth internalising: **when reusing a
prior lesson, restate the mechanism, not the correlation.** "A rebuild is needed
because `ros2 launch` runs the installed copy" transfers safely and immediately
tells you it can't explain time-sync. "A rebuild fixes the cone" does not
transfer, and there is no way to notice that from the sentence itself. (This is
the same shape as the CLAUDE.md rule about justifying by root cause rather than
appeal to authority — a compressed rule with the *why* stripped out is a rule
that will eventually be applied where it doesn't hold.)

### Adjacency is not concurrency — the attribution that did not survive

This entry's first draft made, on its own evidence, **the same mistake it had just
spent two sections refuting in the operator's model**: it took a correlation for a
mechanism. Having refuted "a build fixes the cone", it turned around and asserted
"bench runs bracketed the dead sessions, therefore the bench harness starved
them" — and the word doing the work, *bracketed*, is precisely the word that hides
the problem. **A bracket is an adjacency claim. The mechanism needs a concurrency
claim.** Two sockets have to be bound *at the same time*; a bench run that
finished 23 seconds before the session started shares no instant with it.

The draft never checked. The manifests carry `created` **and** `finished`; the
bags carry `message_start_time` **and** `message_end_time`. Four numbers, one
comparison — and it takes about a minute to write the loop over all 22 × 17 pairs
(zero overlaps). The reason it went unchecked is worth naming, because it is the
generalisable part: **the hypothesis was load-bearing for a story the author
liked.** The co-bind is a genuinely elegant finding — a silent, class-wide,
contract-shaped bug of exactly the sort this codebase rewards discovering. Once
the morning sessions were "explained" by it, the entry had a satisfying arc:
refute the operator, find the real bug, propose the interlock. Checking the
timestamps could only have spoiled that, and so it didn't happen.

Three cheap defences, in increasing order of what they would have cost:

1. **Timestamps are two numbers, not one.** "Around 10:37" is not a window. Any
   claim of the form *"X was running during Y"* must be written as two intervals
   and an intersection, or it is not a claim about concurrency at all.
2. **Ask what the mechanism predicts that alternatives don't** — then go look.
   The co-bind is all-or-nothing per bind; a device fault comes and goes. The bags
   already contained the discriminator (`18-34-09`'s 830/1010, `20-03-36`'s
   155/593); nobody had asked them the question, because the question only occurs
   to you if you take the alternative seriously.
3. **Check whether the symptom predates the suspect.** The dead-cone signature is
   in the `cone_test_*` bags of 2026-05-24 — six weeks before the bench harness
   was written. One `git log --diff-filter=A` would have killed the attribution
   outright, and it is the single cheapest test that was available.

What survives is the part that was never leaning on the incident: **the hazard is
real because the code says so**, and that claim was always independent of whether
it fired that morning. The entry now says exactly that — hazard (HIGH, from code),
attribution (LOW, refuted by artifacts), leading explanation (cone-side
intermittency) — and the reader can see which is which. This one was caught by the
pre-commit `/audit` pass rather than by the author, which is the strongest argument
in this repo's history for keeping that gate on narrative documents: the entry was
*hours* from being committed with a confident, wrong, operationally-misleading
attribution in it.

### The real hazard: two owners of one physical link

The investigation's actual value is not the refutation — it is what fell out of
looking at the morning failures. There is exactly **one** physical link from the
Jetson to the can-bridge, and the codebase currently lets **two processes claim
it simultaneously without either of them noticing**. This is true whether or not
it has ever fired.

Both `teensy_bridge_node` and the bench system-ID harness construct a
`TeensyLinkClient`, which in `start()` binds `0.0.0.0:5005` (stream) and
`0.0.0.0:5006` (RPC) with `SO_REUSEADDR`. `SO_REUSEADDR` on a UDP wildcard bind
means the second `bind()` **succeeds silently** — there is no error, no warning,
no log line.

What happens next is worth being precise about, because the precise version is
load-bearing. There is **no fan-out and no `SO_REUSEPORT` load-balancing**: the
kernel resolves each inbound unicast datagram to a **single** socket, so **one
process receives the entire stream and the other receives nothing**. Which one
wins is decided **at bind time** and holds for the lifetime of the binds. It is
**all-or-nothing**, not a per-datagram coin flip.

That distinction is the difference between two very different bag signatures, and
it is how Step 5 tells this mechanism apart from the one that actually fired.
All-or-nothing is exactly what a `connected = 0/N`-for-the-whole-session bag looks
like — total starvation, never a partial. A per-datagram split would instead show
**partial** connectivity, the bridge still winning roughly half the cone
heartbeats. So when `20-03-36` comes back with **155/593** — connected for part of
a session and not the rest — that is *not* what a port race looks like. It is a
device coming and going. (Getting this wrong in the first draft is what let the
co-bind attribution survive as long as it did: a "random split" story is loose
enough to accommodate almost any bag, which is precisely what should have made it
suspect.)

The consequence, when it *does* fire, is worse than degraded throughput, because
the two consumers want *different* frames. The bench harness subscribes leg
telemetry; it does **not** subscribe `CONE_FRAME`. If the bench socket is the
winner, every cone datagram is **dropped on the floor without a trace** — the
bench harness has no reason to log it, and the bridge never learns the frame
existed. `teensy_bridge_node` sees the cone stop talking and dutifully publishes
`connected=False`, `time_synced=False`. **A perfectly healthy cone presents as a
disconnected cone**, and the only visible artifact is a GUI panel saying the wrong
thing.

Deliberately described at the **transport** level (`TeensyLinkClient`), not by
pinning line numbers inside the bench harness — that file is being actively
edited by the parallel gain-tuning session, and a line reference would rot within
the hour. The role ("the bench system-ID harness") is the durable identifier; the
mechanism lives in shared transport regardless of which harness happens to be the
second binder. **Any** future second `TeensyLinkClient` — a probe, a one-off
script, a second node — inherits this exact failure. The cone is just the loudest
victim because it is the frame type the other consumer happens not to want.

### Why no code landed

Two independent reasons, and both would have been sufficient on their own.

**First: nothing was reproduced in the reported path.** The symptom under
investigation ("cone doesn't sync on subsequent launches") is *not real* — the
bags say so, fourteen healthy sessions running. Landing a fix for a defect the
ground truth refutes is how you get code that exists to satisfy a mental model
rather than a failure mode. And the one *real* failure found (the morning
sessions) is now attributed — at LOW confidence, and against the co-bind — to a
**cone-side hardware** cause, which no change to `TeensyLinkClient` would have
prevented. That is worth sitting with: had the interlock been landed as a "fix"
for the morning incident, it would have shipped, the morning failure would have
stayed possible, and the entry would have claimed a fix it never made.

**Second: the correct fix is in shared transport that a parallel workstream is
actively depending on right now.** `controller/teensy_link/client.py` is the
common path for `teensy_bridge_node` **and** the bench harness the gain-tuning
session is running today. Changing `TeensyLinkClient.start()` to refuse a second
bind would, by construction, change the behaviour of a harness someone else is
mid-session with — and would do it from a session they didn't know was running.
That is a **coordinate-first change, not a drive-by**. The cost of waiting is one
sentence of operational discipline (below); the cost of not waiting is breaking
someone else's live bench run to fix a bug that didn't reproduce.

**The proposed fix, concretely:** a **single-owner advisory lock** in
`TeensyLinkClient.start()` — e.g. an `flock` on a well-known lockfile taken
*before* the sockets are bound, with the holder's PID/argv recorded — so that a
second binder **fails loudly, naming the current owner** ("can-bridge link
already owned by PID 12345 `teensy_bridge_node`"), instead of silently stealing
half the frames. Note what this does *and does not* target: it does **not** patch
cone-sync, and it does **not** special-case `CONE_FRAME`. It closes the entire
**two-owners-of-one-link** class at the single point where ownership is
established, which is the only place the invariant ("exactly one process owns the
can-bridge UDP link") can be enforced. Cone frames were the symptom that made the
class visible; a leg-telemetry probe launched next to the bridge would have
produced a different symptom from the identical cause. This is the
contract-over-patch move from CLAUDE.md: one normative invariant, one enforcement
point, one test — not a cone-specific band-aid.

### The GUI-display caveat

There remains a class of moment where the bags say `time_synced=True` but the
operator honestly saw *Disconnected* on screen, and it is worth naming so it is
not mistaken for a recurrence.

The GUI's cone panel is driven by `cone/heartbeat` over rosbridge. When the
browser (re)connects — a tab left open across a ROS session, a page load, a
websocket reconnect — the panel has **no heartbeat sample yet** and renders its
disconnected state until the first one lands. At ~1 Hz heartbeats that is a
visible **flash of "Disconnected"** that resolves on its own within a second. If
the tab is stale in a deeper way (the long-lived-browser state-latching class
this project has now hit three times in a week — see
[[2026-07-12-gui-bb-stale-calibrated]]), it can persist until refreshed.

**The cure is a tab refresh; the authoritative check is decoding the bag**, not
reading the panel and not `ros2 topic echo` — which false-negatives on high-rate
RELIABLE topics on this Foxy box ([[memory reference_ros2_topic_echo_flaky_foxy]])
and would have "confirmed" the operator's hypothesis if it had been trusted.
That is precisely why this investigation went to the bags first, and it is the
standing recommendation for the next cone report: **decode, don't echo.**

## Fix

**None. No code change landed in this investigation.** This is deliberate and is
argued in Discussion → *Why no code landed*: the reported defect does not exist
in the ground truth, and the one real hazard found has its fix in shared
transport that a parallel workstream is actively using. The proposed fix (a
single-owner advisory lock in `TeensyLinkClient.start()`) is carried as Open Item
(a), to be landed **in coordination with** the bench/gain-tuning workstream.

The only artifacts of this investigation are:

1. `tools/probes/cone_bag_decode.py` — the bag-decode probe, promoted to
   `tools/probes/` because it is a reusable harness (any future "was the cone
   actually connected/synced in session X?" question is one command away, and
   this is the *authoritative* answer channel on a box where `ros2 topic echo`
   lies). The pre-commit `/audit` pass hardened it in three ways, each of which
   was a real defect in the tool this investigation was leaning on:
   - **A cone reboot is no longer bridged.** The tap accumulator resumed straight
     across a `last_catch_seq` reset, which would have invented ~250 phantom taps
     had any catch followed either of the two in-session reboots. Resets are now
     detected (`have_any_catch` clearing, or an implausible >64-tap step between
     consecutive heartbeats), the accumulator restarts, and the row is marked `*`.
     Consequence for the record: **the `taps == catch_ev` 17/17 agreement validates
     the cone→ROS transport only *conditionally* — because no catch happened to
     follow a reboot. It is not a general property of the reconstruction**, and the
     probe docstring now says so.
   - **A genuine zero no longer reads as "topic absent".** `catch_event_count` was
     collapsing a real `0` into `None`, and `None` means *absent* — which
     `taps_mismatch()` skips. A session where the cone counted N taps and the ROS
     graph published **zero** `catch_event`s would therefore have been **silently
     omitted** from the "catch frames lost" summary. That is precisely the
     `catch_correlation_node` orphan of
     [[2026-07-12-gui-cone-catch-correlation-regression]] — the probe would have
     been blind to the very regression its sibling entry exists to document. Zero
     and absent are now distinguished from the bag's channel list (mcap omits
     zero-count channels from `channel_message_counts`, so this bit the *main*
     decode path too, not just the summary-less fallback).
   - **A missing `--root` now errors cleanly** instead of raising a
     `FileNotFoundError` traceback — which is the no-args path on the non-ROS2
     clone the probe advertises support for.
2. This logbook entry.

Interim, the mitigation is **operational** (Open Item (a)).

## Verification

**This was a READ-ONLY investigation.** Nothing was actuated, no node was
started by this session, no code path was modified. The evidence:

- **Rosbag decode of all 17 bags from the day** via the new committed probe
  `tools/probes/cone_bag_decode.py` (`python tools/probes/cone_bag_decode.py --all
  --match 2026-07-12`, re-run 2026-07-12 after the audit) — `/cone/heartbeat`
  decoded per session, reporting first-heartbeat state, the `connected` /
  `time_synced` sample fractions, and the `*` cone-reboot marker. This is the
  ground truth behind the per-session table in Diagnosis and behind both
  refutations. **17/17 sessions decoded**; 2 dead (`10-37-19`, `10-42-51`), 1
  partial (`20-03-36`, 155/593), 2 in-session cone reboots (`18-34-09`,
  `18-44-56`), 0 sessions with catch frames lost between cone and ROS graph.
- **Bench-vs-session overlap check** — every `bench_sysid_*` manifest's
  `created` / `finished` pair intersected against every bag's
  `message_start_time` / `message_end_time`: **22 bench runs × 17 sessions = 0
  overlapping pairs.** This is what refutes the co-bind attribution (Diagnosis
  Step 5).
- **Provenance check on the signature** — `git log --diff-filter=A` puts the bench
  harness at **2026-07-11** (`f218acf`) and `TeensyLinkClient` at **2026-06-02**
  (`2b605af`), while the `cone_test_*` bags of **2026-05-24** already show three
  full-session `connected=0/N` dropouts. The symptom predates both.
- **Repo suite gate for the one new file** (the probe is the only code this entry
  lands; `tools/` is outside pytest's `testpaths`, so it is gated only as a
  regression check on the tree): `pytest tests/ -q` (run 2026-07-13,
  immediately pre-commit) — **2575 passed, 1 xfailed in 599.25 s**. No test
  triple is claimed for the investigation itself: it landed no production code.
- **`~/.ros/log` node rosters and teardown records** — per-session node lists
  (confirming which nodes actually came up) and the shutdown/exit records that
  surfaced Open Item (b).
- **Live process forensics** — no zombie `teensy_bridge_node`, `rosbridge`,
  `ros2 launch`, or bench process; only the systemd GUI daemon
  (`jugglebot-gui.service`) and the `ros2` CLI daemon.
- **Code read** of `controller/teensy_link/client.py` (`TeensyLinkClient.start()`,
  the `SO_REUSEADDR` + wildcard-bind lines) establishing the co-bind hazard. This
  is the *only* evidence the hazard claim rests on — and it is sufficient for it.

**No test-count triple is cited, because none is applicable — this entry lands no
code.** The CLAUDE.md rule requires the (date, command, result) triple for any
*passing-test claim*; making one up for a read-only investigation would be
inventing evidence. The test suite was neither required nor run as a gate here.
The sibling probe promotion and this entry are the only artifacts.

**Operator status:** cone syncing is confirmed working after the investigation.
**This entry does not claim a proven fix, and no code fix exists to credit.** Note
carefully what the operator's confirmation does and does not support: it is
consistent with an **intermittent** cone fault that simply is not currently
manifesting, and it is *not* evidence that the single-owner discipline is what
cured anything. The honest statement is: *the reported symptom was never real, and
the one real fault is intermittent and still uncharacterised.*

## Outcome

- The operator's **zombie-process** hypothesis is **refuted** (live forensics:
  no zombies).
- The operator's **build-correlation** hypothesis is **refuted** (bag ground
  truth: **14/17** sessions synced from their first heartbeat, with a colcon build
  boundary leaving no signature — `19-16-18` / `19-17-31` / `19-19-12` identical
  — and three further launches 45 min to 3 h later, no build between, all synced).
- The only real failures of the day (`10-37-19`, `10-42-51`) show a
  **no-cone-frames-at-all** signature (`connected=False` for 100 % of
  heartbeats), i.e. a transport failure, not a CAN2 time-sync failure.
- A **real structural hazard** was found and is now documented: two
  `TeensyLinkClient` owners can co-bind the can-bridge UDP ports with
  `SO_REUSEADDR`; the kernel gives the *entire* inbound stream to one of them, and
  the non-cone-subscribing consumer silently drops every cone frame.
  **HIGH confidence the hazard is real** (read straight off the code).
- **That hazard did NOT cause the morning failures.** 22 bench runs × 17 sessions
  → **zero overlapping pairs**; the two morning bench runs are 3-second aborts;
  the signature flips mid-session (impossible for an all-or-nothing bind race) and
  **predates the bench harness by six weeks**. The attribution is downgraded to
  **LOW / candidate-only, not supported by the timing artifacts**.
- **Leading explanation for the morning sessions: cone MCU restart /
  power-wiring intermittency** (Open Item (c)) — three confirmed cone reboots on
  2026-07-12, one of them (`20-03-36`) with no bench process within an hour, plus
  identical full-session dropouts in the May bags.
- **No code changed.** Three open items are carried below, each actionable.

## Open Questions

See **Open Items** — all three are actionable follow-ups, not speculation.

## Open Items

### (a) Single-owner interlock in `TeensyLinkClient` — *needs coordination*

**What:** add an advisory single-owner lock (e.g. `flock` on a well-known
lockfile) to `TeensyLinkClient.start()`, taken **before** the sockets are bound,
recording the owner's PID/argv. A second binder then **fails loudly, naming the
current owner**, instead of silently stealing datagrams.

**Why it matters:** today, two processes can own the one physical can-bridge link
and *neither is told*. The kernel hands the **entire** inbound datagram stream to
whichever socket it resolved to at bind time, and any frame type that consumer
doesn't subscribe (e.g. `CONE_FRAME` for the bench harness) is dropped without a
trace — a healthy device presents as a disconnected one. This is a **class**, not
a cone bug: any future probe, script, or node that constructs a second
`TeensyLinkClient` inherits it, with a different symptom each time. A loud failure
at bind time closes the whole class at the single point where ownership is
established.

**Note on priority — this is a LATENT hazard, not a live incident.** Step 5
establishes that it has (as far as the artifacts show) **never actually fired**:
there is no recorded instance of two `TeensyLinkClient` processes overlapping.
That lowers the urgency but **not** the value — the reason to fix it is that when
it does fire it is *undetectable by construction* (no error, no warning, no log
line, on either side), which is exactly the property that made it so plausible as
a culprit here and cost this investigation a wrong attribution. Fix it because a
silent failure mode with no observable is a bad thing to keep, not because it is
currently hurting us.

**Why not now:** the fix is in shared transport that the parallel **bench /
gain-tuning workstream is actively depending on**. Changing bind behaviour under
a live bench session is a drive-by. **Coordinate with that workstream first.**

**Operational mitigation regardless — do this every time:**

> **Never run `jugglebot_launch.py` concurrently with the bench system-ID
> harness.** One owner of the can-bridge link at a time.

Pre-flight check before launching the ROS stack:

```bash
pgrep -af bench_leg_sysid
ss -lup | grep -E ":5005|:5006"
```

If either returns anything, **do not launch** — something already owns the link.

### (b) `teensy_bridge_node.on_shutdown` is killed mid-profiled-stow, EVERY session

**What:** in **every** 18:xx and 19:xx session, `teensy_bridge_node`'s
`on_shutdown` **exceeded the launch file's 5 s SIGINT→SIGTERM escalation window**
and was SIGTERM-killed (**exit `-15`**) partway through its profiled stow. Its
shutdown budget is ~**8 s** (settle + up to 6 s `DEACTIVATE`) against a 5 s
window — it cannot win, and it never does.

**Why it matters:** the **firmware deferred-stow is the backstop**, so there is
**no immediate danger** — the platform still stows. But the clean teardown is
**truncated every single session**, which means the graceful path is effectively
untested in practice and any future regression in it would be invisible behind
the backstop.

**Why not fixed here:** the fix is either (i) raise the launch escalation timeout
above the ~8 s shutdown budget, or (ii) tighten the budget under 5 s. **Either
way it touches shutdown timing**, which per CLAUDE.md requires the
control-system-implications analysis (what happens to the stow profile, the
`DEACTIVATE` handshake, and the firmware backstop's assumptions if the window
moves?). That analysis is real work and was **deliberately not** done inside a
read-only cone investigation. **Flagged, not fixed.**

### (c) Cone MCU restarts — *now the LEADING explanation; needs its own investigation*

**Promoted.** In the first draft this was a residual curiosity. After the Step-5
refutation it is **the leading candidate for the morning failures** and the most
important open item in this entry.

**What:** the cone MCU restarted **at least three times on 2026-07-12**, each time
preceded by the bridge losing cone frames entirely:

| Session | dropout | comes back | evidence of reboot |
|---|---|---|---|
| `18-34-09` | t+32.3 s | t+50.3 s | `last_catch_seq` 35 → **0**, `have_any_catch` cleared |
| `18-44-56` | t+12.4 s | t+26.7 s | `last_catch_seq` 5 → **0**, `have_any_catch` cleared |
| `20-03-36` | **down at launch** | t+43.9 s | comes up with no catch history; the next session (`20-04-49`) takes its first catch as seq **1**, while `20-02-47` had ended at seq **3** |

The probe now marks in-session resets with `*` so this cannot be missed again.
(`20-03-36`'s reboot is only visible **across** bags — the seq evidence is in its
neighbours — which is why the probe's `*` marks it on the first two but not the
third.)

**Why it matters:** the operator was **not** running the bench harness at 20:03
(nearest run finished 54 min earlier), and the same full-session-dead signature
appears in the `cone_test_*` bags of **2026-05-24**, six weeks before the bench
harness existed. So this is not a software-contention artifact: **the cone MCU is
restarting on its own**, and the morning failures look like the same fault, held
down for ~9 minutes instead of ~20 seconds. Candidate causes: supply brown-out on
the cone MCU, a loose connector or power lead, a CAN2 stub/termination fault, or a
firmware watchdog reset.

**Next steps, in order:**

1. **Ask the operator** whether the cone was power-cycled at ~18:35, ~18:45 and
   ~20:03 on 2026-07-12, and whether anything was touched/moved around 10:37.
   If the answer is "no" to all, this is an unprompted self-restart and the
   attribution is settled.
2. **Instrument the cone's own boot path** — does the firmware log a reset cause
   (brown-out vs watchdog vs external reset)? That single register distinguishes
   a power fault from a firmware hang and costs one heartbeat field.
3. **Then** investigate physically (wiring, connector seating, supply headroom
   under load). This is a hardware investigation and deserves its own entry.

**Do not** re-open the co-bind attribution without direct evidence of two live
`TeensyLinkClient` processes — see Step 5.

## Withdrawn claims

### The morning failures were caused by bench-harness UDP co-binding — WITHDRAWN

**Withdrawn before publication**, by the pre-commit `/audit` pass on this entry's
own draft. It never shipped, but it is recorded here rather than quietly edited
out, because the *reasoning* failure is the most reusable thing in this entry
(Discussion → *Adjacency is not concurrency*).

**What the draft claimed:** MEDIUM confidence that the two dead morning sessions
(`10-37-19`, `10-42-51`) were caused by the bench system-ID harness co-binding the
can-bridge UDP ports, on the evidence that bench runs "bracketed the dead sessions
to the minute".

**Why it was withdrawn:** the mechanism requires two sockets bound *simultaneously*
and the artifacts show only *adjacency* — 0 overlapping pairs across 22 bench runs
× 17 sessions; both morning bench runs are 3-second aborts (too short to starve a
163 s session even if they had overlapped); the signature flips mid-session in
three other bags, which an all-or-nothing bind race cannot do; and it appears in
the May `cone_test_*` bags, six weeks before the bench harness was written. See
Diagnosis Step 5.

**What replaces it:** cone MCU restart / power-wiring intermittency (Open Item
(c)), at LEADING confidence.

**What is NOT withdrawn:** the co-bind **hazard** itself (Step 4, HIGH confidence)
and the operational mitigation (Open Item (a)). Those were always read off the
code, never off the incident, and they stand unchanged.

<!-- The two hypotheses refuted in Diagnosis Steps 1-2 (zombies, build-correlation)
     belonged to the incoming operator report, not to a claim this entry published,
     so they are not retractions and are not listed here. -->
