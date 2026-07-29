---
title: The operator surfaces for the hand ball sensor — a tri-state GUI pill and the Phase 7 runbook
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 6 (GUI pill + session runbook)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/css/panels.css
  - ros_ws/gui/js/main.js
  - tests/ros/test_gui_geometry.py
  - tests/hardware/session_hand_ball_sensor.md
# backfilled after the Phase 6 commit lands
commits:
  - 73d70c6
subsystem:
  - gui
  - ros
tags:
  - testing
  - docs
  - safety
---

# The operator surfaces for the hand ball sensor — a tri-state GUI pill and the Phase 7 runbook

## Summary

Phase 5 put the sensor on ROS. Phase 6 — the **final software phase of the
plan** — puts it in front of a human, twice: once as a live pill in the GUI,
once as the written procedure for the sitting that will first prove any of it
on hardware.

The **ball-in-hand pill** lands in the System panel's flags grid
(`flag-level-pill` precedent), driven from the existing `onHandTelemetry`
handler. Three visual looks and a marker:

| Look | Meaning | Condition |
|------|---------|-----------|
| filled green `HELD` | ball seated | `ball_held_valid && ball_held` |
| hollow `EMPTY` | no ball | `ball_held_valid && !ball_held` |
| greyed **dashed** `UNKNOWN` | we don't know | `!ball_held_valid`, or no message |
| trailing `" ~"` | spotty contact | raw disagrees with a *stable* verdict |

UNKNOWN is a **third look, not a dimmed EMPTY** — the whole point of the
tri-state is that "we don't know" can never be misread as "no ball". The
flicker marker is **single-encoded as text**, and its latch is cleared on every
verdict change, so the ~100 ms of raw-vs-debounced disagreement that accompanies
*every ordinary edge* — the debounce doing its job — never wears the marker. A
persisting `~` is therefore unambiguous mid-carry flicker. (One edge does still
latch, by design: the first valid sample *after* a validity gap, whose verdict is
newly **defined** rather than unchanged.)

Two staleness paths, because there are two ways to go blind:
`ball_held_valid` covers a **dead bridge** (the node still publishes and says
"I can't vouch for this"); a **1 s message-stop watchdog** covers a **dead
publisher**. The initial DOM state is born UNKNOWN, which Phase 5's open
question #4 forced.

A new **7-test tripwire** (`TestBallPillFieldNameContract`) pins the three field
names in *both* the `.msg` and `panels.js`. The **runbook**
(`tests/hardware/session_hand_ball_sensor.md`, 952 lines) runs the plan's **five**
Phase 7 steps as **six runbook steps** (a bring-up step inserted as step 3, so plan
3/4/5 become runbook 4/5/6), every command with in-tree provenance.

The **end-of-plan full-suite gate is green**: `python -m pytest tests/ -q` —
**4288 passed, 3 xfailed in 1412.22 s**.

## Motivation

The plan's Phase 7 is an operator-run sitting, and both halves of this phase
exist to serve it.

The **pill** is the operator's live read. Step 2 — the mandatory blocking
raw-word toggle gate — asks a human to press a switch by hand and watch bit 2
move; `/link_status` gives the numeric truth, but a glanceable pill is what
tells them at a distance whether the thing they just did registered. Step 6's
ball soak asks them to note flicker "and what the platform was doing at the
time", which is only possible if flicker has a visible marker.

The **runbook** exists because of a standing project rule: during powered
sittings the operator runs the robot-actuating commands, from a written
procedure, not ad hoc. Phase 7's five steps include flashing firmware, arming
the robot, running a leg battery, and repeatedly putting a hand inside the cup
of an energised hand axis. None of that should be improvised at the bench with
a Claude session composing commands live.

There is also a failure mode specific to this phase that no earlier phase could
have: rosbridge delivers plain JSON, so a field renamed in the `.msg` leaves
`msg.ball_held_valid` permanently `undefined` in `panels.js` — which coerces to
false, and the pill sits at UNKNOWN forever with nothing logged anywhere. That
is indistinguishable at a glance from a genuinely dead sensor, during the one
session whose entire purpose is reading this pill. Hence the tripwire.

## Design

### Three looks, deliberately

The pill is a new `type: 'pill'` row in the `FLAGS` table, carrying **no flag
icon** — it is driven by `/hand_telemetry` rather than `robot_state`, and the
pill itself *is* the state, so a tick/cross/dot beside it would be dead
furniture.

The three CSS looks are structurally distinct, not gradations:

```css
.flag-ball-pill.held    { background: var(--accent-green); color: #000; }
.flag-ball-pill.empty   { color: var(--text-secondary); border-color: var(--border-color); }
.flag-ball-pill.unknown { color: var(--text-muted); border-style: dashed; }
```

HELD is *filled*, EMPTY is *hollow*, UNKNOWN is *dashed*. The dash is the
load-bearing bit: fill-vs-hollow is a single axis, and a greyed hollow pill is
just a quiet EMPTY. A dashed border is a different kind of thing on the screen,
which is the honest rendering of a different kind of statement.

### The flicker latch — an event marker with edge suppression

```js
if (!valid) {
    ballFlickerAtMs = 0;
    ballLastHeld = null;
} else {
    if (held !== ballLastHeld) ballFlickerAtMs = 0;   // verdict flip: not flicker
    ballLastHeld = held;
    if (!!msg.ball_held_raw !== held) ballFlickerAtMs = Date.now();
}
```

Set only on a valid sample where raw disagrees with an **unchanged** verdict;
cleared whenever the debounced verdict changes value, and cleared on any invalid
sample. The last clause stops a sub-second UNKNOWN gap from relighting a pre-gap
marker on the far side of it.

"Unchanged" needs one qualification, because the clear and the set live in the
same block and the **set runs second**. The verdict-flip clear only survives a
sample on which raw *agrees* with the new verdict; if raw disagrees, the set
immediately re-latches. So the marker does light during an ordinary edge's
debounce window and then clears when the verdict follows — which is the "at most
one 100 ms frame" the runbook tells the operator to expect, not a fault.

The case worth naming explicitly is the **first valid sample after a validity
gap**. The invalid branch resets `ballLastHeld` to `null`, so that sample always
takes the flip-clear branch (`held !== null`) — but there was no prior verdict
for it to be "unchanged" from: the verdict is *newly defined*, not *unchanged*,
and a raw disagreement there still latches. That is wanted. A disagreement on the
first reading back from a stale window is exactly what an operator should see.

Held for **1000 ms**, because the sensor polls at 50 Hz while this subscription
is throttled to 10 Hz — an instantaneous marker would blink for one frame and be
unseeable. It is an event marker, not a live state; the quantitative surface is
a rosbag of `/hand_telemetry` (the ball-soak step — **runbook step 6**, which is
the **plan's Phase 7 step 5**; the runbook renumbers, see below).

The marker is a trailing `" ~"` in the pill's **text**, with no CSS class. The
first draft also drew an amber ring; the review deleted it. The plan asks for
"a flicker marker", singular — and text is colourblind-safe and assertable from
a DOM string, which a border colour is not.

### Two ways to go blind, two mechanisms

`ball_held_valid` is the bridge's own vouching, forwarded by Phase 5's two-hop
gate. It covers a **dead bridge**: `teensy_bridge_node` keeps publishing at
100 Hz and each message honestly says "I can't vouch for this".

It cannot cover a **dead publisher**. If `teensy_bridge_node` dies, or the
subscription drops, the last message ever received was a perfectly valid HELD —
and the pill would freeze on it forever, which is exactly the failure the
tri-state exists to prevent, arriving through the one hop the message cannot
describe. Hence:

```js
const HAND_TELEM_TIMEOUT_MS = 1000;   // 10× the 100 ms GUI throttle
handTelemTimeout = setTimeout(() => updateBallHeld(null), HAND_TELEM_TIMEOUT_MS);
```

restarted on every message — the `setMocapConnected` watchdog idiom already in
this file.

The first draft *also* reset the pill in the websocket-disconnect branch. The
minimalism review deleted it as strictly redundant: a websocket drop stops
`/hand_telemetry`, so the watchdog drives the pill to UNKNOWN within one period
anyway. A comment was left in the disconnect branch explaining the absence,
because the neighbouring handlers (`setBBDisconnected`, the cone reset) *do*
reset there, and an unexplained omission reads as an oversight.

### Born UNKNOWN

The initial state is written into `initFlagsGrid`'s `innerHTML`:

```html
<span class="flag-ball-pill unknown" id="flag-ball-held-pill">UNKNOWN</span>
```

This is a direct consequence of **Phase 5's open question #4**:
`_publish_hand_telemetry` early-returns until the bridge's first `Telemetry`
frame lands, so `/hand_telemetry` publishes *nothing at all* during the boot
window. The topic cannot be relied on to drive the pill to UNKNOWN — the pill
has to be born there. An UNKNOWN-by-default DOM plus a watchdog means there is
no reachable state in which the pill asserts something it hasn't been told.

### The runbook's shape

`tests/hardware/session_hand_ball_sensor.md`, 952 lines, patterned on
`session_phase7_reload.md`. The plan's five Phase 7 steps became **six runbook
steps** — a bring-up step (launch → home → activate → confirm the 40 Hz hold
stream, the literal S1/S2 sequence) was inserted after the raw-word gate, for
reasons in the Discussion.

Every command has in-tree provenance. Notable determinations made while writing
it, all read out of the source rather than assumed:

- **Flashing** is `pio run -e teensy41 -t upload`, with `upload_command` explicit
  in `platformio.ini`. The banner-check recipe is
  `pio run -e teensy41 -t upload -t monitor` in one invocation, because
  **no standalone soft-reboot mechanism exists** —
  verified in source, the bridge console has exactly one command. You cannot
  re-print the boot banner without re-flashing or power-cycling.
- The existence of the `gpio_poll` console command **is itself a v4
  discriminator** — a v3 build doesn't have it, so a typo'd version check has a
  second, independent witness.
- **`ros2 topic echo --once` does not exist on this Foxy box.** Older runbooks
  in `tests/hardware/` use it; those recipes are wrong here. This one uses
  `PYTHONUNBUFFERED=1 timeout N ros2 topic echo …` and records the known
  echo-false-negative retry rule for high-rate RELIABLE topics.
- **The can-bridge Teensy's USB cable was not connected at authoring time**
  (`lsusb` shows no `16c0`, no `/dev/ttyACM*`). That is a blocking precondition
  for steps 1, 2 and 5, and it is called out as such rather than discovered at
  the bench.

Both A/B arms pin their motion to a literal repeatable sequence —
`traj_ramp_battery.py` for platform motion (MOTION-A), nine explicit
`go_to_pose` calls at reduced amplitude for carry/tilt (MOTION-B) — each with a
*record-your-substitution* clause, because an A/B comparison is meaningless to a
future reader who doesn't know what B was.

The **ABORT-park** block matters beyond this sitting: if step 2 fails, turning
`gpio_poll off` leaves the bridge **v4-with-the-poller-silent**, which is zero
RxSdo traffic and therefore a v3-identical CAN3 load. `HAND_SENSOR` is additive
and `PROTOCOL_VERSION` stays 4, so the parked bridge is safe to leave in place
for subsequent unrelated sittings. The catch, stated in the block: the toggle is
**not persistent** (`s_enabled` boots true), so the park must be re-applied after
every bridge reboot.

## Discussion

### Why the flicker latch suppresses edges

The naive marker — "light up whenever `ball_held_raw != ball_held`" — is wrong,
and wrong in the direction that destroys the marker's value.

Phase 3's debounce is asymmetric: HELD→EMPTY requires `MAX_MISSING_SAMPLES` (5,
at a 50 Hz poll) consecutive disagreeing samples before the verdict follows the
raw bit. So **every ordinary release produces ~100 ms of disagreement**, by
construction. A naive marker would light on every press and every release — the
debounce working exactly as designed, reported to the operator as a fault. And
since the marker is held for a full second to be visible at all, an operator
doing step 2's five press/release cycles would see a nearly continuous `~`.

A marker that fires on every normal event carries no information. Worse, it
*inverts* the intended signal: the operator learns to ignore it, and then misses
the one occurrence that mattered.

Clearing the latch on verdict change is what buys the semantics. After the
clear, disagreement can only latch while the verdict is *stable* — raw bouncing
underneath a settled verdict, which is precisely "the contact is intermittent
but the debounce is absorbing it". That is the condition the operator cannot see
any other way, and the condition that decides whether the shipped
`max_missing_samples = 5` is correctly sized. The marker now means **"spotty
contact"**, not "an edge happened", and a `~` that persists past an edge is
unambiguous.

The general shape: a diagnostic that fires on the healthy path is worse than no
diagnostic, because it trains the reader to discard the channel.

### Why UNKNOWN got its own look rather than a dimmer EMPTY

The tempting design is two looks and an opacity: green when held, hollow when
not, faded when unsure. It is less CSS and it reads fine on a mockup.

It fails on the one axis that matters, which is what a *glance* returns. Fill
and hollow are the two ends of a single visual dimension, so a faded hollow pill
is perceptually a *weaker EMPTY* — the reader's takeaway is "no ball, probably".
That is a specific, dangerous mistranslation: this signal's whole reason for
existing as a tri-state is that "no ball" and "we don't know" have different
consequences for a possession gate, and Phase 5 went to real trouble
(never-seen ⇒ all-False, `empty` reachable only from a valid reading, `unknown
(never seen)` rather than fabricated zeros) to keep them apart at every layer
below this one. Collapsing them in the render would undo that at the last hop,
in the layer a human actually reads.

A dashed border is *categorically* different rather than *quantitatively*
weaker. It doesn't sit on the fill/hollow axis at all, so it can't be read as a
point on it.

There is a deployment argument too. The pill will be looked at during a sitting
where the bridge may be unflashed, the link may be down, or the endpoint id may
be wrong — three separate UNKNOWN-producing faults, all live possibilities at
step 1. UNKNOWN is not an edge case in this sitting; for part of it, UNKNOWN is
the expected state. A state you expect to see deserves a design, not a filter.

### The two blocking safety findings, and why an operator-safety lens found them

The phase ran as an Opus implementer plus three parallel read-only reviewers
with distinct lenses — correctness + plan-conformance, operator-safety, and
minimalism — feeding a 23-item adjudicated fix batch. **Both blocking findings
came from the operator-safety lens, and neither was reachable from the other
two.**

**Finding 1 — the motion-framing contradiction.** The runbook's safety framing
described when commanded motion begins, and the step ordering contradicted it:
the framing implied motion started later than it actually did. Every individual
command was correct and had provenance; the conformance lens had nothing to flag
because each step matched the plan. The defect lived *between* the safety
preamble and the step sequence, and it is only visible if you read the document
the way an operator reads it — front to back, forming a mental model of "when
does this thing start moving" — rather than the way a reviewer reads it,
checking each block against a spec. The fix inserted an explicit bring-up step
(launch → home → activate → confirm hold) after the raw-word gate, so the
document's claim about when motion begins and the sequence's actual behaviour
are the same statement.

**Finding 2 — step 6's hand-in-cup actions.** The ball soak has the operator put
a hand into the cup repeatedly: seat the ball, nudge it mid-run to provoke the
failure mode, remove it at the end. The draft required a safe state before the
step; it did not require one before **each** action. The hand axis is in
closed-loop hold whenever the robot is ACTIVE, and step 6 deliberately runs
carry motion between those actions — so "safe at the start of the step" is not
safe at action three. The fix makes the checklist per-action and explicit
(commanded motion stopped, platform at rest, E-STOP in reach, and the
hand-axis-energised-while-ACTIVE fact stated outright), and establishes
deactivate → seat → re-activate as the preferred order rather than seat-while-armed.

What is worth recording is *why* a correctness reviewer cannot find these.
Correctness and conformance lenses ask "does this document say true things, and
does it match the plan?" Both drafts passed that bar — the commands worked, the
provenance was real, the steps mapped to the plan's five. The operator-safety
lens asks a different question: "if a human follows this literally, at a bench,
tired, with a powered robot, what happens to them?" That question is about the
*gap* between what a document says and what a person will do with it, and it has
no purchase on any single line. Both findings were structural — an ordering and
a scope — and structural defects in a procedure are invisible to line-oriented
review. The lens existed because a 950-line procedure for a powered sitting
warranted it; it returned two blocking findings, which is the justification.

### Tooling honesty as a runbook principle

Step 4 was supposed to measure SDO request→reply latency. Writing it surfaced
that **nothing shipped can measure that**: the bridge stamps the reply at
arrival, the console prints the resulting cache, and **the request send time is
recorded nowhere** — not on the wire, not on the console, and
`GpioPollSnapshot` has no latency field. Differencing anything available yields
*reply cadence* (poll interval + RTT jitter + lost round trips), not RTT.

The easy move is to write the step as if it measured RTT and let the operator
discover otherwise at the bench — or worse, to let them record a cadence number
under the label "latency", which then propagates into a logbook entry, sizes a
constant, and is believed for a year.

Instead the step carries an explicit *Tooling honesty* block stating what is
not measurable, what **is** (inter-reply intervals from distinct
`ball_held_stamp` values, healthy mode ≈20 ms, plus valid/stale/miss fractions),
and that the tail of that gap distribution is what should size the provisional
`REPLY_STALE_US = 2 × (20 + 100) = 240 ms`. Getting true RTT is then framed as
an **operator decision** with two costed options — a follow-up firmware counter
(small, contained, costs a second flash) or a CAN analyser on CAN3 (no firmware,
needs the tool) — with an instruction to record which was chosen, *including*
"deferred RTT entirely and shipped on cadence" as a legitimate answer.

The principle generalises, and belongs to runbooks specifically: **never let a
step imply a measurement the shipped system cannot make.** A procedure is read
by someone who is not going to re-derive its premises — that is the entire point
of writing it down. A step that asks for an unobtainable number will be answered
with an obtainable one, silently, and the substitution will not survive into the
results log. Naming the limit inside the step is the only place the honesty
sticks.

The same instinct produced the smaller notes: `ros2 topic echo --once` doesn't
exist on this box, the USB cable wasn't plugged in when the runbook was written,
`gpio_poll off` doesn't survive a reboot. Each is a thing the operator would
otherwise discover the expensive way.

### Why a string tripwire, and its honest limits

`TestBallPillFieldNameContract` is a string-level check, and the test class says
so in its docstring rather than pretending otherwise. It cannot prove the pill
renders correctly — nothing in this repo's Python test surface can execute
`panels.js`.

It catches exactly one failure mode, and that mode is nasty: rosbridge delivers
plain JSON with no schema negotiation, so renaming a field in the `.msg` doesn't
break anything loudly. `msg.ball_held_valid` becomes `undefined`, `undefined`
coerces to false, and the pill sits at UNKNOWN forever. No exception, no console
error, nothing in a log. During Phase 7 step 2 — whose live surface is this pill
— that is indistinguishable from a dead sensor, and the operator's most likely
conclusion is that the endpoint id is wrong. A field rename would send a sitting
chasing a firmware ghost.

One detail is load-bearing: `ball_held` is a **prefix** of both `ball_held_raw`
and `ball_held_valid`, so a naive substring check for `msg.ball_held` passes on
`msg.ball_held_raw` alone. The bare-verdict read is pinned with
`\bmsg\.ball_held\b(?!_)`. Without the negative lookahead the test would be
green with the debounced verdict deleted — the single most important field
gone, and the tripwire cheerfully silent. The whole set was **mutation-verified**:
renaming a field in a scratch copy fails the specific test that should fail.

## Implementation

- **`ros_ws/gui/js/panels.js`** — a `type: 'pill'` entry in `FLAGS` (row 5,
  `flag-ball-held`); the `initFlagsGrid` branch that renders it icon-less and
  born-UNKNOWN; and the exported `updateBallHeld(msg)` with the flicker latch
  (`ballFlickerAtMs`, `ballLastHeld`, `BALL_FLICKER_HOLD_MS = 1000`). The
  function's docstring carries the tri-state contract, the flicker semantics and
  the single-encoding rationale, citing the plan's § Architecture as normative.
- **`ros_ws/gui/css/panels.css`** — `.flag-ball-pill` plus the three
  state classes, with a header comment stating that UNKNOWN is deliberately a
  third look and that the marker is single-encoded in text.
- **`ros_ws/gui/js/main.js`** — `updateBallHeld` imported and called from
  `onHandTelemetry`; `HAND_TELEM_TIMEOUT_MS = 1000` and the restart-on-message
  watchdog; the explanatory comment in the disconnect branch where the deleted
  reset used to be.
- **`tests/ros/test_gui_geometry.py`** — `TestBallPillFieldNameContract`, 7
  tests: 3 parametrised over the `.msg` declarations, 3 over the `panels.js`
  reads, and the bare-`ball_held` lookahead test. Plus the `hand_telem_msg`
  module fixture.
- **`tests/hardware/session_hand_ball_sensor.md`** — new, 952 lines. Roles &
  safety framing, preconditions, pre-flight P0–P4 (repo state, bridge
  power-cycle, the `colcon build` of both packages from Phase 5's Deployment
  section, the synthetic-publisher pill check, and a confirm-what-you-flash
  step), then steps 1–6, a results-recording section, and a deferred/open list
  for the debrief.

### Process

One Opus implementer; three parallel read-only reviewers (correctness +
plan-conformance / operator-safety / minimalism); a 23-item adjudicated fix
batch. The operator-safety lens produced both blocking findings, discussed
above. Minimalism produced the deleted amber ring and the deleted
disconnect-branch reset. Correctness produced the regex lookahead and the
mutation verification.

## Verification

All runs 2026-07-29.

**END-OF-PLAN FULL-SUITE GATE** — the one full run the operator's 2026-07-29
gating direction requires for this plan. `python -m pytest tests/ -q`:
**4288 passed, 3 xfailed in 1412.22 s (0:23:32)**.

The count reconciles exactly against the plan's phase arithmetic:

```
4271  (Phase 4)
+ 10  (Phase 5 — teensy_bridge_node hand-sensor tests)
+  7  (Phase 6 — ball-pill field-name tripwires)
= 4288
```

**Scoped file** — `python -m pytest tests/ros/test_gui_geometry.py -q`:
**65 passed in 0.38 s** (58 pre-existing + 7 new). The tripwires are
mutation-verified: renaming a field in a scratch copy fails the right test.

**ES-module syntax check** — `node --check` on `.mjs` copies of the two edited
JS files, rc=0 both, with a deliberately-broken control returning rc=1. The
control was necessary: **plain `node --check` is vacuous for top-level-export
files on Node 22** — it accepts them without parsing as a module. Discovered
while building the check, worked around by copying to `.mjs` first. A syntax
check that cannot fail is not a check, which is why the broken control is part
of the recipe.

**Served-file check** — `curl localhost:8081/js/panels.js` (and the CSS) with
grep counts confirming the live service serves the *edited* files.
`jugglebot-gui.service` is untouched: `ActiveEnterTimestamp` unchanged from
2026-07-27.

**Deferred to the operator — and this IS a deviation from Phase 6's Done-when,**
not a carve-out the plan already granted. That Done-when asks for **browser
verification against `jugglebot-gui.service`**, and no browser has rendered this
pill. The reason is mechanical rather than discretionary: **no headless check in
this repo can render `panels.js`.** There is no JS test runner, no DOM harness,
and the Python test surface can only read the file as text — which is precisely
why the coverage that *does* exist is a string tripwire and says so.

So the browser check is deferred into the runbook's pre-flight **P3**, which
drives all four states from a synthetic `ros2 topic pub` table against the live
service, and it is the operator who closes it. The plan's Phase 6 row will read
**done (`<sha>`; P3 browser check outstanding)** — the outstanding item named in
the row rather than absorbed into it.

**Not verified.** No `HAND_SENSOR` frame has ever crossed the link on hardware.
The pill has never displayed a state derived from a real sensor reading, and the
runbook has never been executed. Everything above is source-level and
synthetic-message verification.

## Deployment

**Live as served, immediately.** The GUI is static files with no build step;
`gui_server` sends `no-cache`, so a browser hard-refresh is the entire deploy.
No `colcon build`, no service restart — and `jugglebot-gui.service` was
confirmed untouched rather than assumed so.

The one prerequisite is Phase 5's: `/hand_telemetry` does not carry the four
fields until `colcon build --packages-select jugglebot_interfaces jugglebot` +
relaunch. Until then the pill correctly renders UNKNOWN, because
`msg.ball_held_valid` is absent and absent coerces to false — the tripwire's
failure mode, arriving here as the *right* answer.

The runbook is the **Phase 7 hand-off artifact**. It is committed before the
sitting, by design: the operator runs robot-actuating commands from it.

## Open questions

1. **The pill has never been seen in a browser by a human.** P3 is written and
   the synthetic publisher table is exact, but the render is unobserved —
   contrast, the dashed border's legibility at the panel's font size, and
   whether the `~` suffix is noticeable at 10 Hz are all unvalidated design
   claims.
2. **`BALL_FLICKER_HOLD_MS = 1000` is reasoned, not measured.** It is 10× the
   GUI throttle, chosen so a single-sample event survives to a frame the
   operator can see. Whether one second is long enough to notice, or long enough
   that consecutive events smear into a solid marker during genuine chatter, is
   a step 6 observation.
3. **The watchdog and `ball_held_valid` have never both been exercised.** Each
   covers a failure the other cannot, but neither has fired against a real
   outage — the dead-publisher path in particular requires killing
   `teensy_bridge_node` with the GUI open, which nobody has done.
4. **True SDO RTT remains unmeasurable with shipped tooling**, and the choice
   between a firmware counter and a bench instrument is an open operator
   decision carried into step 4. `REPLY_STALE_US = 240 ms` stays provisional
   until one of them happens.
5. **The runbook's substitution clauses are untested for comparability.** Both
   A/B arms permit the operator to swap in their usual sequence provided they
   record it. Whether a recorded substitution actually preserves the A/B
   comparison depends on the substitution, and no sitting has stressed that yet.
6. **`gpio_poll off` is not persistent.** The ABORT-park path depends on the
   operator re-applying it after every bridge reboot. Nothing enforces that, and
   a forgotten re-park silently restores RxSdo traffic to CAN3 on a subsequent
   unrelated sitting.

## Related

- `plans/active/hand-ball-sensor.md` — Phase 6 (this phase, the final software
  phase); § Architecture remains **normative** for the tri-state this pill
  renders; Phase 7 is the sitting the runbook drives. This entry carries the
  plan's **end-of-plan full-suite gate**.
- `logbook/2026-07-29-hand-sensor-ros-surface.md` — Phase 5. Supplies
  `ball_held` / `ball_held_raw` / `ball_held_valid`, the two-hop validity gate
  the pill renders, the `colcon build` prerequisite, and open question #4 —
  which is why the pill is born UNKNOWN.
- `logbook/2026-07-29-hand-sensor-uplink-message.md` — Phase 4. The 50 Hz wire
  rate the 1 s flicker hold is sized against.
- `logbook/2026-07-29-hand-sensor-bridge-gpio-poller.md` — Phase 3. Owns the
  asymmetric 5-sample debounce whose ~100 ms edge disagreement the flicker latch
  suppresses, the `gpio_poll on|off` console toggle the ABORT-park uses, and the
  provisional `REPLY_STALE_US`.
- `logbook/2026-07-29-hand-sensor-endpoint-id-contract.md` — Phase 2. The
  endpoint id that step 2's raw-word gate exists to prove.
- `logbook/2026-07-29-hand-sensor-ball-detect-config.md` — Phase 1.
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0. Its
  fw-version launch line is step 1's confirmation surface.
- `tests/hardware/session_hand_ball_sensor.md` — the runbook itself.
- `tests/hardware/session_phase7_reload.md` — the pattern it follows.
