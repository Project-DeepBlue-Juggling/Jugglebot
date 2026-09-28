# Release-side discriminators for the self-toss lateral scatter (2026-09-29)

Context: ticket 01/02 (`.scratch/r4-throw-precision/evidence/result_t01.md`, `result_t02.md`)
found a rest-throw landing 1σ of 16–23 mm that a still, well-positioned platform cannot explain;
17–22 mm of it is a release-side residual with no marker on the hand/cup to instrument directly.
This note answers two offline-discriminator questions using tools that already exist, without
touching the hand.

All work is read-only. No production code was edited. Scratchpad scripts:
`qa_launch_velocity.py`, `qa_group_apex.py`, `qa_parse_logs.py`, `qb_cup_visibility.py` (this
scratchpad directory).

---

## Q-A: does the lateral launch scatter scale with launch speed?

### Method

`tools/probes/selftoss_landing_decomposition.py`'s ground-truth flight fit (gravity-fixed LSQ
on raw `/mocap_data`, gated to the `/balls` track) gives the launch velocity `(vx,vy,vz)` at
`t_rel` for every self-toss directly — this is what was reused for the 2026-09-22 bags (imported
the module, called `analyse()`, binned by the fit's own `apex` field).

Two bags predate the 2026-09-20 `MocapDataMulti.stamp` fix (`/mocap_data` there carries **no**
header/stamp field at all — confirmed by inspecting one raw message; `selftoss_landing_decomposition.py`
raises `AttributeError` on them as-is). For those (2026-09-16, 2026-09-18) I wrote a standalone
script (`qa_launch_velocity.py`) that keeps every topic in the mcap `log_time` domain — the same
trick `tools/probes/throw_outcome_bag_probe.py` already documents and uses for this exact reason
(median header-stamp-vs-log-time offset from `/balls`, applied to the unlabelled mocap markers) —
rather than editing the committed probe.

Self-tosses only (hops excluded, `angle-from-vertical > 2°`). "rest" = first throw of an attempt
(announced < 0.8 s ahead, platform stationary, nothing just caught); "carried" = released from a
catch. Two fits were excluded as tracking-fit outliers (`|vx|` or `|vy|` > 150 mm/s, one order of
magnitude above every other row — almost certainly a mis-associated marker, not a real throw):
`2026-09-16_16-22-22` carried apex 0.634 (vx=248.5, vy=-391.0), rest apex 0.573 (vx=243.5,
vy=-150.9); `2026-09-18_13-25-15` carried apex 0.552 (vx=-97.0, vy=-182.1); `2026-09-18_16-16-17`
one degenerate row (apex fit −0.085 m, discarded).

### Per-apex-group results

**Cleanest comparison — 2026-09-22 cup-contact block A** (post-tracker-fix, lateral learner
**pinned at 0**, so no learner-wander confound; `logbook/2026-09-20-cup-contact-contract-implemented.md`
/ `2026-09-23-cup-contact-first-sitting.md`):

| apex group | n (rest/carried) | vz mean mm/s | σ(vx) mm/s | σ(vy) mm/s | σ(angle_x)=σ(vx/vz) deg | σ(angle_y)=σ(vy/vz) deg |
|---|---|---|---|---|---|---|
| 0.6 m (actual 0.604±0.010) | 32 (5/27) | 3611 | 12.3 | 15.8 | 0.197 | 0.257 |
| — rest only | 5 | — | 17.4 | 15.5 | 0.285 | 0.253 |
| 0.9 m (actual 0.895±0.010) | 37 (4/33) | 4349 | 22.5 | 24.4 | 0.295 | 0.321 |
| — rest only | 4 | — | 18.0 | 20.4 | 0.246 | 0.278 |

Rest-only, n=4–5 per group: **σ(vx) is essentially flat (17.4 → 18.0 mm/s) while vz rises 20 %**,
and σ(angle_x) if anything *falls* (0.285° → 0.246°). That is the signature of an **additive
velocity kick of roughly constant size**, not a constant-angle wobble (which would need σ(vx) to
track vz, i.e. rise to ~21 mm/s while angle stayed flat). The all-throws (rest+carried) numbers do
grow with apex, but carried throws carry the known re-aim/catch-translation channel from ticket 01
— not a clean read on the release itself.

**2026-09-18 sittings** (`2026-09-18_13-25-15` mixed rungs in one sitting, `2026-09-18_16-16-17`
"flawless" run — see `logbook/2026-09-18-tracker-aim-carried-the-lateral-bias-park-race-splice-budget.md`;
this sitting is known to carry a QTM-base-alignment lateral bias and an active tracker-following
re-aim, so treat as a secondary check):

| apex bin | n (rest/carried) | vz mean mm/s | σ(vx) mm/s | σ(vy) mm/s | σ(angle_x) deg | σ(angle_y) deg |
|---|---|---|---|---|---|---|
| 0.6 m | 24 (9/15) | 3772 | 12.2 | 33.3 | 0.190 | 0.517 |
| — rest only (n=9) | | | 15.3 | 42.8 | 0.235 | 0.674 |
| 0.7 m | 10 (6/4) | 4149 | 10.5 | 42.8 | 0.143 | 0.591 |
| — rest only (n=6) | | | 11.0 | 31.9 | 0.153 | 0.451 |
| 0.8 m | 20 (4/16) | 4396 | 16.5 | 52.7 | 0.225 | 0.683 |
| — rest only (n=4) | | | 3.7 | 16.8 | 0.049 | 0.227 |
| 0.9 m | 23 (6/17) | 4620 | 22.7 | 33.0 | 0.288 | 0.420 |
| — rest only (n=6) | | | 22.9 | 40.9 | 0.296 | 0.538 |

Here rest-only σ(vx) does grow with apex (15.3 → 22.9 mm/s, roughly tracking the 1.22× vz rise),
and angle stays roughly flat-to-slightly-up (0.235° → 0.296°) — closer to the angular-wobble
signature. This sitting's own known confounds (tracker-aim lateral bias, still-live re-aim) make
it the noisier of the two; n per bin is 4–9.

**2026-09-16 apex ladder** (`2026-09-16_16-22-22`, arm A K=0 then arm B K=0.7 hand torque-FF,
5 rungs 0.5–0.9 m, **pre-**tracker-fix, highest per-rung overspeed of any bag here — see
`logbook/2026-09-16-apex-ladder-k07-ab-result.md`):

| apex bin | n (rest/carried) | vz mean mm/s | σ(vx) mm/s | σ(vy) mm/s | σ(angle_x) deg | σ(angle_y) deg |
|---|---|---|---|---|---|---|
| 0.6 m | 9 (2/7) | 3793 | 7.3 | 23.8 | 0.113 | 0.354 |
| — rest only (n=2) | | | 3.1 | 3.7 | 0.053 | 0.048 |
| 0.7 m | 4 (1/3) | 4350 | 24.8 | 52.3 | 0.300 | 0.654 |
| 0.8 m | 6 (3/3) | 4273 | 14.8 | 29.0 | 0.199 | 0.382 |
| — rest only (n=3) | | | 11.8 | 6.5 | 0.171 | 0.087 |
| 0.9 m | 4 (2/2) | 4789 | 24.4 | 23.2 | 0.310 | 0.279 |
| — rest only (n=2) | | | 35.4 | 0.4 | 0.446 | 0.028 |
| 1.0 m | 6 (2/4) | 4860 | 21.9 | 52.2 | 0.264 | 0.615 |

Rest bins here have n=2–3 — too thin to read a trend; included for completeness, not as evidence.

### Correlation with hand peak acceleration

A crude proxy (`max|Δvel_meas/Δt|` over the 150 ms before `t_rel`, from `/hand_telemetry`) was
computed for the 09-16 and 09-18 bags (106 fitted throws after outlier exclusion). Per-throw
lateral-velocity deviation from its apex-bin mean vs. this accel proxy: **r = 0.13** — a weak
correlation, i.e. throw-to-throw lateral kick size is not strongly predicted by how hard the hand
accelerated on that particular throw (accel itself correlates with vz at r=0.35, as expected —
higher apex needs a harder stroke). Not computed for the 2026-09-22 bags (script didn't capture
accel there); given the weak result on 106 throws, this was not pursued further under the token
budget.

### Interpretation

The **cleanest, best-controlled** dataset (2026-09-22, learner-pinned, post-fix) leans toward a
**constant-magnitude additive velocity kick** (σ(vx) flat across a 20 % vz change, angle-σ if
anything shrinking) — pointing at the ball's contact/seating in the cup (rolling, lip contact) or
a hand-axis misalignment, rather than a platform/carriage angular wobble that scales with speed.
The 09-18 secondary dataset (more throws, but confounded by a known tracker/learner lateral bias
that sitting) shows the opposite lean. **Given the conflict and the small n per rest-only bin
(4–9), this is not a confident verdict — it is a lean, not a proof.** The recommended next step is
still ticket 01's proposed sitting-A/B: a same-day apex ladder with the lateral learner pinned at 0
and re-aim off, run and analysed with this exact method, so apex is the only thing that changes
within one controlled dataset.

---

## Q-B: is the ball visible to QTM while it rides in the cup during the stroke?

Bag: `~/Desktop/rosbags/2026-09-28_23-55-19`. Reused the raw-data cache
`temp/probes/selftoss_decomp_cache_2026-09-28_23-55-19.npz` that
`selftoss_landing_decomposition.py` had already written for ticket 01/02 — no bag re-read. Method
(`qb_cup_visibility.py`): seed a nearest-neighbour track at the announced release
position/instant, walk it backward frame-by-frame through the unlabelled-marker stream
(`/mocap_data`, `z > 600 mm` gate already applied by the cache), report frame continuity, then
express the tracked marker's xy relative to the mocap `Platform` body.

5 throws examined (announcements 1–5 of this bag): #1 `hop` (site −125,0), #2 `rest` (site −50,0,
i.e. fresh into that site), #3–5 `carried` (continuations of the same chain).

| # | kind | frames in −350..0 ms | span (s) | median dt (Hz) | gaps > 30 ms | full-window xy drift vs Platform (mm) | **last-150-ms-only** xy drift (mm) | z travel, last 150 ms |
|---|---|---|---|---|---|---|---|---|
| 1 | hop | 57 | 0.344 | 6.6 ms (152 Hz) | none | 4.4 max | **6.1 max** | +108.9 mm |
| 2 | rest | 59 | 0.346 | 6.6 ms (152 Hz) | none | 2.6 max | **2.5 max** | +108.1 mm |
| 3 | carried | 50 | 0.341 | 6.6 ms (152 Hz) | none | 22.7 max | **3.0 max** | +73.1 mm |
| 4 | carried | 50 | 0.342 | 6.6 ms (152 Hz) | none | 9.4 max | **2.7 max** | +97.2 mm |
| 5 | carried | 46 | 0.339 | 6.6 ms (152 Hz) | none | 25.8 max | **2.2 max** | +84.5 mm |

**Yes, tracked continuously.** All 5 throws: a single physical marker is followed back the full
300+ ms with no gap over 30 ms, at a median 152 Hz (mocap's native rate) — the ball is not
occluded while riding in the cup for this sitting.

For the carried throws (#3–5), the **full-window** drift (9–26 mm) is not a cup-compliance signal:
it mostly reflects the ball's real incoming free-flight trajectory before catch — z falls
290–400 mm over the window, i.e. the marker is still airborne for a good part of it (the same
physical marker, correctly followed continuously through catch into the next stroke). Restricting
to the **last 150 ms** (the stroke itself, after catch/settle) removes that confound: z travel in
that window (73–109 mm) matches the throw's own upward excursion for every one of the 5 throws
(including #1/#2, whose whole 300+ ms window is already inside the stroke/rest phase). **In that
last-150-ms window, the ball's lateral position relative to the Platform body moves only 2.2–6.1 mm
across all 5 throws** — an order of magnitude below the ~17–22 mm (1σ) release-side residual
ticket 01/02 need to explain.

**Caveat, stated plainly:** this rules out a *visible* lateral roll/slide of >~6 mm happening more
than ~7 ms (one frame) before the release knot, on 5 throws in one sitting. It does **not** rule
out (a) a kick concentrated in the final few ms right at separation, which could fall in or after
the last visible frame or under cup-rim occlusion at the exact moment of release; or (b) the
marker being mounted such that it doesn't move even if the ball spins/rolls without a net centroid
shift. It is a genuine negative result for "gross lateral drift throughout the stroke," not a
clearance of "lip contact at the instant of separation."

---

## Commands run: (date, command, result)

- **2026-09-29**, `python tools/probes/selftoss_landing_decomposition.py ~/Desktop/rosbags/2026-09-22_23-21-58 ~/Desktop/rosbags/2026-09-22_23-40-54 ~/Desktop/rosbags/2026-09-22_23-52-57`
  — built the three `.npz` raw-data caches under `temp/probes/`; OK, ~9 min wall time (292+94+59 MB
  of mcap).
- **2026-09-29**, `python <scratchpad>/qa_group_apex.py`
  — imports `tools/probes/selftoss_landing_decomposition.py` and bins its fit output by apex for
  the three 2026-09-22 bags: 69 fitted self-tosses (0 hops), 32 at ~0.6 m / 37 at ~0.9 m.
- **2026-09-29**, `python <scratchpad>/qa_launch_velocity.py ~/Desktop/rosbags/2026-09-16_16-22-22`
  → `/tmp/qa_0916.log` — 56 announcements, 31 fitted (mocap-format bag, no header.stamp; used the
  log_time-domain workaround). ~4.5 min wall time (259 MB).
- **2026-09-29**, `python <scratchpad>/qa_launch_velocity.py ~/Desktop/rosbags/2026-09-18_13-25-15 ~/Desktop/rosbags/2026-09-18_16-16-17`
  → `/tmp/qa_0918.log` — 119 announcements, 80 fitted. ~6 min wall time (166+80 MB).
- **2026-09-29**, `python <scratchpad>/qa_parse_logs.py /tmp/qa_0916.log ...` and
  `.../qa_parse_logs.py /tmp/qa_0918.log ...` — apex-binned σ tables above; 4 fits excluded as
  outliers (listed above with their bag/kind/apex/v).
- **2026-09-29**, inline correlation script (accel proxy vs lateral-velocity deviation, n=106
  from the 09-16+09-18 fits): r=0.13; corr(vz, accel)=0.35.
- **2026-09-29**, `python <scratchpad>/qb_cup_visibility.py` (reads
  `temp/probes/selftoss_decomp_cache_2026-09-28_23-55-19.npz` only, no bag read) — 5-throw table
  above; all continuous at 152 Hz, last-150-ms lateral drift 2.2–6.1 mm.

## Caveats / things not done under the 60-call budget

- Q-A's per-bag classification of "rest" mirrors `selftoss_landing_decomposition.py`'s own
  heuristic (`t_rel - t_announce_log < 0.8 s` and angle-from-vertical); not independently
  re-derived from the schedule.
- Hand peak-acceleration proxy is a first-difference of `vel_meas` over a 150 ms window, not a
  proper filtered/derived acceleration channel — good enough for a rough correlation, not a
  precision number.
- The 2026-09-22 bags were not run through the accel-correlation step (time budget); the 09-16/18
  result (r=0.13, weak) is the only evidence on this sub-question.
- No new `.npz` caches were written to the repo's `temp/probes/` for 09-16/09-18 (the standalone
  script keeps its own pickle in the scratchpad only) — a future run of
  `selftoss_landing_decomposition.py` on those two bags would still need either the mocap-stamp
  fix backported or the same log-time workaround, since the committed probe throws on them as-is.
