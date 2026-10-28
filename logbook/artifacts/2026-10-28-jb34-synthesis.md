> Discussion artefact of 2026-10-28 (JB-34 logbook backfill survey), kept verbatim below. Owner changes: E2 declined (the skills/check live-state rehearsal is not filed);
> status value is `resolved` everywhere, never `superseded`; everything else approved as recommended. The filed items are JB-82 to JB-87 (see the logbook entry).

# JB-34 synthesis: the logbook backfill as a proposal

Inputs: `merged.csv` (228 rows from 7 surveyors) and the seven `out/*.notes.md` files, plus 26 targeted
checks against the repo on 2026-10-28. Nothing is filed. This is the discussion artefact for the owner.

## (a) Headline

The owner's guess holds. **Of 228 entries, 16 have residue that is still open and that nothing tracks.**
Those 16 group into 12 themes. The proposal: **FILE 6 items, FOLD 3 themes into existing items or plans,
DROP 3**. No new Pathway is recommended. **93 status flips** are in `flips.csv`.

| verdict | surveyed | after corrections | full_* (102) | skim_* (126) |
|---|---|---|---|---|
| SUPERSEDED | 98 | 99 | 39 | 60 |
| CLOSED | 67 | 68 | 33 | 35 |
| STALE | 24 | 29 | 6 | 23 |
| LIVE | 16 | 16 | 14 | 2 |
| STILL-OPEN | 23 | **16** | 10 | 6 |

The two right-hand columns are the after-corrections counts.

## (b) Verdict corrections (evidence-backed only)

| entry (logbook/) | old | new | why |
|---|---|---|---|
| 2026-07-16-max-deviation-guard-tracking-lag (skim) | CLOSED | SUPERSEDED | `plans/archived/accel-ff-inertia.md:4` says `status: superseded`, and its Archival note says "Nothing implemented" (the stale-hold decay was never written). ⚠ The memory index still calls accel FF "the next chapter". That line is stale. |
| 2026-07-17-wobble-latch-unshaped-traverse (skim) | STILL-OPEN | CLOSED | `bus_voltage` is carried in `/robot_state`: `MotorStateSingle.msg:26`, populated at `teensy_bridge_node.py:3639,3697` |
| 2026-08-28-pipeline-first-contact-deadlock (skim) | STILL-OPEN | CLOSED | The GUI has a cancel: the Juggle panel has Start plus an immediate Stop (`ros_ws/gui/js/juggle-panel.js:4,479`). Only a `bb/cancel` button is missing, and that is already a row in the residue plan's § 2 |
| 2026-08-21-mjcf-generator-reconciled (skim) | STILL-OPEN | STALE | The 95 g vs 71 g gap is documented as deliberately unreconciled at `config/hardware_config.yaml:38-45`. The heavier implied ball is the conservative side |
| 2026-07-29-hand-sensor-bridge-gpio-poller | STILL-OPEN | STALE | The sensor has been in production since 2026-08, and possession verdicts have been sensor-only since 2026-08-26. The tuning rows were re-homed to `tests/hardware/session_hand_ball_sensor.md` |
| 2026-07-29-hand-sensor-gui-pill-runbook | STILL-OPEN | STALE | Same as the row above |
| 2026-09-23-block-b-lateral-learner-and-hand-endstop-push | STILL-OPEN | STALE | A watch item only. No recurrence in a month of Sittings, and the lateral-learner half shipped |
| 2026-10-17-bb-calibration-residual-cap-1p5 (skim) | STILL-OPEN | STALE | The state was accepted with the TEMPLATE_RESIDUAL advisory (`plans/archived/bb-calibration-one-attempt.md:322-327`). The vertical-mode gap is a named non-goal of `plans/parked/bb-marker-never-matched-advisory.md:67` |
| 2026-09-30-skill-stack-r5-sitting-1 | STILL-OPEN | STILL-OPEN (new evidence) | The old item was fixed by BallButler commit 587e732 (2026-10-10), but the class came back. The over-CAN `upload_command` (`BallButler/ball_butler_main/platformio.ini:94`) and `scripts/bb_link_check.py:25` call `~/Desktop/Jugglebot-skills`, a worktree retired 2026-10-23 that no longer exists. **The next over-CAN Ball Butler flash fails.** |

These unsure verdicts were checked and stand:
- `suppress_announcement` is still set (`ball_butler_node.py:595,800`, `BallButlerThrowCmd.action:20`).
- The `holds_ball` wire gap is still excluded by name (`tests/ros/test_install_segment.py:1413-1417`).
- The RETARGET arbitration was never fixed. The entry routes around it with the jam anchor.
- The caught-gate split-track stays SUPERSEDED. Possession is sensor-only, and the tracker defect has no consumer.
- The cup-seating-cone angle gap stays LIVE under the oval plan, but it is untracked inside it (theme J).
- The Kt entry stays CLOSED: its own work, Kt and gravity FF, shipped.

The 13 blank-theme rows were all skim STALE rows. They are now themed:
- *nothing survives*: 04-19 leg1-hold-twitch, 08-24 arrival-band, 09-08 fw18-bundle, 10-13 gui-replay-rename
- *retired line*: 05-19 spacemouse (sim/input deleted 2026-10-14), 07-16 lean-planning test caveat (unified planner)
- *CAN3 hardware fault*: 06-30 phase6-reboot-latch, 07-05 canhub-marginal-can3
- *observability watch items*: 07-11 gui-leg-setpoint-echo, 08-25 observability-closeout, 08-25 udp-channel-health
- *hand bench tooling*: 09-05 fw17-hand-ladder-3
- *superseded Sitting questions*: 10-05 sitting-5-afternoon

## (c) Themes and recommendations

The hardware, firmware and safety themes come first.

**A. Hand ODrive state the can-bridge cannot see** (Hand/firmware, safety-bearing). Entries: 09-04 fw17-hand-sitting-unflashed-idle-axis, 09-14 r3-first-powered-sitting, 10-05 sitting-5-launch-limits-and-jam-anchor.
Residue:
- A Hand in Closed-loop but in the wrong input mode swallows the stream silently, and no telemetry sees it.
- The Guard measures the Hand's deviation against the raw plan, not the post-slew command (`leg_interp.cpp`).
- RETARGET sends `Set_Traj_Vel_Limit` (0x011) and `Set_Input_Pos` (0x00C) in the same tick. 0x00C wins arbitration, so the move can be planned at the old cruise speed.

**Recommendation: FOLD into JB-73** (vel_ff held after MOTOR_FB_STALE). It is the same class: ODrive-side Hand state that drifts from what the can-bridge believes it commanded. Retitle JB-73 to *"Hand ODrive state can silently diverge from what the can-bridge commanded"*, keep priority medium, and keep vel_ff as the first instance.
Trade-off: JB-73 is small and concrete today, and folding turns it into an enumeration. The alternative is a separate bug at medium.

**B. Bridge link ownership and the shutdown Stow** (can-bridge, safety-bearing). Entry: 07-12 cone-sync-udp-link-contention.
Residue:
- `TeensyLinkClient` has no single-owner interlock (no flock/fcntl in `teensy_link/`). The memory index lists this as a standing bench hazard.
- `teensy_bridge_node`'s on_shutdown Stow takes about 8 s. The launch escalates SIGINT to SIGTERM after 5 s, and no `sigterm_timeout` is set in the launch files. The firmware deferred-stow is the only backstop.
- Item (c) of the entry, the Cone MCU restarts, is dropped: it predates the bus swap.

**Recommendation: FILE as bug**: *"teensy_bridge_node's shutdown Stow can be cut off by the launch's 5 s SIGTERM; teensy_link has no single-owner interlock"*. Priority medium, no pathway.
Trade-off: if the owner trusts the firmware deferred-stow, the Stow half is benign. Then file only the interlock, at low.

**C. CAN3 drive-path hardware fault** (can-bridge hardware). Entries: 07-31 can3-drive-path-fault, plus STALE 06-30 and 07-05.
Residue:
- The load-dependent analog fault was never repaired, and the firmware still runs the bus swap "until the CAN3 path is repaired".
- CAN3 now carries the Cone and, since 2026-10-25, the wave camera, so the faulty bus gained load. Whether JB-52 (CLIP_MISSING on first start) relates is unchecked.

**Recommendation: FILE as bug**: *"can-bridge CAN3 analog drive path has a load-dependent hardware fault (the Jugglebot loom stays on CAN2)"*. Priority low, no pathway.
Trade-off: DROP, and keep it as the standing memory and firmware-comment fact it already is.

**D. Robot frame and Fit bias** (tracking/kincal). Entries: 09-23 kinematic-calibration-design-and-fit-tool, 09-23 cup-contact-first-sitting, 09-29 r4-sitting-3.
Residue:
- Kincal steps 6-7 were never done: the tilt-map recapture after Level and the flying frame check. `_frame_offset_check` is still live (`skill_node.py:605`).
- `flight_fit` assumes gravity along -z, which gives a +62 mm/s² y bias.
- The early Fit lands about 9 mm short in x.

**Recommendation: FILE as feature**: *"Kinematic calibration acceptance: the flying frame check, and a Fit that assumes gravity along -z"*. Priority medium, Pathway **learner-generalisation**. Its P2 open questions (centre-pull cause, apex drift) are candidate symptoms of an uncorrected frame tilt, and a global residual would otherwise absorb a geometry error.
Trade-off: give it its own Pathway instead (see d).

**E. `RestTerminal.holds_ball` never crosses the wire** (skill-stack). Entry: 10-02 r5-sitting-2, plus the 10-06 sitting-6 and 09-14 apex-ladder-prep remnants.
Residue:
- The node rebuilds every REST as holding a ball, which over-constrains an empty-cup REST's cup-contact floor. The over-constraint is conservative: it costs refusals, not safety.
- The 10-06 entry notes that a live-state rehearsal (a `skills/check` that plans each Pattern's first Skills from `/robot_state`) was "Not built". Today's `skills/check` (`skill_node.py:31,1410`) reports only ladder refusals and box/limits.

**Recommendation: FILE as bug**: *"RestTerminal.holds_ball never reaches the node: every REST is rebuilt as holding a ball"*. Priority low, Pathway **two-ball-oval**.
- DROP the apex-edge tie-break, the CHAIN_CATCH margin ring and `t_bridge_us` (R3-era; no recurrence since the R5 gate).
- The closing-REST LIMIT_JERK is already JB-24.
- The rehearsal is an owner question (g2).

**F. Wire, interface and docs hygiene** (trivial). Entries: 06-08 bb-cutover-bench, 06-29 canbridge-phase0, 09-21 agent-skills-config, 10-13 wave-2 item 7.
Residue:
- The dead `suppress_announcement` field still names the deleted `can_node`.
- `teensy_bridge_node.py:809-816` hard-codes the 0x8 `MPC_ACTIVE` and 0x30/4 cone-health masks instead of using `p.HeartbeatT2JFlags`.
- `docs/teensy-udp-protocol.md` is missing from the `mkdocs.yml` nav.
- `HAND_LANE_REFUSED` should be renamed `LANE_REFUSED`, and `gpio_poll.cpp:71-74` still says "PROVISIONAL".

**Recommendation: FILE as feature**: *"Retire dead wire fields and literal flag masks; put teensy-udp-protocol.md in the docs nav"*. Priority low, no pathway. It is one chore item, not a Pathway. The `.action` edit needs a colcon rebuild and a launch restart.
Trade-off: DROP, and let the next edit that touches each file take it.

**G. Ball Butler residue** (Ball Butler). Entries: 09-30 r5-sitting-1, 10-13 bb-throw-self-cancel, plus STALE 10-17, 07-24 and 06-24.
Residue:
- The over-CAN flash path points at the retired worktree (see the corrections table).
- F3, F4, yaw-homing and the aim refit are already rows of `plans/archived/skill-stack-dress-rehearsals-and-residue.md` § 2.

**Recommendation: FILE as bug**: *"Ball Butler over-CAN flash calls the retired Jugglebot-skills worktree"*. Priority medium, because it blocks the next Ball Butler flash. Pathway **ball-butler**. The fix is in the BallButler repo, `platformio.ini:44,94` and `bb_link_check.py:13,25`.
FOLD F3, F4, yaw-homing and the refit: they stay as residue-plan rows. JB-25, JB-21 and JB-30 already cover the rest.

**H. Hand ball sensor tuning** (Hand/can-bridge). Entries: 07-29 gpio-poller, 07-29 gui-pill-runbook, 07-29 endpoint-id.
Residue: the ~240 ms `REPLY_STALE_US` dead-sensor window was never measured, and the `gpio_poll` off setting is not persistent.

**Recommendation: DROP as stale.** The sensor has run every Sitting since August, and its tuning rows live in the hardware runbook. The comment fix rides in F.

**I. Launch CPU and the emitter gap**. Entries: 09-14 rosbridge-cpu-leak, 10-10 mocap-frame-queue, 05-22 mpc-compute-bound (lesson only).

**Recommendation: FOLD into JB-76, JB-23 and JB-31**. Add `mocap_node` priority/pinning and the `TRACKING_MAX_FRAMES` revisit to JB-76's next steps, unless they are already there.

**J. Oval cup-seating cone angle gap**. Entry: 10-19 cup-seating-cone-refuses-columns-stroke-braking. The offline replica reads 16.5-21.7° at the knot after the Catch, while hardware read 31.6-41.7°. The gap is unexplained.

**Recommendation: FOLD** as one open-question line in `plans/active/two-ball-oval.md`. No item. P4 already moved to `pose` mode partly because `level` stays CUP_SEATING_CONE-refused (2026-10-24 entry, line 181).

**K. GUI producer-liveness contract**. Entry: 07-12 gui-cone-catch-correlation-regression. The ask is a test that every GUI-subscribed topic has a live producer. Only the KeyValue half exists (`tests/ros/test_gui_geometry.py:522-556`).

**Recommendation: DROP.** No GUI-topic regression has been recorded since July.
Trade-off: it is exactly the one-invariant/one-test shape the Engineering Philosophy favours. If the owner wants it, it fits as a fourth bullet of F.

**L. Unreconciled constants and watch items**. Entries: INERTIA_RATIO (08-21), bridge clock ppm (08-15), FK cold-start branch (07-25), Hand end-stop push (09-23), the observability watch items, the sitting-5 questions, cone-sync (c), the apex-ladder remnants.

**Recommendation: DROP as stale.** Each is documented, bounded or harmless, and none recurred. The per-row reasons are in `merged.csv` `evidence`.

## (d) Pathways

**No new Pathway recommended.** One candidate has a reachable end state:
- **Robot frame agreement** (theme D). Destination: *"The kinematic model, the Robot frame and the Tracker's Fit agree closely enough that the flying frame check passes and `_frame_offset_check` is deleted."*
  It is about one plan's worth of work, and learner-generalisation is its consumer, so it is proposed as an item under that Pathway.
- Theme A's class ("every Hand ODrive state the bridge relies on is observed") has no reachable end state, so it stays an item.

Glossary check on the six titles: they use Hand, can-bridge, Stow, Ball Butler, Fit and REST. None of them uses an `_Avoid_` word: not toss, reload, session, leg 6, butler/feeder, can-hub, or "the guard" for `motor_guard`. Words like "issue" or "defect" do not appear either.

## (e) Flips (`flips.csv`, 93 rows, all to `resolved`)

By source verdict: SUPERSEDED 39, CLOSED 33, LIVE where the entry's own work is done 12, STALE 6 (3 surveyed plus 3 corrected), and 3 skim rows with off-ladder statuses (`complete`, `done`, `open`).

**Status value.** The ladder in `logbook/README.md:171-178` is `open | in-progress | tuned | resolved`. `tests/sim/test_logbook_front_matter.py` checks only that a status is present, not its value.
- `superseded` has one precedent (`2026-07-12-bench-leg-gain-tuning-stage1.md:5`) and a colour in `tools/render_report.py:52`, but it is not on the documented ladder.
- So the flips use **`resolved`**, and the closure line says "superseded" where it applies.
- Three surveyed rows carry the off-ladder `resolved-with-open-items`. Two of them flip.

Not flipped:
- The 10 full STILL-OPEN entries keep their status. Suggestion: flip them to `resolved` in the same commit that files their item, with a closure line naming the JB-n.
- Two LIVE rows stay unflipped:
  - `2026-10-18-learner-generalisation-investigation`: by its own terms it closes with plan P5.
  - `2026-10-19-cup-seating-cone-refuses-columns-stroke-braking`: the survey did not confirm that the amended cone rule was flown.
- The skim `superseded` row waits on question g1.

## (f) Live-skipped (11 entries after 2026-10-20, no verdict, all `in-progress`)

10-22 catch-observation-and-catch-time-offset; 10-23 columns-last-ball-parity-rule-and-stop-throw-count; 10-23 deactivate-rejected-state-machine-divergence; 10-23 feed-line-latches-the-in-flight-fit; 10-23 iplan11-extended-to-the-rest-slew-seed; 10-23 rest-rebase-stale-seed-hand-bump; 10-24 leg-lead-clamp-age-aware-p1-firmware; 10-25 wave-camera-gpio-findings; 10-28 jb35-records-ledger; 10-28 jb75-jb72-arm-edge-stale-and-home-epoch; 10-28 jb76-solve-tail-is-environment-starvation-instrumented.

## (g) Questions for the owner (each answerable in one line)

1. Should flips use `resolved` everywhere, or should `superseded` be added to the README ladder (a normative-doc edit, so audited) and used for the 39 superseded rows?
2. Theme E: should the `skills/check` live-state rehearsal (planning each Pattern's first Skills from `/robot_state`) be filed as a feature on two-ball-oval, or left alone?
3. Theme A: fold into JB-73 and retitle it to the class, or file a separate bug?
4. Theme B: is the firmware deferred-stow a sufficient backstop, so that only the single-owner interlock is filed?
5. Theme C: file the CAN3 drive-path fault as a low bug, or keep it as a memory fact only?
6. Theme G: fix the Ball Butler flash path now (a two-line BallButler edit) instead of filing it?
7. Should the 10 STILL-OPEN entries be flipped to `resolved` when their item is filed, or keep `in-progress` until the item closes?
