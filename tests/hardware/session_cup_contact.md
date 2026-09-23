# Cup-contact contract — first powered sitting (single-site self-toss, 0.6 m and 0.9 m)

First flight of `plans/active/cup-contact-contract.md` (implemented 2026-09-21, commit
`2cc8631`, logbook `2026-09-20-cup-contact-contract-implemented.md`). Same machine, site,
driver and services as `session_skills_r3.md` — **that sheet stays the reference for every
shared row** (perception preconditions, bring-up, refusal table, recovery); this sheet only
adds what is new and the order to fly it in. Read § 0 before powering anything.

**If your physical intuition disagrees with the framing below, that is load-bearing signal —
say so before the first throw.**

## 0. What changed since the last sitting (2026-09-18 16:16), and what this sitting decides

1. **The cup no longer falls away from a ball** (C-CUP-2): from 0.125 s before the planned
   touch-down to rest / the knot before release, the cup's downward acceleration is held to
   ≤ 0.7 g. The cup planner is stroke-limited, so it cannot coast at the match speed through
   that window: it rides the 0.7 g floor and takes its dive in the one knot before the window.
   **Cost: the cup is slower at touch-down — −1.54 m/s instead of −2.74 m/s at 0.9 m, so the
   ball closes on the cup at ~2.66 m/s instead of ~1.46 m/s.** The simulator cannot show a
   rebound (its capture model is blind to closing speed). **This sitting is the test.**
2. **Banking is amplitude-aware** (C-CUP-1/3): a 4 mm lateral aim now commands 0.2° of tilt, not
   2.2°; leg jerk rises smoothly with the lateral offset and saturates at half the 150 k limit.
3. **Lateral authority defaults to 20 mm** (was pinned to 0; 40 mm through the first Block B on
   2026-09-23, which dropped balls on the 0.9 m 25-throw chains): the learner may aim throws
   laterally, and a tracker-aimed catch may move up to the authority from the schedule's site.
4. **A session-start frame check guards (3)**: with authority > 0, a start is REFUSED
   (`REJECTED_FRAME_OFFSET`) if the mocap `Platform` body is more than 5 mm from the commanded
   platform position over 1 s, or if that cannot be measured. `learner_lateral_authority_mm:=0`
   flies pinned and only logs the offset.

**Two questions, flown in this order so a bad catch is attributable to ONE change:**

| Block | Authority | Question | 
|---|---|---|
| A (pinned) | `learner_lateral_authority_mm := 0` | Does the new dive catch cleanly — no rebound, `seat=` in the smooth band? |
| B (unpinned) | `:= 20` (the launch default since 2026-09-23; 40 was the first Block B's) | Does the learner pull the lateral miss toward zero without the platform chasing the ball into a drop? |

**Pre-registered fallback (decided now, not at the rig):** if Block A shows a rebound signature
(§ 4) on **3 or more of the first 10 catches at either apex**, STOP — do not fly Block B. The
next session re-decides τ vs the two-tier approach floor (plan § 7) before anything else.

## 1. Before the robot is powered

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at `2cc8631` or later. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash` | 2 packages finished. **Mandatory** — the installed tree still carries authority 0, no frame check and the old planner. |
| 3 | `source ~/Desktop/PDJ_venv/venv/bin/activate && PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as ab; print(ab.gate_hash())"` then `grep -m1 gate_hash config/generated/admissible_box.yaml` | The two hashes are EQUAL. If not, the box is stale against the gate: `python tools/admissible_sweep.py --site-pairs both --single-apex 0.5 0.6 0.7 0.8 0.9` (~29 min) before flying. |
| 4 | `cat ~/Desktop/Jugglebot/temp/reports/nightly/status` | Fresh GREEN (or a RED you have read against `git status`). |
| 5 | `./run_tests.sh --full 2>&1 \| tee temp/logs/presitting_full_$(date +%Y%m%d).log` — **with the launch DOWN** | `RESULT: PASS`. (Rule: full tier before ANY hardware sitting.) |
| 6 | QTM: `Catching Cone` rigid body DISABLED, Ball Butler reflectors MASKED (R3 sheet row 10); a rigid body named **`Platform`** defined and tracked (R3 sheet row 10a); QTM aligned to the base with the new base marker. | The frame check needs `Platform`. |

## 2. Bring-up and the dress rehearsal (robot powered, NOT yet throwing)

Run R3-sheet rows **11–20** unchanged (load capture, launch with `record:=true auto_arm:=true`
teed to `temp/logs/launch_cupcontact_$(date +%Y%m%d_%H%M).log`, `blas threads: 1`, services
listed, GUI + QTM streaming, Home → Activate, site −50/0, `dwell_s 0.30`, a FRESH
`plant_id` e.g. `cc-$(date +%Y%m%d)`, `set_limits` 300 / 5000 / 150000, ball seated). Then:

| # | Step | Expect |
|---|---|---|
| 7 | `ros2 param get /skill_node learner_lateral_authority_mm` | `20.0` — proves the rebuilt install is the one running. `0.0` here = step 2 was skipped. |
| 8 | `ros2 param set /skill_node apex_m 0.6` ; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `box OK: ('P1', 'P1') …` with a band containing 0.6, and a **`frame check OK: mocap Platform is +N.N mm (x …, y …) … body spread S mm (limit 25.0 mm)`** line in the RESPONSE, plus a `tracker landings are now corrected by (x …, y …) mm` line in the launch log. **Record N, x, y, S.** Since 2026-09-23 an offset under 25 mm is MEASURED AND SUBTRACTED from every tracker landing, not refused — the 8.5 mm lever arm of 09-22 is expected here. |
| 9 | If row 8 says `REJECTED_FRAME_OFFSET` | Over 25 mm: the base alignment is wrong outright (not the lever arm) — re-align QTM to the base marker and repeat row 8. Cannot-evaluate names the missing input: no `Platform` body, stale mocap, stale commanded position, the platform not at rest, or the Platform body moving > 2 mm inside the 1 s window — fix that one. Block A may still fly (authority 0 only logs it, and still adopts an in-bound offset). |
| 10 | Loaded-box solve check: with the GUI open, bag recording and QTM streaming, leave it 60 s, then `grep -c SPLICE_TOO_LATE temp/logs/launch_cupcontact_*.log` after the first attempt of § 3 | 0. (Measured 2026-09-20 on a loaded box: catch-and-throw solve p95 146 ms against the 150 ms splice budget — the margin is thin. Two or more `SPLICE_TOO_LATE` in a block is a finding; note the load average at that time from the row-11 capture.) |

## 3. Block A — pinned: is the new dive a good catch?

| # | Step | Expect |
|---|---|---|
| 11 | `ros2 param set /skill_node learner_lateral_authority_mm 0.0` ; `ros2 service call skills/check std_srvs/srv/Trigger` | The response's frame-check line now reads `… (informational — learner_lateral_authority_mm=0)`. |
| 12 | **0.6 m.** `ros2 param set /skill_node apex_m 0.6` ; `n_throws 1` ; `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` — repeat until the memory holds 2 rows (cold-start policy A, R3 sheet § 4), then `n_throws 10` and one chained attempt. | Accepted every time. Per catch, read the `OUTCOME ball N: … seat=+0.0xx s vs scheduled landing` line and the hand sensor. **Record per throw: `seat=`, caught Y/N, HELD/EMPTY/HELD Y/N, your eyes' verdict (smooth / hard / bounced).** |
| 13 | **0.9 m.** `ros2 param set /skill_node apex_m 0.9` and repeat row 12 (singles until 2 rows at this apex, then a 10-chain). | Same record. |
| 14 | Apply the § 4 verdict to Block A before going on. | Rebound on ≥ 3 of the first 10 catches at either apex → STOP (pre-registered). |

## 4. What a good and a bad catch look like now

| Signal | Good (expected) | Bad — the rebound the sim cannot show |
|---|---|---|
| `seat=` on the OUTCOME line | **+0.05 … +0.15 s** (sim with the contract: +0.063 … +0.085; the 09-18 16:16 sitting's median was +0.12) | **> ~+0.20 s**, or no seat inside the 0.70 s window on a ball you watched land in the cup |
| Hand sensor across touch-down | EMPTY → HELD, once | HELD → EMPTY → HELD (the ball left the cup floor and came back) |
| By eye / ear | the ball "settles in" | a click and a visible hop; ball rattles or leaves |
| `caught=` on the memory row | True | False with the ball in the cup = late seat (count it as a rebound, not a drop) |

What is NOT a contract failure: a `seat=` near +0.02 s with a clean hold (early ball, the cup
still holds ≥ 0.3 g of seating force now — this is the case the contract exists for; on 09-17
this was HELD/EMPTY/HELD). **If early arrivals now hold cleanly, that is the contract working —
say so in the results.**

## 5. Block B — unpinned: does the learner take out the lateral miss?

Only if Block A passed § 4, row 8's `skills/check` read `frame check OK` (offset under the 25 mm sanity bound, adopted) and § 9 has been run.

| # | Step | Expect |
|---|---|---|
| 15 | `ros2 param set /skill_node learner_lateral_authority_mm 20.0` ; `ros2 service call skills/check std_srvs/srv/Trigger` | `frame check OK: …` (not `informational`). Same `plant_id` — Block A's rows already carry the observed lateral miss at zero lateral command, which is exactly what the learner needs (rows written BEFORE 2026-09-23 are in the raw mocap frame and were quarantined — only a Block A flown on this software is consistent with Block B). **Run § 9 first.** |
| 16 | **0.9 m first** (its box admits the full ±40 × ±40 mm): `n_throws 1` ×2, then `n_throws 10`. | Record per throw: landing error x/y from the memory-row line, `seat=`, caught. **Criterion (plan § 5): median lateral miss moves from ≈ +31 mm toward 0 within five throws**, `seat=` stays in +0.05 … +0.15 s. |
| 17 | **0.6 m** (box admits ±40 mm in x but only **±30 mm in y** — the learner's throw command is clipped there; the catch-side clamp is still 40). | Same record. A learner command sitting on the ±30 mm y face is the box binding, not a fault. |
| 18 | During both: `grep -E "AIM-LATERAL-CLAMPED\|RESEND \|RESEND-SKIPPED\|REJECTED_CYCLE_INFEASIBLE\|CUP_CONTACT_ACC\|LIMIT_JERK" temp/logs/launch_cupcontact_*.log \| tail -40` between attempts | `AIM-LATERAL-CLAMPED` = the tracker asked for more than 40 mm and was held (fine, count them). Re-send refusals on `LIMIT_JERK` should now be RARE (18 of 21 re-aims refused on 09-18; the dive is no longer at the jerk ceiling). **Any `CUP_CONTACT_ACC` refusal is a finding** — the planner should never produce a plan its own gate refuses; record the whole line (it names the knot and the value). |
| 19 | Watch the platform during the dive. | It may now move a few mm laterally in the catch window — that is commanded and tracked. The 09-16 "wobble" (13.5 mm excursion / 2–3° of tilt for a 4 mm aim) must NOT reappear; if it does, stop Block B and `:=0`. |

## 6. If something refuses or latches

R3 sheet § 6 (verdict table — it already carries `REJECTED_FRAME_OFFSET`) and § 7 (recovery:
`/recover` parks the hand; an armed off-band hand REFUSES → DEACTIVATE/ACTIVATE) apply
unchanged. Never re-dispatch a hand move by hand. A guard latch during the dive at the new
profile is a NEW finding: note the axis ("leg 6" = the hand), the knot time relative to
touch-down, and keep the bag.

## 7. Close-out

| # | Step |
|---|---|
| 20 | `skills/stop`, GUI Deactivate, launch down. Note the bag folder and the launch log path. |
| 21 | Hand Claude the PATHS, not pastes: the launch log, the bag folder, the loadavg capture, this sheet's § 8 filled in. |
| 22 | Logbook entry (`/investigate` or `/log`): the § 8 tables, the verdict on τ = 0.125 s, the lateral-miss trajectory, every finding. Update `plans/active/cup-contact-contract.md` status (§ 5 step 4's sitting) and the memory file. |

## 8. Results (fill in during the sitting)

**Frame check (row 8):** offset ____ mm (x ____, y ____) · QTM re-aligned? Y/N

**Block A — pinned**

| apex | throw | seat= (s) | caught | HELD/EMPTY/HELD | by eye | notes |
|---|---|---|---|---|---|---|
| 0.6 | 1 | | | | | |
| 0.9 | 1 | | | | | |

Block A verdict (§ 4): rebounds __/10 at 0.6 m, __/10 at 0.9 m → PROCEED / STOP

**Block B — unpinned (20 mm; the first Block B flew 40)**

| apex | throw | err x (mm) | err y (mm) | seat= (s) | caught | clamped? | notes |
|---|---|---|---|---|---|---|---|
| 0.9 | 1 | | | | | | |
| 0.6 | 1 | | | | | | |

Median lateral miss, first 5 → last 5 throws: 0.9 m ____ → ____ mm · 0.6 m ____ → ____ mm

`SPLICE_TOO_LATE` __ · `AIM-LATERAL-CLAMPED` __ · re-sends accepted/refused __/__ ·
`CUP_CONTACT_ACC` __ (expect 0) · guard latches __ · drops __

## 9. Frame-offset z-sweep — which frame is tilted? (robot powered, no throwing, BEFORE Block B)

Added 2026-09-23 (`logbook/2026-09-23-cup-contact-first-sitting.md` Diagnosis § 2). The
(−1.56, −8.53) mm offset equals 574.3 mm × the levelling pose offset (0.015, 0.002) rad. If it
is a lever arm (the QTM Base-body frame tilted from the machine's base plane), it scales with
the platform's height above the base; if it is a translation (a body-origin or marker
definition), it does not. Six commands, ten minutes, and it also exercises the new
"offset adopted, landings corrected" path with the robot standing still.

Run after § 2 (Home → Activate, QTM streaming, `set_limits` applied) with the launch log open
in another terminal. Authority pinned so nothing refuses on the number:

| # | Step | Expect / record |
|---|---|---|
| S1 | `ros2 param set /skill_node learner_lateral_authority_mm 0.0` | `Set parameter successful`. |
| S2 | For each z in **100, 170, 240, 300** (mm, STOW-relative — 170 is the ACTIVE height): `ros2 service call /trajectory/go_to_pose jugglebot_interfaces/srv/GoToPose "{pose: {position: {x: -50.0, y: 0.0, z: Z}, orientation: {w: 1.0}}, duration_s: 3.0}"` then wait 3 s at rest, then `ros2 service call skills/check std_srvs/srv/Trigger` | `accepted: true` (a `WORKSPACE` refusal at 300: record it and use 270 instead). The RESPONSE's `frame check: mocap Platform is +N.N mm (x …, y …) … body spread S mm` line — **record z, x, y, S for each height.** The launch log also prints `tracker landings are now corrected by (…)` at each check. |
| S3 | Return to the ACTIVE height: the same call with `z: 170.0`. | `accepted: true`. |
| S4 | Read the table. | **Lever arm:** y changes by ≈ −0.014 mm per mm of z, i.e. ≈ −7.5 / −8.5 / −9.5 / −10.3 mm at 100 / 170 / 240 / 300 (x by ≈ +0.002 mm per mm). **Translation:** all four within ±0.3 mm of each other. Either way the software subtracts the value measured at the site height, so Block B may proceed; a lever arm additionally means the `Base` body's definition should be redone with the base plane's normal (QTM: define the body from the base markers with the calibration frame aligned to the base plate), after which the check should read < 2 mm. |
| S5 | Fill in: z=100 (___, ___) · z=170 (___, ___) · z=240 (___, ___) · z=300 (___, ___) · verdict: LEVER ARM / TRANSLATION | |
