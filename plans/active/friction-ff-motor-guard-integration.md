# Friction-FF Integration into motor_guard

**Status:** Proposal, awaiting approval
**Predecessor:** [plans/active/motion-onset-deadtime-investigation.md](motion-onset-deadtime-investigation.md) — bench-validation phase complete
**Bench-validation logbook:** [logbook/2026-04-27-friction-feedforward-bench-validation.md](../../logbook/2026-04-27-friction-feedforward-bench-validation.md)
**Author note:** This is a research/integration deliverable. The plan describes the contract, the implementation, the per-leg tuning protocol, and the acceptance gates. Do not begin implementation until §6's open questions are resolved with the user.

---

## 0. Context and goal

The bench-validation phase established that:

- The leg's friction-vs-velocity is well captured by a four-parameter Stribeck model (R² = 0.983):
  ```
  τ_friction(ω) = τ_c + (τ_s − τ_c)·exp(−(|ω|/ω_s)²) + b·|ω|
  ```
- Adding `torque_ff = τ_friction(v_cmd) × Kt` to ODrive's `set_input_pos.torque_ff` reduces motion-onset latency by ~2× on the bench (81 → 32 ms).
- A "stiction-boost" — applying full τ_s for `|v_cmd| < 0.20 rev/s` — closes a residual ~12 % FF undersize at low velocities and yields another 1.7–1.9× latency improvement.
- Sending the trajectory's instantaneous velocity in `set_input_pos.vel_ff` is the largest single FF improvement on the bench: 90 %-target tracking lag drops ~5× (31 → 6 ms) on top of friction FF, contributing the dominant share of the full FF stack's ~9× improvement (53 → 6 ms total).

The bench is single-leg, single-pose, and runs `pos_step` (a trapezoid). The platform is six legs, full pose space, and runs Hermite-interpolated MPC trajectories. Bench parameters are the *starting estimate*; per-leg refinement on platform is the protocol described in §4.

Goal: **integrate the FF stack into `motor_guard.py` and `hardware_config.yaml` such that motion-onset dead-time on the platform's existing 7-move test battery drops from the current ~100–200 ms baseline to below the §5 acceptance thresholds.**

---

## 1. The pipeline as it exists today (verified, not assumed)

Tracing the existing code at the time of this plan (read by grep, not memory):

| Location | What's there |
|---|---|
| [motor_guard.py:849-850](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L849-L850) | `msg['leg_vel'] = self._commanded_vel_ff_rps.tolist()` and `msg['leg_torques'] = self._commanded_torque_ff_Nm.tolist()` — the IPC envelope is already shaped to carry both feedforward channels. |
| [motor_guard.py:784](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L784) | `self._commanded_torque_ff_Nm = self._mpc_base_torque_Nm.copy()` — `_mpc_base_torque_Nm` is the per-tick torque_ff carried via the MPC IPC envelope. On hardware paths today this is **zero in operational runs** ([controller/hardware_plant.py:126-127, 442-443](../../controller/hardware_plant.py): `_ff_torque_buf` starts as `np.zeros(6)` with `_has_ff_torque = False`, and only `set_pose()` flips the flag on — the MPC hardware path uses `command()` which keeps it off). The friction FF must still **add** to this rather than replace it, so a future MPC version that populates torque_ff (e.g. gravity compensation) stacks correctly. |
| [motor_guard.py:700-782](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L700-L782) | `self._commanded_vel_ff_rps` is computed via Hermite C¹ interpolation when the MPC stream is fresh (lines 703-741), with cubic Taylor extrapolation (743-754) and a coast-down ramp-to-zero (755-782) as fallbacks. The motor_guard's interpolator already produces a per-tick cmd_vel via the Hermite-derivative basis. |
| [motion_bridge_node.py:148-149](../../ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py#L148-L149) | `velocities = telem.get('leg_vel'); torques = telem.get('leg_torques')` — bridge ingests both fields. |
| [can_node.py:1023](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1023) | `self._send_position_target(axis_id, positions[axis_id], vel_ff=vel_ffs[axis_id], torque_ff=torque_ffs[axis_id])` — vel_ff and torque_ff are already routed into the position-target call. |
| [can_node.py:703-723](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L703-L723) | `_send_position_target` quantises both feedforwards into int16 (`INPUT_SCALE_LEG_VEL = 1000`, `INPUT_SCALE_LEG_TOR = 10000`), clamps to int16 range, and emits `encode_set_input_pos`. |

**The FF plumbing is already complete end-to-end.** The integration is computational, not architectural — friction FF goes into `_commanded_torque_ff_Nm` (additive to whatever MPC put there), and vel_ff is *already* being computed and shipped (so we need to verify it's not zero in operational runs).

This is significantly less invasive than I'd worried about. It's "compute Stribeck, add to the torque_ff that's already being sent" — three things to add: a friction model, per-leg parameters, a feature flag.

### One sanity check the plan must perform on day one

Before writing any FF code, verify on the platform that motor_guard is *actually* populating `vel_ff` with non-zero values today (not zero out of habit). The bench result showed vel_ff alone produced the largest single improvement of any change. If today's platform runs are sending `vel_ff = 0` (a latent regression), enabling vel_ff alone would already be a substantive improvement before any friction FF lands. **This is a pre-implementation diagnostic step, not a hypothesis to debate.**

---

## 2. Contract

The friction FF is implemented as a **contract**, in the sense the project's engineering philosophy uses (one normative document, one canonical enforcement point, one test). The contract:

> **`motor_guard._commanded_torque_ff_Nm` is the sum of the MPC-supplied base torque and a per-leg Stribeck friction feedforward computed from `_commanded_vel_ff_rps`. The friction FF is gated by a single config flag (`feedforward.enable_friction_ff`) and parameterised by a per-leg parameter block in `hardware_config.yaml`. There is no other code path that writes friction-derived torque to the ODrive.**

Three parts, one each:

1. **Normative:** §3 of this document, plus per-leg parameter docstrings in `hardware_config.yaml`.
2. **Enforcement:** a single function `motor_guard._compute_friction_ff_Nm(self) -> np.ndarray` that produces the per-leg Nm vector. Called once per interpolation tick. No friction computation anywhere else.
3. **Test:** `tests/motion/test_motor_guard_friction_ff.py` — unit tests that the function (a) is monotonically signed in v_cmd, (b) produces ≈τ_s at v_cmd=0+ when boost is on, (c) tapers to ≈τ_c at high v, (d) returns zero when the feature flag is off, (e) clamps to int16-encodable range when summed with `_mpc_base_torque_Nm`.

---

## 3. Specification

### 3.1 Per-leg parameters in `hardware_config.yaml`

Add a new section under `jugglebot_odrive_defaults`:

```yaml
# Friction feedforward — Stribeck model τ_friction(ω) = τ_c + (τ_s − τ_c)·exp(−(|ω|/ω_s)²) + b·|ω|
# Per-leg values; bench-fit parameters are the starting estimate, refined
# per-leg via the diagnose detector following plans/active/friction-ff-motor-guard-integration.md §4.
friction_ff:
  enabled: false                              # global feature flag
  stiction_boost_threshold_rps: 0.20          # |v_cmd| below this gets full τ_s instead of Stribeck taper
  ff_sign:        [-1, -1, -1, -1, -1, -1]    # per-leg sign convention (motor wiring)
  tau_c_A:        [1.094, 1.094, 1.094, 1.094, 1.094, 1.094]  # kinetic Coulomb floor
  tau_s_A:        [1.953, 1.953, 1.953, 1.953, 1.953, 1.953]  # stiction peak
  omega_s_rps:    [0.251, 0.251, 0.251, 0.251, 0.251, 0.251]  # Stribeck breakaway scale
  b_A_per_rps:    [0.0173, 0.0173, 0.0173, 0.0173, 0.0173, 0.0173]  # viscous slope
  load_offset_A:  [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]  # constant load — bench had 0.173 from cable preload; platform legs each see different gravity, populated by per-leg tuning
```

`config/generate_config.py` regenerates the Python/C++/JS constants. The friction-FF block lives in `hardware_config.py` exposed as `FRICTION_FF_*` attributes.

**Why per-leg from day one:** every previous per-leg tuning pass (`leg-gain-tuning-methodology.md`) has shown leg-to-leg variation. Starting with uniform values is fine, but the schema must support per-leg from the first commit so that the diagnose-driven refinement protocol (§4) can land per-leg adjustments without a config schema migration.

**Default `enabled: false`:** the FF lands behind a flag so the platform integrates safely. The first operational test enables it explicitly per-session, validating before persistent deployment.

### 3.2 The `_compute_friction_ff_Nm` function

Single canonical implementation in motor_guard:

```python
def _compute_friction_ff_Nm(self) -> np.ndarray:
    """Per-leg Stribeck friction torque feedforward, in Nm.

    Returns zeros if hw.FRICTION_FF_ENABLED is False.  Otherwise computes
    τ_friction(|v_cmd|) per leg from self._commanded_vel_ff_rps using the
    Stribeck four-parameter model, applies the stiction-boost band, signs
    the result by sign(v_cmd) × per-leg ff_sign, and adds the per-leg
    constant load offset.

    Result is written into self._friction_ff_buf (preallocated
    np.zeros(6) in __init__).  Vectorised intermediates are allowed to
    allocate small length-6 temporaries — motor_guard's 500 Hz loop
    isn't formally bound by the MPC hot-loop zero-allocation contract,
    so the discipline here is "no per-tick growth, no large allocations",
    not strict zero-alloc.
    """
    if not hw.FRICTION_FF_ENABLED:
        self._friction_ff_buf.fill(0.0)
        return self._friction_ff_buf

    v = self._commanded_vel_ff_rps           # (6,) rps, signed
    av = np.abs(v)                            # (6,) magnitude
    sgn = np.sign(v)                          # (6,) ±1, 0 if v == 0

    # Stiction-boost band: τ_s instead of Stribeck-tapered for |v| < threshold.
    # Both branches share +b·|v| so there's no discontinuity from the viscous term.
    in_boost = av < hw.FRICTION_FF_BOOST_THRESHOLD_RPS
    stribeck = (hw.FRICTION_FF_TAU_C
                + (hw.FRICTION_FF_TAU_S - hw.FRICTION_FF_TAU_C)
                  * np.exp(-(av / hw.FRICTION_FF_OMEGA_S) ** 2)
                + hw.FRICTION_FF_B_VISC * av)
    boosted = hw.FRICTION_FF_TAU_S + hw.FRICTION_FF_B_VISC * av
    iq_friction = np.where(in_boost & (av >= 1e-4), boosted, stribeck)

    # At exactly v=0, no friction term — only the constant load
    iq_friction = np.where(av < 1e-4, 0.0, iq_friction)

    # Sign convention: per-leg ff_sign × motion direction × friction magnitude
    iq_total = -sgn * iq_friction + hw.FRICTION_FF_LOAD_OFFSET_A
    self._friction_ff_buf[:] = (iq_total
                                * hw.FRICTION_FF_SIGN
                                * hw.MOTOR_KT_NM_PER_A)
    return self._friction_ff_buf
```

Invariants:
- Vectorised over all 6 legs (no per-leg loop).
- The result buffer is preallocated; the final write uses `self._friction_ff_buf[:] = ...` so the caller's reference stays stable.
- Length-6 temporaries from `np.abs`, `np.sign`, `np.where`, `np.exp` are acceptable — motor_guard at 500 Hz has headroom that the MPC 40 Hz hot loop does not. If profiling shows allocation pressure, swap to in-place ops on scratch buffers as a follow-up.
- The function is pure: no state mutation beyond the result buffer.
- The flag check is first so disabled mode is one branch + memset.

### 3.3 The single integration point

In `motor_guard.py` where `_commanded_torque_ff_Nm` is currently set:

```python
# OLD:
# self._commanded_torque_ff_Nm = self._mpc_base_torque_Nm.copy()

# NEW:
np.add(self._mpc_base_torque_Nm,
       self._compute_friction_ff_Nm(),
       out=self._commanded_torque_ff_Nm)
```

That is the *only* place friction FF enters the torque_ff signal. Any other path is a contract violation.

### 3.4 vel_ff: verify, don't reimplement

`_commanded_vel_ff_rps` is already populated by motor_guard. Step one of integration is to **verify on platform that this is non-zero in operational runs** (e.g. by sniffing the IPC stream during a 7-move battery). If it's always zero (latent regression), find and fix the gap before doing anything else — the bench result says vel_ff is the single largest improvement, so getting it operational is higher priority than friction FF itself.

If vel_ff is already non-zero and active, no work needed; friction FF just stacks on top.

### 3.5 The constant `MOTOR_KT_NM_PER_A`

Add to `hardware_config.yaml` under `dynamics`:
```yaml
motor_kt_nm_per_a: 0.0624   # measured from Phase 2 multi-weight bench test (R²=0.994)
```

This already exists in `tests/hardware/single_leg_test.py:92` as a measured constant; the integration just promotes it to config so it's a single source of truth.

---

## 4. Per-leg tuning protocol (post-integration)

After the integration lands with uniform bench-derived parameters, per-leg refinement uses the diagnose detector that already exists. This is a Level-1-style outlier-bisection protocol mirroring `plans/active/leg-gain-tuning-methodology.md`'s §"Empirical A/B on the outlier".

### 4.1 Procedure

1. Run the existing 7-move test battery from the leg-gain-tuning methodology with the FF flag ON and uniform bench params.
2. Run `python sim/analysis/diagnose.py temp/logs/mpc_<ts>.csv --json`.
3. Read the `motion_onset.per_leg_summary` block: per-leg `median_latency_ms` and `max_latency_ms`.
4. **Compute the per-leg latency ratio** to the median across legs. Any leg with ratio > 1.5 is an outlier.
5. For each outlier, **bisect `tau_s_A` for that leg** by ±20 % and re-run the battery. If outlier ratio drops, accept the change. If it rises, revert and try `omega_s_rps` instead.
6. Continue until no leg exceeds 1.3× the best leg's median latency, or until further changes don't improve the worst case.

### 4.2 In-experiment controls

Per the lesson from `2026-04-19-leg1-pose-dependent-hold-twitch.md`:
- Always change one leg's parameter at a time and leave one nearby leg untouched as a control.
- Watch for cross-leg interactions (a tuned leg's faster response may shift the platform's pose-error allocation, changing other legs' apparent latency).
- Document each iteration in a single logbook entry as a chronological table of `(leg, param, before, after, latency_before, latency_after)`.

### 4.3 What to do if the protocol doesn't converge

Three escalation paths, in order:

1. **The bench fit is wrong for this leg.** Re-fit by running a brief platform-leg friction sweep (the cogging_bench_test.py protocol, but on a platform leg with the leg detached or constrained — TBD with the user). Replace that leg's bench-default params with the platform-leg-specific fit.
2. **The friction model itself is missing a term.** Most likely missing: a position-dependent gravity term (per-leg gravity-from-kinematics). If this is the case, the load_offset_A column should be replaced with a kinematics call.
3. **The dead-time isn't friction-dominated.** If the FF doesn't help on a particular leg even after refit, revisit the cogging hypothesis or the inertia floor: an acceleration-FF refinement (`τ_ff += J_eff × dv/dt`) is on the table.

---

## 5. Acceptance criteria

The integration ships when **all** of the following hold on the existing 7-move test battery (from `plans/active/leg-gain-tuning-methodology.md`):

| Metric | Target | Stretch |
|---|---|---|
| Median per-session `motion_onset_latency_ms` | ≤ 60 ms | ≤ 40 ms |
| Max per-session `motion_onset_latency_ms` | ≤ 100 ms | ≤ 60 ms |
| Per-leg latency asymmetry (max / min) | ≤ 1.5 | ≤ 1.3 |
| First-tick leap magnitude (post-onset) | ≤ 1.0 mm | ≤ 0.5 mm |
| Hold-phase per-leg `act_std` (regression check) | unchanged from current ~3–11 µm | unchanged |
| Move 5 (long Z-up) tracking RMS | unchanged from current baseline | improved |
| Move 6/7 (extreme poses) hold-phase asymmetry | unchanged or improved from current 8.5× | ≤ 5× |

The hold-phase regression check is critical: a friction FF that improves motion-onset at the cost of hold-phase quality reverses the gain-tuning chapter's hard-won results. The `act_std` floor of ~3–11 µm is the published outcome from `2026-04-20`; any drift above 15 µm is a regression and the FF must be backed out or tuned down.

The asymmetry metric guards against per-leg parameter drift; the gain-tuning chapter's worst-case asymmetry of 8.5× is the floor to clear.

---

## 6. Open questions

These need user guidance before implementation:

1. **Is motor_guard already populating vel_ff in operational runs?** Verify before doing anything else — see §3.4.
   - **If zero (latent regression)**: PR 1 stays as planned (Kt promotion + vel_ff fix). The vel_ff fix alone may be the largest single platform improvement available and could ship before any friction-FF work.
   - **If non-zero**: PR 1 collapses to just the Kt promotion, and we proceed directly to PR 2 (friction-FF schema + `_compute_friction_ff_Nm`).

2. **Should friction FF land as a single PR (FF + per-leg config + tuning protocol + tests) or staged?** I'd recommend staged:
   - PR 1: Promote `motor_kt_nm_per_a` from `single_leg_test.py` to `hardware_config.yaml`. Verify vel_ff is on. (Trivial; 1 file changed; no behaviour change.)
   - PR 2: Add the friction-FF schema to `hardware_config.yaml` (defaults, `enabled: false`). Add `_compute_friction_ff_Nm` and the integration point. Add unit tests. (Behaviour-neutral until flag flips.)
   - PR 3: Flip `enabled: true` after on-platform validation. Document in a logbook entry per the in-experiment-control protocol.
   - PR 4 (later): Per-leg parameter refinement based on §4 protocol.

3. **Per-leg parameter granularity at first deployment:** ship uniform bench-derived params, OR run a brief platform-leg friction characterisation first (single leg, isolated CAN, brake-resistor protocol — same as bench but on a platform leg)? The first option is faster; the second is more rigorous.

4. **Hold-phase behaviour of the friction FF.** At v_cmd = 0 (rotor at hold), the FF returns only the constant load_offset (per §3.2, friction term zeroed). On the bench this produced clean hold (no integrator wind-up fight). On the platform, where there's gravity per pose, the load_offset value would need to come from kinematics, not a constant. **First-platform deployment can hardwire load_offset_A = 0 and let the existing per-leg gain-tuning integrator handle gravity** (which is what's working today). Per-leg gravity-from-kinematics is a Phase-2 refinement.

5. **Sign convention on platform legs.** Bench is `ff_sign = -1`. Platform should be the same per-leg if motor wiring is consistent across all six. If can_node's leg-inversion (the `negate, scale by appropriate value` step in §"Critical Conventions" of CLAUDE.md) is applied at a different stage than I think, the sign may need to be `+1` on platform. **Verify on day one** by sniffing iq_meas vs vel_meas on a single platform leg under closed-loop velocity command — same diagnostic the bench used.

6. **Do we want runtime FF disable?** The flag in YAML requires a config regen + reboot to flip. A per-session command-line override (e.g. `run_mpc.py --no-friction-ff`) would let on-platform A/B testing happen without config churn during the tuning phase. **I'd recommend yes** — it's a 5-line argparse flag, and the gain-tuning chapter showed how valuable in-experiment controls are.

---

## 7. Test plan

### 7.1 Unit tests (`tests/motion/test_motor_guard_friction_ff.py`)

- `test_friction_ff_disabled_returns_zero`: with flag off, function returns `np.zeros(6)`.
- `test_friction_ff_zero_at_v_zero`: at `v_cmd = 0`, returns just `load_offset × Kt × ff_sign` (no friction term).
- `test_friction_ff_signs_with_velocity`: for each leg, FF for `v_cmd > 0` has same sign as FF for `−v_cmd` × `−1`, accounting for ff_sign.
- `test_friction_ff_taper_to_coulomb`: at `v_cmd = 5 × ω_s`, FF magnitude is within 5 % of `τ_c`.
- `test_friction_ff_boost_at_low_v`: at `v_cmd = 0.5 × boost_threshold` with boost on, FF magnitude is within 5 % of `τ_s`.
- `test_friction_ff_continuity_at_boost_threshold`: discontinuity at `v_cmd = boost_threshold` is bounded by the predictable Stribeck-vs-τ_s gap (sanity, not a tight assert).
- `test_friction_ff_per_leg_independence`: setting different params per leg produces leg-independent FF outputs.
- `test_friction_ff_buffer_identity`: `_friction_ff_buf` Python `id()` is stable across 1000 calls (i.e. no rebinding of the result buffer; small per-call temporaries are allowed).
- `test_friction_ff_int16_clamp`: with extreme params (τ_s = 5 A), the resulting torque_ff sum stays within int16 encode range.

### 7.2 Integration tests (existing tests pass)

- `pytest tests/ros/test_can_node.py -v` — must still pass.
- `pytest tests/motion/test_motor_guard.py -v` — must still pass.
- Full `pytest tests/ -v` — full suite green.

### 7.3 Hardware acceptance (the 7-move battery)

After PR 2 lands and before flipping the flag (PR 3), run the 7-move battery at uniform bench params with `enabled: false` and again with `enabled: true`. Both runs go through `diagnose.py --json` and the per-leg `motion_onset` block is compared. The flag-on run must satisfy §5's targets.

### 7.4 In-experiment control during platform tuning

Per §4.2, when bisecting per-leg `tau_s_A`, always leave one nearby leg's params unchanged. Document in a logbook entry as a chronological table.

---

## 8. Reversibility

The integration is reversible at three levels:

1. **Per-session reversal** via `run_mpc.py --no-friction-ff` (per §6.6). Zero config churn.
2. **Persistent reversal** via `feedforward.enable_friction_ff: false` in YAML. Config regen + ros2 rebuild.
3. **Code reversal** via reverting the integration commit. The single integration point in motor_guard makes this a one-file revert.

The `_compute_friction_ff_Nm` function is independent of any other motor_guard logic, so reverting it doesn't touch anything else.

---

## 9. Sequencing

| Step | Owner | Dependency | Done when |
|---|---|---|---|
| Pre-flight: verify vel_ff is non-zero on platform | User on platform | none | confirmed via IPC sniff or rosbag inspection |
| PR 1: Kt promotion + vel_ff verify | claude | pre-flight | PR merged, no behaviour change |
| Plan review: §6 questions resolved with user | user + claude | PR 1 merged | this plan updated, all open questions answered |
| PR 2: Schema + `_compute_friction_ff_Nm` + tests, flag off | claude | plan review | unit tests pass; full pytest green; flag remains off |
| PR 3a: Flag on at uniform bench params, validate | user + claude | PR 2 merged | 7-move battery passes §5 targets at uniform params, OR per-leg tuning required |
| PR 3b (if needed): Per-leg refinement loop | user + claude | PR 3a results | per-leg params committed; logbook entry per §4.2 |
| Logbook closeout | claude | PR 3 merged | platform-validation entry written; this plan archived |

---

## 10. References

- Bench-validation logbook: [logbook/2026-04-27-friction-feedforward-bench-validation.md](../../logbook/2026-04-27-friction-feedforward-bench-validation.md) — model fit, demo results, withdrawn claims, per-leg tuning rationale.
- Original investigation: [plans/active/motion-onset-deadtime-investigation.md](motion-onset-deadtime-investigation.md) — symptom recap, candidate-mechanism enumeration, fix-option ranking.
- Hot-loop zero-allocation contract: [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md) — scoped to the MPC 40 Hz hot loop; motor_guard's 500 Hz loop is not formally bound by it.
- Gain-tuning methodology: [plans/active/leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) — per-leg tuning patterns, in-experiment controls, the 7-move battery referenced in §5.
- Bench scripts and analysers: [tests/hardware/friction_ff_demo.py](../../tests/hardware/friction_ff_demo.py), [tools/friction_study_analyse.py](../../tools/friction_study_analyse.py).
