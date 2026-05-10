"""Phase 5 + Phase 6 enforcement tests for PLANT_INTERFACE_CONTRACT.md P1–P4.

Phase 5 covers the two invariants implementations are expected to honour
from the moment they're constructed:

  * P1 — PlantState aliasing: ``get_state()`` returns the same instance,
    every ndarray field is the same array object across calls.
  * P2 — ``can_reset: bool`` capability flag; ``reset()`` raises
    ``NotImplementedError`` iff not ``can_reset``.

Phase 6 covers the two invariants on ``command()`` and ``control_dt``:

  * P3 — ``command()`` is a trusted-callee boundary; implementations
    MUST NOT silently coerce malformed inputs (NaN, out-of-range).
    Either raise or forward; silent correction is prohibited.
  * P4 — ``control_dt: float`` is a mandatory constructor kwarg
    (default 0.025 s = 40 Hz); time-window thresholds derive from it,
    not hard-coded magic numbers.

Tests are parameterised over both shipping implementations:
  * ``MuJoCoPlant`` (sim)
  * ``HardwarePlant`` (hardware) — via the in-memory ZMQ-mock helper
    at ``tests/sim/_hardware_plant_stub.py``.
"""

from __future__ import annotations

import contextlib

import numpy as np
import pytest

from controller.plant import PlantInterface, PlantState
from plant.mujoco_plant import MuJoCoPlant
from tests.sim._hardware_plant_stub import build_hardware_plant_stub


# ---------------------------------------------------------------------
# Plant fixture — parameterised over MuJoCoPlant + HardwarePlant
# ---------------------------------------------------------------------

@contextlib.contextmanager
def _build_mujoco_plant():
    """Yield a fresh MuJoCoPlant.  No teardown needed — Python GC handles it."""
    yield MuJoCoPlant()


@pytest.fixture(
    params=[
        pytest.param(
            ('mujoco', _build_mujoco_plant, True),
            id='MuJoCoPlant',
        ),
        pytest.param(
            ('hardware', build_hardware_plant_stub, False),
            id='HardwarePlant',
        ),
    ],
)
def plant_with_capability(request):
    """Yield ``(name, plant, expected_can_reset)`` triples.

    ``name`` is a short label for diagnostics.  ``expected_can_reset`` is
    the value the contract requires the implementation to declare.
    """
    name, builder, expected_can_reset = request.param
    with builder() as plant:
        yield name, plant, expected_can_reset


# ---------------------------------------------------------------------
# P1 — PlantState aliasing
# ---------------------------------------------------------------------

# Number of get_state() calls used for the identity invariant tests.
# 100 is generous: the contract claim is "every call returns the same
# instance", so any deviation will surface in the first handful — but
# 100 amortises Python attribute-cache flakes and gives the test
# meaningful tracemalloc evidence in the post-failure report.
_N_CALLS = 100


class TestP1PlantStateAliasing:
    """T-U-P1-1, T-U-P1-2 — instance identity + ndarray-field identity."""

    def test_get_state_returns_same_instance(self, plant_with_capability):
        """T-U-P1-1: ``id(plant.get_state())`` invariant over 100 calls."""
        name, plant, _ = plant_with_capability
        first = plant.get_state()
        first_id = id(first)
        # Confirm the returned object IS a PlantState (the type check is
        # part of the ABC contract — implementations MUST honour the
        # dataclass shape, not just the alias).
        assert isinstance(first, PlantState), (
            f"{name}: get_state() returned {type(first).__name__}, "
            f"expected PlantState"
        )
        for i in range(_N_CALLS):
            s = plant.get_state()
            assert id(s) == first_id, (
                f"{name}: get_state() returned a different instance on "
                f"call {i + 1} (id={id(s)} vs {first_id} on first call)"
            )

    def test_ndarray_fields_are_same_array_objects(self, plant_with_capability):
        """T-U-P1-2: every ndarray field is the same array object across calls.

        The contract claim is identity, not equality — ``id(state.field)``
        MUST be invariant.  This is the property that lets consumers
        rely on ``np.copyto(local_buf, state.field)`` rather than a
        defensive ``.copy()`` per access."""
        name, plant, _ = plant_with_capability
        first = plant.get_state()
        # Snapshot the id() of every ndarray field on the first call.
        ndarray_field_names = [
            'leg_extensions_mm',
            'leg_velocities_mmps',
            'platform_pos_mm',
            'platform_rot',
            'platform_twist',
        ]
        first_ids = {
            field: id(getattr(first, field)) for field in ndarray_field_names
        }
        for call_idx in range(_N_CALLS):
            s = plant.get_state()
            for field in ndarray_field_names:
                got_id = id(getattr(s, field))
                assert got_id == first_ids[field], (
                    f"{name}: state.{field} is a different array object on "
                    f"call {call_idx + 1} (id={got_id} vs {first_ids[field]} "
                    f"on first call) — P1 violation"
                )

    def test_ndarray_fields_remain_ndarrays(self, plant_with_capability):
        """Defensive: every ndarray field MUST stay an ndarray across calls.

        A naive in-place rewrite can accidentally rebind a field to a
        scalar or list (e.g., ``state.platform_pos_mm = pos_m * 1000.0``
        when pos_m is a fresh array).  This test catches that
        regression."""
        name, plant, _ = plant_with_capability
        for call_idx in range(_N_CALLS):
            s = plant.get_state()
            for field in (
                'leg_extensions_mm', 'leg_velocities_mmps',
                'platform_pos_mm', 'platform_rot', 'platform_twist',
            ):
                value = getattr(s, field)
                assert isinstance(value, np.ndarray), (
                    f"{name}: state.{field} on call {call_idx + 1} "
                    f"is {type(value).__name__}, expected ndarray"
                )

    def test_field_values_change_across_calls_when_state_changes(self):
        """A meaningful test that aliasing doesn't freeze field VALUES.

        Aliasing means the SAME array gets new values written into it;
        it does NOT mean the values are stuck.  This test moves the
        plant and asserts the same array object now reflects the new
        state.  Sim-only because hardware needs a live motor guard to
        produce changing motor_pos values."""
        plant = MuJoCoPlant()
        s1 = plant.get_state()
        # Snapshot the pose values (not just the array reference).
        pose_before = s1.platform_pos_mm.copy()
        pose_array_id = id(s1.platform_pos_mm)
        # Move the plant.
        target = np.array([20.0, 10.0, 50.0, 0.0, 0.0, 0.0])
        plant.command(plant.pose_to_extensions(target))
        for _ in range(80):  # ~2 s at 25 ms — enough to settle
            plant.step(0.025)
        s2 = plant.get_state()
        # SAME array object, NEW values.
        assert id(s2.platform_pos_mm) == pose_array_id, (
            "P1 violation: platform_pos_mm rebound to a different array"
        )
        assert not np.allclose(s2.platform_pos_mm, pose_before, atol=0.5), (
            "Sanity: plant should have moved, but platform_pos_mm is "
            "indistinguishable from initial.  Either the plant didn't "
            "move (test setup bug) or the field is frozen (worse bug)."
        )


# ---------------------------------------------------------------------
# P2 — can_reset capability + reset() raise behaviour
# ---------------------------------------------------------------------

class TestP2ResetCapability:
    """T-U-P2-1, T-U-P2-2 — can_reset declaration + reset() behaviour."""

    def test_can_reset_matches_implementation(self, plant_with_capability):
        """T-U-P2-1: ``can_reset`` returns the documented value per impl.

        MuJoCoPlant: True (sim can rewind).
        HardwarePlant: False (homing is owned by the orchestrator)."""
        name, plant, expected = plant_with_capability
        assert plant.can_reset is expected, (
            f"{name}: can_reset returned {plant.can_reset!r}, "
            f"expected {expected!r}"
        )

    def test_reset_behaviour_matches_capability(self, plant_with_capability):
        """T-U-P2-2: ``reset()`` raises ``NotImplementedError`` iff
        ``not can_reset``.  The biconditional is the load-bearing
        property — a plant that claims ``can_reset=True`` must NOT raise
        on a vanilla ``reset()`` call, AND a plant that claims
        ``can_reset=False`` MUST raise."""
        name, plant, expected = plant_with_capability
        if expected:
            # can_reset=True: reset() must succeed.
            plant.reset()  # no exception expected
        else:
            # can_reset=False: reset() must raise NotImplementedError.
            with pytest.raises(NotImplementedError):
                plant.reset()

    def test_reset_with_pose_behaviour_matches_capability(self, plant_with_capability):
        """Same biconditional with the optional ``pose_6dof`` argument.

        The ``pose_6dof`` overload follows the same contract — capability
        is a property of the implementation, not of the call shape."""
        name, plant, expected = plant_with_capability
        target_pose = np.array([0.0, 0.0, 40.0, 0.0, 0.0, 0.0])
        if expected:
            plant.reset(target_pose)  # no exception expected
        else:
            with pytest.raises(NotImplementedError):
                plant.reset(target_pose)

    def test_hardware_reset_error_message_names_alternative(self):
        """The contract requires P2's NotImplementedError message to
        name the implementation and point at the correct lifecycle API.
        Audit the error text directly against the prose contract."""
        with build_hardware_plant_stub() as plant:
            with pytest.raises(NotImplementedError) as excinfo:
                plant.reset()
            msg = str(excinfo.value)
            assert 'HardwarePlant' in msg
            assert 'orchestrator' in msg
            assert 'can_reset' in msg


# ---------------------------------------------------------------------
# Class-level invariants — the ABC enforces declaration
# ---------------------------------------------------------------------

class TestPlantInterfaceABC:
    """Static checks on the ABC itself (don't need a live plant)."""

    def test_can_reset_is_abstract_on_the_abc(self):
        """Without an implementation declaring ``can_reset``, instantiating
        a subclass MUST fail with ``TypeError`` from Python's ABCMeta.

        This is the structural enforcement that prevents a future
        ``PlantInterface`` implementation from silently inheriting an
        ambiguous default.  See PLANT_INTERFACE_CONTRACT.md P2."""

        class _MissingCapability(PlantInterface):
            # Deliberately omit can_reset to verify ABCMeta rejects.
            def command(self, leg_extensions_mm, vel_mm_s=None,
                        cmd_next_mm=None, cmd_next2_mm=None):
                pass

            def get_state(self):
                pass

            def step(self, dt):
                pass

            def reset(self, pose_6dof=None):
                pass

        with pytest.raises(TypeError, match='can_reset'):
            _MissingCapability()

    def test_implementations_register_as_subclasses(self):
        """Both shipped implementations are formal subclasses of
        ``PlantInterface``.  This is what enables the parameterised
        contract test — it guarantees the API surface is uniform."""
        with build_hardware_plant_stub() as hw:
            assert isinstance(hw, PlantInterface)
        assert issubclass(MuJoCoPlant, PlantInterface)

    def test_control_dt_is_abstract_on_the_abc(self):
        """Without an implementation declaring ``control_dt``,
        instantiating a subclass MUST fail with ``TypeError`` from
        Python's ABCMeta.  See PLANT_INTERFACE_CONTRACT.md P4."""

        class _MissingControlDt(PlantInterface):
            # Declares can_reset but omits control_dt.
            can_reset = True

            def command(self, leg_extensions_mm, vel_mm_s=None,
                        cmd_next_mm=None, cmd_next2_mm=None):
                pass

            def get_state(self):
                pass

            def step(self, dt):
                pass

            def reset(self, pose_6dof=None):
                pass

        with pytest.raises(TypeError, match='control_dt'):
            _MissingControlDt()


# ---------------------------------------------------------------------
# P3 — Trusted-callee command boundary
# ---------------------------------------------------------------------

# Helper: read the post-command "what got forwarded" snapshot from the
# implementation under test.  Both implementations expose the forwarded
# values via different surfaces; abstracting the read here keeps the
# test bodies symmetric.

def _read_forwarded_extensions(name: str, plant) -> np.ndarray:
    """Return the (6,) extensions that ``command()`` forwarded.

    For ``MuJoCoPlant``, this is ``self._data.ctrl[:6]`` translated
    back to extensions via ``_slide_to_extensions``.
    For ``HardwarePlant``, this is the pre-allocated ``_cmd_ext_buf``
    that ``command()`` overwrites via ``np.copyto`` at the head of
    the body; see ``hardware_plant.py``.  The buffer is not the
    message but the canonical "what got forwarded" surface."""
    if name == 'mujoco':
        slide_m = plant._data.ctrl[:6].copy()
        return plant._slide_to_extensions(slide_m)
    elif name == 'hardware':
        return plant._cmd_ext_buf.copy()
    else:
        raise AssertionError(f"unknown plant name {name!r}")


class TestP3TrustedCallee:
    """T-U-P3-1 — command() does not silently coerce malformed inputs.

    The contract permits raise (defensive) or pass-through (trusted);
    silent correction (clip, NaN→default, reshape) is prohibited.
    Both shipped implementations take the pass-through path post-
    Phase-6.
    """

    def test_nan_input_is_not_silently_zeroed(self, plant_with_capability):
        """``plant.command(np.full(6, NaN))`` MUST either raise or
        forward NaN to the layer below.  A silent zero-replacement
        would hide the upstream bug that produced NaN in the first
        place."""
        name, plant, _ = plant_with_capability
        nan_ext = np.full(6, np.nan)
        try:
            plant.command(nan_ext)
        except (ValueError, RuntimeError):
            # Defensive raise is P3-compliant — and stops the test.
            return
        # No raise — the implementation took the pass-through path.
        # Verify NaN actually reached the layer below (no silent
        # zero-replacement).
        forwarded = _read_forwarded_extensions(name, plant)
        assert np.all(np.isnan(forwarded)), (
            f"{name}: command(NaN) did not raise AND did not forward "
            f"NaN — implementation silently coerced "
            f"(forwarded={forwarded}).  P3 violation."
        )

    def test_out_of_range_input_is_not_silently_clipped(
            self, plant_with_capability):
        """An out-of-range extension (e.g., 10 000 mm — 35× full
        stroke) MUST either raise or forward the value to the layer
        below.  A silent ``np.clip`` to the actuator envelope is the
        canonical P3 violation that motivated the contract."""
        name, plant, _ = plant_with_capability
        big_ext = np.full(6, 10_000.0)
        try:
            plant.command(big_ext)
        except (ValueError, RuntimeError):
            return
        forwarded = _read_forwarded_extensions(name, plant)
        # If the implementation clipped to the actuator envelope,
        # forwarded values would all equal the upper limit.  The
        # forwarded values must EITHER all equal 10 000 (pure pass-
        # through, what we expect) OR be larger than any plausible
        # clip target (e.g., > stroke + margin).  Anything in the
        # range [0, stroke] is silent clipping — fail.
        assert np.all(forwarded > 1000.0), (
            f"{name}: command(10000) forwarded {forwarded} — looks "
            f"like silent clipping to the actuator envelope.  P3 "
            f"violation."
        )

    def test_negative_input_is_not_silently_clipped(
            self, plant_with_capability):
        """Same property in the negative direction: a wildly negative
        extension MUST either raise or forward.  Silent clip-to-zero
        is the failure mode the test catches."""
        name, plant, _ = plant_with_capability
        neg_ext = np.full(6, -500.0)
        try:
            plant.command(neg_ext)
        except (ValueError, RuntimeError):
            return
        forwarded = _read_forwarded_extensions(name, plant)
        # Pure pass-through: forwarded == -500.  Silent clip-to-zero:
        # forwarded == 0.  Anything in [0, ε] is the clipping signal.
        assert np.all(forwarded < -100.0), (
            f"{name}: command(-500) forwarded {forwarded} — looks "
            f"like silent clipping to zero.  P3 violation."
        )


# ---------------------------------------------------------------------
# P4 — control_dt awareness
# ---------------------------------------------------------------------

class TestP4ControlDtAwareness:
    """T-U-P4-1, T-U-P4-2 — control_dt parameterisation."""

    def test_default_control_dt_matches_40hz(self, plant_with_capability):
        """Both implementations default to control_dt = 0.025 (40 Hz)
        — the production operating point.  Constructing without the
        kwarg yields this value."""
        name, plant, _ = plant_with_capability
        assert plant.control_dt == pytest.approx(0.025), (
            f"{name}: default control_dt = {plant.control_dt}, "
            f"expected 0.025"
        )

    def test_mujoco_plant_explicit_control_dt(self):
        """``MuJoCoPlant(control_dt=...)`` echoes the constructor arg."""
        plant = MuJoCoPlant(control_dt=0.05)
        assert plant.control_dt == pytest.approx(0.05)

    def test_hardware_plant_explicit_control_dt(self):
        """``HardwarePlant(control_dt=...)`` echoes the constructor arg."""
        with build_hardware_plant_stub(control_dt=0.05) as plant:
            assert plant.control_dt == pytest.approx(0.05)

    def test_hardware_plant_staleness_thresholds_scale_with_control_dt(self):
        """T-U-P4-1: ``HardwarePlant(control_dt=0.05)`` produces
        staleness thresholds 2× the ``control_dt=0.025`` values.

        At 40 Hz (control_dt=0.025): warn=0.075, hard=0.125, estop=0.5.
        At 20 Hz (control_dt=0.050): warn=0.150, hard=0.250, estop=1.0.

        The biconditional is the key property — every threshold
        scales linearly, so a regression that hard-codes any one of
        them trips the test."""
        with build_hardware_plant_stub(control_dt=0.025) as plant_40hz:
            warn_40 = plant_40hz._telem_stale_warn_s
            hard_40 = plant_40hz._telem_stale_hard_s
            estop_40 = plant_40hz._telem_stale_estop_s
        with build_hardware_plant_stub(control_dt=0.05) as plant_20hz:
            warn_20 = plant_20hz._telem_stale_warn_s
            hard_20 = plant_20hz._telem_stale_hard_s
            estop_20 = plant_20hz._telem_stale_estop_s
        # Linear scaling (within float tolerance).
        assert warn_20 == pytest.approx(2.0 * warn_40)
        assert hard_20 == pytest.approx(2.0 * hard_40)
        assert estop_20 == pytest.approx(2.0 * estop_40)

    def test_hardware_plant_default_thresholds_match_pre_phase6_constants(
            self):
        """Regression guard: at the default ``control_dt=0.025`` the
        derived thresholds reproduce the pre-Phase-6 magic numbers
        (0.075 / 0.125 / 0.5).  This pins behaviour for the production
        operating point — a refactor that changes the multipliers
        without updating this test trips it."""
        with build_hardware_plant_stub(control_dt=0.025) as plant:
            assert plant._telem_stale_warn_s == pytest.approx(0.075)
            assert plant._telem_stale_hard_s == pytest.approx(0.125)
            assert plant._telem_stale_estop_s == pytest.approx(0.5)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
