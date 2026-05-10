"""Phase 5 enforcement tests for PLANT_INTERFACE_CONTRACT.md P1, P2.

These cover the two invariants implementations are expected to honour
from the moment they're constructed:

  * P1 — PlantState aliasing: ``get_state()`` returns the same instance,
    every ndarray field is the same array object across calls.
  * P2 — ``can_reset: bool`` capability flag; ``reset()`` raises
    ``NotImplementedError`` iff not ``can_reset``.

Tests are parameterised over both shipping implementations:
  * ``MuJoCoPlant`` (sim)
  * ``HardwarePlant`` (hardware) — via the in-memory ZMQ-mock helper
    at ``tests/sim/_hardware_plant_stub.py``.

Phase 6 will extend this file with P3 (trusted-callee command) and P4
(``control_dt`` awareness) tests.
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


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
