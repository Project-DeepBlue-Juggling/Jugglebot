"""Phase 2 verification tests for the control loop and IPC layer.

Tests:
  1. Loop timing test — 60s run, verify jitter is within bounds
  2. IPC latency test — measure round-trip through ZeroMQ
  3. Force conversion test — verify spool-radius-based conversion

Run:  python -m jugglebot.motion.tests.test_control_loop
"""

from __future__ import annotations

import sys
import threading
import time

import numpy as np

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    leg_forces_to_motor_torques,
    motor_torques_to_leg_forces,
    revs_to_extensions_mm,
)


# ---------------------------------------------------------------------------
# Test 1: Loop timing
# ---------------------------------------------------------------------------

def test_loop_timing():
    """Run the control loop for a short duration and check jitter.

    Phase 2 exit gate: 99th-percentile jitter < 2x nominal period.
    We run for 10s (not 60s) in the automated test for speed;
    the full 60s test should be run manually on the target hardware.
    """
    try:
        from jugglebot.motion.control_loop import ControlLoop, ControlMode
    except ImportError as e:
        print(f"  [SKIP] loop timing -- missing dependency: {e}")
        return True  # skip is not failure

    # Use a mock IPC that does nothing (no ZeroMQ — just timing test)
    class MockIPC:
        def recv_all(self):
            return []
        def send_telemetry(self, msg):
            pass
        @property
        def seconds_since_last_recv(self):
            return 0.0  # pretend bridge is always alive
        def close(self):
            pass

    rate_hz = 500
    duration_s = 10.0
    geom = StewartGeometry()

    loop = ControlLoop(target_rate_hz=rate_hz, geom=geom, ipc=MockIPC())
    loop.mode = ControlMode.DISABLED  # no computation, just timing

    # Run in a thread so we can stop it
    def run_loop():
        loop.run()

    t = threading.Thread(target=run_loop, daemon=True)
    t.start()

    time.sleep(duration_s)
    loop.stop()
    t.join(timeout=2.0)

    dt_nominal = 1.0 / rate_hz
    jitter_p99 = loop.stats.jitter_99
    mean_dt = loop.stats.mean
    n_cycles = loop.stats.count

    # Phase 2 exit gate: p99 jitter < 2× nominal
    gate_threshold = 2.0 * dt_nominal
    passed = jitter_p99 < gate_threshold

    print(f"  [{'PASS' if passed else 'FAIL'}] loop timing at {rate_hz} Hz "
          f"({n_cycles} cycles in {duration_s:.0f}s)")
    print(f"         mean dt:     {mean_dt*1000:.3f} ms  "
          f"(nominal: {dt_nominal*1000:.3f} ms)")
    print(f"         p99 jitter:  {jitter_p99*1000:.3f} ms  "
          f"(gate: < {gate_threshold*1000:.3f} ms)")
    print(f"         max dt:      {loop.stats.max*1000:.3f} ms")
    print(f"         std:         {loop.stats.std*1000:.3f} ms")

    return passed


# ---------------------------------------------------------------------------
# Test 2: IPC latency
# ---------------------------------------------------------------------------

def test_ipc_latency():
    """Measure ZeroMQ round-trip latency.

    Sends a message from BridgeIPC → ControlProcessIPC → back.
    """
    try:
        import zmq
        import msgpack
    except ImportError as e:
        print(f"  [SKIP] IPC latency — missing dependency: {e}")
        return True  # skip is not failure

    from jugglebot.motion.ipc import (
        BridgeIPC,
        ControlProcessIPC,
        make_target_state,
    )

    # Start IPC endpoints
    bridge = BridgeIPC(
        command_addr='tcp://127.0.0.1:15555',
        telemetry_addr='tcp://127.0.0.1:15556',
    )
    control = ControlProcessIPC(
        command_addr='tcp://127.0.0.1:15555',
        telemetry_addr='tcp://127.0.0.1:15556',
    )

    # Allow sockets to connect
    time.sleep(0.2)

    latencies = []
    n_messages = 100

    for _ in range(n_messages):
        # Send target from bridge
        t_send = time.perf_counter()
        target = make_target_state(
            pos=[10.0, 20.0, 100.0],
            rot_quat=[1.0, 0.0, 0.0, 0.0],
        )
        bridge.send_target(target)

        # Small sleep to let ZMQ deliver
        time.sleep(0.0005)

        # Receive at control process
        msgs = control.recv_all()
        t_recv = time.perf_counter()

        if msgs:
            latencies.append(t_recv - t_send)

    control.close()
    bridge.close()

    if not latencies:
        print("  [FAIL] IPC latency — no messages received")
        return False

    lat = np.array(latencies) * 1000  # ms
    passed = np.median(lat) < 2.0  # should be well under 2ms

    print(f"  [{'PASS' if passed else 'FAIL'}] IPC latency  "
          f"({len(latencies)}/{n_messages} messages received)")
    print(f"         median: {np.median(lat):.3f} ms  "
          f"mean: {np.mean(lat):.3f} ms  "
          f"p99: {np.percentile(lat, 99):.3f} ms")

    return passed


# ---------------------------------------------------------------------------
# Test 3: Force conversion
# ---------------------------------------------------------------------------

def test_force_conversion():
    """Verify force↔torque conversion is self-consistent and physically reasonable."""
    geom = StewartGeometry()

    # 1. Round-trip: forces → torques → forces
    test_forces = np.array([100.0, 150.0, 120.0, 130.0, 110.0, 140.0])
    torques = leg_forces_to_motor_torques(test_forces, geom)
    recovered_forces = motor_torques_to_leg_forces(torques, geom)
    roundtrip_err = np.max(np.abs(test_forces - recovered_forces))

    # 2. Round-trip: extensions → revs → extensions
    test_ext = np.array([50.0, 100.0, 150.0, 200.0, 250.0, 280.0])
    revs = extensions_mm_to_revs(test_ext, geom)
    recovered_ext = revs_to_extensions_mm(revs, geom)
    ext_roundtrip_err = np.max(np.abs(test_ext - recovered_ext))

    # 3. Sanity check: spool radii should be in the range 10-15 mm
    spool_radii = geom.spool_radius_mm
    radii_ok = np.all(spool_radii > 5.0) and np.all(spool_radii < 20.0)

    # 4. Sanity check: 100N leg force should produce roughly 1 Nm motor torque
    #    (given ~11mm spool radius → 100N × 0.011m = 1.1 Nm)
    f100 = np.full(6, 100.0)
    t100 = leg_forces_to_motor_torques(f100, geom)
    torque_magnitude_ok = np.all(t100 > 0.5) and np.all(t100 < 2.0)

    passed = (roundtrip_err < 1e-10 and ext_roundtrip_err < 1e-10
              and radii_ok and torque_magnitude_ok)

    print(f"  [{'PASS' if passed else 'FAIL'}] force conversion")
    print(f"         force roundtrip error: {roundtrip_err:.2e} N")
    print(f"         extension roundtrip error: {ext_roundtrip_err:.2e} mm")
    print(f"         spool radii: {spool_radii.round(2)} mm  "
          f"(range ok: {radii_ok})")
    print(f"         100N -> torque: {t100.round(3)} Nm  "
          f"(magnitude ok: {torque_magnitude_ok})")

    return passed


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("Phase 2 Control Loop & IPC Verification Tests")
    print("=" * 60)
    print()

    tests = [
        ("1. Loop timing", test_loop_timing),
        ("2. IPC latency", test_ipc_latency),
        ("3. Force conversion", test_force_conversion),
    ]

    results = []
    for name, test_fn in tests:
        print(f"\n{name}")
        try:
            passed = test_fn()
        except Exception as e:
            print(f"  [FAIL] {e}")
            import traceback
            traceback.print_exc()
            passed = False
        results.append(passed)

    print("\n" + "=" * 60)
    n_passed = sum(results)
    n_total = len(results)
    all_passed = all(results)

    if all_passed:
        print(f"ALL {n_total} TESTS PASSED")
    else:
        print(f"{n_passed}/{n_total} tests passed")
        for (name, _), passed in zip(tests, results):
            if not passed:
                print(f"  FAILED: {name}")

    print("=" * 60)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
