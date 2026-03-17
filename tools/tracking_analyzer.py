#!/usr/bin/env python3
"""Replay rosbag data through the ball tracker with tunable parameters.

Reads /mocap_data and /throw_announcements from a rosbag, feeds them
through the BallTracker, and reports detection statistics. Use this to
tune parabolic detection thresholds and matching parameters.

Usage:
    # Analyse a bag with no balls (false positive check):
    python tools/tracking_analyzer.py ~/Desktop/rosbags/bag_no_balls

    # Analyse a bag with BB throws:
    python tools/tracking_analyzer.py ~/Desktop/rosbags/bag_bb_throws

    # Override parameters:
    python tools/tracking_analyzer.py bag_bb_throws --accel-threshold 2000

    # Sweep a parameter range:
    python tools/tracking_analyzer.py bag_no_balls --sweep-accel 1000,2000,3000,4000,5000

Requires: rosbags (pip install rosbags)
"""
from __future__ import annotations

import argparse
import sys
import os
from pathlib import Path
from dataclasses import dataclass, field

import numpy as np

# Add the ROS2 package to the path so we can import tracking/ without ROS2
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'ros_ws' / 'src' / 'jugglebot'))

from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2


# ---------------------------------------------------------------------------
# Rosbag reader (uses rosbags library, not rclpy)
# ---------------------------------------------------------------------------

def read_bag(bag_path: str) -> tuple[list, list]:
    """Read /mocap_data and /throw_announcements from a rosbag.

    Returns:
        (mocap_frames, announcements) where:
        - mocap_frames: list of (timestamp_s, markers) where markers is list of
          (x, y, z, residual, label) tuples
        - announcements: list of (timestamp_s, announcement_dict) where dict has
          keys: initial_position, initial_velocity, throw_time, target_id,
          landing_position, landing_velocity, landing_time, thrower_name
    """
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import get_typestore, Stores
    except ImportError:
        print("ERROR: 'rosbags' package not installed.")
        print("Install with: pip install rosbags")
        sys.exit(1)

    # Build a typestore and register our custom message types
    typestore = get_typestore(Stores.ROS2_FOXY)

    # Register jugglebot_interfaces messages
    msg_dir = Path(__file__).resolve().parent.parent / 'ros_ws' / 'src' / 'jugglebot_interfaces' / 'msg'
    if msg_dir.exists():
        _register_custom_msgs(typestore, msg_dir)
    else:
        print(f"WARNING: Message directory not found at {msg_dir}")
        print("  Custom message deserialization may fail.")

    mocap_frames = []
    announcements = []

    with Reader(bag_path) as reader:
        # Get connections for our topics
        mocap_connections = [c for c in reader.connections if c.topic == '/mocap_data']
        announce_connections = [c for c in reader.connections if c.topic == '/throw_announcements']

        if not mocap_connections:
            print(f"WARNING: No /mocap_data topic found in {bag_path}")
            print(f"  Available topics: {[c.topic for c in reader.connections]}")

        for conn, timestamp, rawdata in reader.messages():
            t_s = timestamp / 1e9  # nanoseconds → seconds

            if conn.topic == '/mocap_data':
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                markers = []
                for m in msg.markers:
                    markers.append((
                        m.position.x, m.position.y, m.position.z,
                        m.residual,
                        m.label if hasattr(m, 'label') else '',
                    ))
                mocap_frames.append((t_s, markers))

            elif conn.topic == '/throw_announcements':
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                throw_time = msg.throw_time.sec + msg.throw_time.nanosec * 1e-9
                landing_time = msg.landing_time.sec + msg.landing_time.nanosec * 1e-9

                announcements.append((t_s, {
                    'initial_position': np.array([
                        msg.initial_position.x,
                        msg.initial_position.y,
                        msg.initial_position.z,
                    ]),
                    'initial_velocity': np.array([
                        msg.initial_velocity.x,
                        msg.initial_velocity.y,
                        msg.initial_velocity.z,
                    ]),
                    'throw_time': throw_time,
                    'target_id': msg.target_id if hasattr(msg, 'target_id') else '',
                    'thrower_name': msg.thrower_name if hasattr(msg, 'thrower_name') else 'ball_butler',
                    'landing_position': np.array([
                        msg.landing_position.x,
                        msg.landing_position.y,
                        msg.landing_position.z,
                    ]),
                    'landing_velocity': np.array([
                        msg.landing_velocity.x,
                        msg.landing_velocity.y,
                        msg.landing_velocity.z,
                    ]),
                    'landing_time': landing_time,
                }))

    return mocap_frames, announcements


def _register_custom_msgs(typestore, msg_dir: Path):
    """Register all .msg files from jugglebot_interfaces with the typestore."""
    from rosbags.typesys import get_types_from_msg

    # Collect all .msg definitions
    msg_texts = {}
    for msg_file in sorted(msg_dir.glob('*.msg')):
        name = msg_file.stem
        full_name = f'jugglebot_interfaces/msg/{name}'
        msg_texts[full_name] = msg_file.read_text()

    # Parse and register (handles inter-message dependencies)
    add_types = {}
    for name, text in msg_texts.items():
        add_types.update(get_types_from_msg(text, name))

    typestore.register(add_types)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

@dataclass
class AnalysisResult:
    """Results from one replay run."""
    parabolic_accel_threshold: float
    total_frames: int = 0
    total_markers_seen: int = 0

    # Balls created
    announced_balls: int = 0
    parabolic_detections: int = 0  # Human throw detections (potential false positives if no balls thrown)

    # Confirmed balls (matched to mocap)
    confirmed_balls: int = 0

    # Terminal states
    caught_count: int = 0
    unknown_count: int = 0

    # Acceleration residuals from parabolic detection (for threshold tuning)
    accel_residuals: list = field(default_factory=list)

    # Per-ball details
    ball_details: list = field(default_factory=list)


def replay(
    mocap_frames: list,
    announcements: list,
    parabolic_accel_threshold: float = 3000.0,
    gate_radius_mm: float = 50.0,
    match_threshold_base_mm: float = 100.0,
    parabolic_min_frames: int = 3,
    landing_z: float = 734.3,
    dt: float = 0.005,
    verbose: bool = False,
) -> AnalysisResult:
    """Replay recorded data through the tracker and collect statistics."""

    tracker = BallTracker(
        dt=dt,
        landing_z=landing_z,
        match_threshold_base_mm=match_threshold_base_mm,
        gate_radius_mm=gate_radius_mm,
        parabolic_accel_threshold_mmps2=parabolic_accel_threshold,
        parabolic_min_frames=parabolic_min_frames,
    )

    result = AnalysisResult(parabolic_accel_threshold=parabolic_accel_threshold)

    # Index announcements by time for injection
    announce_idx = 0
    seen_ball_ids = set()
    prev_balls = {}

    for frame_idx, (t_s, markers) in enumerate(mocap_frames):
        # Inject any announcements that arrived before this frame
        while announce_idx < len(announcements):
            ann_t, ann = announcements[announce_idx]
            if ann_t > t_s:
                break
            tracker.handle_announcement(
                initial_position=ann['initial_position'],
                initial_velocity=ann['initial_velocity'],
                throw_time=ann['throw_time'],
                source=ann['thrower_name'],
                destination=ann.get('target_id', ''),
                landing_position=ann['landing_position'],
                landing_velocity=ann['landing_velocity'],
                landing_time=ann['landing_time'],
            )
            result.announced_balls += 1
            announce_idx += 1

        # Extract unlabelled markers
        unlabelled = []
        for x, y, z, residual, label in markers:
            if not label:
                unlabelled.append(np.array([x, y, z]))

        result.total_frames += 1
        result.total_markers_seen += len(unlabelled)

        # Run tracker
        balls = tracker.process_frame(unlabelled, t_s)

        # Track new balls and state changes
        for ball in balls:
            if ball.id not in seen_ball_ids:
                seen_ball_ids.add(ball.id)
                if ball.source == "human_throw":
                    result.parabolic_detections += 1
                    if verbose:
                        print(f"  [{t_s:.3f}s] PARABOLIC DETECTION: ball {ball.id} "
                              f"at ({ball.position[0]:.0f}, {ball.position[1]:.0f}, {ball.position[2]:.0f})")

            # Track confirmation
            prev = prev_balls.get(ball.id)
            if prev is not None:
                if (prev.tracking == TrackingConfidence.ANNOUNCED and
                        ball.tracking == TrackingConfidence.CONFIRMED):
                    result.confirmed_balls += 1
                    if verbose:
                        print(f"  [{t_s:.3f}s] CONFIRMED: ball {ball.id} "
                              f"after {ball.frames_tracked} frames")

            # Track terminal transitions
            if prev is not None and prev.status == BallStatus.IN_FLIGHT:
                if ball.status == BallStatus.CAUGHT:
                    result.caught_count += 1
                    if verbose:
                        print(f"  [{t_s:.3f}s] CAUGHT: ball {ball.id}")
                elif ball.status == BallStatus.UNKNOWN:
                    result.unknown_count += 1
                    if verbose:
                        print(f"  [{t_s:.3f}s] UNKNOWN: ball {ball.id}")

            prev_balls[ball.id] = Ball(
                id=ball.id,
                status=ball.status,
                tracking=ball.tracking,
                source=ball.source,
                position=ball.position.copy(),
                velocity=ball.velocity.copy(),
                landing_position=ball.landing_position.copy(),
                landing_velocity=ball.landing_velocity.copy(),
                landing_time=ball.landing_time,
                frames_tracked=ball.frames_tracked,
            )

    # Collect acceleration residuals from internal marker tracks
    # (These tell us how close each candidate was to gravity)
    _collect_accel_residuals(tracker, result)

    # Summarise per-ball details
    for ball_id, ball in prev_balls.items():
        result.ball_details.append({
            'id': ball.id,
            'source': ball.source,
            'status': ball.status.name,
            'tracking': ball.tracking.name,
            'frames_tracked': ball.frames_tracked,
            'landing_time': ball.landing_time,
        })

    return result


def _collect_accel_residuals(tracker: BallTracker, result: AnalysisResult):
    """Collect acceleration residuals from the tracker's diagnostic log."""
    result.accel_residuals = list(tracker.accel_residuals)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_report(result: AnalysisResult, label: str = ""):
    """Print a human-readable report."""
    print(f"\n{'=' * 60}")
    if label:
        print(f"  {label}")
        print(f"{'=' * 60}")
    print(f"  Parabolic accel threshold: {result.parabolic_accel_threshold:.0f} mm/s²")
    print(f"  Frames processed:          {result.total_frames}")
    print(f"  Total markers seen:         {result.total_markers_seen}")
    print(f"  Avg markers/frame:          {result.total_markers_seen / max(result.total_frames, 1):.1f}")
    print()
    print(f"  Announced balls (BB):       {result.announced_balls}")
    print(f"  Parabolic detections:       {result.parabolic_detections}")
    print(f"  Confirmed (mocap matched):  {result.confirmed_balls}")
    print()
    print(f"  Terminal: CAUGHT={result.caught_count}, UNKNOWN={result.unknown_count}")
    print()

    if result.announced_balls > 0 and result.parabolic_detections == 0:
        pct = result.confirmed_balls / result.announced_balls * 100
        print(f"  BB confirmation rate: {result.confirmed_balls}/{result.announced_balls} ({pct:.0f}%)")

    if result.announced_balls == 0 and result.parabolic_detections > 0:
        print(f"  WARNING — FALSE POSITIVES: {result.parabolic_detections} parabolic detections "
              f"with no announcements")

    if result.accel_residuals:
        residuals = [r for r, _ in result.accel_residuals]
        promoted = [r for r, p in result.accel_residuals if p]
        rejected = [r for r, p in result.accel_residuals if not p]

        print(f"  Acceleration residuals (parabolic detector):")
        print(f"    Total checks: {len(residuals)}")
        if residuals:
            print(f"    All:      min={min(residuals):.0f}  median={np.median(residuals):.0f}  "
                  f"max={max(residuals):.0f} mm/s²")
        if promoted:
            print(f"    Promoted: min={min(promoted):.0f}  median={np.median(promoted):.0f}  "
                  f"max={max(promoted):.0f} mm/s²  (n={len(promoted)})")
        if rejected:
            print(f"    Rejected: min={min(rejected):.0f}  median={np.median(rejected):.0f}  "
                  f"max={max(rejected):.0f} mm/s²  (n={len(rejected)})")

        # Suggest threshold if we have both populations
        if promoted and rejected:
            max_promoted = max(promoted)
            min_rejected = min(rejected)
            if max_promoted < min_rejected:
                mid = (max_promoted + min_rejected) / 2
                print(f"    CLEAN SEPARATION. Suggested threshold: {mid:.0f} mm/s2")
            else:
                print(f"    OVERLAP between promoted ({max_promoted:.0f}) and "
                      f"rejected ({min_rejected:.0f}) -- may need more frames or tighter gate")
        print()

    if result.ball_details:
        print(f"\n  Per-ball summary:")
        for d in result.ball_details:
            print(f"    Ball {d['id']:3d}: {d['source']:12s} | {d['status']:10s} | "
                  f"{d['tracking']:10s} | {d['frames_tracked']:4d} frames")

    print(f"{'=' * 60}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Replay rosbag data through ball tracker for tuning")
    parser.add_argument('bag_path', help="Path to rosbag directory")
    parser.add_argument('--accel-threshold', type=float, default=3000.0,
                        help="Parabolic acceleration threshold (mm/s², default: 3000)")
    parser.add_argument('--gate-radius', type=float, default=50.0,
                        help="Frame-to-frame gate radius (mm, default: 50)")
    parser.add_argument('--match-threshold', type=float, default=100.0,
                        help="Announced ball match threshold base (mm, default: 100)")
    parser.add_argument('--min-frames', type=int, default=3,
                        help="Minimum frames for parabolic detection (default: 3)")
    parser.add_argument('--landing-z', type=float, default=734.3,
                        help="Landing plane Z height (mm, default: 734.3)")
    parser.add_argument('--verbose', '-v', action='store_true',
                        help="Print per-event details")
    parser.add_argument('--sweep-accel', type=str, default=None,
                        help="Comma-separated list of accel thresholds to sweep "
                             "(e.g. '1000,2000,3000,4000,5000')")

    args = parser.parse_args()

    bag_path = args.bag_path
    if not os.path.exists(bag_path):
        print(f"ERROR: Bag path not found: {bag_path}")
        sys.exit(1)

    print(f"Reading rosbag: {bag_path}")
    mocap_frames, announcements = read_bag(bag_path)
    print(f"  Loaded {len(mocap_frames)} mocap frames, "
          f"{len(announcements)} throw announcements")

    if not mocap_frames:
        print("ERROR: No mocap data found in bag.")
        sys.exit(1)

    duration = mocap_frames[-1][0] - mocap_frames[0][0]
    print(f"  Duration: {duration:.1f}s")

    if args.sweep_accel:
        # Sweep mode: run with multiple thresholds
        thresholds = [float(x) for x in args.sweep_accel.split(',')]
        print(f"\nSweeping {len(thresholds)} accel thresholds...")

        for thresh in thresholds:
            result = replay(
                mocap_frames, announcements,
                parabolic_accel_threshold=thresh,
                gate_radius_mm=args.gate_radius,
                match_threshold_base_mm=args.match_threshold,
                parabolic_min_frames=args.min_frames,
                landing_z=args.landing_z,
                verbose=False,
            )
            # Compact summary for sweep
            fp = result.parabolic_detections if result.announced_balls == 0 else 0
            confirmed = result.confirmed_balls
            print(f"  threshold={thresh:6.0f} mm/s² | "
                  f"parabolic={result.parabolic_detections:3d} | "
                  f"confirmed={confirmed:3d}/{result.announced_balls} | "
                  f"false_pos={fp:3d}")

    else:
        # Single run
        result = replay(
            mocap_frames, announcements,
            parabolic_accel_threshold=args.accel_threshold,
            gate_radius_mm=args.gate_radius,
            match_threshold_base_mm=args.match_threshold,
            parabolic_min_frames=args.min_frames,
            landing_z=args.landing_z,
            verbose=args.verbose,
        )
        print_report(result, label=os.path.basename(bag_path))


if __name__ == '__main__':
    main()
