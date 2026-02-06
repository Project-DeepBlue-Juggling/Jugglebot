"""
Extract Throw Targets from MCAP
================================
Reads a ROS2 .mcap recording (standalone file or bag directory),
extracts target positions from /throw_announcements messages,
and saves them as a CSV with ball labels matching the QTM JSON convention.

Usage:
    python extract_targets.py <recording.mcap> [output.csv]
    python extract_targets.py <bag_directory>   [output.csv]

If output path is omitted, it defaults to <input_stem>_targets.csv

Dependencies:
    pip install mcap rosbags
"""

import csv
import sys
from pathlib import Path

import numpy as np

from mcap.reader import make_reader
from rosbags.typesys import get_typestore, get_types_from_msg, Stores


# ── Configuration ───────────────────────────────────────────────────────────

TOPIC = "/throw_announcements"

# The custom message definition — must match the ROS2 msg used during recording
THROW_ANNOUNCEMENT_MSGDEF = """\
std_msgs/Header header
string thrower_name
geometry_msgs/Point initial_position
geometry_msgs/Vector3 initial_velocity
string target_id
geometry_msgs/Point target_position
builtin_interfaces/Time throw_time
"""


# ── MCAP Reading ────────────────────────────────────────────────────────────


def setup_typestore() -> tuple:
    """Set up rosbags typestore with the custom ThrowAnnouncement type.
    Returns (typestore, msg_type_name)."""
    store = get_typestore(Stores.ROS2_HUMBLE)

    # We don't know the exact package name used at record time, so we register
    # under a placeholder. The CDR payload doesn't care about the type name —
    # it just needs the field layout to match.
    msg_type_name = "throw_interfaces/msg/ThrowAnnouncement"
    add_types = get_types_from_msg(THROW_ANNOUNCEMENT_MSGDEF, msg_type_name)
    store.register(add_types)

    return store, msg_type_name


def read_mcap_standalone(filepath: Path, store, msg_type_name: str) -> list[dict]:
    """Read throw announcements from a standalone .mcap file."""
    records = []

    with open(filepath, "rb") as f:
        reader = make_reader(f)
        count = 0

        for schema, channel, message in reader.iter_messages(topics=[TOPIC]):
            msg = store.deserialize_cdr(message.data, msg_type_name)
            records.append(
                {
                    "label": f"b{count}",
                    "target_id": msg.target_id,
                    "target_x": msg.target_position.x,
                    "target_y": msg.target_position.y,
                    "target_z": msg.target_position.z,
                }
            )
            count += 1

    return records


def read_bag_directory(bag_dir: Path, store, msg_type_name: str) -> list[dict]:
    """Read throw announcements from a ROS2 bag directory (contains metadata.yaml + .mcap)."""
    from rosbags.rosbag2 import Reader

    records = []
    count = 0

    with Reader(bag_dir) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC]
        if not connections:
            print(f"  Warning: topic '{TOPIC}' not found in bag")
            return records

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = store.deserialize_cdr(rawdata, msg_type_name)
            records.append(
                {
                    "label": f"b{count}",
                    "target_id": msg.target_id,
                    "target_x": msg.target_position.x,
                    "target_y": msg.target_position.y,
                    "target_z": msg.target_position.z,
                }
            )
            count += 1

    return records


def load_targets(input_path: Path) -> list[dict]:
    """Auto-detect whether input is a standalone .mcap or a bag directory."""
    store, msg_type_name = setup_typestore()

    if input_path.is_dir():
        print(f"Detected bag directory: {input_path}")
        return read_bag_directory(input_path, store, msg_type_name)
    elif input_path.suffix == ".mcap":
        print(f"Detected standalone .mcap: {input_path.name}")
        return read_mcap_standalone(input_path, store, msg_type_name)
    else:
        print(f"Error: unrecognised input: {input_path}")
        print("Provide a .mcap file or a ROS2 bag directory.")
        sys.exit(1)


# ── CSV Output ──────────────────────────────────────────────────────────────


def save_csv(records: list[dict], output_path: Path) -> None:
    """Save extracted target positions to CSV."""
    fieldnames = ["label", "target_id", "target_x", "target_y", "target_z"]

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    print(f"Saved {len(records)} target positions to {output_path}")


# ── Summary ─────────────────────────────────────────────────────────────────


def print_summary(records: list[dict]) -> None:
    """Print a quick summary of extracted targets."""
    if not records:
        print("No throw announcements found!")
        return

    print(f"\n  Throws extracted: {len(records)}")

    # Unique target IDs
    target_ids = set(r["target_id"] for r in records)
    print(f"  Unique targets:   {len(target_ids)}")

    # Position ranges
    xs = [r["target_x"] for r in records]
    ys = [r["target_y"] for r in records]
    zs = [r["target_z"] for r in records]
    print(f"  X range: [{min(xs):.1f}, {max(xs):.1f}] mm")
    print(f"  Y range: [{min(ys):.1f}, {max(ys):.1f}] mm")
    print(f"  Z range: [{min(zs):.1f}, {max(zs):.1f}] mm")

    # Throws per target
    from collections import Counter

    counts = Counter(r["target_id"] for r in records)
    throws_per = list(counts.values())
    print(f"  Throws per target: {min(throws_per)}–{max(throws_per)} (mean {np.mean(throws_per):.1f})")


# ── Entry Point ─────────────────────────────────────────────────────────────


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("Error: Provide a .mcap file or ROS2 bag directory.")
        sys.exit(1)

    input_path = Path(sys.argv[1])
    if not input_path.exists():
        print(f"Error: not found: {input_path}")
        sys.exit(1)

    # Output path: explicit or auto-generated
    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2])
    else:
        output_path = input_path.with_name(f"{input_path.stem}_targets.csv")

    records = load_targets(input_path)
    print_summary(records)
    save_csv(records, output_path)


if __name__ == "__main__":
    main()