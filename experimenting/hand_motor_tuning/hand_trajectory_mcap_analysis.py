#!/usr/bin/env python3
"""
hand_trajectory_mcap_analysis.py

Compare commanded hand-motor trajectory to measured position.

Terminal:
    python hand_trajectory_mcap_analysis.py  path/to/file.mcap

VS Code:
    Edit BAG_FILE at the bottom and press ▶ Run.
"""

from __future__ import annotations
import argparse, sys
from pathlib import Path
from typing import Tuple, Union

import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from ament_index_python import get_package_share_directory

# ─── CONFIG ──────────────────────────────────────────────────────────────────
CMD_TOPIC   = '/hand_trajectory'   # custom command message
STATE_TOPIC = '/robot_state'       # RobotState telemetry
MOTOR_INDEX = 6                    # element in motor_states to plot
# ─────────────────────────────────────────────────────────────────────────────


def extract_data(bag: Path):
    """
    Extract data from the bag file.
    Returns:
        Tuple of time and position arrays for commanded and measured data.
    """
    cmd_t, cmd_p = [], []
    meas_t, meas_p = [], []

    typestore = get_typestore(Stores.ROS2_FOXY)

    # Register the custom messages
    pkg_share = get_package_share_directory('jugglebot_interfaces')
    msg_dir = Path(pkg_share) / 'msg'
    for msg_file in msg_dir.glob('*.msg'):
        # Ignore any message with type "PlatformPoseMessage"
        if msg_file.stem == 'PlatformPoseMessage':
            continue
        typestore.register(
            get_types_from_msg(
                msg_file.read_text(encoding='utf-8'),
                f"jugglebot_interfaces/msg/{msg_file.stem}"
            )
        )

    with Reader(bag) as reader:
        for con, _, raw in reader.messages():
            topic = con.topic

            if topic == CMD_TOPIC:
                # Extract commanded trajectory
                msg = typestore.deserialize_cdr(raw, con.msgtype)
                # Extract every position and time command from their respective lists
                for i in range(len(msg.time_cmd)):
                    cmd_t.append(msg.time_cmd[i].sec + msg.time_cmd[i].nanosec * 1e-9)
                    cmd_p.append(msg.input_pos[i])

            elif topic == STATE_TOPIC:
                # Extract measured state
                msg = typestore.deserialize_cdr(raw, con.msgtype)
                meas_t.append(msg.timestamp.sec + msg.timestamp.nanosec * 1e-9)
                meas_p.append(msg.motor_states[MOTOR_INDEX].pos_estimate)

    print(f"Extracted {len(cmd_t)} commanded and {len(meas_t)} measured points.")
    return np.array(cmd_t), np.array(cmd_p), np.array(meas_t), np.array(meas_p)


def plot_data(cmd_t, cmd_p, meas_t, meas_p, title_suffix=''):
    plt.figure(figsize=(10, 5))
    order = np.argsort(cmd_t)
    plt.plot(cmd_t[order], cmd_p[order], drawstyle='steps-post',
             label='Commanded', linewidth=1.4)
    plt.plot(meas_t, meas_p, label='Measured', linewidth=1.0)
    plt.xlabel('Time [s]');  plt.ylabel('Position [rev]')
    plt.title(f'Hand-motor commanded vs measured{title_suffix}')
    plt.legend();  plt.grid(True);  plt.tight_layout()
    plt.show()


# ─── Inline-run convenience ──────────────────────────────────────────────────
def run_on_file(bag_path: Union[str, Path]):
    bag = Path(bag_path).expanduser()
    if not bag.exists():
        sys.exit(f'❌ Bag file not found: {bag}')

    cmd_t, cmd_p, meas_t, meas_p = extract_data(bag)

    if cmd_t.size == 0 or meas_t.size == 0:
        sys.exit("❌ Required topics missing in bag.")

    t0 = min(cmd_t.min(), meas_t.min())
    plot_data(cmd_t - t0, cmd_p, meas_t - t0, meas_p,
              title_suffix=f'  ({bag.name})')


# ─── CLI entry-point ─────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='Plot hand-motor trajectory from an MCAP bag.')
    ap.add_argument('bag', type=Path, help='.mcap file')
    args = ap.parse_args()
    run_on_file(args.bag)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        main()            # normal CLI path
    else:
        # VS Code quick-run
        BAG_FILE = "/home/jetson/Desktop/Jugglebot/experimenting/hand_motor_tuning/test"
        run_on_file(BAG_FILE)
