---
description: Analyse hardware test logs — MPC telemetry and rosbag recordings. Invoke with /diagnose [csv_filename | --latest | --all-new].
---

# Hardware Diagnosis Agent

Analyse hardware test session data from MPC telemetry CSVs and rosbag (MCAP) recordings.  Cross-reference findings against known failure patterns and present a structured diagnostic report.

**Data sources:**
- **MPC telemetry CSV** (`sim/logs/mpc_*.csv`) — 55-field StepRecord at 40 Hz: pose, tracking error, solve times, leg extensions, torques
- **Rosbag MCAP** (`~/Desktop/rosbags/<timestamp>/`) — 19 ROS2 topics recorded automatically: motor state, leg commands, hand telemetry, state transitions, diagnostics, etc.

Note: ROS2 Foxy does NOT write per-node text log files (that's a Humble+ feature).  The rosbag is the primary source for ROS2 event data.

## Arguments

The user may provide arguments after `/diagnose`:
- **No args** or **`--latest`**: analyse the most recent unanalysed log in `sim/logs/`
- **`<csv_filename>`**: analyse a specific MPC log (filename or full path)
- **`--all-new`**: analyse all unanalysed logs

## Protocol

Follow these steps in order:

### Step 1: Determine target log(s)

1. Read `sim/analysis/log_index.json` to find session metadata
2. If the user specified a filename, locate it in `sim/logs/`
3. If `--latest` or no args: scan `sim/logs/mpc_*.csv`, find the most recent file not marked `analyzed: true` in the index
4. If `--all-new`: find all unanalysed files

### Step 2: Find correlated rosbag

For each target CSV:
1. Parse the CSV filename timestamp (format: `YYYYMMDD_HHMMSS`)
2. Scan `~/Desktop/rosbags/` for a directory whose timestamp is closest and within 1 hour
3. Multiple MPC sessions may exist within one rosbag session — use the CSV timestamp to identify the relevant time window

### Step 3: Run the analysis engine

Run the diagnosis script with all available data sources:

```bash
python sim/analysis/diagnose.py <csv_path> [--rosbag <path>] --json
```

Read the JSON output.

### Step 4: Cross-reference against known issues

Read `sim/analysis/known_issues.yaml`.  For each flag in the analysis output:
- Check if the flag matches any known issue's signatures
- If matched: report the known issue ID, description, and fix suggestion
- If unmatched: flag as a **new/unknown anomaly** for investigation

### Step 5: Gather context

Read `sim/HARDWARE_BRINGUP.md` to understand:
- What bringup phase this test belongs to
- What the pass/fail criteria are for this phase
- Whether the observed behaviour is expected for this phase

### Step 6: Present the report

Use this format:

```
## Hardware Diagnosis: <csv_filename>
**Phase:** <phase>  |  **Duration:** <N>s  |  **Source:** <mpc/hardware>
**Rosbag:** <dir or "none">

### Verdict: PASS / NEEDS ATTENTION / FAIL

<1-2 sentence overall assessment>

### Event Timeline
<Chronological timeline from rosbag: state transitions (/orchestrator_state), mode changes (/control_mode_topic), errors>
<Include MPC anomaly timestamps (discontinuities, solve spikes) interleaved>
<Only include if rosbag is available; skip section otherwise>

### Tracking Performance
| Leg | RMS (mm) | Peak (mm) | Mean (mm) |
|-----|----------|-----------|-----------|
| 0   | ...      | ...       | ...       |
...
| **Aggregate** | pos RMS: ... | pos peak: ... | ori RMS: ...deg |

<Flag worst leg if ratio > 1.5x median>
<Report transient vs steady-state metrics if available>

### MPC Solver
- p50: ... ms, p95: ... ms, p99: ... ms, max: ... ms
- Budget violations: N (X%)
- First-sample: ... ms (expected cold-start if > 15ms)
<Compare against baseline if available>

### Motor & CAN Health
<From rosbag: motor errors (/robot_state), disarms, CAN rejections, firmware validation>
<Skip section if rosbag not available>

### Stability
- Oscillation: <detected/not detected>, chatter ratios per leg
- Discontinuities: <count> command jumps, <count> actual jumps
- Torque: max <N> Nm, mean <N> Nm

### Workspace
- Extension range: [<min>, <max>] mm
- Margin to lower limit: <N> mm
- Margin to upper limit: <N> mm

### Flagged Issues
For each flag:
- **[SEVERITY]** [source] description
  - **Known issue:** <ID> — <fix suggestion> (if matched)
  - **Unknown:** Suggest investigation path (if not matched)

### Recommendations
<Prioritised list of next steps based on flags and phase context>
```

### Step 7: Update the log index

Update `sim/analysis/log_index.json`:
- Set `analyzed: true` and `last_analyzed` to current ISO timestamp
- Set `verdict` to PASS, NEEDS_ATTENTION, or FAIL
- Set `flags_count` to the number of flags
- Set `phase` if identifiable from context
- Set `rosbag` path if found

## Verdict Criteria

- **PASS**: No error-severity flags, all metrics within expected ranges for the phase
- **NEEDS ATTENTION**: Warning-severity flags only, or minor threshold exceedances
- **FAIL**: Any error-severity flag, or metrics significantly outside expected ranges

## Important Notes

- This is a **read-only** analysis agent.  Never send commands to the robot.
- The analysis engine (`diagnose.py`) handles all numeric computation.  The LLM's role is interpretation, cross-referencing, and presentation.
- MPC CSV time is relative (starts at 0).  Rosbag timestamps are absolute.  Use the session start marker for correlation.
- If `rosbags` library is not installed, rosbag analysis will be skipped gracefully.
- The `known_issues.yaml` is a living document.  If you discover a new failure pattern during analysis, suggest adding it.
