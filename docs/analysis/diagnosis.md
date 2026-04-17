# Diagnosis Tools

## Session Diagnosis (`/diagnose`)

Analyse MPC telemetry CSVs and rosbag (MCAP) recordings from hardware test sessions.

### Usage

```
/diagnose                          # Analyse most recent unanalysed session
/diagnose mpc_20260401_143845.csv  # Analyse a specific session
/diagnose --all-new                # Analyse all unanalysed sessions
/diagnose --compare <a> <b>        # Compare two sessions side-by-side
```

### Analysis Engine

The core engine (`sim/analysis/diagnose.py`) runs these analyses:

| Function | What it checks |
|----------|---------------|
| `analyse_tracking()` | Per-leg + aggregate tracking error (RMS, peak, mean) |
| `analyse_solve_times()` | Solver performance (p50/p95/p99, budget violations, consecutive violations) |
| `analyse_oscillation()` | Chatter ratio detection, amplitude growth (sign-change analysis) |
| `analyse_discontinuities()` | Single-step command/actual jumps exceeding thresholds |
| `analyse_workspace()` | Stroke usage and margins to hard limits |
| `analyse_steady_state()` | Transient vs steady-state segmentation, settling time |
| `analyse_torques()` | Feedforward torque statistics |

### Standalone Usage

The analysis engine can be run directly without the slash command:

```bash
# JSON output (for programmatic use)
python sim/analysis/diagnose.py temp/logs/mpc_20260401_143845.csv --json

# With rosbag correlation
python sim/analysis/diagnose.py temp/logs/mpc_20260401_143845.csv \
    --rosbag ~/Desktop/rosbags/2026-04-01_14-37-59 --json

# With interactive Plotly report
python sim/analysis/diagnose.py temp/logs/mpc_20260401_143845.csv --plots auto
```

### Verdict Criteria

- **PASS** -- No error-severity flags, all metrics within expected ranges
- **NEEDS_ATTENTION** -- Warning-severity flags only, or minor threshold exceedances
- **FAIL** -- Any error-severity flag, or metrics significantly outside expected ranges

### Interactive Reports

Diagnosis generates interactive HTML reports (Plotly) with 8 plot categories:

| Category | Content |
|----------|---------|
| `legs` | Commanded vs actual extension per leg (3x2 grid) |
| `pose` | Reference vs actual 6-DoF pose |
| `tracking` | Position + orientation error over time |
| `solver` | Solve time distribution vs 24ms budget |
| `velocity` | Leg velocity per step |
| `hand` | Hand position and velocity |
| `workspace` | Stroke usage vs limits |
| `chatter` | Oscillation detection per leg |

Categories are auto-selected based on detected anomalies. All plots support zoom, pan, hover tooltips, and legend toggling.

---

## Session Comparison (`/diagnose --compare`)

Compare two hardware sessions side-by-side to answer: *"Did my fix actually improve things?"*

### Usage

```
/diagnose --compare mpc_20260401_143845.csv mpc_20260401_152048.csv
```

### Output

A delta table showing key metrics with improvement/regression indicators:

```
  SESSION COMPARISON: mpc_20260401_143845.csv vs mpc_20260401_152048.csv

  Metric                     Before          After              Delta
  -------------------------- --------------- --------------- ---------
  Tracking RMS (mm)              2.341           0.187       92% down (improved)
  Solve time p50 (ms)           24.8            22.1         11% down (improved)
  Budget violations (%)         89%             12%          77% down (improved)
  Command discontinuities        4               0           resolved
  Verdict                       FAIL            PASS         improved
```

An interactive Plotly HTML report is also generated with dual-trace overlays (session A in blue, session B in red) for legs, tracking error, and solve times.

### Standalone Usage

```bash
# Text table
python sim/analysis/compare_sessions.py <csv_a> <csv_b>

# JSON output
python sim/analysis/compare_sessions.py <csv_a> <csv_b> --json

# With interactive HTML report
python sim/analysis/compare_sessions.py <csv_a> <csv_b> --html

# Custom labels
python sim/analysis/compare_sessions.py <csv_a> <csv_b> --labels "before fix" "after fix"
```

---

## Known Issues Catalog

`sim/analysis/known_issues.yaml` tracks failure patterns with machine-readable detection signatures.

### Issue Lifecycle

```
active  -->  fixed (kept for regression detection)
  |
  +--->  persistent (hardware characteristic, not a bug)
  |
  +--->  known (understood, no action needed)
```

### Entry Format

```yaml
- id: VEL_FF_BUG
  name: "Velocity feedforward semantic mismatch"
  severity: critical
  status: fixed
  fixed_date: "2026-03-30"
  logbook_entry: "2026-03-30-velocity-feedforward-oscillation.md"
  signatures:
    - type: oscillation
      chatter_threshold: 0.5
      amplitude_growing: true
  fix: "Use (cmd - prev_cmd)/dt for velocity feedforward."
```

### Signature Types

| Type | What it detects | Key thresholds |
|------|----------------|----------------|
| `oscillation` | Command chatter ratio, amplitude growth | `chatter_threshold`, `amplitude_growing` |
| `discontinuity` | Single-step jumps in commanded/actual extension | `field`, `threshold_mm` |
| `solve_time` | Consecutive MPC budget violations | `max_consecutive_violations` |
| `per_leg_tracking` | One leg tracking significantly worse than median | `leg`, `rms_ratio_vs_median` |
| `solve_status_pattern` | Specific solve status strings (e.g., cold_hold) | `pattern` |
| `ros2_log_pattern` | ROS2 log message patterns | `pattern`, `level` |

### Current Issues

As of 2026-04-01:

| ID | Severity | Status | Description |
|----|----------|--------|-------------|
| VEL_FF_BUG | critical | fixed | Velocity feedforward semantic mismatch |
| COLD_HOLD_STROKE_MIN | critical | fixed | Cold-hold fallback to stroke minimum |
| ASSIGNMENT_ORDER | critical | fixed | Assignment-order bug in feedforward |
| MPC_STALENESS | high | active | Solve time exceeding 24ms budget |
| LEG2_TRACKING | medium | active | Leg 2 consistently worst tracker |
| CAN_BUS_WATCHDOG | high | persistent | CAN bus watchdog restore |
| ODRIVE_DISARM | critical | persistent | ODrive axis disarmed in CLOSED_LOOP |

---

## Session Index

`sim/analysis/log_index.json` tracks metadata for every analysed session:

```json
{
  "mpc_20260401_143845.csv": {
    "analyzed": true,
    "last_analyzed": "2026-04-01T15:05:00Z",
    "verdict": "FAIL",
    "flags_count": 4,
    "phase": "solver-fix-validation-motion",
    "rosbag": "/home/jetson/Desktop/rosbags/2026-04-01_14-37-59",
    "logbook_entry": "2026-04-01-cold-hold-fallback-stroke-minimum.md"
  }
}
```

The `/diagnose` command updates this file after each analysis. The `logbook_entry` field links sessions to their investigation narrative.
