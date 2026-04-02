# Hardware Analysis & Engineering Logbook

The analysis subsystem provides tools for diagnosing hardware test sessions, comparing before/after results, searching past investigations, and maintaining a structured engineering logbook.

## Architecture

```
sim/analysis/
  diagnose.py            MPC telemetry + rosbag analysis engine
  compare_sessions.py    Side-by-side session comparison (before/after)
  logbook_search.py      Search logbook by symptoms for prior art
  compare.py             Sim-vs-hardware accuracy comparison
  plot_interactive.py    Plotly interactive reports
  plot_diagnosis.py      Matplotlib static plots (fallback)
  report_html.py         HTML report generation
  known_issues.yaml      Signature catalog for regression detection
  log_index.json         Per-session metadata index

logbook/
  INDEX.md               Auto-maintained summary table
  README.md              System guide and workflows
  TEMPLATE.md            Entry format reference
  YYYY-MM-DD-<slug>.md   Investigation entries

.claude/commands/        Slash commands (diagnose, investigate, log, logbook)
.claude/agents/          Dedicated agents (fix-proposer, logbook-updater)
```

## Workflow Overview

The typical hardware debugging workflow follows this pipeline:

```
Hardware test session (CSV + rosbag)
         |
         v
    /diagnose           Analyse telemetry, cross-reference known issues
         |
         v
    /investigate        Full gated pipeline: diagnose -> log -> fix -> test -> commit
         |
    +----+----+
    |         |
    v         v
 fix-proposer    logbook-updater
 (prior art      (create entry,
  search,         update index)
  risk assess)
         |
         v
    /diagnose --compare    Before/after validation
         |
         v
    Engineering logbook entry (full traceability)
```

## Key Concepts

### Telemetry Data

MPC telemetry CSVs contain 55 fields recorded at 40 Hz:

- **Pose**: reference and actual 6-DoF (x, y, z, rx, ry, rz)
- **Legs**: commanded and actual extension per leg (mm), leg velocities
- **Solver**: solve time (ms), solve status, cost, constraint violation, IPOPT iterations
- **Tracking**: position error (mm), orientation error (deg)
- **Dynamics**: feedforward torques, overhead timing

### Known Issues Catalog

`sim/analysis/known_issues.yaml` is a machine-readable signature catalog. Each entry has:

- **Detection signatures** with thresholds (oscillation chatter ratio, discontinuity magnitude, solve time violations)
- **Status tracking**: `fixed` (kept for regression detection), `active`, `persistent`, `known`
- **Logbook references** linking to the full investigation narrative

### Traceability Chain

Every code change traced back to its investigation:

```
code line  ->  git blame  ->  commit  ->  Logbook-Entry trailer  ->  logbook entry
                                          (full symptoms, diagnosis, rejected alternatives, outcome)
```

## Pages

- **[Diagnosis Tools](diagnosis.md)** -- Session analysis, comparison, and known issue detection
- **[Engineering Logbook](logbook.md)** -- Structured investigation records and prior-art search
