# Jugglebot
For all CAD and code related to Project: DeepBlue Juggling

I'll try to release an updated folder here for every episode in which major changes happen.

For further information about the project as a whole, as well as more detail on specific elements of the project, please head to:
pdj.zulipchat.com

## Hardware bench-rig & analysis tools

A single-ODrive bench rig is set up for isolated single-leg characterisation
(brake resistor, separate PSU, isolated CAN bus). The bench scripts and
analysers are documented in [tools/README.md](tools/README.md):

- **Friction characterisation** — Stribeck four-parameter fit pipeline
  (`cogging_bench_test.py` → `friction_study_analyse.py`)
- **Stiction breakaway** — torque-ramp test with position-only escape detector
  (`breakaway_ramp_test.py` → `breakaway_analyse.py`)
- **Friction feedforward demo** — bench-validation of the FF stack
  (`friction_ff_demo.py`)

Canonical reference: [logbook/2026-04-27-friction-feedforward-bench-validation.md](logbook/2026-04-27-friction-feedforward-bench-validation.md).
