"""Online per-throw task-space planner for the BB-led juggle demo.

Pure-Python (no ROS2) CasADi reference planner — see ``plans/archived/
bb-led-two-ball-juggle-demo.md`` for the demo history.

**Formerly ``controller/demo/``** (moved 2026-08-01, refactor-2026-07 Phase
6; older logbook entries and plan documents still name the old path). Every
consumer is a sim entry script, a sim test or a ``tools/probes/juggle_*``
probe — nothing on the hardware control path imports it — so it belongs
beside its consumers rather than inside ``controller/``, which the hardware
MPC loop imports.

The offline optimiser (``juggle_optimizer``), the open-loop player
(``player``), the master event schedule (``timeline``), the periodic
platform-pose trajectory (``trajectory``) and the oval pattern spec
(``pattern``) were deleted 2026-09-09 (R0 dead-layer deletion,
``plans/active/two-ball-skill-stack.md`` § 6) along with their consumer,
``sim/juggle_demo.py`` — the offline demo they served is superseded by the
online loop (``sim/juggle_online.py``).

  - ``juggle_planner`` — online per-throw CasADi planner (jerk-minimising,
    re-solved once per throw against the observed incoming ball); the QP
    fixtures captured by ``tools/probes/capture_cup_cycle_refs.py`` are
    generated from this module.

Consumers import the submodule explicitly
(``from sim.juggle_planner.juggle_planner import ...``); this package
intentionally does not re-export, to keep import order under the consumer's
control.
"""
