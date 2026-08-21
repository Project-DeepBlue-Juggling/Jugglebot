"""Open-loop juggling-demo trajectory tooling.

Pure-Python (no ROS2) modules for the Ball-Butler-initiated two-ball oval
juggling demo — see ``plans/archived/bb-led-two-ball-juggle-demo.md``.

**Formerly ``controller/demo/``** (moved 2026-08-01, refactor-2026-07 Phase 6;
older logbook entries and plan documents still name the old path).  Every
consumer is a sim entry script, a sim test or a ``tools/probes/juggle_*``
probe — nothing on the hardware control path imports it — so it belongs beside
its consumers rather than inside ``controller/``, which the hardware MPC
loop imports.  The move also let the optimiser's seven hand-copied stroke
constants become real imports from ``sim.hand`` (see ``juggle_optimizer``).

  - ``pattern``    — oval geometry + tempo specification
  - ``trajectory`` — periodic C2 platform-pose trajectory + analytic builder
  - ``player``     — runtime open-loop trajectory player (wall-time -> command)
  - ``timeline``   — master event schedule (BB priming + hand events) + abort
                     path (``ExitTransient``)
  - ``juggle_optimizer`` — offline CasADi jerk-minimising optimiser
    (level-platform first cut; replaces the analytic baseline). Banking +
    leg-space jerk objective deferred to a Phase 2 follow-up.

Consumers import the submodules explicitly (e.g.
``from sim.juggle_planner.pattern import JugglePattern``); this package
intentionally does not re-export, to keep import order under the consumer's
control.
"""
