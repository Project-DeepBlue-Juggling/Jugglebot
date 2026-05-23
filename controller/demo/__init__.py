"""Open-loop juggling-demo trajectory tooling.

Pure-Python (no ROS2) modules for the Ball-Butler-initiated two-ball oval
juggling demo — see ``plans/active/bb-led-two-ball-juggle-demo.md``.

  - ``pattern``    — oval geometry + tempo specification
  - ``trajectory`` — periodic C2 platform-pose trajectory + analytic builder
  - ``player``     — runtime open-loop trajectory player (wall-time -> command)
  - ``timeline``   — master event schedule (BB priming + hand events) + abort
                     path (``ExitTransient``)
  - ``juggle_optimizer`` — offline CasADi jerk-minimising optimiser
    (level-platform first cut; replaces the analytic baseline). Banking +
    leg-space jerk objective deferred to a Phase 2 follow-up.

Consumers import the submodules explicitly (e.g.
``from controller.demo.pattern import JugglePattern``); this package
intentionally does not re-export, to keep import order under the consumer's
control.
"""
