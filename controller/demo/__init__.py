"""Open-loop juggling-demo trajectory tooling.

Pure-Python (no ROS2) modules for the Ball-Butler-initiated two-ball oval
juggling demo — see ``plans/active/bb-led-two-ball-juggle-demo.md``.

  - ``pattern``    — oval geometry + tempo specification
  - ``trajectory`` — periodic C2 platform-pose trajectory + analytic builder
  - ``player``     — runtime open-loop trajectory player (wall-time -> command)
  - ``timeline``   — master event schedule (BB priming + hand events) + abort
                     path (``ExitTransient``)
  - ``juggle_optimizer`` — offline CasADi jerk-minimising optimiser (Phase 2,
    pending — replaces the analytic trajectory builder)

Consumers import the submodules explicitly (e.g.
``from controller.demo.pattern import JugglePattern``); this package
intentionally does not re-export, to keep import order under the consumer's
control.
"""
