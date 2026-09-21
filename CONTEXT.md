# Jugglebot

The shared language of the Jugglebot project: a Stewart-platform robot that throws and catches balls, the machines around it, and the way work on it is organised. This file is a glossary and nothing else — use these terms, and not the words listed under _Avoid_, in code, plans, logbook entries and issues.

## Language

### The machine

**Jugglebot**:
The whole robot: the Stewart platform, the Hand, and the electronics that drive them.
_Avoid_: "the platform" for the whole robot

**Stewart platform**:
The six-Leg parallel mechanism that positions and tilts the Platform. Used only when the mechanism itself is the subject.

**Platform**:
The moving top plate of the Stewart platform, and the coordinate frame fixed to it.
_Avoid_: Platform for the robot as a whole; Platform for the Platform Teensy

**Leg**:
One of the six linear actuators of the Stewart platform, indexed 0–5. The Hand is never a Leg.
_Avoid_: strut, actuator (for a specific Leg)

**Hand**:
The seventh actuated axis: a linear stroke, carried on the Platform, that throws and catches the ball.
_Avoid_: leg 6, seventh leg

**Cup**:
The part of the Hand that holds the ball. The Hand is one monolithic object; the Cup is its ball-holding end, not a separate part.
_Avoid_: cone

**Cone**:
The catching cone: a removable, sensorized catching attachment with its own electronics. Never a word for the Cup.

**Axis 6**:
The Hand's index on the wire and in firmware. Valid only in protocol and firmware contexts; everywhere else it is the Hand.

**Ball Butler (BB)**:
The separate ball-feeding machine that throws balls to Jugglebot.
_Avoid_: butler, feeder

### Electronics and links

**can-bridge**:
The Teensy 4.1 that owns all CAN traffic to the motor controllers and streams motion to them; the Leg-path safety authority. "The bridge", unqualified, means this board and nothing else. Its Jetson-side counterpart is always written `teensy_bridge_node`, and rosbridge (the GUI's websocket) is unrelated.
_Avoid_: can-hub, canhub, "the Teensy" (there are two)

**Platform Teensy**:
The Teensy 4.0 mounted on the robot. Always written in full.
_Avoid_: Platform (alone), "the Teensy"

### States and poses

**Idle**:
An axis whose motor is unpowered and free to move.

**Closed-loop**:
An axis whose motor controller is actively holding or tracking a position.
_Avoid_: armed (for a motor controller)

**Homing**:
The power-up procedure that finds an axis's zero. "Home" is never a pose.
_Avoid_: home pose, "at home"

**Stow**:
The Platform's lowered rest pose, with every axis Idle; also the origin of the planning frame.
_Avoid_: home

**Active**:
The state reached by the operator's ACTIVATE: the Legs Closed-loop and holding the Active pose.

**Active pose**:
The raised pose the Platform holds once Active, from which every Attempt starts.
_Avoid_: home, operating pose

**Park**:
The Hand at the bottom of its stroke.

**Armed**:
Streaming motion to the motors is permitted. Armed means this and nothing else.
_Avoid_: armed for the Guard (it is enabled), armed for a motor controller (it is Closed-loop)

**Tilt**:
Any inclination of the Platform.

**Level**:
The Platform's normal aligned with gravity — not with the base.
_Avoid_: level for "parallel to the base"

**Tilt map**:
The measured calibration of the Platform's Tilt error across its workspace, used to make it Level.

**Banking**:
Tilt commanded on purpose so the Cup meets or releases the ball along the right axis.
_Avoid_: tilt (when Banking is meant)

### Safety

**Guard**:
The can-bridge's protective layer; the only safety authority on the Leg path. `motor_guard` is a different thing and is always named by its filename.
_Avoid_: "the guard" for `motor_guard`, safety layer

**Soft E-stop**:
A latched stop raised in software by the Guard, cleared only by an explicit operator command.
_Avoid_: guard E-STOP, bare "E-STOP"

**E-stop button**:
The operator's physical emergency stop, independent of all software.
_Avoid_: bare "E-STOP"

**Watchdog**:
One of the Guard's liveness detectors. Some raise a Soft E-stop; others only suppress output until the signal returns.

**Fault**:
Any abnormal state the can-bridge reports, latched or not. Every Soft E-stop is a Fault; not every Fault is a Soft E-stop.

### Juggling

**Throw**:
Any release of a ball into flight, by the Hand or by Ball Butler. In capitals, THROW is a Skill kind.
_Avoid_: toss, self-toss (say self-throw)

**Feed**:
A Throw by Ball Butler to Jugglebot.
_Avoid_: reload

**Catch**:
The Cup receiving a ball in flight. In capitals, CATCH is a Skill kind.

**Site**:
A named position of the Cup opening at which balls are thrown and caught.
_Avoid_: catch pose, catch position

**Landing**:
The arrival of a ball at the catch plane — where, when and how fast — whether predicted or observed. A Landing belongs to the ball; a Site belongs to the robot.
_Avoid_: catch pose

**Apex**:
The peak height of a ball's flight above the catch plane.
_Avoid_: height, throw height

**Dwell**:
The time a ball spends in the Cup between being caught and being thrown.

**Beat (β)**:
The interval between consecutive Throws of a Pattern, whichever Sites they leave from.
_Avoid_: cycle, period (for the rhythm)

**Pattern**:
A juggling arrangement — its Sites, Apex and Beat — from which a Schedule is compiled.

**Columns**:
The two-ball, one-Hand Pattern in which vertical Throws alternate between two Sites.

### Skills and motion

**Skill**:
The atomic unit of a Schedule: one THROW, CATCH or REST, of a given ball, at a given Site, at an absolute time.
_Avoid_: primitive, action

**REST**:
The Skill kind that brings the robot to rest at a Site. Every Attempt opens and ends with one.

**Schedule**:
The list of Skills, at absolute wall-clock times, that realises a Pattern. A Schedule never delays.
_Avoid_: plan (for a Schedule — a plan is a document of intended work), trajectory, sequence

**Segment**:
The planned motion, across the six Legs and the Hand, that realises one Skill. Every Segment ends at rest.
_Avoid_: trajectory, cycle

**Knot**:
One point of a Segment at which the full state of every axis is pinned.
_Avoid_: waypoint, sample, keyframe

**Splice**:
The join of a new Segment onto the motion already being streamed, at a future Knot.

**Admissible box**:
The pre-computed bound on a Skill's command inside which every Segment is known to be feasible.
_Avoid_: feasibility box, bare "box"

**Feasibility gate**:
The check that a candidate Segment respects every limit of the machine.
_Avoid_: bare "gate"

**Sitting limits**:
The velocity, acceleration and jerk limits in force for the current Sitting.
_Avoid_: session limits

### Ball tracking

**Mocap**:
The motion-capture system that reports marker positions. QTM names the vendor software only.

**Tracker**:
The estimator that turns Mocap markers into ball flights and their Landings.
_Avoid_: mocap, QTM (for the estimator)

**Fit**:
The ballistic fit to one observed flight, and the Landing it yields; the trusted landing estimate once it has converged.

### Learning

**Learner**:
The memory-based method that corrects each Throw's command from the outcomes of past Throws.
_Avoid_: ILC

**Memory**:
The Learner's store of past Throws: what was commanded and what landed.

### Process

**Sitting**:
One continuous powered period at the robot, with the operator present.
_Avoid_: session, run

**Attempt**:
One execution of a Schedule, from its opening REST to its ending REST. A Sitting holds many Attempts.
_Avoid_: run, trial

**Session**:
One Claude Code conversation. Never used for time at the robot.

**Rung**:
A step of a plan that closes only on a hardware acceptance criterion met at a Sitting.
_Avoid_: phase (for a hardware-gated step)

**Phase**:
A step of a plan that closes without a Sitting.

**Rung gate**:
The acceptance criterion that closes a Rung.
_Avoid_: bare "gate"

**Suite gate**:
The test run that must pass before a commit.
_Avoid_: bare "gate", "the tests"

**Ladder**:
A graduated sweep of test points flown within one Sitting (an Apex ladder). Rungs are not the steps of a Ladder.

**Runbook**:
The written, step-by-step procedure an operator follows at a Sitting.
_Avoid_: runsheet, script
