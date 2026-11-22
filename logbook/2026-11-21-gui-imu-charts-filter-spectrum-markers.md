---
title: "GUI charts: panel height model, low-pass filter and spectrum view for the IMU charts, Throw and Catch markers, IMU capture bands on the replay trackbar, IMU columns in Export CSV"
type: feature
date: 2026-11-21
status: complete
phase: "IDEA-20 tooling"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/gui/js/imu-charts.js
  - ros_ws/gui/js/imu-charts-core.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/ball-events.js
  - ros_ws/gui/js/event-store.js
  - ros_ws/gui/js/command-history.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/js/replay/ui/imu-bands.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/css/theme.css
  - ros_ws/gui/css/replay.css
subsystem: [gui, platform-imu]
tags: [gui, charts, imu, replay]
---

# GUI charts: second round of owner requests (2026-11-21)

Follows `2026-11-21-gui-imu-charts-followups`.

- **Height model (corrected).** The top handle sets the height of the whole chart panel; the middle divider only
  moves the boundary between the IMU charts and the ODrive charts (double-click: even split; each keeps 100 px). The
  panel has one stored height with one group shown and another with both (default: the first plus 200 px), so
  switching the IMU row on does not squash the ODrive grid. The "Charts" label in the top handle and the collapsed
  panel state are removed; a stored collapsed flag is cleared at start.
- **Filter.** Off / 100 / 50 / 15 Hz zero-phase low-pass (second-order Butterworth run forward and backward,
  coefficients from the capture's own sample rate, never across gaps, cached per capture and cutoff). Why: Catch and
  Throw impacts reach 11 g and set the scale, hiding the slow Platform motion.
- **Spectrum view.** Welch amplitude spectra (Hann, up to 4096 points, half overlap) of the raw samples inside the
  visible time range, on a log axis to 10 Hz, 50 Hz or Nyquist, with the largest peak above 1 Hz labelled per series.
  Spectrum charts leave the shared cursor and zoom. On capture `20261120T120634Z_AEE1` the view reads a gyro line at
  77.6 Hz (an independent scipy Welch gives 78.1 Hz) and acceleration peaks at 1.4 and 2.1 Hz; no dominant peak near
  5 Hz, so the ring-mode frequency is not yet read from this data.
- **Throw and Catch markers** on all eleven charts, from `/balls` (already subscribed, recorded and replayable):
  Throw = `throw_time` of the Hand's own balls, the announced release, labelled "(planned)" because no measured
  release time is published; Catch = `time_at_land` from the converged flight fit once the ball is CAUGHT. They are
  chart-only event types, kept out of the Event Log list. Measured against the Platform IMU on the 32-Throw Attempt:
  the IMU spike sits 2 to 6 ms after the Throw marker, and most Catch markers are within 36 ms of the impact spike.
  The Hand's stroke end comes about 0.12 s after the Throw marker (the marker sits near the velocity peak). Ball
  Butler Feeds get no Throw marker. The label number is the ball id.
- **Trackbar bands.** In REPLAY each IMU capture of the recording is a purple band on the trackbar with a tooltip.
- **Export CSV.** With the IMU group shown and a capture loaded, the existing export gains six `imu_*` columns (raw,
  sensor frame); rows are the time-sorted union of motor and IMU sample times with empty cells where a source has no
  sample. With the group hidden the export is as before (same code path; not compared byte for byte).

## Verification

Headless Chromium against the worktree server and recording `2026-11-20_22-53-34` (2026-11-21): each item above was
exercised with no console errors (heights and persistence, bands at the four capture times, an 88-column export,
the 15 Hz filter, spectrum peaks, markers and tooltips on IMU and motor charts). Not exercised: a live launch (the
Throw and Catch mapping of a live `/balls` stream is unit-tested only). Tests: see the commit message.
