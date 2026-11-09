# id_cutover: proofs for the per-kind Item ID renumbering

These are the P4 cutover's frozen evidence. They ran against P4's tree and name files that P6 deleted
(the sweeper, its overrides and its tests), so to re-run one, check out the renumber commit `38058788`.

Plan `plans/active/item-id-per-kind.md` section 4 P4. Pre-sweep tree = `git HEAD`, post-sweep
tree = the working tree. Run them before the renumber commit: they read `HEAD` as the old side.
Needs the venv (`source ~/Desktop/PDJ_venv/venv/bin/activate`), `mapkit/legacy_ids.csv`
and the sweep's `temp/id-sweep/` reports. Evidence goes to `temp/probes/id_cutover/`.

- `t_r2_round_trip.py`: inverting the crosswalk over every changed file reproduces `HEAD` line
  by line. Pass = 0 differences; exceptions are counted by kind (override, list-range,
  generated, hand-edited, fixture) and listed in the evidence file.
- `t_r3_python_ast.py`: changed `.py` outside tests/mapkit/night_shift/sweeper have an identical
  AST and token stream once strings and comments are masked, and each changed string is
  explained by the crosswalk. Pass = 0 failures, 0 unexplained strings.
- `t_r4_firmware_hash.sh`: builds (never uploads) both Teensy images and compares each
  `firmware.hex` sha256 with `BASELINE` (default `temp/id-migration/p4-fw-before.txt` in the
  main checkout); no `__DATE__`/`__TIME__`/`__FILE__`/`__LINE__` in either tree. Pass = hex equal.
- `t_r5_generated_constants.py`: every changed line under `config/`, both firmware trees and
  the profiling probes is a comment or an Item ID inside a string or comment; generated
  Python modules load with equal non-string constants; `MAGIC` and `PROTOCOL_VERSION` unchanged.
- `t_r6_zero_count.py`: no old `JB-<n>` outside the frozen files, fixture residue, code_literal
  lines, kept override rows and real hyphenated file names; `ids --check` clean (before the
  commit only its `changed:` lines are allowed; `STRICT_IDS=1` drops that); the map's ids are
  exactly 1..N per kind and equal the crosswalk's. Scope: only the upper-case `JB-<n>` shape is
  counted (the proof's own directory is the `proof-self` class and the three hyphenated file names
  are pinned by file:line); lower-case / padded forms (`jb149`, `JB-0149`) are the P5 guard's job, not T-R6's.
- `run_all.sh`: all five in order (T-R4 last), summary table, non-zero if any failed.
