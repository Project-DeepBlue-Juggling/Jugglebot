---
title: <descriptive title>
type: investigation        # investigation | bugfix | refactor | feature | optimization
date: YYYY-MM-DD
status: open               # open | in-progress | tuned | resolved
#
# Status ladder (for investigation entries):
#   open         — nothing done yet
#   in-progress  — diagnosis done, fix/verification still ongoing
#   tuned        — the specific symptom scoped to this entry is addressed
#                  and verified on hardware, but there is an intentionally
#                  open sibling investigation elsewhere. Use `tuned` rather
#                  than leaving a real ship at `in-progress` forever.
#   resolved     — every symptom in scope is addressed and verified; no open
#                  follow-ups inside this entry's scope
#
# --- Context (use what's relevant) ---
phase: ""                  # bringup phase, if applicable
related_plan: ""           # filename only (e.g., hardware-bringup.md), searched in both plans/active/ and plans/archived/
related_issues:            # known_issues.yaml IDs, if applicable
  - <KNOWN_ISSUE_ID>
sessions:                  # MPC telemetry CSVs, if applicable
  - mpc_YYYYMMDD_HHMMSS.csv
# --- Traceability ---
files_changed:             # every file modified (enables reverse lookups)
  - <path/to/file.py>
commits:                   # commit short hashes (filled after committing)
  - <abcdef0>
# --- Classification ---
subsystem:                 # primary subsystem(s) affected
  - <subsystem>            # mpc | can | tracking | motion | sim | gui | config | controller | ros
tags:                      # additional descriptors
  - <tag>                  # safety | performance | ux | testing | docs | IPC | kinematics | dynamics
---

# <title>

## Summary

<2-3 sentence overview: what changed and why.>

<!---------------------------------------------------------------------
  SECTION GUIDE — use the sections relevant to your entry type.
  Delete or leave empty any sections that don't apply.

  investigation:  Symptoms, Diagnosis, Discussion, Fix, Outcome
  bugfix:         Problem, Root Cause, Fix, Verification
  refactor:       Motivation, Changes, Migration Notes, Verification
  feature:        Motivation, Design, Implementation, Verification
  optimization:   Motivation, Approach, Benchmarks, Verification
---------------------------------------------------------------------->

<!-- === Investigation sections === -->

## Symptoms

<What was observed. Be concrete.>

## Diagnosis

<Analysis results, verdict, key metrics.>

### Flagged Issues

<Flags from /diagnose with severity and known-issue cross-references.>

<!-- === Bugfix sections === -->

## Problem

<What's broken and how it manifests.>

## Root Cause

<Why it's broken — trace to the specific code path.>

<!-- === Refactor / Feature / Optimization sections === -->

## Motivation

<Why this change is needed. What problem does it solve or what capability does it add?>

## Design

<Architecture decisions, API changes, key design choices. For refactors, describe
the before/after structure.>

## Changes

<For refactors: what moved where, what was renamed, what was deleted.>

## Implementation

<For features: how it works, key code paths, integration points.>

## Approach

<For optimizations: what was tried, what worked, what didn't.>

## Benchmarks

<For optimizations: before/after measurements.>

<!-- === Common sections (all types) === -->

## Fix

<What code changes were made. File paths, brief description of each change.
For non-bugfix types, rename this to "Changes" or "Implementation" above.>

## Verification

<How the change was verified: tests run, hardware re-test, manual checks,
before/after metrics.>

## Outcome

<Results. Was the goal achieved? Any unexpected side effects?>

## Withdrawn claims

<!--
  Use this section when an earlier interpretation within this entry
  (or an earlier message in the investigating conversation) turned out
  to be wrong. Record the retraction here rather than silently editing
  the wrong claim out of the Diagnosis / Verification sections.

  Format:
    - [YYYY-MM-DD HH:MM] <one-line summary of the wrong claim>
      WITHDRAWN: <why it was wrong, with evidence>
      Superseded by: <what's true instead, with pointer to the section
                      or commit that has the corrected finding>

  Example (from the 2026-04-18 hold-fighting investigation):
    - [2026-04-19 09:30] Claimed "gain halving applied" from a
      hold-stdev drop across multiple legs.
      WITHDRAWN: comparison used a 3 s window vs a 0.5 s window on
      different pose targets; the drop was from the different operating
      point, not from the gain change. User's ODriveGUI verification
      (all ODrives read 40/0.2/0.32) is correct.
      Superseded by: Fix section Part 2, commit 56b7514.

  Leave this section empty if no claims were withdrawn. Do NOT delete
  wrong claims to hide them — that destroys the investigation history
  and makes the next person repeat the mistake.
-->

## Open Questions

<Anything unresolved. Follow-up items. Future work.>
