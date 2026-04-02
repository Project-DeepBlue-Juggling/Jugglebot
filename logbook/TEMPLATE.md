---
title: <descriptive title>
type: investigation        # investigation | bugfix | refactor | feature | optimization
date: YYYY-MM-DD
status: open               # open | in-progress | resolved
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

## Open Questions

<Anything unresolved. Follow-up items. Future work.>
