---
description: Generate a detailed implementation report for a proposed refactor, feature, or architectural change. Produces a markdown document with context, architecture, phased implementation plan, rigorous testing plan, and collaborator notes.
---

# Implementation Report Generator

You are generating a detailed implementation report for the following task:

$ARGUMENTS

## Process

### Phase 1: Understand the Request

Parse the user's description to identify:
- **What** is being changed (module, subsystem, architecture)
- **Why** it's being changed (problem statement, motivation)
- **Scope** — which files, interfaces, and protocols are affected

If the description is ambiguous, ask clarifying questions before proceeding.

### Phase 2: Deep Codebase Exploration

Launch Explore agents (up to 3 in parallel) to thoroughly understand the current implementation:

1. **Primary target** — Read every file that will be modified or replaced. Extract exact function signatures, class structures, line counts, and key algorithms.
2. **Integration points** — Identify all callers, subscribers, publishers, IPC channels, and data flows that touch the target code. Trace the full path of data through the system.
3. **Protocols and conventions** — Document message formats, encoding conventions, safety invariants, sign conventions, scaling factors, and any "magic numbers" with their origins.

For each file explored, note:
- File path and approximate line count
- Public API (classes, functions, constants)
- Key internal details relevant to the refactor
- Safety-critical invariants that must be preserved

### Phase 3: Architecture Design

Based on your exploration:

1. Draw the **current architecture** (ASCII diagram showing data flow, processes, IPC)
2. Draw the **proposed architecture** (same format, showing the changes)
3. Identify **new IPC channels, ports, or message formats** needed
4. Identify **files to create, modify, and archive**

### Phase 4: Write the Report

Save the report as a markdown file in `plans/active/` using kebab-case (e.g., `can-process-refactor.md`, `kalman-filter-redesign.md`). Add YAML frontmatter with `title`, `created` (today's date), and `status: active`.

The report MUST contain these sections, in this order:

#### 1. Context
- What problem or limitation motivates this change
- What the change achieves (concrete benefits, not vague improvements)
- When to do it (prerequisites, dependencies, recommended timing)
- Link to any related incidents, analysis documents, or prior work

#### 2. Architecture
- Current architecture (ASCII diagram + explanation)
- Proposed architecture (ASCII diagram + explanation)
- New IPC channels, ports, message formats (with exact field definitions)
- What changes vs what stays the same

#### 3. Implementation Phase Summary
A table with these columns:

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|

- **Status** must be one of: `NOT STARTED`, `IN PROGRESS`, `COMPLETE`
- **Date** is blank for NOT STARTED, the start date for IN PROGRESS, or the completion date for COMPLETE (format: YYYY-MM-DD)
- Phases must be incremental — each phase produces a testable, deployable intermediate state
- Earlier phases must not depend on later phases

Initialize all phases as `NOT STARTED` with blank dates. The report author or implementer updates status and dates as work progresses.

#### 4. Implementation Phases (detailed)
For each phase, include a status header and then the details:

```
### Phase N: Title — STATUS (date)
```

Where STATUS is `NOT STARTED`, `IN PROGRESS (started YYYY-MM-DD)`, or `COMPLETE (YYYY-MM-DD)`.

Then for each phase:
- **New/modified files** with exact paths
- **Scope** — what this phase adds or changes
- **IPC protocol** — exact message format (Python dict literals) for any new messages
- **Critical details** — safety invariants, encoding conventions, edge cases
- **Dependencies** — what must exist before this phase can start

#### 5. Testing Plan
This section must be **extremely rigorous**. Organize tests into categories:

- **Unit tests** (offline, no hardware) — Mock external dependencies. Test every encoding/decoding path, every safety check, every error handler. Include edge cases: NaN inputs, timeout conditions, concurrent operations.
- **Integration tests** (real system, safe conditions) — Test IPC connectivity, latency measurement, state propagation. Compare output against current system.
- **Hardware tests** (real actuators, E-stop ready) — Replicate existing passing tests under the new architecture. Include endurance tests.
- **Regression tests** — Bit-exact comparisons against current system output. Verify no behavioral changes in preserved interfaces.

Each test must have:
- A test ID (T-U1, T-I1, T-H1, T-R1 format)
- A clear description of what it validates
- Explicit pass/fail criteria where applicable

#### 6. Notes for Collaborators
- **Safety-critical invariants** — List every convention, threshold, or encoding rule that must be preserved exactly, with file locations and consequences of getting it wrong
- **Architecture decisions** — Explain non-obvious design choices and their rationale
- **Startup/shutdown ordering** — Process dependencies and sequencing
- **Files affected** — Table of all files created, modified, or archived
- **Rollback plan** — How to revert to the previous architecture if issues arise

### Phase 5: Review

Before presenting the report:
- Verify all file paths mentioned actually exist in the codebase
- Verify all function/class names mentioned are current (grep for them)
- Verify IPC port numbers don't conflict with existing assignments
- Ensure the phase ordering makes sense (no forward dependencies)
- Ensure the testing plan covers every new code path

## Style Guidelines

- Use concrete values, not vague descriptions ("0.3 rev" not "the step limit")
- Include exact file paths relative to repo root
- Show message formats as Python dict literals, not prose descriptions
- ASCII diagrams for architecture (not Mermaid or other formats)
- Keep each section scannable — use tables, bullet lists, and code blocks
- Do not pad with filler or repeat information across sections
