# ADR-0015: Each kind of Item has its own ID prefix and its own counter

- **Status**: Accepted
- **Date**: 2026-11-07
- **Deciders**: Harrison + Claude
- **Related**: plan `item-id-per-kind.md` (the implementation), archived plan `map-item-ids.md` § 2 decision 1 (the decision this reverses), Items JB-118 and JB-131

## Context

Item IDs arrived on 2026-10-26 as `JB-<n>`, one sequence across every kind of Item. That design rejected kind-prefixed sequences in writing, for four reasons: one-letter prefixes (`B-12`, `F-7`) clashed with letters that already meant Phases, invariants and test IDs; a mistyped prefix could resolve to a different real Item, since `B-12` and `F-12` would both exist; with one sequence the number alone is the identity and sorts by filing order; and showing the kind was the only gain, which the app's tile already did. Only the first reason was specific to the letters. The second and third still apply to this decision, and the Consequences below accept them. The fourth did not survive: the tile stops at the edge of the app.

Within days the owner found the shared prefix confusing, because an ID says nothing about what it names (JB-118). The first answer, on 2026-11-01, kept the sequence and added a kind pill beside the ID in the map app (JB-131). The pill only exists in the app. An ID is mostly read somewhere else: logbook entries, commit subjects, Night Shift reports, code comments and the terminal.

The migration cost only grows. Twelve days of IDs had already produced 4,066 citations across 665 files (measured at `a058d6d1`) when the owner reversed the answer on 2026-11-07.

## Decision

An Item ID is its kind's prefix and a number counted within that kind: `BUG-12`, `FEAT-7`, `IDEA-3`, `PWAY-2`, `TKT-31`.

- Question Tickets are Tickets and use `TKT`. A Ticket's type is not part of its ID.
- Every Item ID reference that can change is rewritten, historical logbook entries and archived plans included.
- Filenames, stems and `Logbook-Entry:` values keep their names. Renaming them breaks commit trailers that cannot be rewritten and changes the Item identity the map app derives from the stem.
- A frozen crosswalk keeps old commit messages readable. A read that names a `JB-<n>` translates to the new ID, and a write refuses it and names the new ID.
- Plans, Steps and logbook entries stay without IDs.

## Consequences

**Accepted:**

1. **A one-word slip can name a different real Item.** `BUG-12` and `FEAT-12` both exist, the risk the 2026-10-25 design refused. Every write echoes the ID, stem and title, and the Night Shift gate checks that the prefix matches the Item's kind. A slip between two real Items of one kind stays possible, as it was before.
2. **A number alone names nothing.** Up to five Items share one. A bare number on a read lists them all, where it used to find exactly one.
3. **IDs no longer sort by filing order across kinds.** Anything that needs that order reads `created`.
4. **Historical text changes once.** An old entry no longer matches what was written on its date in that one token. The documentation guide's rule against rewriting history gains a narrow, dated exception for this, rather than being dropped, and a blame-ignore entry keeps `git blame` reaching the right `Logbook-Entry:` trailer.

**Gained:**

- `BUG-12` is a bug wherever it is read, with no app needed to say so.
- Concurrent filings of different kinds can no longer collide. Two filings of the same kind still can, so JB-100 narrows but does not close.

## Alternatives considered

- **The kind pill alone** (JB-131): no migration, but it stops at the edge of the app.
- **Per-kind prefixes on one shared counter** (`BUG-13`, `IDEA-14`): a smaller migration in which the number stays globally unique, so a bare number still finds one Item. Rejected for option 3's cleaner read: each kind counts from 1 with no gaps.
- **One-letter prefixes:** the letters are taken (above).
- **A separate `QUESTION-n` prefix:** it shows at a glance that an Item waits on the owner, but it splits one kind across two counters and lets an ID disagree with the Ticket's type.
- **Renaming filenames too:** the tree would hold no `jb` token at all, but every old `Logbook-Entry:` trailer would point at a missing file and the app would lose the identity of 22 Items.
- **`PLAN-n` for plans:** nothing needs a plan number, and plans already have immutable short filenames.
