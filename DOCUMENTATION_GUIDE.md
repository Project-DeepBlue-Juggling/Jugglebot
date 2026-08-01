# Documentation Guide

The single reference for **how documentation is organised in Jugglebot** — where
each kind of information lives, what format it takes, and how the layers cross-
reference each other. Both human collaborators and Claude instances should
consult this before creating or editing any prose/markdown artifact.

If you're making a code change and wondering "which file(s) do I need to
update?", jump straight to [§ Decision Tree](#decision-tree).

---

## 1. Documentation layers at a glance

Jugglebot has **eight distinct documentation layers**. Each has a specific
purpose; duplication between them is a bug. Pick the right layer and keep
authoritative content in exactly one place — cross-reference from the others.

| # | Layer | Path | Audience | What belongs here |
|---|-------|------|----------|-------------------|
| 1 | **Claude instructions** | [CLAUDE.md](CLAUDE.md) | Claude | Project overview, workflow rules, critical conventions |
| 2 | **Public README** | [README.md](README.md) | Public visitors | One-paragraph project blurb + link to the project chat / external project home |
| 3 | **Published technical docs** | [docs/](docs/) + [mkdocs.yml](mkdocs.yml) | Engineers reading the MkDocs site | Subsystem architecture, kinematics, algorithms, usage guides |
| 4 | **Subsystem docs** | `<subsystem>/docs/`, `<subsystem>/README.md`, in-tree `*.md` | Engineers editing that subsystem | Normative specs & conventions local to one area |
| 5 | **Engineering logbook** | [logbook/](logbook/) | Engineers (human + Claude) investigating past work | Per-change entries: *why* a commit looks the way it does |
| 6 | **Plans** | [plans/active/](plans/active/), [plans/archived/](plans/archived/) | Implementers and reviewers | Forward-looking implementation reports & bringup plans |
| 7 | **Automation layer** | [.claude/commands/](.claude/commands/), [.claude/agents/](.claude/agents/) | Claude (and anyone editing the workflow) | Slash-command protocols and dedicated agent specs |
| 8 | **Inline (source code)** | `*.py`, `*.h`, `*.yaml` | Engineers reading the code | Docstrings, module headers, non-obvious *why* comments |

> Claude's **auto-memory** at `~/.claude/.../memory/` is Claude-private and is
> deliberately **not** a documentation layer. Do not copy project docs there.

---

## 2. Layer-by-layer reference

### 2.1 [CLAUDE.md](CLAUDE.md) — Claude-facing instructions

The project root's `CLAUDE.md` is the top-of-context file every Claude session
loads. It is deliberately terse. It must contain, and only contain:

- One-paragraph project identity
- Architecture snapshot (directory tree, boundaries)
- Required environment / venv
- Top-level commands (build, run, test, simulate)
- **Workflow rules** (grep-before-refactor, test-after-change, etc.)
- **Critical conventions** (sign conventions, unit conventions, Py 3.8 rules)
- Pointers to deeper docs (including this file)

**Do not** put investigation write-ups, plan content, or large reference
material here. Point to the authoritative source instead.

### 2.2 [README.md](README.md) — Public landing

Minimal. One-paragraph project blurb plus a link to the external project home
(Zulip chat). Does not duplicate internal docs.

### 2.3 [docs/](docs/) + [mkdocs.yml](mkdocs.yml) — Published technical docs

The MkDocs site at `docs/` is the **authoritative technical reference** for
how subsystems work. Preview locally with `mkdocs serve` (port 8000).

Layout conventions:

```
docs/
  index.md                 # top-level overview (mirrors mkdocs.yml nav)
  <section>/index.md       # section landing page
  <section>/<topic>.md     # one topic per page
  stylesheets/extra.css    # site-wide styling
```

Every new page must be registered in `mkdocs.yml → nav:`. Section names map
to directories; filenames are lowercase, hyphenated (or underscore where
matching existing style — `motion_planner/`, `sim_mpc/`, `analysis/`).

**When to add here:** a stable piece of knowledge about how a subsystem
works that will be referenced more than once. If the content is specific to
one investigation or one change, it belongs in the **logbook**, not here.

**Math** — `pymdownx.arithmatex` is enabled; use `$...$` and `$$...$$`.

### 2.4 Subsystem docs — normative specs living next to the code

Three patterns exist, each justified by the content's tight coupling to one
subtree:

- **`<subsystem>/docs/<topic>.md`** — narrow reference docs that don't warrant
  a top-level MkDocs page. Example: [ros_ws/docs/safety.md](ros_ws/docs/safety.md),
  [ros_ws/docs/control_modes.md](ros_ws/docs/control_modes.md).
- **`<subsystem>/README.md`** — convention/usage guide for the files in that
  directory. Example: [tools/README.md](tools/README.md) (describes the
  `--preview` convention that all harnesses must follow).
- **`<subsystem>/<CONTRACT_NAME>.md`** — normative contracts that the code
  in that subtree must satisfy. Example:
  [controller/REFERENCE_LAYER_CONTRACT.md](controller/REFERENCE_LAYER_CONTRACT.md)
  (the K1–K6 reference-layer invariants).

**When to use:** the content is a *rule* or *convention* scoped to one
subtree and changes together with that subtree. If a human editing that
directory would want the rule on-screen, put it in the subtree; if it's a
general-audience reference, put it in [docs/](docs/).

### 2.5 [logbook/](logbook/) — Engineering logbook

The logbook is the authoritative record of **every code change**: symptoms,
diagnosis, alternatives considered, fix, outcome. It's the answer to "why
does this code look this way?"

Full spec: [logbook/README.md](logbook/README.md). Template:
[logbook/TEMPLATE.md](logbook/TEMPLATE.md). Index: [logbook/INDEX.md](logbook/INDEX.md).

Essentials you must know:

- **Filename:** `YYYY-MM-DD-<slug>.md` (slug is 3–6 hyphen-separated words).
- **Frontmatter** (YAML between `---` delimiters) is mandatory. Fields:
  `title`, `type`, `date`, `status`, and optional `phase`, `related_plan`,
  `related_issues`, `sessions`, `files_changed`, `commits`, `subsystem`,
  `tags`.
- **`type` (controlled):** `investigation | bugfix | refactor | feature | optimization`.
  Each type has its own required body sections — see
  [logbook/README.md § Entry types](logbook/README.md#entry-types-and-their-sections).
- **`status` (ladder):** `open → in-progress → tuned | resolved`.
  Use `tuned` (not `in-progress`) when this entry's scope is shipped but a
  sibling investigation is deliberately parked elsewhere.
- **`related_plan`:** filename only (e.g. `hardware-bringup.md`), never a path.
  Search both `plans/active/` and `plans/archived/` to resolve. This keeps the
  reference stable when a plan is archived.
- **`subsystem` taxonomy (controlled):** `mpc | controller | motion | can | tracking | ros | gui | sim | config | tools`.
  Auto-derivable from the file paths in `files_changed`.
- **`tags` taxonomy (controlled):** `safety | performance | IPC | kinematics | dynamics | testing | docs`.
- **Withdrawn claims:** investigation entries never delete incorrect earlier
  conclusions — they move them into a "Withdrawn claims" section near the
  bottom (format in [logbook/TEMPLATE.md](logbook/TEMPLATE.md)). This is a
  hard rule: silently editing the wrong claim out destroys investigation
  history.
- **[INDEX.md](logbook/INDEX.md)** is the single sortable table of all entries.
  It is updated by the `logbook-updater` agent — do not hand-edit unless
  fixing a bug the agent introduced.

**Creating entries:**
- Hardware investigation → `/investigate`
- Non-hardware code change → `/log <type> <title>` (types listed above)
- Manual / blank → `/logbook --new "title"`

**Cross-references from a logbook entry:**
- `related_plan:` → a plan file (by filename)
- `sessions:` → rows in [sim/analysis/log_index.json](sim/analysis/log_index.json)
  (bidirectional — `log_index.json` gets `logbook_entry` pointers added)
- the commit's `Logbook-Entry: <slug>` trailer → the canonical link in both
  directions (`git blame` → commit → entry, and
  `git log --grep "Logbook-Entry: <slug>"` → every commit for the entry).
  Entries do **not** carry a `commits:` list; SHA backfill was retired
  2026-08-01 (historical entries keep theirs)

### 2.6 [plans/active/](plans/active/) + [plans/archived/](plans/archived/) — Plans

Plans are **forward-looking** implementation reports or bringup roadmaps. The
logbook is retrospective; plans are prospective.

Layout:

```
plans/active/<name>.md                    # in progress
plans/archived/YYYY-MM-DD <name>.md       # completion date prefixed at archival
```

Filename is kebab-case. Archived files keep their original filename but
gain a leading `YYYY-MM-DD ` date prefix (the completion/supersession date).

**Frontmatter:**

```yaml
---
title: <descriptive title>
created: YYYY-MM-DD
status: active            # active | completed | superseded
completed: YYYY-MM-DD     # only when status != active
# Optional, for tuning/methodology plans:
owner: <name>
last_updated: YYYY-MM-DD
related_logbook:          # logbook filenames this plan discusses
  - YYYY-MM-DD-slug.md
related_config:           # config files this plan pins values in
  - path → field.name
related_code:             # specific functions this plan governs
  - path::symbol
---
```

**Body structure** for implementation reports is defined by the
`/implementation-report` slash command — see
[.claude/commands/implementation-report.md](.claude/commands/implementation-report.md).
The canonical section order is **Context → Architecture → Implementation Phase
Summary (table) → Implementation Phases (detailed) → Testing Plan → Notes for
Collaborators**.

**Voice & style.** Plans are *informational roadmaps*, not conversations.

- **Future tense, active voice.** "The detector will be added to
  `diagnose.py`" — not "I'll add the detector" and not "the detector is
  added". The plan describes work that *will* be done; once done it
  moves to the logbook.
- **No first-person pronouns.** No "I", "we", "me", "my", "us". A plan
  has no author voice — it's a deliverable. Replace "I propose X" with
  "The proposal is X" or just "X".
- **No second-person addressing the reader.** No "do you buy this
  ranking?", "should I add iq logging?", "what's your appetite for…".
  Open questions become declarative: *"Whether iq is logged at 500 Hz
  is unconfirmed; resolution required before §4.5"*. Decision points
  become *"Decision required: …"*, not *"Awaiting your call on …"*.
- **No conversation-closer artefacts.** No "Awaiting approval before
  writing any code", no "let me know if…", no "happy to revise". The
  plan stands as the deliverable; approvals and revisions are
  conversation-layer concerns, not document content.
- **Declarative throughout.** State the mechanism, the predicted
  signature, the proposed fix, the acceptance criterion. Use
  imperative for procedures ("Run X. Capture Y. Compare Z."). Avoid
  hedges that read as deliberative speech ("I'd recommend", "I think
  we should"); prefer "The recommended approach is X because Y".

The voice rule applies to *new* plan prose. Plans authored before this
guidance was added (pre-2026-05-08) may carry residual first-person
text; rewrite on next substantive edit, not pre-emptively.

A worked example of a rewrite (pre-2026-05-08 → post): see the
diff in [plans/archived/2026-05-08 motion-onset-deadtime-investigation.md](plans/archived/2026-05-08%20motion-onset-deadtime-investigation.md)
where §2 intro, §6 (Open Questions), and the closing line were
restyled at supersession.

**Lifecycle:**
1. Create with `/implementation-report <task description>` (or by hand for
   simple bringup plans).
2. Live in `plans/active/`. Update status/dates on each phase as work
   progresses.
3. Archive with `/archive-plan <name>` — the `plan-reviewer` agent critically
   checks every phase against the actual codebase before allowing archival.

**Cross-reference hygiene** — when a plan references another plan, reference
by filename-with-date-prefix once archived (e.g. `plans/archived/2026-03-30
mpc-oscillation-analysis.md`). `/archive-plan` updates inbound references
automatically; verify with a grep if you move a plan by hand.

### 2.7 [.claude/commands/](.claude/commands/) + [.claude/agents/](.claude/agents/) — Automation layer

Slash commands and agents are themselves documented as markdown. They define
the *protocols* the logbook and plans layers depend on.

- **Slash commands** — every file is the full specification of one command.
  Frontmatter: `description` (one line) and optionally `disable-model-invocation`.
  The body is the Protocol: step-by-step instructions, arguments, gates, and
  example output formats.
- **Agents** — specialised sub-agents invoked by slash commands. Frontmatter:
  `name` and `description`. Body documents the agent's input, process,
  output format, and guidelines.

**When editing:** the slash command or agent file *is* the source of truth.
If you change behaviour in practice (e.g. a new gate, a new output field),
update the command/agent definition in the same commit. The motor-guard-style
rule of "never drift between code and contract" applies here too.

Full inventory:

| File | Purpose |
|------|---------|
| [commands/investigate.md](.claude/commands/investigate.md) | Hardware diagnose → fix → commit pipeline (gated) |
| [commands/diagnose.md](.claude/commands/diagnose.md) | Telemetry + rosbag analysis |
| [commands/log.md](.claude/commands/log.md) | Non-hardware logbook entry creation |
| [commands/logbook.md](.claude/commands/logbook.md) | Browse / search / filter entries |
| [commands/archive-plan.md](.claude/commands/archive-plan.md) | Move a plan to `archived/` (reviewed) |
| [commands/implementation-report.md](.claude/commands/implementation-report.md) | Generate a new plan document |
| [commands/audit.md](.claude/commands/audit.md) | Review recent diff + propose fixes |
| [commands/commit.md](.claude/commands/commit.md) | Propose and execute commit groupings |
| [agents/fix-proposer.md](.claude/agents/fix-proposer.md) | Proposes 1–3 fixes with risk + prior-art search |
| [agents/logbook-updater.md](.claude/agents/logbook-updater.md) | Mechanical logbook file management |
| [agents/plan-reviewer.md](.claude/agents/plan-reviewer.md) | Critical pre-archive plan review |
| [agents/audit-reporter.md](.claude/agents/audit-reporter.md) | Read-only diff audit → findings |
| [agents/audit-fixer.md](.claude/agents/audit-fixer.md) | Takes approved findings → fix proposals |

### 2.8 Inline source-code documentation

Source-level docs are the lowest-level layer but governed by the same
economy-of-words principle. Rules:

- **Module / file header** — optional one-line purpose docstring. Use when
  the filename alone doesn't make the role obvious.
- **Function docstrings** — only when behaviour is non-obvious from the
  signature and body. No `"""Compute foo"""` for `def compute_foo(...)`.
- **Inline comments** — reserved for **why**, never **what**. A comment is
  warranted when it documents a hidden constraint, a subtle invariant, a
  workaround for a specific bug, or behaviour that would surprise a reader.
  If deleting it wouldn't confuse a future reader, don't write it.
- **No task-journal comments.** Do not reference the current task, PR, or
  caller ("added for the X flow"). That belongs in the commit message and
  logbook entry, not in code that outlives them.
- **Control-system comments are load-bearing.** Sign conventions, Jacobian
  conventions, and unit conventions documented inline are part of the safety
  invariants. Treat them as normative — if the code changes, update the
  comment in the same commit.

---

## 3. Frontmatter cheat sheet

One-glance reference for what goes in the `---` YAML block of each artifact.

| Layer | Required | Optional | Controlled vocabulary |
|-------|----------|----------|-----------------------|
| **logbook entry** | `title`, `type`, `date`, `status` | `phase`, `related_plan`, `related_issues`, `sessions`, `files_changed`, `commits`, `subsystem`, `tags` | `type`, `status`, `subsystem`, `tags` |
| **plan (active)** | `title`, `created`, `status: active` | `owner`, `last_updated`, `related_logbook`, `related_config`, `related_code` | `status` |
| **plan (archived)** | `title`, `created`, `status`, `completed` | same as active | `completed` / `superseded` |
| **slash command** | `description` | `disable-model-invocation` | — |
| **agent** | `name`, `description` | — | — |

**Naming rules:**

- Logbook: `YYYY-MM-DD-<slug>.md` (3–6-word slug).
- Plan (active): `<kebab-case-name>.md`.
- Plan (archived): `YYYY-MM-DD <kebab-case-name>.md` (note the space after
  the date — preserved for historical parity; `/archive-plan` handles this).
- Slash command / agent: `<name>.md` matching the invocation name.

---

## 4. Cross-reference map

Who points to whom:

```
CLAUDE.md ──► DOCUMENTATION_GUIDE.md (this file)
          ──► logbook/README.md
          ──► plans/active/, plans/archived/

docs/<section>/<topic>.md ──► logbook/<entry>.md      (per-issue narrative)
                          ──► plans/archived/<plan>   (historical decisions)

logbook/<entry>.md ──► plans/<plan>.md                (via related_plan)
                  ──► sim/analysis/known_issues.yaml   (via related_issues)
                  ──► sim/analysis/log_index.json      (via sessions)
                  ──► git commits                      (via commits, bidirectional)

plans/<plan>.md ──► logbook/<entry>.md                (via related_logbook)
              ──► config/<file>.yaml                  (via related_config)
              ──► code symbols                        (via related_code)

.claude/commands/ ──► logbook/, plans/, sim/analysis/  (the protocols)
.claude/agents/   ◄── .claude/commands/                (invoked by)

git commit ──► logbook entry (via `Logbook-Entry:` trailer)
```

The critical traceability chain for "why does this line of code exist?" is:

```
source line → git blame → commit → Logbook-Entry trailer → logbook entry
            → (entry's related_plan) → plan → (plan's related_logbook) → sibling entries
```

One hop each, no broken links.

---

## 5. Decision tree — "I need to document X"

Use this to pick the right layer *before* writing.

- **Is this the *why* behind a concrete code change?**
  → Logbook entry. Use `/investigate` (hardware) or `/log` (non-hardware).
  The commit for the code change gets a `Logbook-Entry:` trailer.

- **Is this a forward-looking plan of work yet to be done?**
  → Plan in [plans/active/](plans/active/). Use `/implementation-report` for
  non-trivial architectural changes; write by hand for simple bringup
  sequences.

- **Is this stable, general knowledge about how a subsystem works?**
  → Page under [docs/](docs/), registered in `mkdocs.yml`.

- **Is this a convention or contract local to one subtree?**
  → In-tree markdown next to the code:
  `<subsystem>/docs/<topic>.md`, `<subsystem>/README.md`, or
  `<subsystem>/<CONTRACT>.md`.

- **Is this a workflow rule or repo-wide convention Claude must follow?**
  → Add to [CLAUDE.md](CLAUDE.md) under "Workflow Rules" or "Critical
  Conventions". Keep it terse — expand in this guide or a docs page.

- **Is this a protocol for a slash command or agent?**
  → The relevant file under [.claude/commands/](.claude/commands/) or
  [.claude/agents/](.claude/agents/). That file **is** the spec.

- **Is this the *why* behind a single line or block of code?**
  → Inline comment, only if the why is non-obvious. Usually the logbook entry
  is the better place; a commit trailer links back.

- **Is this a public-facing project description?**
  → [README.md](README.md). Keep it minimal; link to the external project home.

- **None of the above fit.** → Ask before inventing a new layer.

---

## 6. Idiosyncrasies & gotchas

Things that are **not** derivable from reading the layout and will trip you up
if you don't know them:

- **`related_plan` is filename-only, not a path.** A logbook entry that writes
  `related_plan: plans/active/hardware-bringup.md` will break when the plan
  is archived to `plans/archived/2026-XX-XX hardware-bringup.md`. Always use
  the bare filename; consumers search both directories.

- **Archived plans keep the original name with a date prefix and a space.**
  Not a hyphen — a literal space (`2026-03-30 mpc-oscillation-analysis.md`).
  Filename-matching code must account for this.

- **`logbook/INDEX.md` is auto-maintained.** Every slash command that touches
  entries refreshes it via the `logbook-updater` agent. Hand-edits that don't
  follow the row format (`| date | status | phase | title | [slug](file.md) |`)
  will be clobbered on the next agent run.

- **`sim/analysis/log_index.json` mirrors logbook ↔ session.** The logbook
  entry's `sessions:` field is one direction; the JSON's `logbook_entry`
  field is the other. `/diagnose` and `/investigate` keep them in sync —
  hand-edits must update both.

- **`status: tuned` is not a synonym for `in-progress`.** Use it when a
  symptom is verified fixed *and* the entry intentionally leaves a sibling
  investigation open elsewhere. Stale `in-progress` entries are a known
  failure mode.

- **Withdrawn claims are never silently removed.** If you realise a
  conclusion in an existing entry was wrong, add a "Withdrawn claims" entry
  with date, the retracted claim, evidence, and a pointer to the correct
  finding. The wrong claim stays in the body of Diagnosis/Verification.

- **`docs/` math uses `pymdownx.arithmatex`, not raw KaTeX.** Use `$...$` and
  `$$...$$`; MathJax is loaded via `extra_javascript` in `mkdocs.yml`.

- **Two index files with different meanings.** [logbook/INDEX.md](logbook/INDEX.md)
  is the logbook index; [sim/analysis/log_index.json](sim/analysis/log_index.json)
  is the *session* (telemetry CSV) index. They cross-reference but are not
  the same thing.

- **`known_issues.yaml` is a living catalog, not documentation.**
  [sim/analysis/known_issues.yaml](sim/analysis/known_issues.yaml) pins
  machine-readable signatures for regression detection. When an issue is
  fixed, keep the entry with `status: fixed` — do not delete it. Add a
  `logbook_entry:` pointer so `/diagnose` can surface the fix narrative.

- **Plans without frontmatter exist in the wild.** A handful of older
  planning documents (e.g. ad-hoc investigation proposals) start with a
  Markdown title and an inline `**Status:**` line instead of YAML
  frontmatter. New plans must use YAML frontmatter; older ones should be
  upgraded when touched.

- **Plans use `status: active | completed | superseded`** — not the logbook's
  four-step ladder. The vocabularies are deliberately different because the
  semantics differ.

- **The `controller/REFERENCE_LAYER_CONTRACT.md` pattern is reserved for
  normative specs.** Don't create new `ALLCAPS.md` files for general notes;
  prefer [docs/](docs/) or [logbook/](logbook/).

---

## 7. Maintenance

This guide itself is documentation. It must be kept current with the
structures it describes. Update it when:

- A new documentation layer is added (e.g. a new `<subsystem>/docs/` tree).
- A frontmatter field is added, removed, or renamed in the logbook or plan
  schema — also update [logbook/README.md](logbook/README.md),
  [logbook/TEMPLATE.md](logbook/TEMPLATE.md), and the agent definitions.
- A controlled-vocabulary term (`type`, `status`, `subsystem`, `tags`) is
  added.
- A slash command or agent is added, removed, or substantially renamed.

Changes to this guide are logged with `/log refactor` (type `refactor`,
subsystem `docs`).
