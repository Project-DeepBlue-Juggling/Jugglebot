# Issue tracker: GitHub (public door) and `map/` (planning)

GitHub Issues are the **public door only**: outsiders' bugs and requests, and PR triage. The owner's own planning never lives there; it lives in `map/` (`DOCUMENTATION_GUIDE.md` § 2.10), `plans/` and `logbook/`. Use the `gh` CLI for the public-door operations below; the file-based operations are in the last section.

## Public door: conventions

**`gh` is not on `PATH` here.** It is installed at `~/bin/gh` and already authenticated on this Jetson. In a Bash tool call run `export PATH="$HOME/bin:$PATH"` first (or call it by absolute path), or every command below fails with `command not found`.

- **Create an issue**: `gh issue create --title "..." --body "..."`. Use a heredoc for multi-line bodies.
- **Read an issue**: `gh issue view <number> --comments`, filtering comments by `jq` and also fetching labels.
- **List issues**: `gh issue list --state open --json number,title,body,labels,comments --jq '[.[] | {number, title, body, labels: [.labels[].name], comments: [.comments[].body]}]'` with appropriate `--label` and `--state` filters.
- **Comment on an issue**: `gh issue comment <number> --body "..."`
- **Apply / remove labels**: `gh issue edit <number> --add-label "..."` / `--remove-label "..."`
- **Close**: `gh issue close <number> --comment "..."`

Infer the repo from `git remote -v`; `gh` does this automatically when run inside a clone.

## Repo notes

- The repo is **public**: issue bodies, comments and labels are world-readable.
- GitHub Issues sit alongside `map/`, `plans/` and `logbook/`; they don't replace them (those layers are defined in `DOCUMENTATION_GUIDE.md` §§ 2.5–2.6 and 2.10).
- The logbook front-matter field `related_issues:` holds `known_issues.yaml` IDs (e.g. `VEL_FF_BUG`), never GitHub issue numbers (`DOCUMENTATION_GUIDE.md` § 2.5).

## Pull requests as a triage surface

**PRs as a request surface: no.** _(Set to `yes` if this repo treats external PRs as feature requests; `/triage` reads this flag.)_

When set to `yes`, PRs run through the same labels and states as issues, using the `gh pr` equivalents:

- **Read a PR**: `gh pr view <number> --comments` and `gh pr diff <number>` for the diff.
- **List external PRs for triage**: `gh pr list --state open --json number,title,body,labels,author,authorAssociation,comments` then keep only `authorAssociation` of `CONTRIBUTOR`, `FIRST_TIME_CONTRIBUTOR`, or `NONE` (drop `OWNER`/`MEMBER`/`COLLABORATOR`).
- **Comment / label / close**: `gh pr comment`, `gh pr edit --add-label`/`--remove-label`, `gh pr close`.

GitHub shares one number space across issues and PRs, so a bare `#42` may be either: resolve with `gh pr view 42` and fall back to `gh issue view 42`.

## When a skill says "publish to the issue tracker"

Create a GitHub issue, and only for outsider-facing specs. The owner's own planning never lives there: file it with `/inbox` (a `map/` item). An outsider's issue is imported with `/inbox --from-issue <n>` and then closed with a comment pointing at the map item.

## When a skill says "fetch the relevant ticket"

Run `gh issue view <number> --comments`.

## Wayfinding operations (file-based)

Used by `/wayfinder`. Nothing here touches GitHub; every operation is a file edit in `map/` plus a commit and push (`map: ...`, no `Logbook-Entry:` trailer).

- **Map**: `map/pathway-<slug>.md`. Frontmatter carries the Destination; the body holds Notes / Decisions so far / Not yet specified / Out of scope. "Resolved tickets" is computed from the Pathway's complete ticket children, never hand-kept.
- **IDs**: every ticket is an Item and carries `id: JB-<n>` (`DOCUMENTATION_GUIDE.md` § 2.10); `frontier` prints it first, and `claim` / `resolve` take the ID (`JB-12`, `jb12`), the stem or the slug, and refuse a non-ticket. `blocked_by` and `parent` stay stems. Echoes and subjects cite the ID (`map: claim JB-9 <slug>`).
- **Child ticket**: `map/ticket-<slug>.md` with `kind: ticket`, `type: research|prototype|grilling|task`, `parent: <pathway slug>`, `blocked_by`, `owners` (non-empty means claimed), `rank`, `status: open|complete|dropped`, `links`. The body is the question; the answer is appended under `## Answer`.
- **Blocking**: the `blocked_by` list of filename stems (`ticket-<slug>`, or another item's `<kind>-<slug>`, or a plan's bare filename). A ticket is unblocked when every blocker is complete or dropped (a plan blocker: archived).
- **Frontier query**: `python -m mapkit frontier <pathway>` lists tickets that are open, have no open blocker and no owner, lowest `rank` first.
- **Claim**: `python -m mapkit claim <id|slug> --as <user>` adds the owner, commits `map: claim JB-<n> <slug>` and pushes. It is the session's first write.
- **Resolve**: `python -m mapkit resolve <id|slug> --answer-file <path>` appends the Answer, sets `status: complete`, commits `map: resolve JB-<n> <slug>` and pushes.
- **Item status**: `python -m mapkit item set-status <slug> <status> [--note TEXT] [--link PATH]... [--owner NAME] [--force] [--commit [--no-push]]` validates against the kind's statuses, refuses to re-open a `complete` item or start one another owner holds (`--force` overrides), appends a dated line under `## Progress`, adds links without duplicates and regenerates `docs/project-map.md`. Without `--commit` it stages nothing and prints the two paths to stage (so a fix's commit, with its `Logbook-Entry` trailer, can carry the close); with `--commit` it commits `map: <status> <title>` (no trailer) and pushes.
- **Read side**: `python -m mapkit show <pathway> [--json]` (Destination, active plans with step table, open children by kind) and `python -m mapkit items [--pathway X] [--kind K] [--status S | --open] [--json]` (one line per item).
- **Sharpened tickets**: a ticket that turns out too big is not edited in place; write new ticket files with `blocked_by` on what they need.
- **Legacy GitHub maps**: `python -m mapkit import-issues <n> --pathway <slug>` imports a map that was kept as GitHub issues.
