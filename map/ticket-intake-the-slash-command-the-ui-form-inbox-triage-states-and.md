---
kind: ticket
title: 'Intake: the slash command, the UI form, inbox triage states, and the existing GitHub issues'
type: grilling
parent: project-map
blocked_by:
- ticket-map-item-schema-and-the-map-directory-layout
owners: []
rank: 9
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/30
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

How does a feature, bug or idea get filed and triaged? Decide: the slash command name and its minimum fields (title, type, one line; attach a logbook entry or bag path?); the UI form's fields; inbox triage states and who moves items (owner only? Claude at the end of /investigate?); the relation to GitHub issues (the four open issues #16-19 with needs-triage / ready-for-agent labels: migrate into map/ and close them, keep GitHub issues as the collaborator-facing intake mirrored in, or drop them?); whether /investigate and /log auto-file a Bug or Feature when they close an entry with open residue.

Stress-test: a bug spotted mid-sitting on the Jetson with no app running; an idea filed from a phone (if out of scope, say so).

## Answer

## Answer (grilling, owner, 2026-10-23, one round)

Resolved on the owner's word while #25 (item schema) was still open: field names below are **inputs to #25**, which keeps the final say on the schema; the behaviour is settled here. Working schema assumed: the #24 prototype's item file (`kind`, `title`, `status`, `priority`, `owners`, `parent`, `blocked_by`, body = description).

### The command

- **`/inbox <bug|feature|idea> "<title>" [one-line description]`**, a Jugglebot slash command (`.claude/commands/inbox.md`), the writing sibling of #37's `/work-inbox`. Options: `--pathway <slug>`, `--priority low|medium|high`, `--link <repo-path-or-url>` (repeatable), `--from-issue <n>`.
- **Minimum fields from the filer:** kind, title, one line (the command asks for the line if missing). **Claude fills:** `created` (date), `filed_by` (owner or the session), `status: inbox`. **Optional at filing:** pathway (the parent), priority, `links:` as a plain list of repo paths or URLs (a logbook entry, a bag path, a GitHub issue). No typed `logbook:` / `bag:` fields.
- **The command commits and pushes the item immediately:** `git commit -- map/<file>.md` with message `map: file <kind> <title>` and **no `Logbook-Entry` trailer** (data, not code, same as the Pi's bot commits in #31), then `git push`. An unpushed item is invisible to the map. The push also carries any other unpushed local commits, which is the normal flow.
- **Stress test, bug mid-sitting on the Jetson with no app running:** `/inbox bug "…"` in the Jetson session writes, commits and pushes the file; the Pi's two-minute pull timer puts it on the map. Passes.
- **Stress test, idea from a phone:** in scope at no extra cost. The UI form below is the Pi app's modal; a phone on the LAN opens `http://jeeves:8765`, off-LAN needs Tailscale on the phone. No separate phone page; the modal must fit a 400 px screen (a #35 acceptance criterion).

### The UI form

- The fork's new-item modal is cut to **kind (bug, feature, idea only), title, one line, priority (optional), pathway (optional, default unattached)**. It writes the same record the command writes.
- **Pathways and plans are not created in the UI:** a plan is a file in `plans/`; a pathway is born in a grilling session with a Destination. Promoting an idea is #37's command, not a form.

### Triage states (inputs to #25's status vocabulary)

- For the inbox kinds: **`inbox`** (filed, unreviewed) → **`backlog`** (triaged, kept) → **`in_progress`** → **`complete`**; **`dropped`** from anywhere; **`promoted`** for an idea (terminal, with `promoted_to:` naming the plan or pathway it became).
- **Triage = leaving `inbox`.** Attachment to a pathway is a separate axis (the parent): a bug can be attached at filing and still untriaged; an idea can be triaged and stay unattached. This **replaces #24's "day-one inbox = unparented items"** with "the inbox is status `inbox`"; the Front page wall shows every inbox-kind item regardless, the Now/Next/Later rail shows the unattached ones (#38).
- **Who moves items (owner override of the recommendation):** the owner and Claude both move items freely through every transition, including triage, drop and promote. Claude is expected to do most of the administrative work (filing from sittings and logbook closes, attaching pathways, closing items through `/work-inbox`). No owner-only transitions.

### The existing GitHub issues

- **Migrate and close #17, #18, #19** at implementation time (a #35 step): #17 → Feature "Rename session limits to sitting limits" (unattached); #18 → Bug "can-bridge Guard reports the Hand as leg 6" (unattached); #19 → Feature "Hand Idle, not Parked Closed-loop, while Active" (body keeps the design notes and the plan-first, safety-bearing rule). Each issue is closed with a comment naming its item file.
- **#16 gets no item:** it is #28's question (the Avoid-word test); #28 closes it with a pointer when it resolves.
- **Going forward, GitHub issues are not an intake channel for the owner.** They remain the public door for outsiders; `/inbox --from-issue n` imports one (title, body, `links: [issue URL]`) and closes it with a pointer. Whether wayfinder maps themselves stay on GitHub issues is #34's question.

### /investigate and /log

- When the logbook-updater sets an entry's status to anything other than `resolved`, the command **lists each open follow-up and offers to file it** as a Bug or Feature with `links: [logbook/<slug>.md]`, status `inbox`; one gate, default yes, nothing silent. Exact edits to the two command files are #33's.
- Scale of the residue this addresses (2026-10-23): 458 entries, 79 `in-progress`, 21 `tuned`, 3 `resolved-with-open-items`, 192 with an open-questions or follow-up section.
- **The backfill** of those existing entries is **one Idea item** filed when `map/` exists, not work now.

### Sharpened

- Unblocks #36 (inbox page prototype: uses these states and the wall/rail split of #38) and #37 (`/work-inbox`: consumes items in `backlog`, closes or promotes them, and needs `links:`, `promoted_to:`, `filed_by:`).
- #25 receives: `created`, `filed_by`, `links`, `promoted_to`, the inbox-kind status set above.
- #33 receives: the `/inbox` command file, the gated offer in `/investigate` and `/log`.
- #35 receives: the migration of #17/#18/#19, the 400 px modal criterion.
