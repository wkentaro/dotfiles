---
name: setup-github-labels
description: "Applies one small canonical GitHub label set to the current repo, so every repo you run it in speaks the same label vocabulary: an issue `type:` axis (bug/feature/task, mirroring GitHub Issue Types), the five whose-turn triage roles (needs-triage/needs-info/ready-for-agent/ready-for-human/wontfix), and a PR-verdict axis (recommend-merge/recommend-close). This skill owns the GitHub label objects (name, color, description); `setup-matt-pocock-skills` owns the triage roles' agent wiring. Use when setting up labels on a repo, or after editing this skill's canonical tables. Invoke explicitly; it is not auto-triggered."
---

# Setup GitHub Labels

Apply one intentional label vocabulary to whatever repo you run this in. The
tables below are the source of truth, not the GitHub UI. This is a prompt-driven
skill, not a deterministic script: read the set, explore the repo, confirm with
the user, then create the labels with `gh`.

## The canonical set

Three groups, kept small on purpose. This skill owns the GitHub label *objects*
(name, color, description) for all of them. `setup-matt-pocock-skills` owns the
*agent wiring* for the triage roles — the role→string mapping in
`docs/agents/triage-labels.md` that tells the `triage` skill which string to
apply — and never touches a label's color or description. So the two skills
split by concern, not by which labels: one owns the labels on GitHub, the other
owns how the agent refers to them. Anything a *tool* already owns (Dependabot,
labeler actions) is **not** here.

### Issue `type:` axis

| Label           | Color    | Description (issue-scoped)                          |
| --------------- | -------- | -------------------------------------------------- |
| `type: bug`     | `d73a4a` | issue: Reporting a defect to fix                   |
| `type: feature` | `a2eeef` | issue: Requesting a new capability or improvement  |
| `type: task`    | `cfd3d7` | issue: For other work: maintenance, refactor, docs |

`type: bug` / `type: feature` / `type: task` mirror GitHub's Issue Types
(Bug / Feature / Task). `task` is the catch-all for refactor/docs/chore/test
work. If a repo ever moves under a GitHub org, these map 1:1 onto native Issue
Types and the labels can be retired.

### Issue triage axis

The five whose-turn triage roles the `triage` skill moves an issue through.
`setup-matt-pocock-skills` decides *which string* each role maps to (and records
it in `docs/agents/triage-labels.md`); this skill creates the actual GitHub
labels with a consistent scope prefix, so it reads at a glance whether a state
applies to issues, PRs, or both.

| Label             | Color    | Description                                        |
| ----------------- | -------- | -------------------------------------------------- |
| `needs-triage`    | `fbca04` | issue: Maintainer needs to evaluate this issue     |
| `needs-info`      | `d876e3` | issue/pr: Waiting on reporter for more information |
| `ready-for-agent` | `0e8a16` | issue: Fully specified, ready for an AFK agent     |
| `ready-for-human` | `1d76db` | issue: Requires human implementation               |
| `wontfix`         | `ffffff` | issue: Will not be actioned                        |

`needs-info` is the one `issue/pr:` label: a PR parked on an outside human reuses
it rather than minting a PR-scoped twin, so "waiting on a human" reads
identically on both. Every other triage role is issue-only.

### PR-verdict axis

Whose-turn routing on a PR needs almost no labels: a non-draft PR with no
verdict is, by definition, the agent's to finalize, and a PR parked on an
outside human reuses the `needs-info` triage label above rather than minting a
PR-scoped twin. What this axis adds is the agent's **terminal verdict** once it
has finished finalizing a PR: the single emerald/red signal that tells the
maintainer, at a glance, which open PRs are theirs to ship.

| Label             | Color    | Description (PR-scoped)                              |
| ----------------- | -------- | --------------------------------------------------- |
| `recommend-merge` | `0E8A16` | pr: Agent finalized and endorses it: review and merge |
| `recommend-close` | `D93F0B` | pr: Agent recommends closing: your call to review or close |

The verdict is **binary by design**: the agent finalizes a PR (rebase, green CI,
polish) and then emits exactly one of the two. `recommend-close` is the single
"not confidently mergeable" bucket — it covers both an active reject and a
low-confidence "couldn't tell, your call", with the nuance carried in the
agent's review comment, not a third label. Keeping it one bucket means the
maintainer's world stays two-pile: an emerald ship queue
(`is:pr is:open label:recommend-merge`) and a red triage pile.

The agent **never merges and never closes** — both stay the maintainer's hand;
the verdict is a recommendation, not an action. That is why both labels lead with
`recommend-` rather than the ecosystem-conventional `ready-to-merge`: the matched
`recommend-merge` / `recommend-close` pair names them honestly as recommendations,
and — deliberately — steers clear of the label strings merge bots watch (Kodiak,
Mergify, bors, GitHub auto-merge). A bot wired to merge on `ready-to-merge` would
turn the agent's recommendation into an actual merge and break this invariant, so
do **not** rename it back to that conventional string.

A verdict endorses one specific diff, so it goes **stale** the moment the PR
changes: a new commit after a verdict means the agent must clear that label and
re-review before the emerald queue can be trusted again. Pushes after a verdict
are rare, so this stays a manual step rather than something worth a CI workflow.

**A couple of label families are deliberately left out, because a *tool* already
owns them:**

- Tool-managed labels (`dependencies` from Dependabot, and other labels created
  by labeler actions or bots) are owned by that tooling. Leave their color and
  description alone; do not add them here or you will fight the tool that
  recreates them.
- `area:*` labels are per-repo (add them locally, ideally via path-based
  `actions/labeler`), so they do not belong in a shared cross-repo set.

The "still being worked" state is owned by GitHub's native **draft** flag, not a
label. A PR mid-iteration (including agent- or self-authored PRs, where the
author *is* the maintainer so review-request doesn't apply) stays a draft until
the agent has a verdict. So there is deliberately no `ready-for-maintainer` /
`ready-for-review` label: `recommend-merge` already *is* the "your turn to ship"
signal, and the draft flag already owns "not yet". The maintainer's "needs my
attention" filter is `is:pr is:open label:recommend-merge`. `/make-pr` opens
autonomous PRs as draft for exactly this reason.

The same draft flag is why there is no `do-not-merge` label: a "don't merge this
yet" state is almost always temporary (a PR kept open to exercise CI, or one
mid-iteration), and that is precisely what draft expresses. A draft PR already
cannot be merged, so a separate flag would just be a second, redundant mechanism
for "not yet".

PR *type* and *area* stay absent: PR type comes from the conventional-commit
title, and area is per-repo. Only the PR *verdict* is shared here, because "is
this one mine to ship" generalizes across every repo with PRs. Each description
leads with a scope prefix — `issue:`, `pr:`, or `issue/pr:` — to advertise where
it applies, so tools and people don't cross-apply.

### Issue ↔ PR equivalents

Issues and PRs run the same underlying state machine — *whose turn is it, and
what must they do* — but express it with different signals. This skill creates
both sides' labels; `setup-matt-pocock-skills` owns the issue side's *vocabulary*
(which string each triage role maps to). The PR side also leans on GitHub's
native draft flag. This table lines them up so a state reads the same whether
you're looking at an issue or a PR:

| State (whose turn / what's needed)     | Issue (triage role)                       | PR (verdict + draft flag)                 |
| -------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| Unprocessed — someone must look        | `needs-triage`                            | no verdict label, non-draft               |
| Blocked on an outside human            | `needs-info`                              | `needs-info` (same label, reused)         |
| Still being built / iterated           | *(open issue, no extra label)*            | **draft** flag                            |
| Agent's turn to act                    | `ready-for-agent`                         | no verdict label, non-draft               |
| Maintainer's turn — endorsed           | `ready-for-human` (human implements)      | `recommend-merge` (human reviews/merges)  |
| Won't proceed                          | `wontfix`                                 | `recommend-close`                         |

Two asymmetries are intentional, not gaps:

- **`needs-triage` and `ready-for-agent` collapse into one PR state.** A non-draft
  PR with no verdict already means "agent, finalize this", so the PR side never
  separates "needs evaluation" from "agent's turn" — there is no PR `needs-triage`
  or `ready-for-agent` label. On the issue side they stay distinct because an
  issue can sit triaged-but-not-yet-assigned.
- **The maintainer's terminal action differs.** `ready-for-human` on an issue means
  *implement it*; `recommend-merge` on a PR means *review and merge it*. Same "your
  turn, human" role, different verb — which is why `ready-for-human` stays
  issue-only and `recommend-merge` is its PR counterpart rather than a shared label.

`do-not-merge` is absent on purpose: a "don't merge yet" PR is the **draft** row
above, so it needs no label of its own.

## Process

Detect, preview, confirm, then apply.

### 1. Detect the current repo

```bash
gh repo view --json nameWithOwner -q .nameWithOwner
```

If that fails (no GitHub remote), ask the user which repo to target.

### 2. Preview (read-only)

Show the user which canonical labels are new vs already present on the repo, by
comparing the tables' labels against the repo's existing ones:

```bash
gh label list --repo "$REPO" --limit 200 --json name -q '.[].name'
```

Labels in the tables but not in that list will be **created**; labels already
present will have their color/description **updated**.

### 3. Confirm, then apply

Creating labels on a (often public) repo is an outward-facing action, so confirm
with the user first. Then create each label from the tables with `--force`, which
adds it if missing and updates color/description if it already exists:

```bash
gh label create "type: bug"     --color d73a4a --description "issue: Reporting a defect to fix"                   --force --repo "$REPO"
gh label create "type: feature" --color a2eeef --description "issue: Requesting a new capability or improvement"  --force --repo "$REPO"
gh label create "type: task"    --color cfd3d7 --description "issue: For other work: maintenance, refactor, docs" --force --repo "$REPO"
gh label create "needs-triage"    --color fbca04 --description "issue: Maintainer needs to evaluate this issue"     --force --repo "$REPO"
gh label create "needs-info"      --color d876e3 --description "issue/pr: Waiting on reporter for more information" --force --repo "$REPO"
gh label create "ready-for-agent" --color 0e8a16 --description "issue: Fully specified, ready for an AFK agent"     --force --repo "$REPO"
gh label create "ready-for-human" --color 1d76db --description "issue: Requires human implementation"              --force --repo "$REPO"
gh label create "wontfix"         --color ffffff --description "issue: Will not be actioned"                       --force --repo "$REPO"
gh label create "recommend-merge" --color 0E8A16 --description "pr: Agent finalized and endorses it: review and merge"   --force --repo "$REPO"
gh label create "recommend-close" --color D93F0B --description "pr: Agent recommends closing: your call to review or close" --force --repo "$REPO"
```

To set up several repos, repeat with each `--repo`.

### 4. Report

Tell the user which labels were created vs updated. Non-canonical labels already
on the repo are **left untouched** (this skill only adds/updates the canonical
set). If they want to retire a stray label, that is a manual, deliberate step:
`gh label delete "<name>" --repo "$REPO" --yes`. Deleting strips it off every
issue/PR currently wearing it, so never do it without explicit confirmation.

This skill creates the triage labels, but the engineering skills only *apply*
them if the repo also has the agent wiring — the role→string mapping in
`docs/agents/triage-labels.md`. If that file is absent
(`test -f docs/agents/triage-labels.md`), recommend running
`setup-matt-pocock-skills` to write it. That's a one-line pointer, not a
requirement: the labels stand on their own; the wiring is what lets the `triage`
skill reach for them. Skip the recommendation when the file is already present.

## Changing the set

Edit the tables above: add or remove a row, and keep the `gh label create` lines
in step 3 in sync with them. For a triage role, also keep it consistent with the
role→string mapping `setup-matt-pocock-skills` records in
`docs/agents/triage-labels.md` — this skill owns the label's color and
description, that skill owns which string the role maps to. Keep the set small:
before adding a label, check it carries information the commit title, diff, or an
existing label (or another skill's labels) doesn't.
