---
name: setup-github-labels
description: "Applies one small canonical GitHub label set to the current repo, so every repo you run it in speaks the same label vocabulary. Two axes: the issue `type:` axis (bug/feature/task, mirroring GitHub Issue Types) and a PR-verdict axis (recommend-merge/recommend-close). Use when setting up labels on a repo, or after editing this skill's canonical table. Invoke explicitly; it is not auto-triggered."
---

# Setup GitHub Labels

Apply one intentional label vocabulary to whatever repo you run this in. The
table below is the source of truth, not the GitHub UI. This is a prompt-driven
skill, not a deterministic script: read the set, explore the repo, confirm with
the user, then create the labels with `gh`.

## The canonical set

Two axes, kept small on purpose: anything another tool or skill already owns is
**not** in it.

### Issue `type:` axis

| Label           | Color    | Description (issue-scoped)                       |
| --------------- | -------- | ----------------------------------------------- |
| `type: bug`     | `d73a4a` | Issue reporting a defect to fix                 |
| `type: feature` | `a2eeef` | Issue requesting a new capability or improvement|
| `type: task`    | `cfd3d7` | Issue for other work: maintenance, refactor, docs |

`type: bug` / `type: feature` / `type: task` mirror GitHub's Issue Types
(Bug / Feature / Task). `task` is the catch-all for refactor/docs/chore/test
work. If a repo ever moves under a GitHub org, these map 1:1 onto native Issue
Types and the labels can be retired.

### PR-verdict axis

Whose-turn routing on a PR needs almost no labels: a non-draft PR with no
verdict is, by definition, the agent's to finalize, and a PR parked on an
outside human reuses the `needs-info` label (created by
`setup-matt-pocock-skills` for issue triage) rather than minting a PR-scoped
twin. This skill does not recreate that label. What this skill owns is the agent's
**terminal verdict** once it has finished finalizing a PR: the single
emerald/red signal that tells the maintainer, at a glance, which open PRs are
theirs to ship.

| Label             | Color    | Description (PR-scoped)                                       |
| ----------------- | -------- | ------------------------------------------------------------ |
| `recommend-merge` | `0E8A16` | PR finalized and endorsed by the agent: review and merge     |
| `recommend-close` | `D93F0B` | PR the agent recommends closing: your call to review or close|

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

**Single owner per label, so several common labels are deliberately excluded:**

- Whose-turn triage roles (`needs-triage`, `needs-info`, `ready-for-agent`,
  `ready-for-human`, `wontfix`) are owned by `setup-matt-pocock-skills`; run
  that skill for them. `needs-info` is created there as an issue state, but a PR
  parked on an outside human reuses that same label rather than minting a
  PR-scoped twin, so "waiting on a human" reads identically on both. The agent's
  turn on a PR, by contrast, is just the no-verdict default, so no
  `ready-for-agent` label is applied there. This skill adds only the PR-terminal
  verdicts (`recommend-merge` / `recommend-close`) on top, so the two skills
  together cover the whole flow without overlap. `ready-for-human` stays
  issue-only: on a PR the human reviews and merges (or closes), it does not
  implement.
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
this one mine to ship" generalizes across every repo with PRs. Each `type:`
description leads with "Issue" and each verdict description with "PR" to
advertise scope, so tools and people don't cross-apply.

### Issue ↔ PR equivalents

Issues and PRs run the same underlying state machine — *whose turn is it, and
what must they do* — but express it with different signals. The issue side is
owned by `setup-matt-pocock-skills` (the five triage roles); the PR side is what
this skill owns, plus GitHub's native draft flag. This table lines them up so a
state reads the same whether you're looking at an issue or a PR:

| State (whose turn / what's needed)     | Issue (`setup-matt-pocock-skills`)        | PR (this skill + draft flag)              |
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
comparing the table's labels against the repo's existing ones:

```bash
gh label list --repo "$REPO" --limit 200 --json name -q '.[].name'
```

Labels in the table but not in that list will be **created**; labels already
present will have their color/description **updated**.

While you have that list, also note whether `needs-info` is present. This skill's
PR-verdict flow reuses that label (created by `setup-matt-pocock-skills`) for a
PR parked on an outside human, but deliberately does not create it. Its presence
or absence feeds the recommendation in step 4.

### 3. Confirm, then apply

Creating labels on a (often public) repo is an outward-facing action, so confirm
with the user first. Then create each label from the table with `--force`, which
adds it if missing and updates color/description if it already exists:

```bash
gh label create "type: bug"     --color d73a4a --description "Issue reporting a defect to fix"                   --force --repo "$REPO"
gh label create "type: feature" --color a2eeef --description "Issue requesting a new capability or improvement"  --force --repo "$REPO"
gh label create "type: task"    --color cfd3d7 --description "Issue for other work: maintenance, refactor, docs" --force --repo "$REPO"
gh label create "recommend-merge" --color 0E8A16 --description "PR finalized and endorsed by the agent: review and merge"       --force --repo "$REPO"
gh label create "recommend-close" --color D93F0B --description "PR the agent recommends closing: your call to review or close"  --force --repo "$REPO"
```

To set up several repos, repeat with each `--repo`.

### 4. Report

Tell the user which labels were created vs updated. Non-canonical labels already
on the repo are **left untouched** (this skill only adds/updates the canonical
set). If they want to retire a stray label, that is a manual, deliberate step:
`gh label delete "<name>" --repo "$REPO" --yes`. Deleting strips it off every
issue/PR currently wearing it, so never do it without explicit confirmation.

If the step 2 preview showed `needs-info` is **absent**, recommend running
`setup-matt-pocock-skills` to complete the vocabulary: it owns the issue-triage
roles (`needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`,
`wontfix`), and this skill's "PR parked on a human" state reuses its `needs-info`
label. This is a recommendation, not a requirement: the canonical set applied
here (the `type:` axis plus the PR verdicts) stands on its own; the matt-pocock
set is what rounds out the shared issue+PR triage flow. Skip the recommendation
when `needs-info` is already present.

## Changing the set

Edit the table above: add or remove a row, and keep the `gh label create` lines
in step 3 in sync with it. Keep the set small: before adding a label, check it
carries information the commit title, diff, or an existing label (or another
skill's labels) doesn't.
