---
name: setup-github-labels
description: "Applies one small canonical GitHub label set to the current repo, so every repo you run it in speaks the same label vocabulary. Two axes: the issue `type:` axis (bug/feature/task, mirroring GitHub Issue Types) and a PR-verdict axis (ready-to-merge/recommend-close, plus a do-not-merge flag). Use when setting up labels on a repo, or after editing this skill's canonical table. Invoke explicitly; it is not auto-triggered."
disable-model-invocation: true
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
| `ready-to-merge`  | `0E8A16` | PR finalized and endorsed by the agent: review and merge     |
| `recommend-close` | `D93F0B` | PR the agent recommends closing: your call to review or close|
| `do-not-merge`    | `B60205` | Do not merge this pull request                               |

The verdict is **binary by design**: the agent finalizes a PR (rebase, green CI,
polish) and then emits exactly one of the two. `recommend-close` is the single
"not confidently mergeable" bucket — it covers both an active reject and a
low-confidence "couldn't tell, your call", with the nuance carried in the
agent's review comment, not a third label. Keeping it one bucket means the
maintainer's world stays two-pile: an emerald ship queue
(`is:pr is:open label:ready-to-merge`) and a red triage pile.

The agent **never merges and never closes** — both stay the maintainer's hand;
the verdict is a recommendation, not an action.

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
  verdicts (`ready-to-merge` / `recommend-close`) and
  the `do-not-merge` flag on top, so the two skills together cover the whole
  flow without overlap. `ready-for-human` stays issue-only: on a PR the human
  reviews and merges (or closes), it does not implement.
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
`ready-for-review` label: `ready-to-merge` already *is* the "your turn to ship"
signal, and the draft flag already owns "not yet". The maintainer's "needs my
attention" filter is `is:pr is:open label:ready-to-merge`. `/make-pr` opens
autonomous PRs as draft for exactly this reason.

PR *type* and *area* stay absent: PR type comes from the conventional-commit
title, and area is per-repo. Only the PR *verdict* is shared here, because "is
this one mine to ship" generalizes across every repo with PRs. Each `type:`
description leads with "Issue" and each verdict description with "PR" to
advertise scope, so tools and people don't cross-apply.

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

### 3. Confirm, then apply

Creating labels on a (often public) repo is an outward-facing action, so confirm
with the user first. Then create each label from the table with `--force`, which
adds it if missing and updates color/description if it already exists:

```bash
gh label create "type: bug"     --color d73a4a --description "Issue reporting a defect to fix"                   --force --repo "$REPO"
gh label create "type: feature" --color a2eeef --description "Issue requesting a new capability or improvement"  --force --repo "$REPO"
gh label create "type: task"    --color cfd3d7 --description "Issue for other work: maintenance, refactor, docs" --force --repo "$REPO"
gh label create "ready-to-merge"  --color 0E8A16 --description "PR finalized and endorsed by the agent: review and merge"       --force --repo "$REPO"
gh label create "recommend-close" --color D93F0B --description "PR the agent recommends closing: your call to review or close"  --force --repo "$REPO"
gh label create "do-not-merge"    --color B60205 --description "Do not merge this pull request"                                 --force --repo "$REPO"
```

To set up several repos, repeat with each `--repo`.

### 4. Report

Tell the user which labels were created vs updated. Non-canonical labels already
on the repo are **left untouched** (this skill only adds/updates the canonical
set). If they want to retire a stray label, that is a manual, deliberate step:
`gh label delete "<name>" --repo "$REPO" --yes`. Deleting strips it off every
issue/PR currently wearing it, so never do it without explicit confirmation.

## Changing the set

Edit the table above: add or remove a row, and keep the `gh label create` lines
in step 3 in sync with it. Keep the set small: before adding a label, check it
carries information the commit title, diff, or an existing label (or another
skill's labels) doesn't.
