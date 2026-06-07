---
name: setup-github-labels
description: Applies one small canonical GitHub label set to the current repo, so every repo you run it in speaks the same label vocabulary. Two axes: the issue `type:` axis (bug/feature/task, mirroring GitHub Issue Types) and a PR-flow status axis (needs-author-action/needs-decision/do-not-merge). Use when setting up labels on a repo, or after editing this skill's canonical table. Invoke explicitly; it is not auto-triggered.
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

### PR-flow status axis

These encode only what GitHub's native PR state (checks + review state) can't,
so a maintainer can tell at a glance whose court the ball is in. Anything
machine-detectable (CLA, lint, conflicts, "ready for review") is already a
failing check or review state and gets **no** label.

| Label                 | Color    | Description (PR-scoped)                                        |
| --------------------- | -------- | ------------------------------------------------------------- |
| `needs-author-action` | `d3dddd` | PR is awaiting action from the contributor: code changes or more info |
| `needs-decision`      | `D810FB` | PR is awaiting a maintainer decision before it can proceed     |
| `do-not-merge`        | `B60205` | Do not merge this pull request                                 |

`needs-author-action` is one bucket on purpose: do not split it into
`needs-cla` / `needs-rebase` / `needs-info`, since the contributor only needs a
single signal that the ball is in their court. It is the PR analog of the
issue-triage `needs-info`.

**Single owner per label, so several common labels are deliberately excluded:**

- Issue-triage roles (`needs-triage`, `needs-info`, `ready-for-agent`,
  `ready-for-human`, `wontfix`) are owned by `setup-matt-pocock-skills`; run
  that skill for the issue-triage vocabulary. The PR-flow labels above are this
  skill's own axis and do not overlap them.
- Tool-managed labels (`dependencies` from Dependabot, and other labels created
  by labeler actions or bots) are owned by that tooling. Leave their color and
  description alone; do not add them here or you will fight the tool that
  recreates them.
- `area:*` labels are per-repo (add them locally, ideally via path-based
  `actions/labeler`), so they do not belong in a shared cross-repo set.

PR *type* and *area* stay absent: PR type comes from the conventional-commit
title, and area is per-repo. Only PR *status* is shared here, because "whose
turn is it" generalizes across every repo with PRs. Each `type:` description
leads with "Issue" and each status description with "PR" to advertise scope, so
tools and people don't cross-apply.

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
gh label create "needs-author-action" --color d3dddd --description "PR is awaiting action from the contributor: code changes or more info" --force --repo "$REPO"
gh label create "needs-decision"      --color D810FB --description "PR is awaiting a maintainer decision before it can proceed"            --force --repo "$REPO"
gh label create "do-not-merge"        --color B60205 --description "Do not merge this pull request"                                        --force --repo "$REPO"
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
