---
name: setup-github-labels
description: Applies one small canonical GitHub label set to the current repo, so every repo you run it in speaks the same label vocabulary. The set is the `type:` axis: bug/feature/task, mirroring GitHub Issue Types. Use when setting up labels on a repo, or after editing this skill's canonical table. Invoke explicitly; it is not auto-triggered.
disable-model-invocation: true
---

# Setup GitHub Labels

Apply one intentional label vocabulary to whatever repo you run this in. The
table below is the source of truth, not the GitHub UI. This is a prompt-driven
skill, not a deterministic script: read the set, explore the repo, confirm with
the user, then create the labels with `gh`.

## The canonical set

Three labels: the `type:` axis, and nothing else. The set is small on purpose:
anything another tool or skill already owns is **not** in it.

| Label           | Color    | Description (issue-scoped)                       |
| --------------- | -------- | ----------------------------------------------- |
| `type: bug`     | `d73a4a` | Issue reporting a defect to fix                 |
| `type: feature` | `a2eeef` | Issue requesting a new capability or improvement|
| `type: task`    | `cfd3d7` | Issue for other work: maintenance, refactor, docs |

`type: bug` / `type: feature` / `type: task` mirror GitHub's Issue Types
(Bug / Feature / Task). `task` is the catch-all for refactor/docs/chore/test
work. If a repo ever moves under a GitHub org, these map 1:1 onto native Issue
Types and the labels can be retired.

**Single owner per label, so several common labels are deliberately excluded:**

- Triage roles (`needs-triage`, `needs-info`, `ready-for-agent`,
  `ready-for-human`, `wontfix`) are owned by `setup-matt-pocock-skills`; run
  that skill for the triage vocabulary.
- Tool-managed labels (`dependencies` from Dependabot, and other labels created
  by labeler actions or bots) are owned by that tooling. Leave their color and
  description alone; do not add them here or you will fight the tool that
  recreates them.
- `area:*` labels are per-repo (add them locally, ideally via path-based
  `actions/labeler`), so they do not belong in a shared cross-repo set.

PR type/area is absent for the same reason it was never needed: type comes from
the conventional-commit title, not a label. Each description leads with "Issue"
to advertise that scope, mirroring how Dependabot's `dependencies` label starts
with "Pull requests that…", so tools and people don't apply a `type:` label to
a PR.

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
