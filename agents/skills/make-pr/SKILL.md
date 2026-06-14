---
name: make-pr
description: Push the current branch and create or update a GitHub pull request with a good title, description, labels, and assignee. Use after the branch name is already conventional.
---

# /make-pr — Create or Update a Pull Request

Push the current branch and create (or update) a GitHub PR with title, description, labels, and assignee.

## Workflow

### 0. Check branch

```bash
git branch --show-current
```

**If on `main` (or `master`)**: run `/make-branch` skill first to move their changes to a feature branch. Do NOT proceed with PR creation from the main branch.

**If on a placeholder / petname branch**: run `/make-branch` to rename it before opening the PR. Petname branches are auto-generated names and should not become the public branch name of a PR. A branch is a placeholder if any of these match:

- Petname pattern: `^[a-z]+-[a-z]+$` (two lowercase words joined by a single hyphen, no slash, no digits — e.g. `useful-pony`, `sound-midge`, `novel-aphid`). Claude Code and similar tools auto-generate these.
- Generic placeholder name: `wip`, `tmp`, `temp`, `test`, `scratch`, `work`, or similar.

A branch matching `^(feat|fix|refactor|docs|test|chore|perf|ci)/.+` is conventional — proceed with PR creation.

### 1. Gather context

Run these in parallel:

```bash
git log main..HEAD --oneline
git diff main...HEAD --stat
gh label list --limit 50 --json name,description
git config user.name
```

Also check if a PR already exists for this branch:

```bash
gh pr view --json number,url 2>/dev/null
```

### 2. Draft PR content

**Title**: `<type>: <short description>` (under 70 chars). Conventional types: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`. Match the PR theme, not just the last commit.

**Labels**: pick from `gh label list --json name,description` output. Apply only labels scoped to PRs — read each description and skip issue-scoped ones (e.g. descriptions starting with "Issue ..."). Verdict labels like `recommend-merge` / `recommend-close` / `recommend-triage` are applied during review, not at creation. If no label clearly applies, add none.

**Assignee**: default to the git user (`gh api user --jq '.login'`).

**Body**: read [DESCRIPTION-QUALITY.md](DESCRIPTION-QUALITY.md) before writing. Pick the smallest structure that carries the signal. Do NOT mechanically apply a fixed template; calibrate to the scope of the change.

#### Body by scope

**Small change** (docs, single bugfix, dep bump, small refactor) — one motivation sentence, evidence if visual, one-item test plan. No `## Summary` header — the lead IS the summary.

```
Follow-up to #144. The original example's inputs looked near-identical, so the
diff panels offered no contrast; this rewrite picks an edit-detection scenario
where each mode shows a distinct result.

![diff](https://github.com/owner/repo/raw/<full-sha>/examples/assets/diff.jpg)

## Test plan
- [x] `uv run python examples/diff.py --save` regenerates the asset
```

**Medium change** (new function, behavior change, multi-commit feature):

```
## Summary
- 3-5 bullets describing behavior and API surface, not implementation details

![evidence if visual]

## Test plan
- [x] tests added: `tests/...`
- [x] `make lint`
```

**Large change** (cross-cutting refactor, new subsystem) — add `## Why` with real architectural motivation; link the issue or design doc that authorized the work.

#### Embedded images

Use a commit-SHA-pinned raw URL so the image survives branch deletion after merge:

```
https://github.com/<owner>/<repo>/raw/$(git rev-parse HEAD)/<path>
```

Branch-pinned URLs (`raw/feature-branch/...`) 404 once the branch is deleted, leaving the archived PR description broken.

#### Test plan calibration

Each item should describe a check that actually gated the change. Boilerplate dilutes meaningful items.

- **Docs-only**: one item, usually asset regen or render check. Don't pad with `make lint` / `pytest` for changes that don't touch lintable / testable code.
- **Bugfix**: the new test that proves the fix + manual repro if applicable.
- **Feature**: tests added + lint + manual verification.

#### Anti-patterns to avoid

- **Marketing words** — "headline", "deliberately", "robust", "elegantly", "sells". Cut them.
- **Before/after framing on small PRs** — theater. The reviewer can compare against `main`.
- **Restating code comments in the body** — descriptions are ephemeral; code comments persist. Don't duplicate.
- **`## Why` sections without a real why** — if you can't add context the diff doesn't carry, skip the section.
- **Rigid templates over signal density** — if a lean paragraph carries the meaning, don't wrap it in `## Summary` ceremony.

### 3. Push and create/update

**Draft vs ready** — a draft PR is the native "not ready yet" signal, so the
maintainer's "needs my attention" filter is `is:pr is:open draft:false` (no
label needed). Pick the default from who is running this skill:

- **User invoked it directly** → open **ready**. They are vouching the branch is
  finished; do not add a `--draft` step they have to undo.
- **Running autonomously / unsupervised** (AFK, `/loop`, an agent opening its own
  PR) → open **draft** (`gh pr create --draft`). The work is unvetted; the user
  flips it to ready (`gh pr ready <number>`) or merges after a look.
- An explicit "draft" / "ready" request in the prompt overrides this.

```bash
# Push with tracking
git push -u origin <branch> --force-with-lease

# Create or update ( add --draft per the rule above )
gh pr create --title "..." --body "..." --label "..." --assignee "..."
# If PR already exists:
gh pr edit <number> --title "..." --body "..." --add-label "..." --add-assignee "..."
```

### 4. Return the PR URL

Always print the PR URL at the end so the user can click it.

## Guidelines

- Always use `--force-with-lease` (not `--force`) when pushing.
- Check if a PR already exists before creating — update it (`gh pr edit`) instead of failing.
- For long PR bodies, write to a tempfile and pass via `--body-file` to avoid shell quoting issues.
