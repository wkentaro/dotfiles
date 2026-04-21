# /make-branch — Move changes from main to a new branch

When you've been working on `main` (or a random placeholder branch like a petname) and want to PR, this skill creates a well-named feature branch and moves your commits and uncommitted changes onto it.

## Workflow

### 1. Assess the situation

Run these in parallel:

```bash
git branch --show-current
git log origin/main..HEAD --oneline
git diff --stat
git diff --cached --stat
```

Classify the current branch:

- **`main` / `master`** — proceed with the branch-out flow (Cases A/B/C below).
- **Placeholder / petname branch** — proceed with the in-place rename flow (Case D). A branch is a placeholder if any of these match:
  - Petname pattern: `^[a-z]+-[a-z]+$` (two lowercase words joined by a single hyphen, no slash, no digits — e.g. `useful-pony`, `sound-midge`, `novel-aphid`). Claude Code and similar tools auto-generate these.
  - Generic placeholder name: `wip`, `tmp`, `temp`, `test`, `scratch`, `work`, or similar.
  - The user passes an argument asking to rename (e.g. "because this branch is petname", "rename this branch").
- **Already-conventional branch** — a branch matching `^(feat|fix|refactor|docs|test|chore|perf|ci)/.+` is well-named. Abort and tell the user to use `/make-pr` directly.
- **Protected branches** (`develop`, `dev`, `staging`, `production`, `release/*`) — abort; don't touch these.

**Abort if there's nothing to move** — no commits ahead of `origin/main` and no uncommitted changes (only applies to the branch-out flow; the rename flow can proceed with only committed work).

### 2. Generate a branch name

Analyze the commits and/or diff to understand the theme of the changes. Generate a branch name:

- Format: `<type>/<short-kebab-description>` (e.g., `feat/add-type-annotations`, `fix/url-parsing`, `refactor/simplify-session`)
- Types: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`
- Keep it under 50 chars, descriptive, no issue numbers unless obvious

### 3. Move changes to the new branch

Handle three cases:

**Case A: Commits ahead of origin/main + uncommitted changes**

```bash
git stash --include-untracked
git branch <branch-name>
git reset --hard origin/main
git switch <branch-name>
git stash pop
```

**Case B: Commits ahead of origin/main only (clean working tree)**

```bash
git branch <branch-name>
git reset --hard origin/main
git switch <branch-name>
```

**Case C: Uncommitted changes only (no commits ahead)**

```bash
git switch -c <branch-name>
```

**Case D: Already on a placeholder / petname branch — rename in place**

Safe, non-destructive. Preserves all commits and working-tree state; no stash needed.

```bash
git branch -m <old-placeholder> <branch-name>
```

If the placeholder branch was already pushed to origin, also update the remote:

```bash
git push origin :<old-placeholder> <branch-name>
git push -u origin <branch-name>
```

### 4. Verify and report

```bash
git branch --show-current
git log origin/main..HEAD --oneline
git status --short
```

Print a summary:
- Branch name created
- Number of commits moved
- Working tree status
- Suggest running `/make-pr` next

## Guidelines

- Always fetch before comparing: `git fetch origin main` first
- Never force-delete or lose commits — the stash+reset approach is safe
- If `origin/main` doesn't exist (e.g., fresh repo), use `origin/master` or abort with a message
- Don't commit uncommitted changes — just move them to the new branch as-is
- Pick a branch name that would make sense as a PR title prefix
