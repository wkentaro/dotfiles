# /make-branch — Move changes from main to a new branch

When you've been working on `main` and want to PR, this skill creates a well-named
feature branch and moves your commits and uncommitted changes onto it.

## Workflow

### 1. Assess the situation

Run these in parallel:

```bash
git branch --show-current
git log origin/main..HEAD --oneline
git diff --stat
git diff --cached --stat
```

**Abort if not on `main`** — tell the user they're already on a branch and can use `/make-pr` directly.

**Abort if there's nothing to move** — no commits ahead of `origin/main` and no uncommitted changes.

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
