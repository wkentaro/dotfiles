---
name: git-hunk
description: |
  Split uncommitted changes into focused, logical commits using git-hunk.
  Use when asked to "split changes", "split commits", "organize commits",
  "commit by hunk", or "separate changes into commits".
allowed-tools:
  - Bash
  - Read
  - Edit
  - Grep
  - Glob
---

# /git-hunk — Split Changes into Focused Commits

Split uncommitted changes into focused, logical commits using the `git-hunk` CLI tool.

## Prerequisites

`git-hunk` must be installed (`pip install git-hunk`).

## Workflow

### 1. Survey all changes

```bash
git-hunk list --json
```

This returns JSON with all unstaged hunks. Each hunk has an `id`, `file`, `header`, `additions`, `deletions`.

If there are also staged changes, check those too:

```bash
git-hunk list --staged --json
```

### 2. Inspect hunks that need closer look

```bash
git-hunk show <hunk-id>
```

Shows the full diff with line numbers. Use this to understand what each hunk does.

### 3. Group hunks into logical commits

Analyze the hunks and group them by logical change (e.g., "refactor X", "add feature Y", "fix bug Z"). Each group becomes one commit.

### 4. Stage and commit each group

For each logical group, stage the relevant hunks:

```bash
git-hunk stage <id1> <id2> <id3>
```

For partial hunk staging (only specific lines within a hunk):

```bash
git-hunk stage <id> -l 3,5-7      # include only lines 3, 5, 6, 7
git-hunk stage <id> -l ^3,^5-7    # exclude lines 3, 5, 6, 7
```

Then commit:

```bash
git commit -m "<type>: <description>"
```

Repeat for each group.

### 5. Verify

After all commits, verify no changes are left behind:

```bash
git-hunk list --json
git status
```

## Key Commands Reference

| Command | Description |
|---------|-------------|
| `git-hunk list [--staged] [--json]` | List hunks (unstaged by default) |
| `git-hunk show <id> [--staged]` | Show full diff for a hunk |
| `git-hunk stage <id>... [-l <lines>]` | Stage hunks or specific lines |
| `git-hunk unstage <id>... [-l <lines>]` | Unstage hunks back to working tree |
| `git-hunk discard <id>... [-l <lines>]` | Discard hunks (restore from HEAD) |

## Guidelines

- **Hunk IDs support prefix matching** — use shortest unambiguous prefix
- **IDs are stable** — they survive partial staging, so you can stage incrementally
- **One logical change per commit** — group related hunks across files
- **Use conventional commits** — `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`
- **Don't leave orphaned changes** — verify all hunks are accounted for
- **Ask the user** if grouping is ambiguous — don't guess at commit boundaries
