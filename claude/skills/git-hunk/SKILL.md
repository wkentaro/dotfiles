---
name: git-hunk
description: |
  Split uncommitted changes into focused, logical commits using git-hunk.
  Use when asked to "split changes", "split commits", "organize commits",
  "commit by hunk", or "separate changes into commits".
license: MIT
metadata:
  author: wkentaro
  version: "0.1.0"
allowed-tools:
  - Bash
---

# /git-hunk - split changes into focused commits

Requires: `uv tool install git-hunk` (or `pip install git-hunk`)

## Workflow

1. `git-hunk list` - see all hunks (file, id, +/- stats). No diffs.
2. `git-hunk show <id> [<id>...]` or `git-hunk show --all` when headers aren't clear enough.
3. Group hunks into logical commits. Ask the user if grouping is ambiguous.
4. Stage and commit each group:
   ```bash
   git-hunk stage <id1> <id2> ...
   git commit -m "<type>: <description>"
   ```
5. `git-hunk list` again to check nothing got left behind.

For partial hunks: `git-hunk stage <id> -l 3,5-7` (include) or `-l ^3,^5-7` (exclude).

## Example `git-hunk list` output

```
unstaged:
labelme/app.py
  c43213b  @@ -78,6 +78,7 @@ _AI_CREATE_MODES  +1
  4da0d77  @@ -1364,6 +1365,19 @@ class MainWindow  +13
labelme/translate/de_DE.qm
  7a3befc  Binary file
```

## Notes

- IDs are content-based hashes, stable across partial staging, and support prefix matching
- `--staged` / `--unstaged` to filter the view
- `--json` exists but plain output is usually enough
- One logical change per commit, conventional commit messages
