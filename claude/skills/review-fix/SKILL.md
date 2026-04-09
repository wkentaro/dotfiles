---
name: review-fix
description: >
  Review dirty (uncommitted) code changes with a fresh Claude Code subagent,
  fix what it flags, and loop until clean. Trigger phrases: "review+fix",
  "loop to fix dirty", "clean up before commit", "review and fix".
  Works locally in any git repo. Uses Claude Code only (no Codex).
---

# Review+Fix Skill

Runs a tight review→fix→review loop within your current session.
A fresh Claude Code process reviews the diff with no prior context (unbiased),
you apply the fixes, then repeat until the reviewer has nothing left to flag.

---

## When to use

- After implementing a feature / bug fix — before committing
- When overnight-worker has made changes and you want them tightened up
- Any time you want a quick second opinion on dirty changes

---

## How it works

1. Get the current dirty diff (`git diff HEAD` — staged + unstaged)
2. Spawn Claude Code with `--print` mode to review (fresh context, no bias)
3. Parse the output: list of issues to fix
4. Apply the fixes (edit files directly)
5. Repeat from step 1 — stop when Claude Code reports nothing left to fix
6. Report how many cycles were needed

---

## Step-by-step procedure

### 1. Identify the working directory

Ask the user which repo to target, or infer from context.
Default: current working directory.

```bash
cd <repo-path> && git status
```

If `git status` shows nothing dirty, tell the user and stop.

### 2. Run one review+fix cycle

**Get the diff:**

```bash
cd <repo-path> && git diff HEAD
```

If the diff is empty but there are untracked files:

```bash
git diff HEAD && git ls-files --others --exclude-standard
```

**Spawn Claude Code to review (fresh context):**

```bash
cd <repo-path> && git diff HEAD | claude --permission-mode bypassPermissions --print \
  "You are a code reviewer. Review the following git diff carefully.

IMPORTANT: You MUST check against these project rules (read them before reviewing):
- ~/.claude/rules/python/wkentaro-style.md (Python style: kwargs, type annotations,
  thin entry points, no inline imports, no test classes, naming over comments, etc.)
- ~/.dotfiles/claude/rules/common/kent-beck-style.md (30 Kent Beck principles:
  composed method, intention-revealing names, guard clauses, single responsibility,
  say things once, query/command separation, etc.)

List every concrete issue you find: bugs, style violations, leftover debug code,
missing error handling, inconsistencies with the surrounding codebase,
violations of the wkentaro-style and Kent Beck rules above, etc.
Be specific: file name, line range, what's wrong, which rule is violated, what to do instead.
If the diff looks clean and you have nothing to flag, respond with exactly: LGTM"
```

**Parse the output:**
- If response is `LGTM` (or contains no actionable items) → stop the loop
- Otherwise → proceed to fix

### 3. Apply fixes

Fix every issue the reviewer flagged using your edit/write tools directly.
Do not spawn another agent for the fixes — handle them yourself in this session.

After fixing, briefly summarize what you changed.

### 4. Loop

Go back to step 2. Before each cycle, print:

```
--- Review+Fix cycle N ---
```

### 5. Stop condition

Stop when:
- Claude Code responds with `LGTM`, OR
- Claude Code flags fewer than 3 minor style nits with no bugs/logic issues, OR
- 5 cycles have passed (safety limit — tell the user if you hit this)

### 6. Final report

```
✅ review+fix complete in N cycle(s).
Summary of changes: <brief list>
Ready to commit.
```

---

## Tips

- **Keep the diff focused.** If the repo has massive unrelated changes, pass only the relevant files:
  ```bash
  git diff HEAD -- path/to/file.py | claude --permission-mode bypassPermissions --print "..."
  ```

- **Staged only:** To review only staged changes:
  ```bash
  git diff --cached | claude --permission-mode bypassPermissions --print "..."
  ```

- **After overnight-worker PRs:** Check out the branch first, then run review+fix before merging.
  ```bash
  git checkout <branch> && git diff main...HEAD | claude ...
  ```

- **Cycle limit:** If you hit 5 cycles without LGTM, stop and tell the user — likely a deeper design issue that needs human judgment.
