---
name: review-fix
description: Run /code-review, /simplify, /brooks-review, /review, and /exemplar-review on a change via parallel subagents, address the meaningful findings, fold the fixes into clean commits, and force-push with lease. Use when the user wants to review-and-fix a change before merge — on uncommitted work, the current feature branch, or a GitHub PR / GitLab MR. Triggers include "review-fix", "review and fix this PR/MR", "polish this branch", or "run the reviews, address the suggestions, then force-push".
---

# /review-fix — Review a change, address findings, commit clean, force-push

Run five reviewers over a change, apply the meaningful suggestions, fold the fixes
into clean commits, and force-push with lease. Works on four kinds of target:

- **Uncommitted changes** (no arg, dirty working tree) — apply fixes, commit. No push.
- **Current feature branch** (no arg, commits ahead of `main`) — fold fixes, force-push.
- **GitHub PR** (`#1234`, `1234`, or a `.../pull/1234` URL) — check out, fix, force-push.
- **GitLab MR** (a `.../merge_requests/1234` URL) — check out, fix, force-push.

## Argument

Optional. The target. If omitted, operate on the current branch and working tree.

## Workflow

### 0. Resolve the target into a local working state

```bash
git branch --show-current
git status --short
git log origin/main..HEAD --oneline
```

- **No arg** — work on the current branch / working tree as-is.
- **GitHub PR** (`#N`, bare `N`, or `github.com/.../pull/N`) — the working tree must be
  clean first (`git status --short` empty); if dirty, stop and ask the user to stash or
  commit. Then `gh pr checkout N`.
- **GitLab MR** (`gitlab.../merge_requests/N`) — same clean-tree check, then
  `glab mr checkout N`.

Identify the base branch (usually `main`); the reviewers diff against it.

### 1. Review-and-fix loop (repeat until a round is clean)

This is a **loop**, not a one-shot. Keep cycling until a full review round surfaces no
meaningful findings. Do not stop after the first pass, and do not ask the user to say
"re-run" — the loop is the whole point of this skill.

Each round:

**1a. Run the five reviews in parallel (report-only subagents).** Spawn **five**
subagents in a single message so they run concurrently. Pin each reviewer's model per the
**Model** column below: correctness-focused reviewers run on Opus (a missed bug costs more
than the tokens), taste/structure reviewers on Sonnet (holds up fine, saves the bulk of
the spend across up to 5 rounds). You (the orchestrator) stay on the session model to
triage and apply fixes. They are **read-only**:
each one only *reports* findings — none of them writes files, commits, or pushes. You are
the single writer (1c); concurrent `/simplify` + `/code-review --fix` in one working tree
would race on edits.

| Subagent | Model | Prompt |
| --- | --- | --- |
| code-review | opus | "Invoke the `/code-review` skill (high effort, no `--fix`, no `--comment`) on the current branch diff vs `<base>`. Do not modify, commit, or push anything. Return the meaningful findings as a numbered list: file:line, the issue, and the suggested change." |
| simplify | sonnet | "Run the `/simplify` skill's analysis on the current branch diff vs `<base>`, but DO NOT write any files. Instead return the simplifications it would make as a numbered list: file:line, what to simplify, and the proposed edit." |
| brooks-review | sonnet | "Invoke the `/brooks-review` skill on the current branch diff vs `<base>`. Do not modify any files. Return the findings as Symptom → Source → Consequence → Remedy." |
| review | opus | "Invoke the `/review` skill on the current branch diff vs `<base>` (review the diff directly — do not assume a PR exists). Do not modify, comment, commit, or push anything. Return the meaningful findings as a numbered list: file:line, the issue, and the suggested change." |
| exemplar-review | sonnet | "Invoke the `/exemplar-review` skill on the current branch diff vs `<base>`. Do not modify, commit, or push anything. Return the **Top fixes** as a numbered list: the finding, the fix, and your confidence." |

Wait for all five to return.

**1b. Triage findings → the meaningful set.** Merge and dedupe the five reports. Keep
only what is worth a code change:

- **Keep**: real correctness bugs, genuine simplifications/reuse, true design or
  maintainability problems with a concrete remedy.
- **Drop**: stylistic nits that already match the repo's conventions, anything that
  conflicts with the user's documented code style, speculative or out-of-scope
  "improvements", and likely false positives.

When a finding is borderline low value, prefer skipping it — surgical changes beat
churn. Print the kept set and the dropped set (one line each) so the user sees the call.

**1c. Apply the accepted fixes (you are the only writer).** Read each file before editing
it. Apply the kept findings as tight, surgical edits. Every changed line should trace to
a kept finding — do not "improve" adjacent code. Leave the edits in the working tree;
defer committing until step 2.

**1d. Verify — fast checks *and* behavior.** Run the project's fast checks (tests / lint /
type-check) for the files you touched (`ruff`, `ty`, and the relevant `pytest` here).
Treat green as necessary, not sufficient: a passing suite says nothing about output no
test asserts on. So when the diff touches an **output path** — help/usage text, rendered
or formatted output, ANSI/rich markup, templates, serializers, log formatting — also *run*
the affected surface and read the result, diffing against the base version's output where
feasible. Deleting an escape/quote/encode/sanitize/validate call is the sharpest case: it
reads as a clean simplification but is often a load-bearing guard, and only rendering shows
the breakage. When you find such a bug, lock it with an assertion on the rendered output
so these cheap checks catch a recurrence. Fix any red check or wrong render before the
next round; do not commit while checks are red.

**1e. Decide whether to loop again.**

- If this round kept **one or more** findings → go back to **1a**. The new round reviews
  the now-fixed code and catches issues the fixes introduced or exposed.
- If this round kept **zero** findings (all five reviewers came back clean or
  drop-only) → the change is settled. Exit the loop and go to step 2.

**Stop conditions to avoid spinning.** Cap at **5 rounds**. Also bail out early if you
notice oscillation (a later round re-proposes an edit you deliberately dropped, or
reverses a fix from an earlier round). If you hit the cap or detect oscillation before a
clean round, stop, list what is still outstanding, and ask the user how to proceed
instead of looping further.

### 2. Commit clean

- **Uncommitted target** (nothing ahead of base) — commit the fixes. Use the `git-hunk`
  CLI to split into logical commits. Stop here: there is nothing to force-push.
- **Branch / PR / MR target** (commits ahead of base) — fold each fix into the commit it
  belongs to. `git commit --fixup` only captures *staged* changes, so stage each logical
  group before committing it:

  ```bash
  git add <paths-for-this-group>                          # stage just this group's fix
  git commit --fixup=<sha-of-the-commit-being-fixed>      # repeat per logical group
  GIT_SEQUENCE_EDITOR=true git rebase -i --autosquash --autostash <base>
  ```

  (To split different hunks of one file across separate fixups, use `git-hunk` to stage
  at hunk granularity rather than whole files.) Interactive rebase is unavailable in this
  environment; `GIT_SEQUENCE_EDITOR=true` runs the autosquash non-interactively. If the
  autosquash stops on a merge conflict, resolve it and `git rebase --continue`; if that is
  not clean, `git rebase --abort` and commit the fix as its own commit rather than leaving
  the tree mid-rebase. If a fix is genuinely new behavior rather than a correction to an
  existing commit, make it its own commit via `git-hunk` instead.

Never add a `Co-authored-by` trailer.

### 3. Confirm, then force-push with lease

Force-pushing is hard to undo. **Stop and show the user**: the kept-findings summary,
`git log <base>..HEAD --oneline`, and `git diff <base>..HEAD --stat`. Ask for explicit
permission to push. Earlier approval of the overall task is **not** this permission.

Only after the user says go:

```bash
git push --force-with-lease
```

For a fresh-from-`main` uncommitted target there is nothing to push — end at step 2 and
say so.

## Notes

- The five reviewers must stay report-only. If you ever let `/simplify` or
  `/code-review --fix` write in a subagent, parallel runs corrupt each other's edits.
- "Meaningful" is a judgment call, not a checklist. Defend the drops if asked.
- The five reviewers (1a) are static — they cannot see bugs that only surface at runtime;
  that is what the behavioral half of 1d exists to catch, not them.
- If prior work was stashed to check out a PR/MR in step 0, remind the user it is stashed
  when you finish.
