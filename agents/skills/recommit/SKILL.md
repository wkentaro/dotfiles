---
name: recommit
description: Reshape a branch's commits-that-aren't-in-main into a clean, logical, independently-valid sequence without changing the final tree — splitting grab-bag commits, folding fixups, ordering refactor-before-feature, dropping dead-infra commits — then stop (no push). Use when preparing a PR/MR branch for review or merge, cleaning up messy iterative history (review-fix amendments, fixups, interleaved refactors, WIP commits), or when the user says "clean up the git history", "tidy the commits", "recommit", or "make the history present nicely on the PR".
---

# /recommit — Reshape branch history into a clean, presentable sequence

Take the commits a branch has on top of `main` (often a mess after iterative work:
review-fix amendments, fixups, a refactor tangled into the feature) and rewrite them
into a clean, logically-ordered sequence where each commit tells one story and stands on
its own — **without changing the final tree**. Then stop. The branch will need a
force-push afterward; that is the user's call, not this skill's.

**The one invariant: the working tree at the end is byte-identical to the start.** This
is reshaping history, not editing code. If `git diff <original-HEAD> HEAD` is non-empty,
you made a mistake.

## When to use / not use

- **Use**: a feature branch ahead of `main` whose commit history is untidy and is about
  to be reviewed or merged. All commits in `main..HEAD` are fair game to rewrite.
- **Don't use**: to change behavior, to rewrite commits already in `main`, or on a
  branch others are actively building on without coordination.

## Workflow

### 0. Capture the starting state (your undo)

```bash
git rev-parse HEAD                 # ORIG — record this; it is your safety net
git log main..HEAD --oneline       # the commits you may rewrite
git status --short                 # working tree must be understood before you start
git diff main..HEAD --stat         # the whole change, at a glance
```

- Identify the base (usually `main`). **Only `main..HEAD` is rewritable.**
- Keep `ORIG` (the starting HEAD sha). At any point you can abort with
  `git reset --hard <ORIG>`, and you will verify against it at the end.
- **Uncommitted changes**: if the tree is dirty, decide whether those changes belong in
  the new history (fold them into the right commit) or should be set aside. If it is
  unclear, stop and ask rather than guessing.

### 1. Design the commit narrative (the judgment)

Read the full `git diff main..HEAD` and the existing commits, then write down the target
list — each commit's title and which files/hunks it owns. Principles:

- **One story per commit**, each independently buildable and testable.
- **Refactor/prep first, feature after.** Behavior-preserving changes that "make the
  change easy" land before the change that uses them.
- **Pull out standalone improvements.** A change that justifies itself independently of
  the feature (a general UX fix, a bug fix, a dependency bump) earns its own commit.
- **Infrastructure lands with its first user.** Never create a commit that adds a generic
  mechanism nothing calls yet — that is dead-code-on-arrival. Bundle it with the first
  thing that uses it.
- **Order by dependency.** A commit that uses `X` comes after the commit that introduces
  `X`.
- **Don't over-split.** Artificial atomization is as bad as a grab-bag. Group mechanical
  or generated changes (translations, lockfiles, snapshots) with the change that
  necessitated them, not as their own commit.

### 2. Rebuild the history

Pick the lightest technique that fits:

**A. Whole-commit operations** (reorder, reword, drop, squash, fold a fix into an earlier
commit) — use fixups + non-interactive autosquash:

```bash
git add <paths> && git commit --fixup=<sha>          # repeat per group
GIT_SEQUENCE_EDITOR=true git rebase -i --autosquash --autostash main
```

The interactive rebase *editor* is unavailable in this environment;
`GIT_SEQUENCE_EDITOR=true` runs the plan non-interactively. When you don't want to
hand-pick which ancestor each fix belongs to, `git absorb` (if installed) blames each hunk
to its target commit and emits the `--fixup` commits for you — then run the autosquash
above. If autosquash stops on a conflict, resolve and `git rebase --continue`; if it is
not clean, `git rebase --abort` and fall back to technique B.

**B. Splitting one file's changes across several commits** (the robust method — what
whole-commit rebases can't do):

1. Save the final version of every file that gets split across commits:
   `cp <path> tmp/<name>` (use the repo's `tmp/`).
2. Move HEAD to the base, keep the final tree, stage nothing:
   `git reset --soft <base> && git reset`
3. For each target commit, **in order**: bring its files to that commit's *intermediate*
   state. For a split file, start from the base version
   (`git show <base>:<path> > <path>`) and re-apply only this commit's edits; for files
   owned wholly by this commit, leave them as-is. Then stage exactly this commit's paths
   and commit:
   `git add <paths-for-this-commit> && git commit -m "<message>"`
   Files belonging to later commits stay unstaged (or untracked) until their turn.
4. Restore the saved final files (`cp tmp/<name> <path>`), `git add -A`, and commit the
   remainder.

For hunk-granular staging inside a file, prefer the **/git-hunk** skill over
hand-rebuilding intermediate files.

### 3. Verify (non-negotiable)

```bash
# the invariant, checked exactly: the new tree must equal the original tree
[ "$(git rev-parse <ORIG>^{tree})" = "$(git rev-parse HEAD^{tree})" ] && echo SAME-TREE || echo MISMATCH
```

If that prints `MISMATCH`, you changed content: fix it, or `git reset --hard <ORIG>` and
redo. Then prove each commit stands alone:

```bash
for sha in <new shas in order>; do
  git -c advice.detachedHead=false switch -d $sha
  # run the project's fast checks (discover them — don't assume):
  #   python: uv run ruff check . && uv run ty check . && uv run pytest -q
  #   node:   npm run lint && npm test
done
git switch <branch>
```

Each commit should at least lint and type-check; run tests where they are fast. End on
the branch with a clean tree and `git stash list` empty.

### 4. Stop

- Show the result: `git log main..HEAD --oneline`.
- State plainly that the branch now diverges from its remote and updating the PR/MR needs
  `git push --force-with-lease` — and that **you are leaving that to the user** (per this
  skill's contract and the rule that force-push needs explicit, immediate permission).

## Safety rules

- **`ORIG` is sacred.** Capture it first; it is the one-command undo (`git reset --hard
  <ORIG>`) and the tree-identity oracle. Reflog also recovers any pre-rewrite commit.
- **Never alter the final tree.** Reshaping ≠ editing. Behavior changes belong in a
  separate, explicit step before or after — not smuggled into a history cleanup.
- **Only `main..HEAD`.** Never rewrite commits already merged to `main`.
- **No `Co-authored-by` trailers.** Use `git switch` (not `git checkout`) to move between
  branches. Put scratch files in the repo's `tmp/`.

## Worked example

A "Language setting" PR had two commits: a `_create_combo` refactor, then one grab-bag
`feat` that bundled (a) a generic dialog auto-sizing improvement, (b) a `note` caption
mechanism, (c) the actual language feature + 20 translation files. Recommitted to:

1. `refactor(settings): extract _create_combo …` — behavior-preserving prep, kept first.
2. `feat(settings): fit the dialog height to the active tab` — a standalone UX fix that
   helps the existing dialog regardless of the feature; split out via technique B
   (settings_dialog.py was reconstructed to the sizing-only intermediate, committed, then
   restored to final for commit 3).
3. `feat: add a Language setting …` — the feature, with the `note` mechanism folded in
   (nothing used it until here) and the generated translations grouped with it.

`git diff <ORIG> HEAD` was empty; all three commits passed lint + type-check in detached
checkout. Then stopped, leaving the force-push to the user.
