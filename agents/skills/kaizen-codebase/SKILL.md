---
name: kaizen-codebase
description: Run one autonomous codebase-improvement pass on the current repo. Find the single highest-leverage, low-risk improvement (architecture, tech debt, dead code, test gaps, simplification) and either ship it as a draft PR or, when the change needs human judgment, file a ready-for-human issue. Does exactly one thing then stops, so it composes with /loop for recurring improvement (e.g. `/loop /kaizen-codebase` or `/loop 1d /kaizen-codebase`). Use when the user wants to continuously or periodically improve a repository, asks to "improve the codebase", set up a recurring code-quality agent, or run one improvement iteration.
---

# Kaizen codebase (one pass)

One bounded improvement per invocation, then stop. Compose with `/loop` for
recurrence (`/loop /kaizen-codebase`, `/loop 1d /kaizen-codebase`). Never
schedule yourself; `/loop` owns pacing.

## Contract

- Exactly one outcome per pass: one draft PR, OR one issue, OR a clean no-op.
- **Confident + surgical → draft PR. Uncertain / needs a human call →
  `ready-for-human` issue. Nothing worthwhile → exit.** An empty pass is a
  success, not a failure; never manufacture churn to justify a run.
- Never merge, never force-push without `--force-with-lease`, never push to the
  base branch, never touch a dirty working tree.

## 0. Preflight

- Abort if not a git repo, or if the working tree is dirty (don't mix with the
  user's WIP — say so and stop).
- `git fetch`; identify the base branch (`main`/`master`); ensure local base is
  current and you are on it.
- Detect optional substrate, use what exists and skip what doesn't: `CONTEXT.md`,
  `docs/adr/`, triage labels, a GitHub remote. Repos like gdown/imgviz may have
  none of these — that is fine.

## 1. Pick ONE candidate

- Read any ADRs + `CONTEXT.md` first so you don't re-litigate settled decisions.
- List open PRs and issues; skip anything already proposed or in flight.
- Explore broadly with subagents (architecture depth, tech debt, dead code, test
  gaps, simplification) to keep your context clean. Choose the single
  highest-leverage, lowest-risk candidate. Apply the deletion test and prefer
  surgical seams over sweeping rewrites.

## 2. Classify the candidate

File a **`ready-for-human` issue** (do NOT write code) when ANY holds:

- needs a product or architecture decision with more than one reasonable answer
- large blast radius (public API, serialized/on-disk format, many call sites)
- contradicts or would amend an existing ADR
- cannot be proven correct by cheap or existing tests
- requirements are ambiguous or depend on intent only the maintainer knows
- the change is risky or hard to reverse

Otherwise ship a **draft PR**, but only when ALL hold: one cohesive surgical
change, low-risk, reversible, and verifiable green (tests + lint) before/after.

When in doubt, escalate to an issue. Shipping an uncertain change unattended is
the failure mode this skill exists to avoid.

## 3a. Ship it (confident path)

1. Apply the minimal change. Read the code before editing it; touch only what the
   change requires.
2. Invoke `/review-fix` on the diff (reviewers → triage → fix → verify
   tests+lint). Stop on a clean round or on oscillation.
3. Invoke `/make-branch` to move the commits onto a well-named feature branch.
4. Invoke `/make-pr` and open it as a **draft** (this is an autonomous run).
5. If a load-bearing decision was made or a candidate was rejected for a reason a
   future pass would need, record it as an ADR (or in `CONTEXT.md`) so the loop
   doesn't repeat itself.

## 3b. Escalate (uncertain path)

- Ensure the `ready-for-human` label exists (`/setup-github-labels`, or
  `gh label create ready-for-human` if missing).
- Dedup against open issues, then open ONE issue: the candidate, why it needs a
  human, the options/tradeoffs, and the affected files. Assign the repo owner
  (`--assignee @me`) and apply `ready-for-human`.
- If there is no GitHub remote, append the candidate to a local
  `docs/kaizen-codebase-backlog.md` instead, and say so.

## 4. Report

End with one line: the PR url, the issue url, or "no high-leverage change this
pass." `/loop` reads this to pace the next run.
