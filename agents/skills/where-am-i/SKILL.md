---
name: where-am-i
description: Render the open decision points of a task as an ASCII tree so the user can see where they are and what to answer next. Use when a task has branched into several pending choices, when the user asks "where am I?" / "what do I need to decide?" / "what's left to decide", or invokes /where-am-i. About task decisions, not git branch or working directory. Pure presentation; reads state from the conversation, persists nothing.
---

# where-am-i

Render the task's decisions as an ASCII tree. The point is orientation: show the
path already settled, mark the active fork, and end with the exact questions left
to answer.

## How to render

Read the current state from the conversation, then draw a single tree.

- **Root** = the task in one short phrase.
- **`[N]`** numbers top-level decisions only, in the order they arose; sub-decisions
  stay unnumbered. The closing list references these `[N]` tags.
- **`Q:`** lines at each fork, stating the question being decided there.
- Indent sub-decisions under the option that produced them.

### Markers

Attach a marker after the node with a `◄──` connector (`node  ◄── ✅ chosen`),
as the example shows. Use this one form for every marker.

- `✅` — a decided node; append a terse why/result in parentheses.
- `📍 YOU ARE HERE` — the single active fork, the decision in question right now.
  Use it once per tree.
- `not taken` — an option considered and rejected.
- `(not started)` — a decision that exists but hasn't been reached yet.
- Keep settled branches **visible** (collapsed to one line, not deleted) so the
  user sees the path behind them, not only what is ahead.

### Always end with

A numbered **"Open questions, in order"** list below the tree. Include only the
*undecided* nodes, ordered the way they need answering. Reference each by its
`[N]` tag so the user can point back at the tree.

## Example

```
deploy pipeline rework
│
├─ [1] runner image          Q: pin or float?
│   ├─ pin to :1.4    ◄── ✅ chosen (reproducible CI)
│   └─ float :latest  ◄── not taken
│
├─ [2] cache strategy   ◄── 📍 YOU ARE HERE
│   │   Q: per-branch or shared?
│   ├─ shared ──┐  Q: evict by age or size?
│   │           ├─ age
│   │           └─ size
│   └─ per-branch
│
└─ [3] rollout   ◄── (not started)
        Q: canary or all-at-once?

Open questions, in order:
1. [2] cache strategy — shared or per-branch?
2. [3] rollout — canary or all-at-once?
```

## Rules

- One tree per render. Redraw the whole thing each time rather than patching it.
- Alignment is approximate — don't chase perfect columns; readability beats
  pixel-perfect spacing.
- Keep node labels short; detail goes in the parenthetical or the questions list.
- Do not invent decisions. Only show forks that actually came up in the task.
- If every decision is settled, render the full tree (all `✅`) and write
  `Open questions: none`. If no decision has come up yet, say so in one line
  instead of drawing a tree.
- No files, no state. The conversation is the source of truth.
