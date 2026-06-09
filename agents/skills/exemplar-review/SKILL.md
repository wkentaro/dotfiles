---
name: exemplar-review
description: Review a diff against the best exemplars for its domain — checking both whether it reinvents a community-standard tool or idiom (prior-art), and whether it clears the quality bar a recognized exemplar would hold (excellence-bar, e.g. Apple-grade UI, Stripe-grade docs). Use when the user wants a taste, quality, or idiom pass, asks "is this <X>-grade?", "what would <person> think?", "are we reinventing the wheel?", "is there a standard way?", or runs /exemplar-review. Complements correctness/simplify/maintainability reviewers with the perspective they structurally lack.
---

# exemplar-review

The reviewer that asks two questions local reviewers can't see:

- **Prior-art** — *Is there a community-standard tool, library, or idiom a fluent practitioner would reach for, that this diff reinvents or diverges from?*
- **Excellence-bar** — *Does this clear the bar a recognized exemplar would hold it to?* (Apple-grade UI, Stripe/Vercel-grade docs, Jobs-grade scope, Reitz-grade API ergonomics.)

Correctness/simplify/maintainability reviewers optimize *within* the chosen approach. This one questions the approach and grades the taste.

## Workflow

1. **Read the diff and classify it** by artifact × dimension (filetypes, imports, frameworks, what changed). See [REFERENCE.md](REFERENCE.md) for the routing table.
2. **Pick 1–2 exemplars** and state, in one line each, *why this diff routed to them* (cite the signal). Don't sprinkle names — two well-chosen beats five.
3. **Run Mode A (prior-art)** for each: is there an established tool/pattern? Name it, show its canonical usage, say what to replace and what it deletes.
4. **Run Mode B (excellence-bar)** for each: decompose the exemplar's standard into a **rubric of observable criteria** (see REFERENCE.md templates), score the diff against each, give a specific fix per gap. Never a bare verdict.
5. **Synthesize**: rank the findings by leverage; mark each with a confidence and the tradeoff it carries.

## Non-negotiable rules

- **Lens, not oracle.** Ground every finding in the exemplar's *citable, shipped* work or documented principle ("requests tests against httpbin, so…"; "Apple HIG requires an empty state"). **Never** fabricate "X would say." The name selects the idiom set; the technical merit is the proof.
- **Mode B emits a rubric, never a vibe.** "Make it more Apple" is banned. Decompose into observable criteria → score → concrete fix. Exemplars with public, observable output (Stripe docs, Apple HIG, a maintainer's library) work; a generic "genius" persona drifts to motivational fluff — refuse to route to one.
- **Respect constraints.** Aspirational ≠ ignoring the repo's conventions, the user's documented style, or stated scope. A fix that violates a hard constraint is not a fix; name the tension instead.
- **Guard against authority bias.** A famous name does not make a finding true. If you're not confident, say so. Defend drops on merit.
- **Read-only when run inside another reviewer.** Under `/review-fix`, only *report*; never edit, commit, or push.

## Output

Per exemplar:

```
### <Exemplar> — chosen because <signal in the diff>

Prior-art:
- <standard/tool> — diff does <X>; replace with <canonical usage>; deletes <Y>. [confidence]

Excellence-bar (rubric):
| criterion (observable) | bar | current | fix |
| ... | ... | ... | ... |
Grade: clears / falls short on <n> criteria.
```

Close with a ranked, deduped **Top fixes** list (leverage-ordered), each one line: finding, fix, confidence.

## Modes of invocation

- **Standalone**: `/exemplar-review` on the current diff (or a named target).
- **Embedded in an orchestrator** (e.g. as an extra reviewer under `/review-fix`): report-only; return the **Top fixes** list (one numbered finding per line, each with its fix and confidence) so the orchestrator can triage alongside the other reviewers.

## Litmus

It's working when it surfaces things the other reviewers can't — both *"delete this, use the boring standard tool"* and *"this empty state isn't Stripe-grade; here's the missing copy and the fix."* If a run only restates correctness/style nits, it routed wrong or stayed too generic.
