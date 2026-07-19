---
name: coin
description: Coin a name for a product, feature, or company via a Lexicon-style identify → invent → implement pipeline (blind-briefed lanes, linguistic scoring, conflict screening), or resume a prior run for round 2.
disable-model-invocation: true
---

# coin — name a thing on purpose

Replicates Lexicon Branding's process (David Placek: PowerBook, BlackBerry,
Sonos, Azure, Vercel, Windsurf). You are **coining**, not brainstorming:
generate a wide funnel of directions with judgment suspended, then score
against a fixed rubric. A name is an experience the market will have, not a
word the team likes. Comfort is a warning sign; polarization is a sign of
strength.

## Arguments

- Free text or a path to a brief file → new run (brief file skips the interview).
- Path to an existing `naming-runs/<run>/` workspace → jump to **Round 2**.
- `--depth quick|default|deep` (default `default`) → funnel widths below.
- `--runs-dir <path>` → parent dir for the run workspace (default `./naming-runs/`).

| depth | per-lane directions | triage keep | score keep | shortlist |
|---|---|---|---|---|
| quick | 100 | 30 | 10 | 4–5 |
| default | 250 | 60 | 20 | 5–8 |
| deep | 250 ×2 lanes-per-role | 100 | 30 | 8–10 |

## Phase 1 — Identify

1. **Interview** (skip if a brief file was given). One question at a time:
   - The diamond: how do you define *winning*? What do you *have* to win
     (assets, traction)? What do you *need* to win? What do you have to
     *say* to win?
   - Behavior and experience: how does the product behave now, and how
     should it behave in the future? How should the market behave toward
     it? Which 3–6 experience qualities should the name evoke (e.g. flow,
     precision, warmth)?
   - Constraints: markets (default: global + Japan), name type (company /
     product / feature), must it work as a CLI command or package name?
2. **Landscape**: spawn one research subagent to map competitor names and
   the category's stock language (the words everyone uses — to *avoid*;
   descriptive names imitate, and imitation forfeits distinctiveness).
3. **Framework**: synthesize `framework.md` — target experience qualities,
   behaviors, what the name must be able to say (now and later), category
   language to avoid, and sound priorities chosen from the letter table in
   [references/linguistics.md](references/linguistics.md).
4. **Disguise briefs**: draft the two blind briefs (see lane roles below) —
   a fictional company in an adjacent category, and a physical object with
   the same experience qualities. Neither may mention the real product,
   category, or company.
5. **Sign-off gate**: present the framework and both disguise briefs to the
   user and stop. Phase 1 is complete only when the user has approved them
   — no invent tokens are spent before that. On approval, create the run
   workspace `<runs-dir>/YYYY-MM-DD-<slug>/` and write `framework.md`,
   `landscape.md`, `briefs.md`.

## Phase 2 — Invent + screen (one Workflow run)

Load [references/workflow.md](references/workflow.md) and run its script
verbatim via the Workflow tool, passing the framework, briefs, funnel
widths, and the absolute path of `references/linguistics.md` through
`args`. The script is frozen so every run takes the same shape: four blind
lanes → triage → linguistic scoring → screening. Do not paraphrase the
lane prompts — the disguise lanes work because the script, not you, holds
the briefs.

Lane roles (defined in the script):

1. **truth** — full real brief.
2. **disguise** — the fictional-adjacent-company brief.
3. **object** — the physical-object brief.
4. **mining** — no names: raw words, metaphors, sports, natural phenomena,
   morphemes around the experience qualities; a synthesis step compounds
   them into candidates ("Windsurf was sitting on a word list").

Screening semantics (the script's screeners follow these; hold them when
interpreting results):

- A taken `.com` is a **price flag, never a kill** — domains are area
  codes now; record and move on.
- Trademark results are **advisory — not legal clearance**; the
  presentation must carry that label verbatim.
- Cultural panel: English, Japanese, Spanish, Portuguese, German,
  Mandarin. Eliminate the terrible, flag the worrisome.

Phase 2 is complete when the workflow has returned and you have written
`funnel.md` (every cut logged with what was dropped and why — no silent
truncation), `directions/` (raw lane output), `scores.md`, and
`screening.md` into the workspace. Read the workflow journal if a result
looks empty; never assume.

## Phase 3 — Implement

Produce `presentation.html` in the workspace: a single self-contained page
(no external requests) presenting the shortlist. For every finalist:

- **The story** — rationale written as Lexicon writes it for a client:
  what the name lets the company say, and how the market will behave
  toward it. Start a story, don't make a statement.
- **Sound analysis** — how its letters and rhythm serve the framework's
  target qualities, grounded in [references/linguistics.md](references/linguistics.md).
- **Screening table** — domains, registries, collisions, trademark
  advisory, cultural flags.
- **Prototypes** — the name rendered in developer contexts: a terminal
  (`pip install <name>` / `npx <name>`), a GitHub repo header, a
  landing-page hero, a launch tweet. Seeing the name executed is what
  flips decisions.

End the page with the user's drills, addressed to them:

- Speculate, don't evaluate: ask "what could this name do for us?", never
  "do I like it?".
- Polarization test: if the team splits hard over one name, that's energy
  — Grove chose Pentium over ProChip because of the argument.
- Competitor drill: tell a friend "we just got a new competitor called
  X — what do you think?". You're not collecting opinions; you're watching
  what the name *does* to them. The winning reaction: "I don't know what
  they do, but they're not like the other guys."
- If the team is comfortable with a name, chances are it isn't the name yet.

Phase 3 is complete when every finalist has all four blocks and the page
opens clean from the file system. Close by offering Round 2 and, in one
line, suggest recording the eventual decision wherever the project keeps
decisions.

## Round 2 — resumable, offered never automatic

Given an existing workspace:

1. Read `framework.md`, `funnel.md`, `scores.md`, `screening.md`, plus any
   prior `round-2/` state.
2. Collect the user's reactions to the shortlist and derive **framework
   deltas** ("more tactile, less abstract; compounds outperformed coined
   words") into `round-2/deltas.md`.
3. Re-run the Workflow script with `args.round2`: two lanes seeded by the
   deltas instead of four, plus **rescue** — near-misses from the persisted
   funnel that the deltas now favor re-enter scoring without regeneration.
4. **Co-creation**: any names the user proposes enter the same scoring and
   screening stages as generated ones. Same rubric, no home-team bias.
5. Produce `round-2/presentation.html` showing round-2 finalists
   side-by-side with round-1 survivors — humans decide by comparing.
