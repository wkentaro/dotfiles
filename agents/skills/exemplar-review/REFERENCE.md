# exemplar-review — reference

## Routing table (artifact signal → exemplar → citable basis)

Route on what actually changed. Pick the 1–2 with the strongest signal.

| Diff signal | Exemplar(s) | Authoritative for (citable) |
| --- | --- | --- |
| `SKILL.md`, `.claude/`, agent/prompt/tool defs | Anthropic skill & prompt-engineering docs | skill structure, progressive disclosure, tool-use, eval design |
| `*.ts/*.tsx`, generics, type-level code | Matt Pocock (Total TypeScript) | inference, no `any`, ergonomic types |
| Ruby, `Gemfile`, `app/`, `config/routes.rb` | DHH (Rails Doctrine) | convention over configuration, majestic monolith, no needless ceremony |
| HTTP client / SDK / public API surface, `pyproject`/packaging | Kenneth Reitz (requests), Armin Ronacher (Flask/Click) | API "for humans", sensible defaults, hard-to-misuse |
| Python stdlib idiom, readability | PEP 20 / PEP 8 (Guido & core) | one obvious way, idiomatic Python |
| UI components, CSS, interaction states | Apple HIG, Dieter Rams (10 principles), shadcn/Tailwind norms | states, motion, alignment, affordances |
| Docs, README, DX, blog, changelog | Stripe docs, Vercel, Divio doc system | structure, runnable samples, progressive disclosure |
| Feature scope, "what to cut" | Jobs/Ive (focus), 37signals (Shape Up) | ruthless scoping, one primary action |
| CLI flags / output / UX | `gh`, Charm, Click/Typer norms | noun-verb, `--json`, human default, helpful errors |
| Distributed/data/reliability | Kleppmann (DDIA), Google SRE | failure modes, idempotency, backpressure |
| Tests | Meszaros (xUnit Test Patterns), requests/httpbin, pytest ecosystem | real-vs-mock, fixtures, **prior-art tools** |

Multi-domain diff: pick the dominant axis plus one secondary; say so. Unsure of an exemplar's *actual* documented stance → don't route to them.

## Mode B rubric templates (observable criteria only)

Score each row `clears / falls short`; every gap gets a concrete fix.

**UI (Apple/Rams):** empty + loading + error states designed; optical alignment/spacing; motion meaningful & interruptible; plain action-labeled copy; no dead ends (every state has a next action); focus/keyboard/touch targets; contrast meets WCAG.

**Docs/DX (Stripe/Vercel/Divio):** quickstart copy-pasteable and runs fast; the four doc types not conflated (tutorial / how-to / reference / explanation); every sample runnable; progressive disclosure (no wall of options); errors & edge cases documented; skimmable (one idea per section).

**API/library (Reitz/Ronacher):** 80% case is one obvious call; sensible defaults, config optional; typed actionable errors; no required boilerplate; intent-revealing consistent names; hard to misuse.

**CLI (gh/Click):** noun-verb + discoverable `--help`; human default, `--json` for machines; exit codes + errors to stderr; no interactive prompt blocking scriptable paths.

**Scope (Jobs/Shape Up):** what can be cut with no loss? one clear primary action per surface; solving a stated need vs speculative.

## Worked example (dogfood — the miss that motivated this skill)

Diff: a CLI's `tests/` hand-rolls an `http.server` stub (`StubServer` + `_StubHTTPServer` + `_StubHandler`, ~50 lines in a vaguely-named `support.py`) to test an HTTP client.

- **Signal → exemplar:** Python test infra + HTTP-client-under-test → Kenneth Reitz / the requests testing lineage.
- **Prior-art finding:** requests tests against **httpbin**; the maintained standard today is **`pytest-httpserver`** — a *real* local server fixture (not a mock, so it satisfies "prefer integration over mocks"), with built-in request assertions. Replace the hand-rolled stub with the `httpserver` fixture; deletes ~50 lines *and* `support.py` entirely. **Confidence: high. Tradeoff:** one dev-only dependency (shipped runtime stays zero-dep — a separate axis from "stdlib-only at runtime").

Why the other reviewers missed it: code-review found no bug (there wasn't one), simplify only simplifies *existing* code, brooks-review *praised* the stub as exemplary. None was chartered to ask "is there a standard tool that deletes this file?" — which is Mode A's whole job.
