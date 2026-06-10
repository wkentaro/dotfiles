# wkentaro.md

## Guardrails

- When you change a code, make sure you've read that code. Change shouldn't be made without reading and understanding.
- When the user asks "why did you do X?" or "what about Y?" — they are asking genuinely, not telling you to change. Explain your reasoning, defend it if you believe it's right, and only change course when actually convinced. Don't reflexively revert decisions at the first question.
- When you make changes, know that your changes will be reviewed by your competitor Codex or Claude Code.
- When you write an English sentence, don't use emdash. When you notice you wrote them, fix them.
- When doing rewrite for relicensing, don't disclose that intent in the public PR title, branch name, and description.
- When you create a PR/MR, use /make-pr or /make-mr skill.
- Before any unrevertable (or hard-to-unrevert) action, ask for explicit final permission immediately before doing it. Earlier approval of an approach, plan, or "fix X" is not that permission. This covers merging a PR/MR, tagging a release, publishing to a registry (PyPI, npm), deleting/force-pushing, sending outward-facing messages, and anything else you can't cleanly undo.
- When you create a temporary file, create it in tmp/ directory in the repo.

## Code Style

- Avoid unnecessary comments. Code should be self-explanatory. Only add comments when the logic is truly non-obvious.
- When you commit, don't put Co-authored-by Claude.
- When you switch branches, use `git switch` instead of `git checkout`.
- When you use python, use `uv`.
- When you merge a pull request, ensure the branch is up-to-date with main (rebase if needed), and then merge with merge commit.

## Skill choice

- When you commit code, use the git-hunk CLI to split into logical commits. Run `git-hunk skills get core --full` first to learn its usage.
- When you access a website, use the agent-browser CLI to navigate and extract information. Run `agent-browser skills get core --full` first to learn its usage.
- When you make a pr, use /make-pr.

## Workflow

- For non-trivial tasks (3+ steps or architectural decisions), plan before coding. If it goes sideways, stop and re-plan rather than pushing on.
- Use subagents liberally to keep the main context clean: offload research, exploration, and parallel analysis. One focused task per subagent.
- Don't mark work done until you've proven it: run the tests/app, and diff behavior against main when relevant.
- After a correction, record it in the project's CLAUDE.local.md (gitignored, auto-loaded, so it applies next session in that project). When a lesson generalizes across projects, promote it into ~/.claude/rules/wkentaro.md and delete it from CLAUDE.local.md so the buffer stays small. Discard one-offs.

## Coding style

### Scoping & Constants

- **Scope variables to their usage site** — Don't define globals when a value is only used in one function. Hardcode it locally until reuse demands otherwise. Module-level constants are justified only when shared across modules or across multiple functions; a constant used in a single function belongs inside that function.
- **Don't prematurely parameterize** — A value used in one place should be a local constant, not a function argument with a default. Promote to a parameter only when a caller actually needs to vary it.
- **Constants are `Final` + `UPPER_CASE`** — If a local value is fixed, annotate with `Final` and use caps.

### Functions & Entry Points

- **Thin entry points** — `main()` parses args and delegates. Config data (like lookup tables) lives in `main`, not at module scope.
- **Functions define scope, not just reuse** — Extract functions to limit variable lifetimes and flatten nesting, even if called only once. Prefix with `_` unless the function is an intentional public API.
- **Don't extract trivial single-use helpers** — A 1-2 line helper with one caller adds indirection without payoff; inline it. Extract only when the name adds meaning the expression lacks, it narrows variable scope, or it flattens nesting.
- **Avoid ravioli code** — A function that reads top-to-bottom beats logic shredded into single-caller helpers (each is a jump the reader must reassemble). A module made *mostly* of one-caller helpers is fragmented, not modularized. A `# section` comment inside a long function is the signal to extract; "this could be a function" is not. After any simplification, re-audit touched helpers and inline wrappers a change reduced to a single trivial caller.

### Control Flow

- **Early continue / early return** — In loops, invert the condition and `continue` so the main body lives at the outer indent. Same for functions: handle the negative cases as guard clauses up top, then write the happy path unindented. `if cond: do_a_lot; return` becomes `if not cond: continue` (or `return`); the work after no longer hides behind extra indentation.

### Class Design

- **Data-class-like classes over container-mimicking ones** — Prefer classes that expose their state through named attributes and explicit edit methods, not classes that overload `__getitem__` / `__setitem__` / `__len__` / `__iter__` to pretend to be a list or dict. `shape.points[i]` and `shape.move_vertex(i, pos)` make the data shape and the mutation surface obvious; `shape[i] = pos` hides what's being indexed and conflates the object with its inner collection. Reserve container dunders for types whose entire purpose is to be a container (custom collection classes), not for domain entities that happen to hold a list.

### Imports

- **No inline imports** — keep all imports at the top of the file. Inline/deferred imports are only justified for breaking circular dependencies, which should be rare.

### Naming & Comments

- **Names replace comments** — If removing a docstring makes the function unclear, rename the function. Invest in naming over commenting.
- **Comments explain why, not what** — Only keep comments for non-obvious reasoning. Delete docstrings that restate what the code does.
- **Verb-prefixed function names** — Default to a leading verb that names what the function does (`make_local_mask`, `compute_mask_iou`, `place_mask`, `round_bbox_to_int`). Use a non-verb form only when it reads strictly better for the specific case: predicates (`is_*`, `has_*`, `can_*`, `should_*`), classmethod constructors (`from_*`), and conversion idioms (`to_dict`). Prefer singular `is_*` over `are_*` even for binary relations — name the subject (`is_redundant_pair(new, peer)`, not `are_redundant(new, peer)`). Avoid noun-only names like `mask_iou` or adjective-noun names like `filled_mask_for_bbox` — they read as values, not actions.

### Call Sites

- **Use kwargs** — Unless trivially obvious (`len(x)`, `max(items)`, `shape(aoi)`), spell out keyword arguments.

### Type Annotations

- **Annotate all function signatures** — params + return type, including `-> None` on tests. One rule everywhere.
- **Don't annotate locals** unless the type checker can't infer the full type (e.g., `results: list[Hunk] = []`, `exclude: bool | None = None`).
- **Migration exception** — Annotate locals when upstream functions lack annotations. Remove once upstream is fixed.

### Testing

- **No test classes** — use plain `test_` functions with fixtures.
- **Mirror source layout** — test directories mirror source modules. When a module has multiple test files, use a subdirectory named after the module (e.g., `tests/unit/hunk/` for `hunk.py`). File names describe the aspect, not the module (`id_test.py`, not `hunk_id_test.py`).
- **Split files over grouping comments** — if you need a comment to separate test groups, split into separate files.
- **Deduplicate setup into fixtures** — shared arrange logic belongs in a `@pytest.fixture`, not copy-pasted across tests.
- **Prefer integration over mocks when speed and cost allow** — Mocking a downstream system only verifies your interpretation of its API; the test passes even when the real API changes shape. If a real instance is cheap to bring up in test (existing conftest fixture, ephemeral subprocess, dockerized service, in-memory engine), drive the test through it. Reserve mocks for genuinely impractical dependencies: paid third-party APIs, irreversible side effects (payments, emails to humans), or services with no offline mode. When unsure, measure: a sub-second real-dependency test is almost always better than the equivalent MagicMock, because it catches version drift in the dependency that a mock will silently absorb.

### Linting & Formatting

- **Ruff rules**: `E` (pycodestyle), `F` (pyflakes), `I` (isort), `UP` (pyupgrade), `ANN` (flake8-annotations).
- **Force single-line imports** — one `import` per line, enforced via `force-single-line = true` in isort config.
- **ty with all warnings as errors** — `[tool.ty.rules] all = "error"`.
