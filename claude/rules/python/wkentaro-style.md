---
paths:
  - "**/*.py"
  - "**/*.pyi"
---
# Python Coding Style — wkentaro

## Scoping & Constants

- **Scope variables to their usage site** — Don't define globals when a value is only used in one function. Hardcode it locally until reuse demands otherwise. Module-level constants are justified only when shared across modules or across multiple functions; a constant used in a single function belongs inside that function.
- **Don't prematurely parameterize** — A value used in one place should be a local constant, not a function argument with a default. Promote to a parameter only when a caller actually needs to vary it.
- **Constants are `Final` + `UPPER_CASE`** — If a local value is fixed, annotate with `Final` and use caps.

## Functions & Entry Points

- **Thin entry points** — `main()` parses args and delegates. Config data (like lookup tables) lives in `main`, not at module scope.
- **Functions define scope, not just reuse** — Extract functions to limit variable lifetimes and flatten nesting, even if called only once. Prefix with `_` unless the function is an intentional public API.

## Control Flow

- **Early continue / early return** — In loops, invert the condition and `continue` so the main body lives at the outer indent. Same for functions: handle the negative cases as guard clauses up top, then write the happy path unindented. `if cond: do_a_lot; return` becomes `if not cond: continue` (or `return`); the work after no longer hides behind extra indentation.

## Class Design

- **Data-class-like classes over container-mimicking ones** — Prefer classes that expose their state through named attributes and explicit edit methods, not classes that overload `__getitem__` / `__setitem__` / `__len__` / `__iter__` to pretend to be a list or dict. `shape.points[i]` and `shape.move_vertex(i, pos)` make the data shape and the mutation surface obvious; `shape[i] = pos` hides what's being indexed and conflates the object with its inner collection. Reserve container dunders for types whose entire purpose is to be a container (custom collection classes), not for domain entities that happen to hold a list.

## Imports

- **No inline imports** — keep all imports at the top of the file. Inline/deferred imports are only justified for breaking circular dependencies, which should be rare.

## Naming & Comments

- **Names replace comments** — If removing a docstring makes the function unclear, rename the function. Invest in naming over commenting.
- **Comments explain why, not what** — Only keep comments for non-obvious reasoning. Delete docstrings that restate what the code does.

## Call Sites

- **Use kwargs** — Unless trivially obvious (`len(x)`, `max(items)`, `shape(aoi)`), spell out keyword arguments.

## Type Annotations

- **Annotate all function signatures** — params + return type, including `-> None` on tests. One rule everywhere.
- **Don't annotate locals** unless the type checker can't infer the full type (e.g., `results: list[Hunk] = []`, `exclude: bool | None = None`).
- **Migration exception** — Annotate locals when upstream functions lack annotations. Remove once upstream is fixed.

## Testing

- **No test classes** — use plain `test_` functions with fixtures.
- **Mirror source layout** — test directories mirror source modules. When a module has multiple test files, use a subdirectory named after the module (e.g., `tests/unit/hunk/` for `hunk.py`). File names describe the aspect, not the module (`id_test.py`, not `hunk_id_test.py`).
- **Split files over grouping comments** — if you need a comment to separate test groups, split into separate files.
- **Deduplicate setup into fixtures** — shared arrange logic belongs in a `@pytest.fixture`, not copy-pasted across tests.

## Linting & Formatting

- **Ruff rules**: `E` (pycodestyle), `F` (pyflakes), `I` (isort), `UP` (pyupgrade), `ANN` (flake8-annotations).
- **Force single-line imports** — one `import` per line, enforced via `force-single-line = true` in isort config.
- **ty with all warnings as errors** — `[tool.ty.rules] all = "error"`.
