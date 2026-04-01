---
paths:
  - "**/*.py"
  - "**/*.pyi"
---
# Python Coding Style — wkentaro

## Scoping & Constants

- **Scope variables to their usage site** — Don't define globals when a value is only used in one function. Hardcode it locally until reuse demands otherwise.
- **Don't prematurely parameterize** — A value used in one place should be a local constant, not a function argument with a default. Promote to a parameter only when a caller actually needs to vary it.
- **Constants are `Final` + `UPPER_CASE`** — If a local value is fixed, annotate with `Final` and use caps.

## Functions & Entry Points

- **Thin entry points** — `main()` parses args and delegates. Config data (like lookup tables) lives in `main`, not at module scope.
- **Functions define scope, not just reuse** — Extract functions to limit variable lifetimes and flatten nesting, even if called only once.

## Naming & Comments

- **Names replace comments** — If removing a docstring makes the function unclear, rename the function. Invest in naming over commenting.
- **Comments explain why, not what** — Only keep comments for non-obvious reasoning. Delete docstrings that restate what the code does.

## Call Sites

- **Use kwargs** — Unless trivially obvious (`len(x)`, `max(items)`, `shape(aoi)`), spell out keyword arguments.
