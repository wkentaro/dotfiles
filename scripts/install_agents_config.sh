#!/usr/bin/env bash

# Skills install from wkentaro/brain with `just install`.

set -euo pipefail

main() {
  if command -v claude >/dev/null; then
    claude plugin marketplace add "DietrichGebert/ponytail"
    claude plugin install "ponytail@ponytail" --scope user --yes
  fi

  if command -v codex >/dev/null; then
    codex plugin marketplace add "DietrichGebert/ponytail"
    codex plugin add "ponytail@ponytail"
  fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
