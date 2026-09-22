#!/usr/bin/env bash

set -euo pipefail

_install_ponytail() {
  if command -v claude >/dev/null; then
    claude plugin marketplace add "DietrichGebert/ponytail"
    claude plugin install "ponytail@ponytail" --scope user --yes
  fi

  if command -v codex >/dev/null; then
    codex plugin marketplace add "DietrichGebert/ponytail"
    codex plugin add "ponytail@ponytail"
  fi
}

main() {
  npx skills add "wkentaro/agent-skills" -g -a claude-code codex -y
  npx skills remove where-am-i sending-pull-request -g -y

  if [ -d "./private" ]; then
    npx skills add ./private/agents -g -a claude-code codex -y
  fi

  npx skills add "vercel-labs/before-and-after#main" -g -a claude-code codex -y

  npx skills add "humanlayer/skills" -s show-me -g -a claude-code codex -y

  npx skills add "coreyhaines31/makerskills" -s maker-council -g -a claude-code codex -y

  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/engineering" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/productivity" -s "*" -g -a claude-code codex -y
  npx skills remove ask-matt implement teach tdd to-questionnaire -g -y
  npx skills add "https://github.com/mattpocock/skills/tree/main/skills/in-progress" -s "pr" -g -a claude-code codex -y

  npx skills add "pbakaus/impeccable" -s "impeccable" -g -a claude-code codex -y

  _install_ponytail
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
