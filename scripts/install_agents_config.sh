#!/usr/bin/env bash

set -euo pipefail

_install_renamed() {  # <git-url> <skill-subdir> <new-name>
  local tmp
  tmp=$(mktemp -d)
  git clone -q --depth 1 "$1" "$tmp"
  sed -i "s/^name: .*/name: $3/" "$tmp/$2/SKILL.md"
  npx skills add "$tmp/$2" -g -a claude-code codex -y --copy
  rm -rf "$tmp"
}

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
  npx skills remove workmap where-am-i sending-pull-request session-status resume-note pr-verdict -g -y

  if [ -d "./private" ]; then
    npx skills add ./private/agents -g -a claude-code codex -y
    npx skills remove writing-social-posts -g -y
  fi

  npx skills add "vercel-labs/before-and-after#main" -g -a claude-code codex -y

  npx skills add "humanlayer/skills" -s show-me -g -a claude-code codex -y

  npx skills add "DannyMac180/skills" -s be-ambitious -g -a claude-code codex -y

  # npx skills add "coreyhaines31/makerskills" -s maker-council -g -a claude-code codex -y
  npx skills remove maker-council -g -y

  npx skills add "https://github.com/mattpocock/skills/tree/v1.3.1/skills/engineering" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.3.1/skills/productivity" -s "*" -g -a claude-code codex -y
  npx skills remove ask-matt grill-me implement tdd teach to-questionnaire wizard -g -y

  npx skills add "pbakaus/impeccable" -s "impeccable" -g -a claude-code codex -y

  _install_renamed "https://github.com/cursor/plugins" pstack/skills/unslop unslop-writing
  _install_renamed "https://github.com/heydamir/unslop" skills/unslop unslop-ui
  npx skills add ./agents/skills/unslop -g -a claude-code codex -y

  _install_ponytail

  npx skills add "emilkowalski/skills" -s animate apple-design break-ui find-animation-opportunities mobile-native review-animations -g -a claude-code codex -y

  npx skills add "coreyhaines31/marketingskills" -a claude-code codex -g -y
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
