#!/usr/bin/env bash

set -euo pipefail

_install_brooks_lint() (
  local install_dir
  install_dir="$(mktemp -d)"
  trap 'rm -rf "${install_dir}"' EXIT

  git clone --branch "v1.4.3" --depth 1 "https://github.com/hyhmrright/brooks-lint.git" "${install_dir}/brooks-lint"
  bash "${install_dir}/brooks-lint/scripts/install.sh" claude
  bash "${install_dir}/brooks-lint/scripts/install.sh" agents
)

_install_attention_span() (
  # Output style for Claude Code; enabled via "outputStyle" in claude/settings.json.
  local version="0.6"

  mkdir -p "${HOME}/.claude/output-styles"
  curl -fsSL -o "${HOME}/.claude/output-styles/spartan.md" \
    "https://raw.githubusercontent.com/alexgreensh/attention-span/${version}/output-styles/spartan.md"
)

_install_ponytail() {
  claude plugin marketplace add "DietrichGebert/ponytail"
  claude plugin install "ponytail@ponytail" --scope user --yes

  codex plugin marketplace add "DietrichGebert/ponytail"
  codex plugin add "ponytail@ponytail"
}

main() {
  npx skills add "wkentaro/skills" -s "*" -g -a claude-code codex -y
  npx skills remove to-html coin -g -y

  npx skills add "coreyhaines31/makerskills" -s decide maker-council social-fetch watch-video -g -a claude-code codex -y

  # npx skills add "kunchenguid/vision" -s "vision" -g -a claude-code codex -y
  npx skills remove vision -g -y

  _install_brooks_lint

  _install_attention_span
  npx skills add "ayghri/i-have-adhd" -s "i-have-adhd" -g -a claude-code codex -y

  # npx skills add "dmmulroy/skills" -s "bro" -g -a claude-code codex -y
  npx skills remove bro -g -y

  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/engineering" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/productivity" -s "*" -g -a claude-code codex -y
  npx skills remove ask-matt wait-what -g -y

  npx skills add "jnsahaj/skills" -s "zero-tech-debt" -g -a claude-code codex -y

  npx skills add "pbakaus/impeccable" -s "impeccable" -g -a claude-code codex -y

  # npx skills add "OutThisLife/brooklyn-skills" -s "list-open-work" -g -a claude-code codex -y
  npx skills remove list-open-work -g -y

  npx skills add "humanlayer/skills" -s "show-me" -g -a claude-code codex -y

  _install_ponytail
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
