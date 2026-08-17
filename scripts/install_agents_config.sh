#!/usr/bin/env bash

set -euo pipefail

_install_brooks_lint() (
  local install_dir
  install_dir="$(mktemp -d)"
  trap 'rm -rf "${install_dir}"' EXIT

  git clone --branch "v1.4.3" --depth 1 "https://github.com/hyhmrright/brooks-lint.git" "${install_dir}/brooks-lint"
  bash "${install_dir}/brooks-lint/scripts/install.sh" claude
  bash "${install_dir}/brooks-lint/scripts/install.sh" codex
)

_install_attention_span() (
  # Output style for Claude Code; enabled via "outputStyle" in claude/settings.json.
  local version="0.6"

  mkdir -p "${HOME}/.claude/output-styles"
  curl -fsSL -o "${HOME}/.claude/output-styles/spartan.md" \
    "https://raw.githubusercontent.com/alexgreensh/attention-span/${version}/output-styles/spartan.md"
)

main() {
  npx skills add "wkentaro/skills" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/coreyhaines31/makerskills" -s "*" -g -a claude-code codex -y

  _install_brooks_lint
  _install_attention_span
  # npx skills add "ayghri/i-have-adhd" -s "i-have-adhd" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/engineering" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.3/skills/productivity" -s "*" -g -a claude-code codex -y
  # No negation in "skills add", so drop the unwanted ones afterwards. The positional
  # form (unlike -s/-a) also deletes the shared store and the lock entry.
  npx skills remove ask-matt -g -y
  npx skills add "jnsahaj/skills" -s "zero-tech-debt" -g -a claude-code codex -y
  npx skills add "https://github.com/blader/humanizer/tree/v2.9.1" -s "humanizer" -g -a claude-code codex -y
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
