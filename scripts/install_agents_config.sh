#!/usr/bin/env bash

set -euo pipefail

main() {
  npx skills add "wkentaro/skills" -s "*" -g -a claude-code codex -y

  npx skills add "ayghri/i-have-adhd" -s "i-have-adhd" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.2/skills/engineering" -s "*" -g -a claude-code codex -y
  npx skills add "https://github.com/mattpocock/skills/tree/v1.2.2/skills/productivity" -s "*" -g -a claude-code codex -y
  npx skills add "jnsahaj/skills" -s "zero-tech-debt" -g -a claude-code codex -y
  npx skills add "https://github.com/blader/humanizer/tree/v2.9.1" -s "humanizer" -g -a claude-code codex -y
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
