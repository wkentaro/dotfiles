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

_install_marketing() {  # bundle coreyhaines31/marketingskills into one skill
  local tmp out name desc
  tmp=$(mktemp -d)
  git clone -q --depth 1 "https://github.com/coreyhaines31/marketingskills" "$tmp/src"
  out="$tmp/marketing"
  mkdir -p "$out/playbooks"
  cp -r "$tmp/src/tools" "$out/"  # playbooks link ../../tools/...
  for dir in "$tmp/src/skills"/*/; do
    name=$(basename "$dir")
    cp -r "$dir" "$out/playbooks/$name"
    rm -rf "$out/playbooks/$name/evals"
    mv "$out/playbooks/$name/SKILL.md" "$out/playbooks/$name/PLAYBOOK.md"
  done
  {
    cat <<'MD'
---
name: marketing
description: "Marketing playbooks for a product: positioning and product context, landing page and pricing page conversion, copywriting and copy editing, pricing, SEO and AI search, launches, emails, analytics and A/B tests, competitors, customer research, ads, social, and growth ideas."
---

Each playbook is a former standalone skill in `playbooks/<name>/PLAYBOOK.md`. Its links resolve relative to its own folder.

1. Read `.agents/product-marketing.md` in the project root if it exists. If it is missing and the task depends on the product, audience, or positioning, run the `product-marketing` playbook first to create it.
2. Pick the playbook from the index below that matches the task. Read its `PLAYBOOK.md` in full, then follow it. When a playbook says "see X" or names the X skill, that is `playbooks/X/PLAYBOOK.md`.

## Index

MD
    for f in "$out"/playbooks/*/PLAYBOOK.md; do
      name=$(basename "$(dirname "$f")")
      desc=$(awk '/^description:/{sub(/^description: *"?/,""); print; exit}' "$f" | sed -E 's/^When the user wants (help )?(with |to )?//; s/\. .*//; s/\.?"?$//')
      echo "- \`$name\`: $desc"
    done
  } > "$out/SKILL.md"
  npx skills remove $(ls "$tmp/src/skills") -g -y  # drop the unbundled copies
  npx skills add "$out" -g -a claude-code codex -y --copy
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

  npx skills add "emilkowalski/skills" -s animate animation-performance apple-design break-ui find-animation-opportunities improve-animations mobile-native remove-animations review-animations -g -a claude-code codex -y

  _install_marketing

  npx skills add benjitaylor/skills -s stageshow -g -a claude-code codex -y

  npx skills add shadcn/improve -g -a claude-code codex -y
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
