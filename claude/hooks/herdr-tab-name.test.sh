#!/usr/bin/env bash
# Check herdr-tab-name.sh against a stubbed herdr: bash herdr-tab-name.test.sh
set -euo pipefail
stub=$(mktemp -d)
trap 'rm -rf "$stub"' EXIT
cat >"$stub/herdr" <<'EOF'
#!/bin/sh
case "$1" in
  pane) [ -n "$PANE" ] && echo "$PANE" || { echo boom >&2; exit 1; } ;;
  tab) echo "$TAB" ;;
esac
EOF
chmod +x "$stub/herdr"

check() { # name pane-json tab-json expected-renames
  got=$(PANE=$2 TAB=$3 PATH="$stub:$PATH" HERDR_ENV=1 HERDR_TAB_ID=w:old HERDR_PANE_ID=w:p \
    bash "$(dirname "$0")/herdr-tab-name.sh" |
    jq -r '.hookSpecificOutput.additionalContext' | grep -o 'herdr [a-z]* rename [^ ]*' | tr '\n' ' ' || true)
  [ "$got" = "$4" ] && echo "ok   $1" || { echo "FAIL $1: got '$got', want '$4'"; exit 1; }
}

unnamed_tab='{"result":{"tab":{"label":"3","number":3}}}'
pane='{"result":{"pane":{"tab_id":"w:t3"}}}'
check 'unnamed tab and pane, via the pane'\''s current tab' "$pane" "$unnamed_tab" 'herdr tab rename w:t3 herdr pane rename w:p '
check 'named pane' '{"result":{"pane":{"tab_id":"w:t3","label":"mine"}}}' "$unnamed_tab" 'herdr tab rename w:t3 '
check 'numeric custom tab name' "$pane" '{"result":{"tab":{"label":"2026","number":3}}}' 'herdr pane rename w:p '
check 'pane lookup error JSON' '{"ok":false}' "$unnamed_tab" ''
check 'pane lookup fails' '' "$unnamed_tab" ''
