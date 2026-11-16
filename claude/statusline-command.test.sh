#!/usr/bin/env bash
set -euo pipefail
script="$(cd "$(dirname "$0")" && pwd)/statusline-command.sh"
stub=$(mktemp -d)
trap 'rm -rf "$stub"' EXIT
mkdir "$stub/bin"
cat >"$stub/bin/hostname" <<'EOF'
#!/bin/sh
printf '%s\n' "$TEST_HOST"
EOF
cat >"$stub/bin/python3" <<'EOF'
#!/bin/sh
[ "$*" = '-' ] || exit 1
cat >/dev/null
printf '{"claude":99,"codex":83}\n'
EOF
cat >"$stub/bin/ssh" <<'EOF'
#!/bin/sh
printf '%s\n' "$*" >"$HOME/ssh-args"
[ "$TEST_HOST" != lora ] || exit 255
[ "$*" = '-o BatchMode=yes -o ConnectTimeout=5 lora python3 -' ] || exit 1
exec python3 -
EOF
chmod +x "$stub/bin/"*
export PATH="$stub/bin:$PATH"

for shell in sh bash; do
  for host in lora laptop; do
    export TEST_HOST="$host" HOME="$stub/$shell-$host"
    mkdir "$HOME"
    payload='{"model":{"id":"gpt-6.1-sol"},"rate_limits":null}'
    printf '%s' "$payload" | "$shell" -c '. "$1"; wait' _ "$script" >/dev/null
    if ! jq -e '.codex == 83 and .claude == 99' "$HOME/.cache/claude-gateway-quota.json" >/dev/null 2>&1; then
      printf 'FAIL %s on %s: quota cache was not populated\n' "$shell" "$host"
      exit 1
    fi
    if [ "$host" = lora ]; then
      [ ! -e "$HOME/ssh-args" ]
    else
      [ -s "$HOME/ssh-args" ]
    fi
    output=$(printf '%s' "$payload" | "$shell" "$script")
    [[ "$output" = *'weekly 17% left'* ]]
    output=$(printf '%s' '{"model":{"id":"claude-opus-5-5"}}' | "$shell" "$script")
    [[ "$output" = *'weekly 1% left'* ]]
    printf 'ok %s on %s: refresh and both provider quotas\n' "$shell" "$host"
  done
  export HOME="$stub/$shell-native"
  mkdir "$HOME"
  output=$(printf '%s' '{"rate_limits":{"seven_day":{"used_percentage":25}}}' | "$shell" "$script")
  [[ "$output" = *'weekly 75% left'* ]]
  [ ! -e "$HOME/.cache" ]
  printf 'ok %s: native quota bypasses gateway fetch\n' "$shell"
done
