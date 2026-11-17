#!/usr/bin/env bash
set -Eeuo pipefail
trap 'printf "FAIL line %s: %s\n" "$LINENO" "$BASH_COMMAND"' ERR
script="$(cd "$(dirname "$0")" && pwd)/statusline-command.sh"
stub=$(mktemp -d)
trap 'rm -rf "$stub"' EXIT
mkdir "$stub/bin"
export FIXTURE="$stub/accounts.json"
cat >"$FIXTURE" <<'JSON'
[
  {"provider": "claude", "disabled": false, "windows": [{"label": "weekly", "used_percent": 100}]},
  {"provider": "claude", "disabled": false, "windows": [{"label": "weekly", "used_percent": 0}]},
  {"provider": "claude", "disabled": false, "windows": [{"label": "5-hour", "used_percent": 10}, {"label": "weekly", "used_percent": 50, "resets_at": "2999-01-01T00:00:00.437845Z"}]},
  {"provider": "claude", "disabled": false, "cached": true, "windows": [{"label": "weekly", "used_percent": 95, "resets_at": "2020-01-01T00:00:00.5Z"}], "error": "HTTP 429; retry in 2 min"},
  {"provider": "claude", "disabled": true, "windows": []},
  {"provider": "claude", "disabled": false, "windows": [{"label": "weekly", "used_percent": null}]},
  {"provider": "claude", "disabled": false, "windows": [], "error": "HTTP 429; retry in 2 min"},
  {"provider": null, "disabled": false, "windows": [], "error": "Cannot read credential JSON"},
  {"provider": "codex", "disabled": false, "windows": [{"label": "weekly", "used_percent": 17}]}
]
JSON
cat >"$stub/bin/curl" <<'SH'
#!/bin/sh
[ -z "${TEST_CURL_FAIL-}" ] || exit 7
[ "$*" = '-fsS --max-time 60 https://gateway.test:8317/quota' ] || exit 2
if [ -n "${TEST_ALL_ERRORS-}" ]; then
  printf '[{"provider": "claude", "disabled": false, "windows": [], "error": "HTTP 401"}]\n'
else
  cat "$FIXTURE"
fi
SH
chmod +x "$stub/bin/"*
export PATH="$stub/bin:$PATH"
export ANTHROPIC_BASE_URL=https://gateway.test:8317/

new_home() {
  export HOME="$stub/$1"
}

refresh() {
  printf '%s' '{"model":{"id":"gpt-6.1-sol"},"rate_limits":null}' |
    "$shell" -c '. "$1"; wait' _ "$script" >/dev/null
}

cache() {
  printf '%s' "$HOME/.cache/claude-gateway-quota.json"
}

for shell in sh bash; do
  new_home "$shell"
  refresh
  if ! jq -e '. == {"claude": 50, "codex": 17}' "$(cache)" >/dev/null 2>&1; then
    printf 'FAIL %s: quota cache was not the pool average\n' "$shell"
    exit 1
  fi
  output=$(printf '%s' '{"model":{"id":"gpt-6.1-sol"}}' | "$shell" "$script")
  [[ "$output" = *'weekly 83% left'* ]]
  output=$(printf '%s' '{"model":{"id":"claude-opus-5-5"}}' | "$shell" "$script")
  [[ "$output" = *'weekly 50% left'* ]]
  printf 'ok %s: pool average for both providers\n' "$shell"

  for failure in TEST_CURL_FAIL TEST_ALL_ERRORS; do
    export "$failure=1"
    new_home "$shell-$failure"
    refresh
    [ ! -e "$(cache)" ]
    new_home "$shell-$failure-stale"
    mkdir -p "$HOME/.cache"
    printf '{"claude":40}\n' >"$(cache)"
    touch -t 202001010000 "$(cache)"
    refresh
    [ "$(cat "$(cache)")" = '{"claude":40}' ]
    [ -n "$(find "$(cache)" -mmin +5)" ]
    unset "$failure"
    printf 'ok %s: %s keeps the cache untouched\n' "$shell" "$failure"
  done

  new_home "$shell-direct"
  printf '%s' '{"model":{"id":"claude-opus-5-5"}}' | ANTHROPIC_BASE_URL= "$shell" "$script" >/dev/null
  [ ! -e "$HOME/.cache" ]
  printf 'ok %s: no gateway URL skips the gateway fetch\n' "$shell"

  new_home "$shell-native"
  output=$(printf '%s' '{"rate_limits":{"seven_day":{"used_percentage":25}}}' | "$shell" "$script")
  [[ "$output" = *'weekly 75% left'* ]]
  [ ! -e "$HOME/.cache" ]
  printf 'ok %s: native quota bypasses gateway fetch\n' "$shell"
done
