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
cat >"$stub/bin/hostname" <<'SH'
#!/bin/sh
printf '%s\n' "$TEST_HOST"
SH
cat >"$stub/quota" <<'SH'
#!/bin/sh
printf '%s\n' "$*" >"$HOME/quota-args"
cat "$FIXTURE"
exit 1
SH
cat >"$stub/bin/ssh" <<'SH'
#!/bin/sh
printf '%s\n' "$*" >"$HOME/ssh-args"
[ "$TEST_HOST" != lora ] && [ -z "${TEST_SSH_FAIL-}" ] || exit 255
[ "$*" = '-o BatchMode=yes -o ConnectTimeout=5 lora ~/.local/bin/cliproxy-quota --json' ] || exit 1
if [ -n "${TEST_ALL_ERRORS-}" ]; then
  printf '[{"provider": "claude", "disabled": false, "windows": [], "error": "HTTP 401"}]\n'
else
  cat "$FIXTURE"
fi
exit 1
SH
chmod +x "$stub/quota" "$stub/bin/"*
export PATH="$stub/bin:$PATH"

new_home() {
  export HOME="$stub/$1"
  mkdir -p "$HOME/.local/bin"
  ln -s "$stub/quota" "$HOME/.local/bin/cliproxy-quota"
}

refresh() {
  printf '%s' '{"model":{"id":"gpt-6.1-sol"},"rate_limits":null}' |
    "$shell" -c '. "$1"; wait' _ "$script" >/dev/null
}

cache() {
  printf '%s' "$HOME/.cache/claude-gateway-quota.json"
}

for shell in sh bash; do
  for host in lora laptop; do
    export TEST_HOST="$host"
    new_home "$shell-$host"
    refresh
    if ! jq -e '. == {"claude": 50, "codex": 17}' "$(cache)" >/dev/null 2>&1; then
      printf 'FAIL %s on %s: quota cache was not the pool average\n' "$shell" "$host"
      exit 1
    fi
    if [ "$host" = lora ]; then
      [ ! -e "$HOME/ssh-args" ]
      [ "$(cat "$HOME/quota-args")" = --json ]
    else
      [ -s "$HOME/ssh-args" ]
      [ ! -e "$HOME/quota-args" ]
    fi
    output=$(printf '%s' '{"model":{"id":"gpt-6.1-sol"}}' | "$shell" "$script")
    [[ "$output" = *'weekly 83% left'* ]]
    output=$(printf '%s' '{"model":{"id":"claude-opus-5-5"}}' | "$shell" "$script")
    [[ "$output" = *'weekly 50% left'* ]]
    printf 'ok %s on %s: pool average for both providers\n' "$shell" "$host"
  done

  export TEST_HOST=laptop
  for failure in TEST_SSH_FAIL TEST_ALL_ERRORS; do
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

  new_home "$shell-native"
  output=$(printf '%s' '{"rate_limits":{"seven_day":{"used_percentage":25}}}' | "$shell" "$script")
  [[ "$output" = *'weekly 75% left'* ]]
  [ ! -e "$HOME/.cache" ]
  printf 'ok %s: native quota bypasses gateway fetch\n' "$shell"
done
