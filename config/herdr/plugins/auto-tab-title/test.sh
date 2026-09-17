#!/bin/sh

set -eu

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/herdr-auto-title.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT HUP INT TERM

fake_herdr="$tmp_dir/herdr"
log="$tmp_dir/rename.log"
pane_calls="$tmp_dir/pane-calls"
printf '0\n' >"$pane_calls"
# shellcheck disable=SC2016
printf '%s\n' '#!/bin/sh' \
  'case "$1 $2" in' \
  '  "pane get") calls=$(cat "$HERDR_TEST_PANE_CALLS"); calls=$((calls + 1)); printf "%s\\n" "$calls" >"$HERDR_TEST_PANE_CALLS"; if [ "$calls" -eq 1 ]; then title="renaming... ⠸ | labelme-io"; else title="Review PR 42"; fi; printf '\''{"result":{"pane":{"tab_id":"w1:t2","terminal_title_stripped":"%s"}}}'\'' "$title" ;;' \
  '  "tab get") printf '\''{"result":{"tab":{"label":"%s"}}}'\'' "$HERDR_TEST_TAB_LABEL" ;;' \
  '  "tab rename") printf '\''%s\n'\'' "$*" >>"$HERDR_TEST_LOG" ;;' \
  'esac' >"$fake_herdr"
chmod +x "$fake_herdr"

plugin_dir=$(cd "$(dirname "$0")" && pwd)
event='{"data":{"agent_status":"working"}}'
HERDR_BIN_PATH="$fake_herdr" HERDR_PANE_ID=w1:p2 HERDR_PLUGIN_EVENT_JSON="$event" \
  HERDR_TEST_TAB_LABEL=2 HERDR_TEST_LOG="$log" HERDR_TEST_PANE_CALLS="$pane_calls" sh "$plugin_dir/rename-tab.sh"
[ "$(cat "$log")" = 'tab rename w1:t2 Review PR 42' ]

: >"$log"
HERDR_BIN_PATH="$fake_herdr" HERDR_PANE_ID=w1:p2 HERDR_PLUGIN_EVENT_JSON="$event" \
  HERDR_TEST_TAB_LABEL=manual HERDR_TEST_LOG="$log" HERDR_TEST_PANE_CALLS="$pane_calls" sh "$plugin_dir/rename-tab.sh"
[ ! -s "$log" ]
