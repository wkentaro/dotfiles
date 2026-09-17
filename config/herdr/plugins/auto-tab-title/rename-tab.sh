#!/bin/sh

set -eu

status=$(printf '%s' "$HERDR_PLUGIN_EVENT_JSON" | jq -r '.data.agent_status // empty')
[ "$status" = working ] || exit 0

pane=$("$HERDR_BIN_PATH" pane get "$HERDR_PANE_ID")
tab_id=$(printf '%s' "$pane" | jq -r '.result.pane.tab_id // empty')
title=$(printf '%s' "$pane" | jq -r '.result.pane.terminal_title_stripped // empty')
case "$title" in
  renaming...*)
    # The final title arrives shortly after the working-state event.
    sleep 1
    pane=$("$HERDR_BIN_PATH" pane get "$HERDR_PANE_ID")
    title=$(printf '%s' "$pane" | jq -r '.result.pane.terminal_title_stripped // empty')
    ;;
esac
[ -n "$tab_id" ] && [ -n "$title" ] || exit 0

label=$("$HERDR_BIN_PATH" tab get "$tab_id" | jq -r '.result.tab.label // empty')
case "$label" in
  ''|*[!0-9]*) exit 0 ;;
esac

"$HERDR_BIN_PATH" tab rename "$tab_id" "$title" >/dev/null
