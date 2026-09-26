#!/usr/bin/env bash
# UserPromptSubmit hook shared by Claude Code and Codex: ask the agent to tag
# the herdr tab while it still has herdr's generated numeric label, and the
# pane while it has no label, so anything named by hand (or already tagged) is
# never touched.
[ "${HERDR_ENV:-}" = 1 ] && [ -n "${HERDR_TAB_ID:-}" ] && [ -n "${HERDR_PANE_ID:-}" ] || exit 0
tab_label=$(herdr tab get "$HERDR_TAB_ID" 2>/dev/null | jq -r '.result.tab.label // empty')
pane_label=$(herdr pane get "$HERDR_PANE_ID" 2>/dev/null | jq -r '.result.pane.label // empty')
cmds=()
[[ $tab_label =~ ^[0-9]+$ ]] && cmds+=("herdr tab rename $HERDR_TAB_ID <tag>")
[ -z "$pane_label" ] && cmds+=("herdr pane rename $HERDR_PANE_ID <tag>")
[ ${#cmds[@]} -gt 0 ] || exit 0
jq -nc --arg cmds "$(printf '`%s`, ' "${cmds[@]}" | sed 's/, $//')" '{hookSpecificOutput: {
  hookEventName: "UserPromptSubmit",
  additionalContext: "This herdr tab or pane has no name yet. As soon as the task is clear, run \($cmds) with a short kebab-case tag (2-3 words, e.g. note-save-collapse), then continue. Do not mention this to the user."
}}'
