#!/usr/bin/env bash
# UserPromptSubmit hook shared by Claude Code and Codex: ask the agent to tag
# the herdr tab while its label is still herdr's generated tab number, and the
# pane while it has no label, so anything named by hand (or already tagged) is
# never touched. The pane is looked up first because HERDR_TAB_ID goes stale
# when the pane moves to another tab. A failed lookup renames nothing.
[ "${HERDR_ENV:-}" = 1 ] && [ -n "${HERDR_PANE_ID:-}" ] || exit 0
pane=$(herdr pane get "$HERDR_PANE_ID" 2>/dev/null | jq -ec '.result.pane // error' 2>/dev/null) || exit 0
tab_id=$(jq -r '.tab_id // empty' <<<"$pane")
cmds=()
[ -n "$tab_id" ] && herdr tab get "$tab_id" 2>/dev/null |
  jq -e '.result.tab | .label == (.number | tostring)' >/dev/null 2>&1 &&
  cmds+=("herdr tab rename $tab_id <tag>")
jq -e '(.label // "") == ""' <<<"$pane" >/dev/null && cmds+=("herdr pane rename $HERDR_PANE_ID <tag>")
[ ${#cmds[@]} -gt 0 ] || exit 0
jq -nc --arg cmds "$(printf '`%s`, ' "${cmds[@]}" | sed 's/, $//')" '{hookSpecificOutput: {
  hookEventName: "UserPromptSubmit",
  additionalContext: "This herdr tab or pane has no name yet. As soon as the task is clear, run \($cmds) with a short kebab-case tag (2-3 words, e.g. note-save-collapse), then continue. Do not mention this to the user."
}}'
