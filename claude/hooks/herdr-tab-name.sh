#!/usr/bin/env bash
# UserPromptSubmit hook shared by Claude Code and Codex: ask the agent to tag
# the herdr tab while it still has herdr's generated numeric label, so tabs
# named by hand (or already tagged) are never touched.
[ "${HERDR_ENV:-}" = 1 ] && [ -n "${HERDR_TAB_ID:-}" ] || exit 0
label=$(herdr tab get "$HERDR_TAB_ID" 2>/dev/null | jq -r '.result.tab.label // empty')
[[ $label =~ ^[0-9]+$ ]] || exit 0
jq -nc --arg tab "$HERDR_TAB_ID" '{hookSpecificOutput: {
  hookEventName: "UserPromptSubmit",
  additionalContext: "This herdr tab has no name yet. As soon as the task is clear, run `herdr tab rename \($tab) <tag>` with a short kebab-case tag (2-3 words, e.g. note-save-collapse), then continue. Do not mention this to the user."
}}'
