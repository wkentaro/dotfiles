# Agent instructions

## Git commits

Use only the configured Git author for agent-created commits. Never add `Co-authored-by` trailers.

## Git worktrees

Use `git-wt` to create and manage Git worktrees.

## User questions

- When asking the user one or more questions, use the native structured-input UI whenever it is available: `AskUserQuestion` in Claude Code and `request_user_input` in Codex.
- Put independent questions into one tool call, up to the tool's supported limit. Let the UI present them instead of printing them as ordinary text.
- Do not print a batch of numbered questions as prose. This presentation rule overrides skill-specific prose formats for user questions. Preserve the skill's question order, dependencies, and recommended answers.
- Put the recommended choice first and explain its tradeoff in the option description.
- If the structured-input tool is unavailable, ask one concise plain-text question and wait before asking another.
