# Agent instructions

## Git commits

Use only the configured Git author for agent-created commits. Never add `Co-authored-by` trailers.

## Git worktrees

Use `git-wt` to create and manage Git worktrees.

## User questions

Use native structured input when available: `AskUserQuestion` in Claude Code or `request_user_input` in Codex. Batch independent questions in one call, up to the tool's limit, while preserving order and dependencies. Put the recommended choice first and explain its tradeoff. Structured input takes precedence over skill-specific prose formats; without it, ask one concise plain-text question at a time.
