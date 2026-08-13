# Git

- Commit with the configured Git author; omit `Co-authored-by` trailers.
- Use Conventional Commits (`feat:`, `fix:`, `chore:`, etc.).
- Before committing working-tree changes, run and follow `git-hunk skills get core logical-commits`.
- Manage worktrees with `git-wt`.
- Inspect repository source locally with `ghq get <repository>`.

# Pull and merge requests

- Prepare pull or merge requests for human review, then stop; the user handles approval and all merge actions.
- In comment-style forge Markdown (PR/MR and issue bodies, comments, release notes — including changelog entries pasted into them), write each paragraph or list item as one unwrapped line with blank lines between blocks; a single newline there renders as `<br>`. Repo Markdown files render normally; commit messages stay wrapped at 72 columns.

# User questions

- Ask every question through native structured input (`AskUserQuestion` in Claude Code, `request_user_input` in Codex), including questions a skill spelled out as prose — convert them into the tool's options, keeping the skill's order and recommended answers. Batch independent questions into one call, recommended option first with its tradeoff. Only when the tool is unavailable, ask one plain-text question at a time.
