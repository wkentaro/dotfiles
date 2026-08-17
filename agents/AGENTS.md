# Git

- Commit with the configured Git author; omit `Co-authored-by` trailers.
- Use Conventional Commits (`feat:`, `fix:`, `chore:`, etc.).
- Before committing working-tree changes, run and follow `git-hunk skills get core logical-commits`.
- Manage worktrees with `git-wt`.
- Inspect repository source locally with `ghq get <repository>`.

# Code style

These four fire while writing code, before any review would catch them. The rest of the house conventions live in the `writing-code` skill; invoke it to settle a style call or to audit a change.

- Comment the why. Non-obvious reasoning earns a comment; restating what the code does does not.
- Keep identifiers out of comment prose. A comment naming another function goes stale the moment it is renamed, and nothing catches it.
- If deleting a docstring would make a function unclear, rename the function instead of writing the docstring.
- Start a function name with a verb naming what it does. A noun-only name reads as a value, not an action.

# Pull and merge requests

- Prepare pull or merge requests for human review, then stop; the user handles approval and all merge actions.
- In comment-style forge Markdown (PR/MR and issue bodies, comments, release notes — including changelog entries pasted into them), write each paragraph or list item as one unwrapped line with blank lines between blocks; a single newline there renders as `<br>`. Repo Markdown files render normally; commit messages stay wrapped at 72 columns.

# Reading X posts

- Read an X/Twitter post with `curl -s "https://cdn.syndication.twimg.com/tweet-result?id=<POST_ID>&token=a"`, taking `<POST_ID>` from the status URL; WebFetch on `x.com` or `xcancel.com` hits a login or captcha wall. Single posts only, so a thread needs every link.

# User questions

- Ask every question through native structured input (`AskUserQuestion` in Claude Code, `request_user_input` in Codex), including questions a skill spelled out as prose — convert them into the tool's options, keeping the skill's order and recommended answers. Batch independent questions into one call, recommended option first with its tradeoff. Only when the tool is unavailable, ask one plain-text question at a time.
