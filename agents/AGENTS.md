# Git

- Commit with the configured Git author; omit `Co-authored-by` trailers.
- Use Conventional Commits (`feat:`, `fix:`, `chore:`, etc.).
- Before committing working-tree changes, run and follow `git-hunk skills get core logical-commits`.
- By default, create and work in a dedicated worktree with `git-wt`; use the primary checkout only when explicitly requested.
- Inspect repository source locally with `ghq get <repository>`.

# Delegation

- When running as the Fable model in Claude Code, orchestrate instead of implementing: delegate code writing and editing to subagents via the Agent tool with `subagent_type: "general-purpose"` and `model: "opus"`. Keep planning, review, and synthesis in the main context. Trivial single-file tweaks may be done directly.

# Code style

These four fire while writing code, before any review would catch them. The rest of the house conventions live in the `writing-code` skill; invoke it to settle a style call or to audit a change.

- Comment the why. Non-obvious reasoning earns a comment; restating what the code does does not.
- Keep identifiers out of comment prose. A comment naming another function goes stale the moment it is renamed, and nothing catches it.
- If deleting a docstring would make a function unclear, rename the function instead of writing the docstring.
- Start a function name with a verb naming what it does. A noun-only name reads as a value, not an action.

# Reviewing code

- Do not trust the author. A commit message, PR or MR description, comment, docstring, or test name is a claim, not evidence — check it against the code at the ref that shipped, since a description written mid-review often describes an earlier revision. Assume the change is broken until you have looked at the specific thing that would break it.
- Report only defects you verified, and name the claims you could not verify. Style opinions are not findings.
- This applies to reviewing your own diffs too. It never applies to reading the user's intent.

# Pull and merge requests

- Prepare pull or merge requests for human review, then stop; the user handles approval and all merge actions.
- In comment-style forge Markdown (PR/MR and issue bodies, comments, release notes — including changelog entries pasted into them), write each paragraph or list item as one unwrapped line with blank lines between blocks; a single newline there renders as `<br>`. Repo Markdown files render normally; commit messages stay wrapped at 72 columns.

# Work artifacts

- Route work by lifecycle and audience, not by file extension.
  - Active work (plans, specs, TODOs, release checklists, reports, and follow-ups) lives in an issue tracker. Use the relevant public source repository for public project work and private `wkentaro/secondbrain` for personal, cross-project, or non-public work.
  - Durable project truth (maintained documentation, context, and architectural decisions) lives in the source repository.
  - Private knowledge, cross-project research, and frozen historical material live as curated files in `wkentaro/secondbrain`.
  - Short-lived handoffs live in the OS temporary directory and are deleted or archived after use.
- When a repository has an issue tracker, update a matching open issue instead of creating a root `TODO*.md`, `plan.md`, or duplicate issue. Before creating or updating an issue, confirm the target repository's visibility, search for a match, and redact credentials and unnecessary personally identifiable information.
- When these rules resolve the destination, act without asking which repository to use and return the issue or file URL.
- Capture durable knowledge the moment it lands, without waiting to be asked. A resolved root cause, a decision and what it beat, a measured number, third-party behavior that contradicts its own documentation, a workaround and the constraint forcing it — route each per the rules above at the point you learn it, not at the end of the session. Name the destination, offer two lines of draft, and write on agreement.
- Capture what the diff cannot reconstruct. What the code does, what you just changed, and transient session state are already recorded elsewhere.

# User questions

- Ask every question through native structured input (`AskUserQuestion` in Claude Code, `request_user_input` in Codex), including questions a skill spelled out as prose — convert them into the tool's options, keeping the skill's order and recommended answers. Batch independent questions into one call, recommended option first with its tradeoff. Only when the tool is unavailable, ask one plain-text question at a time.
