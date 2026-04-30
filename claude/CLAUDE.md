# Global Claude Code Preferences

## Guardrails
- When you change a code, make sure you've read that code. Change shouldn't be made without reading and understanding.
- When the user asks "why did you do X?" or "what about Y?" — they are asking genuinely, not telling you to change. Explain your reasoning, defend it if you believe it's right, and only change course when actually convinced. Don't reflexively revert decisions at the first question.
- When you make changes, know that your changes will be reviewed by your competitor Codex or Claude Code.
- When you write an English sentence, don't use emdash. When you notice you wrote them, fix them.
- When doing rewrite for relicensing, don't disclose that intent in the public PR title, branch name, and description.
- When you create a PR, keep description concise.

## Code Style

- Avoid unnecessary comments. Code should be self-explanatory. Only add comments when the logic is truly non-obvious.
- When you commit, don't put Co-authored-by Claude.
- When you switch branches, use `git switch` instead of `git checkout`.
- When you use python, use `uv`.
- When you merge a pull request, ensure the branch is up-to-date with main (rebase if needed), and then merge with merge commit.

## Workflow Orchestration

### 1. Plan Mode Default
- Enter plan mode for ANY non-trivial task (3+ steps or architectural decisions)
- If something goes sideways, STOP and re-plan immediately – don't keep pushing
- Use plan mode for verification steps, not just building
- Write detailed specs upfront to reduce ambiguity

### 2. Subagent Strategy
- Use subagents liberally to keep main context window clean
- Offload research, exploration, and parallel analysis to subagents
- For complex problems, throw more compute at it via subagents
- One task per subagent for focused execution

### 3. Self-Improvement Loop
- After ANY correction from the user: update `tasks/lessons.md` with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these lessons until mistake rate drops
- Review lessons at session start for relevant project

### 4. Verification Before Done
- Never mark a task complete without proving it works
- Diff behavior between main and your changes when relevant
- Ask yourself: "Would a staff engineer approve this?"
- Run tests, check logs, demonstrate correctness

### 5. Demand Elegance (Balanced)
- For non-trivial changes: pause and ask "is there a more elegant way?"
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes – don't over-engineer
- Challenge your own work before presenting it

### 6. Autonomous Bug Fixing
- When given a bug report: just fix it. Don't ask for hand-holding
- Point at logs, errors, failing tests – then resolve them
- Zero context switching required from the user
- Go fix failing CI tests without being told how

## Task Management

1. **Plan First**: Write plan to `tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `tasks/todo.md`
6. **Capture Lessons**: Update `tasks/lessons.md` after corrections

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.
