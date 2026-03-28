# /make-pr — Create or Update a Pull Request

Push the current branch and create (or update) a GitHub PR with title, description, labels, and assignee.

## Workflow

### 1. Gather context

Run these in parallel:

```bash
git log main..HEAD --oneline
git diff main...HEAD --stat
gh label list --limit 50 --json name --jq '.[].name'
git config user.name
```

Also check if a PR already exists for this branch:

```bash
gh pr view --json number,url 2>/dev/null
```

### 2. Draft PR content

From the commit history and diff stats, draft:

- **Title**: `<type>: <short description>` (under 70 chars). Use the overall theme, not just the last commit.
- **Body**: Use this format:

```markdown
## Summary
- Bullet points summarizing the changes

## Why
Brief motivation for the change.

## Test plan
- [x] Completed checks
- [ ] Manual verification steps
```

- **Labels**: Pick from the available labels based on the change type.
- **Assignee**: Default to the git user (`git config user.name` or the GitHub username from `gh api user --jq '.login'`).

### 3. Push and create/update

```bash
# Push with tracking
git push -u origin <branch> --force-with-lease

# Create or update
gh pr create --title "..." --body "..." --label "..." --assignee "..."
# If PR already exists:
gh pr edit <number> --title "..." --body "..." --add-label "..." --add-assignee "..."
```

### 4. Return the PR URL

Always print the PR URL at the end so the user can click it.

## Guidelines

- Always use `--force-with-lease` (not `--force`) when pushing
- Check if a PR already exists before creating — update it instead of failing
- Default assignee to the repo owner / current user
- Use conventional commit style for the title
- Keep the summary focused on *what changed and why*, not implementation details
- Include test results in the test plan if tests were run during the session
