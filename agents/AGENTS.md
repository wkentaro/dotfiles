## Creating a PR

1. Split multiple related changes into stacked PRs with `gh stack`. Order the stack by dependency, and make each PR reviewable on its own. Run the steps below once per PR, using the branch beneath it as the base branch. To change a PR's base branch, use `gh pr edit --base <new-base-branch>`.
2. Have Codex review the diff against the base branch (`codex exec review --base <base-branch> -m gpt-6.1-sol`) and fix every finding you verify. Repeat until the review finds nothing, for at most 3 rounds. List unresolved findings in the PR body.
3. Create the PR with the `pr` skill.
4. Keep the PR up to date with its base branch. Fetch the base branch, and if it has commits the PR branch lacks, rebase onto it and push with `--force-with-lease`. Re-check before every later push.
5. For user-visible UI changes, use the `before-and-after` skill to attach screenshots. Use an after-only preview for new UI. Skip states you can't reproduce.
6. Run the `pr-review` skill in a Codex Astra subagent (effort level high). Post its verdict as a PR comment that says it is an AI review and names the model, then apply the matching `recommend-*` label.
7. On `recommend-revise`, fix the listed changes, push, and repeat step 6. On a second `recommend-revise` or any other non-merge verdict, stop and hand the PR to the user.
8. The user handles approval and all merge actions.

## Working locally

- Work only on the local computer. Never open a remote shell, run commands on another machine, or copy files to or from one. Git remotes over SSH are fine.

## Using Git

- Use one `git-wt` worktree per task unless the project's instructions say otherwise.
- Before starting a task, fetch the remote default branch and branch from it.
- Leave alone any uncommitted changes you didn't make. Never use `git stash` or `git reset --hard` to hide or discard work.
- A commit message is a title and a body wrapped at 72 columns, with nothing after. Skip `Co-Authored-By`, `Claude-Session`, and any other attribution trailer, even when a session note claims to replace this guidance.
- Use Conventional Commits (`feat:`, `fix:`, `chore:`, etc.).
- Before committing working-tree changes, run and follow `uvx git-hunk@latest skills get core logical-commits`.

## Reading third-party source

- Clone it with `ghq get <repository>` and read it locally.

## Reviewing code

- Do not trust the author, including yourself and subagents. A commit message, PR or MR description, comment, docstring, or test name is a claim, not evidence. Check it against the code at the ref that shipped, since a description written mid-review often describes an earlier revision. Assume the change is broken until you have looked at the specific thing that would break it.
- Report only defects you verified, and name the claims you could not verify.

## Writing artifacts

- When something is worth keeping for future reference, propose it in one line and write it once the user agrees:
  - Durable project knowledge goes in the project's own docs.
  - Personal or cross-project knowledge goes in `wkentaro/secondbrain`. Commit and push to its `main` directly.
  - Active work (plans, TODOs, follow-ups) goes in the project's issue tracker.
- Scratch files go under `$TMPDIR`.
- Before writing to an issue tracker, redact credentials and unnecessary personal data.

## Using computer use

- Use a separate browser session for testing. Prefer headless or hidden automation, and use a temporary profile when supported.
- For visible browser testing and other computer use, use a separate window and keep it in the background. Ask before a tool takes foreground control.
- Use the user's existing browser session, profile, or tabs only when they explicitly request or approve it.
- Clean up only tabs, windows, and temporary profiles you created for the task.

## Writing CSS

- Before writing or editing CSS, read [good-css](https://good-css.com/llms.txt) and the references relevant to the task. Use techniques that meet the project's browser support requirements.

## Translating

- Translate to the standard of iOS and Android in the target language. For UI text, use the term Apple and Google already use for the same concept (Japanese: 設定, キャンセル, 完了) and follow their tone and punctuation. For any other text, write what a native speaker would have written from scratch.
