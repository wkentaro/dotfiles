# Agent instructions

## Agent skills

### Issue tracker

Issues are tracked in this repo's GitHub Issues, managed via the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Default five-role vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.

### Recurring automation

Run the four repo-management skills on systemd user timers across many repos, billed to the claude.ai subscription. Managed by `~/.local/bin/claude-skill` + a manifest. See `docs/agents/claude-automation.md`.
