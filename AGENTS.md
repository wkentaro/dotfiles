# Agent instructions

## Layout

A file's path in this repo is its install path under `~` with the leading dot dropped, every directory kept: `~/.pi/agent/mcp.json` lives at `pi/agent/mcp.json`.

## Agent skills

### Issue tracker

Issues are tracked in this repo's GitHub Issues, managed via the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Issue type, issue triage, and PR verdict labels are mapped in `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.
