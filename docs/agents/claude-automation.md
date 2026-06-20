# Recurring Claude Code automation (systemd)

Runs four Claude Code skills on a schedule across many GitHub repos, billed to
the **claude.ai subscription** (not the metered API), executing **locally** via
systemd user timers. Each tick is a fresh `claude -p` session, so context never
accumulates the way `/loop` does.

The four skills (the autonomous repo-management pipeline):

| Skill | Cadence | Gate (skips a tick cheaply when there's nothing to do) |
|---|---|---|
| `process-issues`   | 20min | no open issues |
| `implement-issues` | 30min | no open `ready-for-agent` issues |
| `process-prs`      | 20min | no open PRs |
| `kaizen-codebase`  | 1d    | none (self-directed; manufactures its own work) |

## How it fits together

```
config/claude-skill/repos        manifest: which repos x skills, and each repo's mode
        |  claude-skill apply     reconciles enabled timers to the manifest
        v
claude-<skill>@<repo>.timer  --> claude-skill@<skill>:<repo>.service
                                       |  ExecStart
                                       v
                                 claude-skill run <skill>:<repo>
                                   1. resolve repo via `ghq`
                                   2. flock -n per repo (skip if another skill holds it)
                                   3. per-repo permission mode (bypass | allowlist)
                                   4. gate: skip without a session if nothing to do
                                   5. exec claude -p "/<skill>"
```

- **One generic runner** (`~/.local/bin/claude-skill`) does every (skill, repo).
- **One service template** + **four timer templates**, instanced by repo. Adding
  repos or skills creates **no new files**.
- **Per-repo flock**: only one skill touches a given checkout at a time; the loser
  skips its tick and retries on its next fire (the pipeline is eventually
  consistent by design).

## The manifest

`~/.config/claude-skill/repos` (tracked at `config/claude-skill/repos`) is the
single source of truth:

```
# <repo>  <mode>  <skills>
octomap-python  bypass  process-issues,implement-issues,process-prs,kaizen-codebase
```

- `mode = bypass` runs `--dangerously-skip-permissions`. Fine for your own repos.
- `mode = allowlist` is **reserved, not yet wired** — it will error. Build the
  curated `settings.json` (Edit/Write + `Bash(gh|git|uv|pytest|ruff *)`,
  `--permission-mode acceptEdits`) before pointing a repo at it. Use it for repos
  that take **outside contributions**, where `process-prs`/`process-issues` read
  untrusted external text and full bypass + your GitHub creds is a
  prompt-injection risk.

## Onboarding a repo

```bash
$EDITOR ~/.config/claude-skill/repos   # add a line
claude-skill apply                     # clone if missing, enable timers, prune removed
```

`apply` also enables linger (`loginctl enable-linger`) so timers run while you're
logged out, and warns if subscription creds or headless `gh` auth are missing.

Drop `kaizen-codebase` from a repo's skill list to run only the issue->PR
pipeline. `kaizen` is the most likely to "go crazy" (it invents work) — keep it
daily and add it last when scaling.

## Pausing

- **One pair / one repo**: remove it from the manifest, `claude-skill apply`.
- **Global kill switch** (manifest untouched): `claude-skill pause` stops every
  timer; in-flight runs finish. `claude-skill resume` restarts them.

## Debugging one tick

```bash
claude-skill debug process-issues:octomap-python
```

Runs the exact tick in your terminal, streaming (`--verbose`, not JSON), no
systemd, no turn ceiling. Prints the resolved repo, the mode, and the gate
decision (and runs anyway so you can exercise the skill even when the gate would
skip).

Logs from real ticks:

```bash
claude-skill list                                          # next fire times
journalctl --user -u 'claude-skill@process-prs:octomap-python.service'
systemctl --user list-units --failed 'claude-*'            # ticks that errored
```

## Prerequisites (one-time, headless)

- **Subscription login**: run `claude` once so `~/.claude/.credentials.json` exists.
- **Headless `gh` auth**: a systemd service does not inherit your shell's
  `GITHUB_TOKEN`. Persist credentials to `~/.config/gh/hosts.yml` once:
  `unset GITHUB_TOKEN && gh auth login`. Your interactive shell keeps using the
  env var; headless ticks fall back to the stored creds.

## Caution: the self-feeding pipeline

`process-issues -> implement-issues -> process-prs`, plus `kaizen` adds work, all
on one subscription quota with rate limits. Start with one repo + the three
pipeline skills, watch a day, then add `kaizen`, then scale to more repos.
