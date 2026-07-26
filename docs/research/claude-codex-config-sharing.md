# Sharing Claude Code and Codex configuration from dotfiles

Research date: 2026-07-26

## Recommendation

Use one canonical, vendor-neutral instruction file and one canonical skill tree, then expose them at each tool's documented discovery path. Keep each product's settings separate because their schemas, precedence rules, and state files are not interchangeable.

For this repository, a practical target layout is:

```text
dotfiles/
├── AGENTS.md                    # this repo's project instructions
├── CLAUDE.md -> AGENTS.md       # Claude project adapter (already present)
├── agents/
│   ├── AGENTS.md                # canonical personal instructions
│   └── skills/                  # canonical shared Agent Skills
├── claude/
│   ├── CLAUDE.md -> ../agents/AGENTS.md
│   ├── skills -> ../agents/skills
│   ├── rules/                   # Claude-only behavioral rules
│   └── settings.json            # Claude-only shareable settings
└── codex/
    ├── AGENTS.md -> ../agents/AGENTS.md
    ├── config.toml              # Codex-only shareable settings
    └── rules/                   # Codex command-approval .rules files only
```

With the repository's directories linked into the home directory, the effective paths should be:

```text
~/.claude/CLAUDE.md -> dotfiles/claude/CLAUDE.md
~/.claude/skills    -> ~/.agents/skills
~/.codex/AGENTS.md  -> dotfiles/codex/AGENTS.md
~/.agents/skills    -> dotfiles/agents/skills
```

This gives Claude `~/.claude/CLAUDE.md` and `~/.claude/skills`, while Codex gets `~/.codex/AGENTS.md` and `~/.agents/skills`. Codex's current official personal-skill location is `$HOME/.agents/skills`, not `$HOME/.codex/skills`; it also explicitly supports symlinked skill folders. [OpenAI: Build skills](https://developers.openai.com/codex/skills)

Link only allowlisted files and subdirectories. Do not link the whole product directories because they mix versioned configuration with runtime state.

## Instructions: `CLAUDE.md` versus `AGENTS.md`

Claude Code reads `CLAUDE.md`, not `AGENTS.md`. Anthropic explicitly recommends either a small `CLAUDE.md` containing `@AGENTS.md` or a `CLAUDE.md -> AGENTS.md` symlink when a repository already uses `AGENTS.md`. The import form is better when Claude-specific additions are required; the symlink is better when the content should remain identical. [Anthropic: How Claude remembers your project](https://code.claude.com/docs/en/memory)

Claude's documented scopes are:

- managed policy: the platform-specific system `CLAUDE.md`;
- user: `~/.claude/CLAUDE.md`;
- project: `./CLAUDE.md` or `./.claude/CLAUDE.md`;
- private project override: `./CLAUDE.local.md`, normally gitignored.

Claude walks from the filesystem root down to the current working directory and concatenates discovered instructions; closer files appear later. Within a directory, `CLAUDE.local.md` follows `CLAUDE.md`. Nested files below the working directory load on demand when Claude reads there. [Anthropic: How Claude remembers your project](https://code.claude.com/docs/en/memory)

Codex natively reads `AGENTS.md`. At global scope it reads the first non-empty of `$CODEX_HOME/AGENTS.override.md` and `$CODEX_HOME/AGENTS.md` (`CODEX_HOME` defaults to `~/.codex`). At project scope it walks from the project root to the current directory and, in each directory, selects at most one of `AGENTS.override.md`, `AGENTS.md`, or a configured fallback filename. Later, closer instructions override earlier ones. The combined project-document limit defaults to 32 KiB. [OpenAI: Custom instructions with AGENTS.md](https://developers.openai.com/codex/guides/agents-md)

Therefore:

- Keep shared behavioral guidance in `AGENTS.md`.
- Expose that file to Claude with `CLAUDE.md` adapters.
- Put vendor-only guidance after `@AGENTS.md` in a real `CLAUDE.md`, or in a separate product-specific global file, rather than contaminating the common core.
- Do not expect a file named `Agents.md` or an `Agents` directory to work. Both filenames and paths are case-sensitive conventions: `AGENTS.md`, `.agents/skills`, and `.claude/skills`.

## Imports, includes, and modular rules

Claude supports `@path/to/file` imports in `CLAUDE.md`. Relative imports resolve from the importing file, and absolute paths are also supported. Imports improve organization but do not save context because imported content is expanded at startup. Anthropic recommends concise, concrete instruction files, targeting fewer than 200 lines, and path-scoped rules for larger projects. [Anthropic: How Claude remembers your project](https://code.claude.com/docs/en/memory)

Claude's `.claude/rules/**/*.md` files are behavioral instructions. Files without `paths` frontmatter load unconditionally; files with `paths` load when Claude works with matching files. This is a Claude feature and Codex does not interpret these Markdown rule files. [Anthropic: How Claude remembers your project](https://code.claude.com/docs/en/memory)

Codex's directory named `rules/` has a different purpose. Files such as `~/.codex/rules/default.rules` contain experimental `prefix_rule(...)` command policies controlling whether matching commands may run outside the sandbox. They are not Markdown prompt instructions and are not a substitute for `AGENTS.md`. Project rules live under `<repo>/.codex/rules/` and load only for trusted projects. [OpenAI: Rules](https://developers.openai.com/codex/rules)

The official Codex `AGENTS.md` documentation does not document an `@include` facility. Do not depend on Claude's `@...` syntax inside the canonical shared file: Claude would expand it but Codex would see it as plain text. Prefer nested `AGENTS.md` files for Codex scoping, a concise common file, or generated adapters whose output is checked for drift.

## Skills

Both products use the Agent Skills format: a directory containing `SKILL.md` plus optional scripts, references, and assets. Skill bodies load only when used, making skills the right place for reusable procedures rather than always-on facts. [Anthropic: Extend Claude with skills](https://code.claude.com/docs/en/skills), [OpenAI: Build skills](https://developers.openai.com/codex/skills)

Claude discovers personal skills at `~/.claude/skills/<name>/SKILL.md` and project skills at `.claude/skills/<name>/SKILL.md`. Codex discovers personal skills at `$HOME/.agents/skills`, and repository skills from `.agents/skills` along the path from the working directory to the repository root. [Anthropic: Extend Claude with skills](https://code.claude.com/docs/en/skills), [OpenAI: Build skills](https://developers.openai.com/codex/skills)

The portable pattern is therefore:

1. Author each portable workflow once under `agents/skills/<name>/SKILL.md`.
2. Expose that tree as `~/.agents/skills` for Codex.
3. Expose the same tree as `~/.claude/skills` for Claude.
4. Keep tool-specific extensions optional and documented. Claude supports extra frontmatter and dynamic context features that another Agent Skills host may ignore or reject.

For repository-scoped shared skills in other projects, either commit parallel discovery symlinks (`.agents/skills` and `.claude/skills`) to one canonical tree, or package reusable workflows as a plugin when distribution rather than dotfile authoring becomes the goal. OpenAI recommends plugins for reusable distribution and direct skill folders for local or repo-scoped authoring. [OpenAI: Build skills](https://developers.openai.com/codex/skills)

## Settings and precedence

Claude's supported configuration mechanism is hierarchical `settings.json`. User settings live at `~/.claude/settings.json`; project settings at `.claude/settings.json`; private project settings at `.claude/settings.local.json`, which Claude configures Git to ignore. Other data in `~/.claude.json` includes the OAuth session, MCP configuration, per-project trust and allowed-tool state, preferences, and caches. Do not version or symlink that state file from a public dotfiles repository. [Anthropic: Claude Code settings](https://code.claude.com/docs/en/configuration)

Codex user configuration lives at `~/.codex/config.toml`, with trusted project overrides in `.codex/config.toml`. Its documented precedence, highest first, is CLI overrides, project configs from root to current directory, selected profile, user config, system config, then built-ins. Untrusted projects do not load project `.codex` config, hooks, or rules. [OpenAI: Config basics](https://developers.openai.com/codex/config-basic)

Keep `claude/settings.json` and `codex/config.toml` separate. Share intent rather than trying to share syntax: for example, maintain equivalent sandbox or notification choices in the two native formats and explain intentional differences in comments or documentation.

## Secrets and machine-local state

Version only stable, reviewable inputs:

- instruction Markdown;
- portable skills and their non-secret assets/scripts;
- settings that contain no credentials, hostnames, private paths, account identifiers, or tokens;
- conservative command-policy rules.

Keep these machine-local or secret:

- Claude's `~/.claude.json`, auto-memory under `~/.claude/projects/.../memory/`, local settings, OAuth/session data, caches, logs, and transcripts;
- Codex credential storage (`auth.json` when file storage is selected), `history.jsonl`, logs, sessions, caches, and other generated state;
- API keys, MCP OAuth tokens, bearer headers, cookies, private endpoints, and workstation-specific absolute paths.

Anthropic documents auto memory as machine-local and shared across worktrees of the same repository, not as team configuration. [Anthropic: How Claude remembers your project](https://code.claude.com/docs/en/memory) OpenAI documents `cli_auth_credentials_store` as choosing between file-based `auth.json` and the OS keychain, and `history.persistence` as controlling transcript storage in `history.jsonl`; prefer the OS keychain where available and never commit either file. [OpenAI: Configuration reference](https://developers.openai.com/codex/config-reference)

Use ignore rules as defense in depth, but do not rely on them as the primary boundary. The primary boundary should be an allowlist-based link/install script that links only known configuration paths rather than the entire mutable `~/.claude` or `~/.codex` directory.

## Suggested rollout

1. Preserve the repository-root `AGENTS.md` as the common project contract and its existing `CLAUDE.md -> AGENTS.md` adapter.
2. Add a separate `agents/AGENTS.md` for personal, cross-repository behavior. Do not reuse the root file globally because it contains this repository's paths and workflow.
3. Add `claude/CLAUDE.md` and `codex/AGENTS.md` adapters to `agents/AGENTS.md`.
4. Keep the existing `~/.agents/skills -> agents/skills` hub and expose it at `~/.claude/skills`, so both products discover the same skills through their native paths.
5. Treat `codex/config.toml` separately. Audit it for private paths, tokens, MCP credentials, trust records, and machine-specific values before deciding whether to link it. If Codex writes machine state into the same file, prefer an installer that merges an allowlist of stable keys into the local file.
6. Leave `claude/rules/` Claude-only. Port only their durable behavioral content into the common `AGENTS.md` or portable skills; do not copy them into Codex's `rules/`, whose format and security purpose differ.
7. Make the installer/linker idempotent, fail on conflicting real files, back up before replacement, and validate every link target after installation.
8. Verify in fresh sessions: use Claude's `/memory` to inspect loaded instruction sources, and ask a new Codex run to summarize its loaded instructions. Test one shared skill explicitly in each tool.

## Bottom line

The industry-aligned approach is a small compatibility layer, not a forced universal configuration directory:

- canonical shared instructions: `AGENTS.md`;
- Claude adapter: `CLAUDE.md` or `@AGENTS.md`;
- canonical shared workflows: Agent Skills;
- native product settings kept separate;
- local state and credentials excluded by construction.

This resolves the observed behavior: Codex ignores `claude/rules/*.md` because they are Claude-specific, and neither product discovers skills from a generic `Agents` folder. Codex expects `.agents/skills`; Claude expects `.claude/skills`.
