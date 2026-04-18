# /bookmarks — List, resume, or delete session bookmarks

Read `~/.claude/bookmarks.json`, show the bookmarks, and interpret follow-up
instructions conversationally (`resume 2`, `delete 3`, `rename 1 <new>`).

## Current bookmarks (pre-loaded)

The line below is executed at skill-load time via the `!` inline-bash syntax,
so the bookmark list is already in context — no Bash tool call needed to
render the table or resolve `resume N`.

Format per line: `N \t DATE \t NAME \t FULL_SESSION_ID \t CWD`.
`EMPTY` means the file is missing; `[]` means no bookmarks yet.

!`file=~/.claude/bookmarks.json; [ -f "$file" ] || { echo EMPTY; exit 0; }; jq -r 'if (.bookmarks | length) == 0 then "[]" else (.bookmarks | to_entries[] | "\(.key+1)\t\(.value.date | split("T")[0])\t\(.value.name)\t\(.value.session_id)\t\(.value.cwd)") end' "$file"`

## Workflow

### 1. Render the list as a markdown table

Use the pre-loaded data above. Truncate `FULL_SESSION_ID` to the first 8 chars
for the `SID` column, but remember the full ID for `resume`.

```markdown
| # | DATE | NAME | SID | CWD |
|---|------|------|-----|-----|
| 1 | 2026-04-18 | create-bookmark-skills | 5dd1ab32 | /Users/... |
```

If the pre-loaded data is `EMPTY` or `[]`: reply with
`No bookmarks yet. Use /bookmark to add one.` and stop.

### 2. Prompt for action

After the table, ask:

```
Say `resume N`, `delete N`, or `rename N <new-name>`.
```

Wait for the user's next message and interpret it.

### Supported actions

#### `resume N`

No Bash call needed — the full session ID and cwd are already in the pre-loaded
data above. Render a fenced code block:

```
cd <cwd> && claude --resume <full-session-id>
```

Explain: this must be run in a fresh terminal (or after exiting this session).
The `cd` is required because Claude Code's `/resume` picker filters sessions
by current working directory — without it the session won't appear. Use the
full session id (not the 8-char prefix).

Don't try to run `claude --resume` from inside this session — it would nest.

#### `delete N`

```bash
file=~/.claude/bookmarks.json
tmp=$(mktemp)
jq --argjson i $((N-1)) 'del(.bookmarks[$i])' "$file" > "$tmp" && cat "$tmp" > "$file"
command rm -f "$tmp"
```

Then reprint the list (indexes have shifted — re-read the file).

#### `rename N <new-name>`

```bash
file=~/.claude/bookmarks.json
tmp=$(mktemp)
jq --argjson i $((N-1)) --arg name "<new-name>" \
   '.bookmarks[$i].name = $name' "$file" > "$tmp" && cat "$tmp" > "$file"
command rm -f "$tmp"
```

Then reprint the list.

## Notes

- Indexes are 1-based and refer to the list just shown. They shift after a
  delete — re-read the file and reprint before accepting another action.
- Show all bookmarks, not just those matching the current cwd. The `CWD`
  column disambiguates across projects.
- If the user types something ambiguous, ask for clarification rather than
  guessing.
- If `jq` is missing, the inline `!` block above will fail with a clear
  error — tell the user `brew install jq`.
