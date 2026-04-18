# /bookmarks — List, resume, or delete session bookmarks

Read `~/.claude/bookmarks.json`, show the bookmarks, and interpret follow-up
instructions conversationally (`resume 2`, `delete 3`, `rename 1 <new>`).

## Workflow

### 1. Read the bookmarks

```bash
file=~/.claude/bookmarks.json
[ -f "$file" ] || { echo "EMPTY"; exit 0; }

jq -r '.bookmarks | to_entries[]
       | "\(.key+1)\t\(.value.date | split("T")[0])\t\(.value.name)\t\(.value.session_id[0:8])\t\(.value.cwd)"' "$file"
```

### 2. Render the list in the reply message (not only as bash output)

Bash output blocks can be collapsed by the client, so the user may not see
them. Always render the list as a markdown table inside the assistant
message itself:

```markdown
| # | DATE | NAME | SID | CWD |
|---|------|------|-----|-----|
| 1 | 2026-04-18 | create-bookmark-skills | 5dd1ab32 | /Users/... |
```

Empty bookmarks (`EMPTY` from step 1, or `.bookmarks == []`): reply with
`No bookmarks yet. Use /bookmark to add one.` and stop.

### 3. Prompt for action

After the table, ask:

```
Say `resume N`, `delete N`, or `rename N <new-name>`.
```

Wait for the user's next message and interpret it.

### Supported actions

#### `resume N`

Print the exact command for the user to run:

```
claude --resume <session-id>
```

Explain: this must be run in a fresh terminal (or after exiting this session).
From inside Claude Code, `/resume` opens the picker — pass the session id
shown above.

Don't try to run `claude --resume` from inside this session — it would nest.

#### `delete N`

```bash
tmp=$(mktemp)
jq --argjson i $((N-1)) 'del(.bookmarks[$i])' "$file" > "$tmp" && cat "$tmp" > "$file"
command rm -f "$tmp"
```

Then reprint the list.

#### `rename N <new-name>`

```bash
tmp=$(mktemp)
jq --argjson i $((N-1)) --arg name "<new-name>" \
   '.bookmarks[$i].name = $name' "$file" > "$tmp" && cat "$tmp" > "$file"
command rm -f "$tmp"
```

Then reprint the list.

## Notes

- Indexes are 1-based and refer to the list just shown. They shift after a
  delete — reprint before accepting another action.
- Show all bookmarks, not just those matching the current cwd. The `CWD`
  column disambiguates across projects.
- If the user types something ambiguous, ask for clarification rather than
  guessing.
- If `jq` is missing, abort with `brew install jq`.
