# /bookmark — Bookmark the current Claude Code session

Save a pointer to the current session (session ID + memorable name + date)
to `~/.claude/bookmarks.json` so it can be found and resumed later via
`/bookmarks`.

## Usage

`/bookmark [name]`

- If `name` is provided, use it as-is (normalized to lowercase-kebab-case).
- If omitted, generate a short memorable name (3–6 words, kebab-case) that
  summarizes what this session has been about. Don't ask — pick one and show
  it in the confirmation.

## Workflow

### 1. Find the current session ID

Session files live in `~/.claude/projects/<slug>/<uuid>.jsonl`, where `<slug>`
is the current working directory with `/` and `.` both replaced by `-`. The
current session is the most recently modified `.jsonl` in that folder.

```bash
slug=$(pwd | sed 's#/#-#g; s#\.#-#g')
latest=$(ls -t "$HOME/.claude/projects/$slug"/*.jsonl 2>/dev/null | head -1)
[ -z "$latest" ] && { echo "No session file found under ~/.claude/projects/$slug/"; exit 1; }
session_id=$(basename "$latest" .jsonl)
```

### 2. Pick the name

If the user passed an argument, use it (lowercase, spaces → hyphens, strip
non-`[a-z0-9-]`). Otherwise generate one from the recent conversation topic.
Examples: `add-bookmark-skill`, `debug-cron-retry`, `refactor-auth-flow`.

If the name already exists in the file, append `-2`, `-3`, etc.

### 3. Append to the bookmarks file

```bash
file=~/.claude/bookmarks.json
mkdir -p ~/.claude
[ -f "$file" ] || echo '{"bookmarks": []}' > "$file"

tmp=$(mktemp)
jq --arg name "$name" \
   --arg date "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
   --arg id "$session_id" \
   --arg cwd "$(pwd)" \
   '.bookmarks += [{name:$name, date:$date, session_id:$id, cwd:$cwd}]' \
   "$file" > "$tmp" && cat "$tmp" > "$file"
command rm -f "$tmp"
```

Don't use `mv tmp file` — `mv` is often aliased to `mv -i` and will prompt.
The `cat "$tmp" > "$file"` pattern avoids that.

### 4. Confirm

Print one line:

```
Bookmarked: <name>  (<YYYY-MM-DD> • <8-char session id> • <cwd>)
```

## Notes

- One file, one format: `~/.claude/bookmarks.json`. Shared with `/bookmarks`.
- Never mutate `.jsonl` files directly — only the bookmarks index.
- If `jq` is missing, abort with a message asking the user to install it
  (`brew install jq`). Don't silently fall back to fragile text editing.
