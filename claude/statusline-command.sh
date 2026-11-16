#!/bin/sh
# Codex-style statusline:
# model effort fast · ~/path/to/repo · branch · Context 7% used · 29.4K used · #123 · weekly 55% left
input=$(cat)
cwd=$(echo "$input" | jq -r '.workspace.current_dir // .cwd // empty')
model=$(echo "$input" | jq -r '.model.display_name // empty')
effort=$(echo "$input" | jq -r '.effort.level // empty')
fast=$(echo "$input" | jq -r '.fast_mode // false')
used_pct=$(echo "$input" | jq -r '.context_window.used_percentage // empty')
used_tokens=$(echo "$input" | jq -r '.context_window.total_input_tokens // empty')
pr_number=$(echo "$input" | jq -r '.pr.number // empty')
weekly_used=$(echo "$input" | jq -r '.rate_limits.seven_day.used_percentage // empty')

# Gateway sessions get no rate_limits, so read the weekly quota from the
# proxy's own OAuth tokens on lora, cached and refreshed in the background.
# shortcut: the proxy host is hardcoded; read it from ANTHROPIC_BASE_URL if it moves.
if [ -z "$weekly_used" ]; then
  cache="$HOME/.cache/claude-gateway-quota.json"
  mkdir -p "$HOME/.cache"
  # The lock outlives each attempt, so a failing host is retried at most every 2 minutes.
  find "$cache.lock" -maxdepth 0 -mmin +2 -exec rmdir {} \; 2>/dev/null
  if [ -z "$(find "$cache" -mmin -5 2>/dev/null)" ] && mkdir "$cache.lock" 2>/dev/null; then
    (
      if [ "$(hostname -s)" = lora ]; then
        set -- python3 -
      else
        set -- ssh -o BatchMode=yes -o ConnectTimeout=5 lora python3 -
      fi
      timeout 60 "$@" >"$cache.tmp" <<'EOF' &&
import glob, json, os, urllib.request
out = {}
for path in glob.glob(os.path.expanduser("~/.cli-proxy-api/*.json")):
    try:
        auth = json.load(open(path))
        if auth.get("disabled"):
            continue
        headers = {"Authorization": "Bearer " + auth.get("access_token", "")}
        if auth.get("type") == "claude":
            headers["anthropic-beta"] = "oauth-2025-04-20"
            req = urllib.request.Request("https://api.anthropic.com/api/oauth/usage", headers=headers)
            out["claude"] = json.load(urllib.request.urlopen(req, timeout=10))["seven_day"]["utilization"]
        elif auth.get("type") == "codex":
            headers["ChatGPT-Account-Id"] = auth["account_id"]
            req = urllib.request.Request("https://chatgpt.com/backend-api/wham/usage", headers=headers)
            limit = json.load(urllib.request.urlopen(req, timeout=10))["rate_limit"]
            weekly = [w for w in (limit.get("primary_window"), limit.get("secondary_window")) if w and w["limit_window_seconds"] == 604800]
            out["codex"] = weekly[0]["used_percent"]
    except Exception:
        pass
print(json.dumps(out))
EOF
        { cat "$cache" 2>/dev/null; cat "$cache.tmp"; } | jq -s add >"$cache.new" && mv "$cache.new" "$cache"
    ) </dev/null >/dev/null 2>&1 &
  fi
  case $(echo "$input" | jq -r '.model.id // empty') in
    gpt-*) provider=codex ;;
    *) provider=claude ;;
  esac
  weekly_used=$(jq -r ".$provider // empty" "$cache" 2>/dev/null)
fi

# Colors sampled from Codex's statusline rendering (Catppuccin Mocha)
yellow='\033[38;2;249;226;175m'
green='\033[38;2;166;227;161m'
blue='\033[38;2;137;180;250m'
peach='\033[38;2;250;179;135m'
red='\033[38;2;243;139;168m'
teal='\033[38;2;148;226;213m'
reset='\033[0m'

sep=' · '
line=''

append() {
  if [ -z "$line" ]; then
    line="$1"
  else
    line="$line$sep$1"
  fi
}

if [ -n "$model" ]; then
  seg="$model"
  [ -n "$effort" ] && seg="$seg $effort"
  [ "$fast" = "true" ] && seg="$seg fast"
  append "${yellow}${seg}${reset}"
fi

if [ -n "$cwd" ]; then
  short_cwd=$(echo "$cwd" | sed "s|^$HOME|~|")
  append "${green}${short_cwd}${reset}"
fi

branch=''
if [ -n "$cwd" ] && cd "$cwd" 2>/dev/null; then
  branch=$(git branch --show-current 2>/dev/null)
fi
[ -n "$branch" ] && append "${blue}${branch}${reset}"

if [ -n "$used_pct" ]; then
  used_int=$(printf '%.0f' "$used_pct")
  append "${peach}Context ${used_int}% used${reset}"
fi

if [ -n "$used_tokens" ]; then
  tok=$(awk -v t="$used_tokens" 'BEGIN { if (t >= 1000) printf "%.1fK", t / 1000; else printf "%d", t }')
  append "${peach}${tok} used${reset}"
fi

[ -n "$pr_number" ] && append "${teal}#${pr_number}${reset}"

if [ -n "$weekly_used" ]; then
  weekly_left=$((100 - $(printf '%.0f' "$weekly_used")))
  append "${red}weekly ${weekly_left}% left${reset}"
fi

printf '%b\n' "$line"
