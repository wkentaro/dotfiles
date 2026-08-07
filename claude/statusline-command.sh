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
