#!/bin/sh
input=$(cat)
cwd=$(echo "$input" | jq -r '.workspace.current_dir // .cwd')
model=$(echo "$input" | jq -r '.model.display_name // empty')
used_pct=$(echo "$input" | jq -r '.context_window.used_percentage // empty')
used_tokens=$(echo "$input" | jq -r '.context_window.total_input_tokens // empty')

user=$(whoami)
host=$(hostname -s)

# Host color matches wkentaro.zsh-theme: 215 (orange) local, 171 (pink) over ssh
if [ -n "$SSH_CONNECTION" ]; then
  host_color=171
else
  host_color=215
fi

home="$HOME"
short_cwd=$(echo "$cwd" | sed "s|^$home|~|")

time_str=$(date +"%H:%M")

# Git info matching zsh theme: branch color 206, staged green +, unstaged 222 *, dirty red …
branch=""
git_markers=""
if cd "$cwd" 2>/dev/null && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  branch=$(git branch --show-current 2>/dev/null)
  staged=$(git diff --cached --quiet 2>/dev/null; echo $?)
  unstaged=$(git diff --quiet 2>/dev/null; echo $?)
  if [ "$staged" = "1" ]; then
    git_markers="${git_markers}\033[32m+\033[0m"
  fi
  if [ "$unstaged" = "1" ]; then
    git_markers="${git_markers}\033[38;5;222m*\033[0m"
  fi
  if [ "$staged" = "0" ] && [ "$unstaged" = "0" ] && [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    git_markers="${git_markers}\033[31m…\033[0m"
  fi
fi

# Build the status line with ANSI colors matching wkentaro.zsh-theme
# user: 005 (magenta), host: ssh-dependent, cwd: 156 (light green), branch: 206 (pink), time: 147 (lavender)
line=$(printf '\033[38;5;5m%s\033[0m at \033[38;5;%sm%s\033[0m in \033[38;5;156m%s\033[0m' \
  "$user" "$host_color" "$host" "$short_cwd")

if [ -n "$branch" ]; then
  line="$line on \033[38;5;206m${branch}${git_markers}\033[0m"
fi

line="$line tm \033[38;5;147m$time_str\033[0m"

if [ -n "$used_pct" ]; then
  used_int=$(printf '%.0f' "$used_pct")
  ctx="ctx:${used_int}%"
  if [ -n "$used_tokens" ]; then
    if [ "$used_tokens" -ge 1000 ]; then
      tok="$((used_tokens / 1000))k"
    else
      tok="$used_tokens"
    fi
    ctx="ctx:${tok} ${used_int}%"
  fi
  line="$line  \033[38;5;244m[${ctx}]\033[0m"
fi

if [ -n "$model" ]; then
  line="$line \033[38;5;244m$model\033[0m"
fi

printf '%b\n' "$line"
