#!/bin/bash

SKILLS_DIR="$HOME/.claude/skills"

REPOS=(
  "https://github.com/blader/humanizer.git"
  "https://github.com/garrytan/gstack.git"
)

for repo in "${REPOS[@]}"; do
  name=$(basename "$repo" .git)
  dest="$SKILLS_DIR/$name"

  if [ -d "$dest" ]; then
    echo "skip: $name (already installed)"
    continue
  fi

  echo "install: $name"
  git clone "$repo" "$dest"
done

npx skills add wkentaro/git-hunk -a claude-code -g -y
