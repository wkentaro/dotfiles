#!/usr/bin/env bash

# diff-highlight
# ~~~~~~~~~~~~~~

if type diff-highlight &>/dev/null; then
  echo "diff-highlight is already installed"
  exit 0
fi

mkdir -p $HOME/.local/bin
wget https://raw.githubusercontent.com/git/git/master/contrib/diff-highlight/diff-highlight -O $HOME/.local/bin/diff-highlight --no-check-certificate
chmod u+x $HOME/.local/bin/diff-highlight
