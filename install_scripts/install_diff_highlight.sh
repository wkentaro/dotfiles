#!/usr/bin/env bash

# diff-highlight
# ~~~~~~~~~~~~~~

if type diff-highlight &>/dev/null; then
  exit 0
fi

mkdir -p $HOME/.local/bin
wget https://raw.githubusercontent.com/git/git/master/contrib/diff-highlight/diff-highlight -O $HOME/.local/bin/diff-highlight
chmod u+x $HOME/.local/bin/diff-highlight
