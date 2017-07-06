#!/usr/bin/env bash

# diff-highlight
# ~~~~~~~~~~~~~~

if type diff-highlight &>/dev/null; then
  echo "diff-highlight is already installed"
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget https://github.com/git/git/archive/v2.13.2.tar.gz
tar zxvf v2.13.2.tar.gz
cd git-2.13.2/contrib/diff-highlight
make

mkdir -p $HOME/.local/bin
mv diff-highlight $HOME/.local/bin
