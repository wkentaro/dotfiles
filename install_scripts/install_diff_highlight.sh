#!/usr/bin/env bash

if type diff-highlight &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

URL='https://github.com/git/git/archive/v2.13.2.tar.gz'
if which wget &>/dev/null; then
  wget -q $URL
else
  curl -s -L -O $URL
fi
tar zxf v2.13.2.tar.gz
cd git-2.13.2/contrib/diff-highlight
make -q

mkdir -p $HOME/.local/bin
mv diff-highlight $HOME/.local/bin
