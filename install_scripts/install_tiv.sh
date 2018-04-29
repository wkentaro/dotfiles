#!/bin/bash

if [ $(uname) != Linux ]; then
  exit 0
fi

if [ -e $HOME/.local/bin/tiv ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR
git clone https://github.com/stefanhaustein/TerminalImageViewer.git
cd TerminalImageViewer/src/main/cpp
make
cp tiv ~/.local/bin/tiv
rm -rf $TMPDIR
