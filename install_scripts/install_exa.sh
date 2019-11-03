#!/bin/bash

if [ "$(uname)" = "Darwin" ]; then
  brew install exa
  exit 0
fi

if [ -f $HOME/.local/bin/exa ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR
curl -L -O https://github.com/ogham/exa/releases/download/v0.9.0/exa-linux-x86_64-0.9.0.zip
unzip exa-linux-x86_64-0.9.0.zip
mv exa-linux-x86_64 $HOME/.local/bin/exa
rm -rf $TMPDIR
