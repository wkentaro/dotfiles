#!/bin/bash

if [ "$(uname)" = "Darwin" ]; then
  brew list | egrep '^exa$' &>/dev/null || brew install exa
  exit 0
fi

if [ -f $HOME/.local/bin/exa ]; then
  exit 0
fi

VERSION=v0.10.1

TMPDIR=$(mktemp -d)
cd $TMPDIR
curl -L -O https://github.com/ogham/exa/releases/download/$VERSION/exa-linux-x86_64-$VERSION.zip
unzip exa-linux-x86_64-$VERSION.zip
mv bin/exa $HOME/.local/bin/exa
mv completions/exa.zsh ~/.zsh/completions/_exa
rm -rf $TMPDIR
