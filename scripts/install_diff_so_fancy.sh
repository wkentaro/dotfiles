#!/bin/bash

if [ -f $HOME/.local/bin/diff-so-fancy ]; then
  exit 0
fi

curl -L https://github.com/so-fancy/diff-so-fancy/releases/download/v1.4.4/diff-so-fancy -o $HOME/.local/bin/diff-so-fancy
chmod u+x $HOME/.local/bin/diff-so-fancy
