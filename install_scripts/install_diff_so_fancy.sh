#!/bin/bash

if [ -f $HOME/.local/bin/diff-so-fancy ]; then
  exit 0
fi

curl -L https://raw.githubusercontent.com/so-fancy/diff-so-fancy/master/third_party/build_fatpack/diff-so-fancy -o $HOME/.local/bin/diff-so-fancy
chmod u+x $HOME/.local/bin/diff-so-fancy
