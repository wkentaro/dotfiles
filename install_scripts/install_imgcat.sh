#!/bin/bash

if [ -e $HOME/.local/bin/imgcat ]; then
  exit 0
fi

wget https://raw.githubusercontent.com/gnachman/iTerm2/master/tests/imgcat -O $HOME/.local/bin/imgcat
chmod u+x $HOME/.local/bin/imgcat
