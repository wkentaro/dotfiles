#!/bin/bash

if [ "$(uname)" = "Darwin" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if [ ! -f $HOME/.local/bin/binvox ]; then
  wget -q http://www.patrickmin.com/binvox/linux64/binvox -O $HOME/.local/bin/binvox
  chmod u+x $HOME/.local/bin/binvox
fi

if [ ! -f $HOME/.local/bin/viewvox ]; then
  wget -q http://www.patrickmin.com/viewvox/linux64/viewvox -O $HOME/.local/bin/viewvox
  chmod u+x $HOME/.local/bin/viewvox
fi
