#!/bin/bash

if [ -f ~/.local/bin/fpp ]; then
  echo "[$(basename $0)] fpp is already installed."
  exit 0
fi

set -x

mkdir -p ~/.local/share
cd ~/.local/share
git clone https://github.com/facebook/PathPicker.git
ln -s $(pwd)/PathPicker/fpp ~/.local/bin/fpp

set +x
