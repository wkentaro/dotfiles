#!/bin/bash

if [ -f ~/.local/bin/fpp ]; then
  exit 0
fi

mkdir -p ~/.local/share
cd ~/.local/share
git clone -q https://github.com/facebook/PathPicker.git
ln -s $(pwd)/PathPicker/fpp ~/.local/bin/fpp
