#!/bin/sh

if [ -f ~/.local/bin/fpp ]; then
  echo "fpp is already installed"
  exit 0
fi

mkdir -p ~/.local/share
cd ~/.local/share
git clone https://github.com/facebook/PathPicker.git
ln -s $(pwd)/PathPicker/fpp ~/.local/bin/fpp