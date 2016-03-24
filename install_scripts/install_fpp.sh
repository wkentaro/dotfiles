#!/bin/sh

mkdir -p ~/.local/share
cd ~/.local/share
git clone https://github.com/facebook/PathPicker.git
ln -s $(pwd)/PathPicker/fpp ~/.local/bin/fpp