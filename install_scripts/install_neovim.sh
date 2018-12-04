#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

set -e
set -x

sudo apt-get install software-properties-common

sudo apt-add-repository ppa:neovim-ppa/stable
sudo apt-get update
sudo apt-get install neovim

sudo apt-get install python-dev python-pip python3-dev python3-pip

sudo pip install neovim jedi
