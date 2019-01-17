#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

set -e
set -x

sudo apt-get install -y software-properties-common

sudo apt-add-repository -y ppa:neovim-ppa/stable
sudo apt-get update
sudo apt-get install -y neovim

sudo apt-get install -y python-dev python-pip python3-dev python3-pip

sudo pip install neovim jedi
