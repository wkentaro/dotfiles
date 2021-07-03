#!/bin/bash

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

set -e
set -x

sudo apt-get install -y software-properties-common

sudo apt-add-repository -y ppa:neovim-ppa/unstable
sudo apt-get update
sudo apt-get install -y neovim

sudo apt-get install -y python3-dev python3-pip

bash $HERE/install_python3.7.sh
sudo python3.7 -m pip install neovim jedi pynvim
