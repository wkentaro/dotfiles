#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOTFILES_DIR=$(dirname $HERE)

sudo apt-get install -qq -y dconf-cli

$DOTFILES_DIR/config/gnome-terminal-colors-solarized/install.sh
