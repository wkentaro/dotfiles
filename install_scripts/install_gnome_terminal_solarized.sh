#!/bin/sh


if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

sudo apt-get install dconf-cli
~/.dotfiles/config/gnome-terminal-colors-solarized/install.sh
