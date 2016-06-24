#!/bin/sh


if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

sudo apt-get isntall dconf-cli
~/.dotfiles/config/gnome-terminal-colors-solarized/install.sh
