#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

which aptitude &>/dev/null || sudo apt-get install -qq -y aptitude
which zsh &>/dev/null || sudo apt-get install -qq -y zsh
which vim &>/dev/null || sudo apt-get install -qq -y vim
which xsel &>/dev/null || sudo apt-get install xsel
which xclip &>/dev/null || sudo apt-get install -qq -y xclip
which atool &>/dev/null || sudo apt-get install -qq -y atool
which tmux &>/dev/null || sudo apt-get install -qq -y tmux
sudo apt-get install -qq -y compizconfig-settings-manager compiz-plugins-extra
sudo apt-get install -qq -y indicator-multiload
sudo apt-get install -qq -y uim uim-mozc
which cowsay &>/dev/null || sudo apt-get install -qq -y cowsay
which fortune &>/dev/null || sudo apt-get install -qq -y fortune
which pandoc &>/dev/null || sudo apt-get install -qq -y pandoc
which gthumb &>/dev/null || sudo apt-get install -qq -y gthumb
