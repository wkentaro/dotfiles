#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

which aptitude &>/dev/null || sudo apt-get install aptitude -y
which zsh &>/dev/null || sudo apt-get install zsh -y
which vim &>/dev/null || sudo apt-get install vim -y
which xsel &>/dev/null || sudo apt-get install xsel -y
which xclip &>/dev/null || sudo apt-get install xclip -y
which atool &>/dev/null || sudo apt-get install atool -y
which tmux &>/dev/null || sudo apt-get install tmux -y
which compiz &>/dev/null || sudo apt-get install compizconfig-settings-manager compiz-plugins-extra -y
which cowsay &>/dev/null || sudo apt-get install cowsay -y
which fortune &>/dev/null || sudo apt-get install fortune -y

# python
if ! which pip &>/dev/null; then
  sudo easy_install pip
  sudo pip install -U pip setuptools distribute
fi

# scientific tools
pip show numpy &>/dev/null || sudo pip install numpy
pip show scipy &>/dev/null || sudo pip install scipy
pip show scikit-learn &>/dev/null || sudo pip install scikit-learn
pip show scikit-image &>/dev/null || sudo pip install scikit-image

# ros tools
pip show wstool &>/dev/null || sudo pip install wstool
pip show catkin-tools &>/dev/null || sudo pip install catkin-tools
