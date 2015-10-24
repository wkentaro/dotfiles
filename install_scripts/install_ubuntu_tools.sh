#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

sudo apt-get install aptitude
sudo apt-get install zsh
sudo apt-get install vim
sudo apt-get install xsel xclip
sudo apt-get install atool
sudo apt-get install tmux
sudo apt-get install compizconfig-settings-manager compiz-plugins-extra

# python
sudo easy_install pip
sudo pip install -U pip setuptools distribute

# scientific tools
sudo pip install numpy
sudo pip install scipy
sudo pip install scikit-learn
sudo pip install scikit-image

# ros tools
sudo pip install wstool
sudo pip install catkin_tools
