#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

sudo apt-get install aptitude -y
sudo apt-get install zsh -y
sudo apt-get install vim -y
sudo apt-get install xsel xclip -y
sudo apt-get install atool -y
sudo apt-get install tmux -y
sudo apt-get install compizconfig-settings-manager compiz-plugins-extra -y
sudo apt-get install cowsay fortune -y

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
