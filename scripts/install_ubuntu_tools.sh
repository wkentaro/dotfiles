#!/bin/sh

sudo apt-get install aptitude
sudo apt-get install zsh
sudo apt-get install vim
sudo apt-get install xsel xclip
sudo apt-get install atool
sudo apt-get install tmux
sudo apt-get install compizconfig-settings-manager compiz-plugins-extra

# python
sudo apt-get install python-pip

# upgrade basics
sudo pip install --upgrade pip
sudo pip install --upgrade setuptools
sudo pip install --upgrade distribute

# scientific tools
sudo pip install numpy
sudo pip install scipy
sudo pip install scikit-learn
sudo pip install scikit-image

# ros tools
sudo pip install wstool
sudo pip install catkin_tools
