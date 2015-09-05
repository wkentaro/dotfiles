#!/bin/sh


# latex
# ~~~~~


if [ "`uname`" != "Linux" ]; then
  exit 1
fi


sudo apt-get -y install texlive
sudo apt-get -y install texlive-lang-cjk
sudo apt-get -y install xdvik-ja
sudo apt-get -y install dvipsk-ja
sudo apt-get -y install gv
sudo apt-get -y install texlive-fonts-recommended texlive-fonts-extra
