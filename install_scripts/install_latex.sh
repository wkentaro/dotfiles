#!/bin/sh


# latex
# ~~~~~


if [ "`uname`" != "Linux" ]; then
  exit 1
fi


sudo apt-get -y install --reinstall texlive
sudo apt-get -y install --reinstall texlive-lang-cjk
sudo apt-get -y install --reinstall xdvik-ja
sudo apt-get -y install --reinstall dvipsk-ja
sudo apt-get -y install --reinstall gv
sudo apt-get -y install --reinstall texlive-fonts-recommended texlive-fonts-extra
