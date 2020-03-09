#!/bin/sh

if [ "`uname`" != "Linux" ]; then
  exit 0
fi

sudo apt-get install -qq -y texlive
sudo apt-get install -qq -y texlive-lang-cjk
sudo apt-get install -qq -y xdvik-ja
sudo apt-get install -qq -y dvipsk-ja
sudo apt-get install -qq -y gv
sudo apt-get install -qq -y texlive-fonts-recommended texlive-fonts-extra

if [ "$(lsb_release -cs)" = "xenial" ]; then
  sudo apt-get install -qq -y latexmk
  sudo apt-get install -qq -y texlive-xetex
fi
