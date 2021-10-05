#!/bin/sh

if [ "`uname`" != "Linux" ]; then
  exit 0
fi

set -x

sudo apt-get install -y texlive
sudo apt-get install -y texlive-lang-cjk
sudo apt-get install -y xdvik-ja
sudo apt-get install -y dvipsk-ja
sudo apt-get install -y gv
sudo apt-get install -y texlive-fonts-recommended texlive-fonts-extra

if [ "$(lsb_release -cs)" = "xenial" ]; then
  sudo apt-get install -y latexmk
  sudo apt-get install -y texlive-xetex
fi
