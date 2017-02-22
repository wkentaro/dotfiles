#!/bin/sh

if [ -e ~/.anaconda2 ]; then
  exit 0
fi

cd ~/Downloads
if [ ! -e Anaconda2-4.3.0-Linux-x86_64.sh ]; then
  wget 'https://repo.continuum.io/archive/Anaconda2-4.3.0-Linux-x86_64.sh'
fi
bash ./Anaconda2-4.3.0-Linux-x86_64.sh -p $HOME/.anaconda2 -b
