#!/bin/sh

if [ -e ~/anaconda2 ]; then
  exit 0
fi

cd ~/Downloads
wget 'https://repo.continuum.io/archive/Anaconda2-4.3.0-Linux-x86_64.sh'
bash ./Anaconda2-4.3.0-Linux-x86_64.sh
