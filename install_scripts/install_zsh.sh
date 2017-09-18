#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

if [ "$(lsb_release -sr)" = "16.04" ]; then
  sudo apt-get install -qq -y zsh
  exit 0
fi

if [ -e /usr/local/bin/zsh ]; then
  echo "[$(basename $0)] zsh is already installed."
  exit 0
fi

set -x

sudo apt-get install -qq -y build-essential libncurses5-dev

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget -q http://sourceforge.net/projects/zsh/files/zsh/5.0.7/zsh-5.0.7.tar.bz2
tar jxf zsh-5.0.7.tar.bz2
cd zsh-5.0.7

./configure --with-tcsetpgrp
make -j
sudo make install

if ! grep '/usr/local/bin/zsh' /etc/shells; then
  echo '/usr/local/bin/zsh' | sudo tee --append /etc/shells &>/dev/null
fi

rm -rf $TMPDIR

set +x