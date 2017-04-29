#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if [ -e /usr/local/bin/zsh ]; then
  exit 0
fi

sudo apt-get install -qq -y build-essential libncurses5-dev

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget http://sourceforge.net/projects/zsh/files/zsh/5.0.7/zsh-5.0.7.tar.bz2
tar jxf zsh-5.0.7.tar.bz2
cd zsh-5.0.7

./configure
make
sudo make install

if ! grep '/usr/local/bin/zsh' /etc/shells; then
  echo '/usr/local/bin/zsh' | sudo tee --append /etc/shells &>/dev/null
fi

rm -rf $TMPDIR