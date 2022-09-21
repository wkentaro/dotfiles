#!/bin/sh

if [ -f ~/.local/bin/tmux ]; then
  exit 0
fi

uname=$(uname)
if [ $uname = Linux ]; then
  sudo -H apt-get install -qq -y libevent-dev libncurses-dev
elif [ $uname = Darwin ]; then
  brew install libevent ncurses
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

VERSION=3.2a
wget -q https://github.com/tmux/tmux/releases/download/${VERSION}/tmux-${VERSION}.tar.gz
tar zxf tmux-${VERSION}.tar.gz
cd tmux-${VERSION}

./configure --prefix=$HOME/.local
make -j
make install

rm -rf $TMPDIR