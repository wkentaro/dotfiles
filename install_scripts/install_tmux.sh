#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if [ -f /usr/local/bin/tmux ]; then
  exit 0
fi

sudo -H apt-get install -qq -y libevent-dev libncurses-dev

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget -q https://github.com/tmux/tmux/releases/download/2.2/tmux-2.2.tar.gz
tar zxf tmux-2.2.tar.gz
cd tmux-2.2

./configure
make
sudo make install

rm -rf $TMPDIR