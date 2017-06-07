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

VERSION=2.5
wget -q https://github.com/tmux/tmux/releases/download/${VERSION}/tmux-${VERSION}.tar.gz
tar zxf tmux-${VERSION}.tar.gz
cd tmux-${VERSION}

./configure
make -j
sudo make install

rm -rf $TMPDIR