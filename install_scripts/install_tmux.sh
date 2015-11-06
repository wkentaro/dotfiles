#!/bin/sh

if [ -f $HOME/.local/bin/tmux ]; then
  exit 0
fi

sudo aptitude install libevent-dev

wget http://sourceforge.net/projects/tmux/files/tmux/tmux-1.9/tmux-1.9a.tar.gz
tar zxvf tmux-1.9a.tar.gz
cd tmux-1.9a
./configure --prefix=$HOME/.local
make
make install