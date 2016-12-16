#!/bin/sh

if [ -f ~/.local/bin/tmux ]; then
  exit 0
fi

cd /tmp

wget https://github.com/tmux/tmux/releases/download/2.2/tmux-2.2.tar.gz
tar zxvf tmux-2.2.tar.gz
cd tmux-2.2
./configure
make
make install prefix=~/.local
