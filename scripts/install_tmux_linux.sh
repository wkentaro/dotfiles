#!/bin/sh
#
if which atool >/dev/null 2>&1; then
    sudo aptitude install atool
fi
if $(dpkg --get-selections | grep libevent-dev | wc -l) != 1; then
    sudo aptitude install libevent-dev
fi
wget http://sourceforge.net/projects/tmux/files/tmux/tmux-1.9/tmux-1.9a.tar.gz
atool -x tmux-1.9a.tar.gz
cd tmux-1.9a
./configure && make && sudo make install