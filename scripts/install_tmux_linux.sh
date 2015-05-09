#!/bin/sh
#
wget http://sourceforge.net/projects/tmux/files/tmux/tmux-1.9/tmux-1.9a.tar.gz
atool -x tmux-1.9a.tar.gz
cd tmu-1.9a
./configure && make && sudo make install