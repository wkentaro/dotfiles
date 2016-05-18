#!/bin/sh

cd /tmp
wget https://github.com/vim/vim/archive/v7.4.865.tar.gz
tar zxvf v7.4.865
cd vim-7.4.865/src
make distclean
./configure
make
make install prefix=~/.local
