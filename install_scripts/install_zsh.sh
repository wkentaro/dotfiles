#!/bin/sh

wget http://sourceforge.net/projects/zsh/files/zsh/5.0.7/zsh-5.0.7.tar.bz2

tar jxvf zsh-5.0.7.tar.bz2

cd zsh-5.0.7

./configure --prefix=$HOME/.local

make && make install