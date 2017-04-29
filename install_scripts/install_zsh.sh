#!/bin/sh

wget http://sourceforge.net/projects/zsh/files/zsh/5.0.7/zsh-5.0.7.tar.bz2

tar jxvf zsh-5.0.7.tar.bz2

sudo apt-get install -qq -y build-essential libncurses5-dev

cd zsh-5.0.7

./configure

make && sudo make install