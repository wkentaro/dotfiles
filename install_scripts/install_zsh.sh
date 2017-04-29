#!/bin/sh

sudo apt-get install -qq -y build-essential libncurses5-dev

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget http://sourceforge.net/projects/zsh/files/zsh/5.0.7/zsh-5.0.7.tar.bz2
tar jxf zsh-5.0.7.tar.bz2
cd zsh-5.0.7

./configure
make
sudo make install

rm -rf $TMPDIR